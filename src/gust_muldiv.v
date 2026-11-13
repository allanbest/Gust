// =============================================================================
// Gust v1.0 - multiply / divide
//
//   gust_mul    : combinational 32x32 -> low 32 bits.  Used for MUL when
//                 MUL_FAST=1.  Inputs are operand-isolated by the core, so the
//                 array only switches while a MUL is actually in the X stage.
//
//   gust_muldiv : sequential unit for MULH/MULHSU/MULHU/DIV[U]/REM[U]
//                 (and MUL when MUL_FAST=0).  One 34-bit adder is shared by
//                 every step; 1 bit per cycle.
//                   multiply: 32 cycles (two's-complement shift-add, the last
//                             partial product is subtracted for a signed
//                             multiplier)
//                   divide:   1 + 32 + 1 cycles (restoring on magnitudes with
//                             a sign fix-up step)
//                 RISC-V corner cases fall out of the datapath:
//                   x / 0    -> quotient = all ones, remainder = x
//                   MIN / -1 -> quotient = MIN,      remainder = 0
//
// Handshake: pulse `start` with operands held stable; `done` rises when the
// result is valid and stays up until `ack` (result consumed) or `abort`.
// =============================================================================
module gust_mul (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [31:0] p
);
    assign p = a * b;
endmodule

module gust_muldiv (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        abort,
    input  wire        ack,
    input  wire [2:0]  op,        // funct3 of the M-extension instruction
    input  wire [31:0] a,         // rs1
    input  wire [31:0] b,         // rs2
    output wire        busy,
    output wire        done,
    output wire [31:0] result
);
    localparam [2:0] S_IDLE = 3'd0, S_MUL = 3'd1, S_DIVINIT = 3'd2,
                     S_DIV  = 3'd3, S_DIVFIX = 3'd4, S_DONE = 3'd5;

    reg [2:0]  state;
    reg [5:0]  cnt;
    reg [33:0] hi;      // multiply: upper partial product / divide: remainder
    reg [31:0] lo;      // multiply: multiplier, then low product / quotient
    reg [31:0] res_q;

    wire is_div   = op[2];
    // multiply operand signedness: MULH s*s, MULHSU s*u, MULHU u*u, MUL any
    wire a_signed = is_div ? ~op[0] : (op[1:0] != 2'b11);
    wire b_signed = is_div ? ~op[0] : (op[1:0] == 2'b01);
    wire a_neg    = a_signed & a[31];
    wire b_neg    = b_signed & b[31];
    wire b_zero   = ~|b;

    wire last = (cnt == 6'd31);

    // ----------------------------------------------------- shared adder
    reg  [33:0] ad_x, ad_y;
    reg         ad_c;
    wire [33:0] ad_s = ad_x + ad_y + {33'b0, ad_c};

    // multiply step: add (or, in the last step of a signed multiplier,
    // subtract) the sign-extended multiplicand when the current bit is set
    wire        mul_sub = last & a_signed;
    wire [33:0] b_ext   = {{2{b_neg}}, b};
    // divide step: trial = {rem, next dividend bit} - |b|
    wire [33:0] div_sh  = {1'b0, hi[31:0], lo[31]};
    // final negation select
    wire        want_rem = op[1];
    wire [31:0] fix_in   = want_rem ? hi[31:0] : lo;
    wire        fix_neg  = want_rem ? a_neg : ((a_neg ^ b_neg) & ~b_zero);

    always @* begin
        ad_x = 34'b0;
        ad_y = 34'b0;
        ad_c = 1'b0;
        case (state)
        S_MUL: begin
            ad_x = hi;
            ad_y = lo[0] ? (b_ext ^ {34{mul_sub}}) : 34'b0;
            ad_c = lo[0] & mul_sub;
        end
        S_DIVINIT: begin                 // -a
            ad_y = {2'b11, ~a};
            ad_c = 1'b1;
        end
        S_DIV: begin                     // shifted remainder - |b|
            ad_x = div_sh;
            ad_y = {2'b11, (b_neg ? b : ~b)};
            ad_c = ~b_neg;
        end
        S_DIVFIX: begin                  // -fix_in
            ad_y = {2'b11, ~fix_in};
            ad_c = 1'b1;
        end
        default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            cnt   <= 6'd0;
        end else if (abort) begin
            state <= S_IDLE;
        end else begin
            case (state)
            S_IDLE: begin
                cnt <= 6'd0;
                if (start) state <= is_div ? S_DIVINIT : S_MUL;
            end
            S_MUL: begin
                cnt <= cnt + 6'd1;
                if (last) state <= S_DONE;
            end
            S_DIVINIT: state <= S_DIV;
            S_DIV: begin
                cnt <= cnt + 6'd1;
                if (last) state <= S_DIVFIX;
            end
            S_DIVFIX: state <= S_DONE;
            S_DONE:   if (ack) state <= S_IDLE;
            default:  state <= S_IDLE;
            endcase
        end
    end

    // datapath registers (no reset needed: always written before being read)
    always @(posedge clk) begin
        case (state)
        S_IDLE: if (start) begin
            hi <= 34'b0;
            lo <= a;                      // multiplier, or dividend if a >= 0
        end
        S_MUL: begin
            hi <= {ad_s[33], ad_s[33:1]};
            lo <= {ad_s[0], lo[31:1]};
            if (last) res_q <= (op[1:0] == 2'b00) ? {ad_s[0], lo[31:1]}
                                                   : ad_s[32:1];
        end
        S_DIVINIT: if (a_neg) lo <= ad_s[31:0];      // |a|
        S_DIV: begin
            if (ad_s[33]) begin           // trial negative: keep remainder
                hi <= {2'b00, div_sh[31:0]};
                lo <= {lo[30:0], 1'b0};
            end else begin
                hi <= {2'b00, ad_s[31:0]};
                lo <= {lo[30:0], 1'b1};
            end
        end
        S_DIVFIX: res_q <= fix_neg ? ad_s[31:0] : fix_in;
        default: ;
        endcase
    end

    assign busy   = (state != S_IDLE) & (state != S_DONE);
    assign done   = (state == S_DONE);
    assign result = res_q;
endmodule
