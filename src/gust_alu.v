// =============================================================================
// Gust v1.0 - ALU (RV32I + optional Zba/Zbb/Zbs)
//
// One shared adder/subtractor, one funnel shifter that also does rotates and
// single-bit extract, and a small bit-count block.  The shifter and bit-count
// inputs are operand-isolated: they see zeros unless the current operation
// needs them, so they do not toggle on every add/compare.
//
// `add` exposes the raw adder output (used for load/store addresses and JALR
// targets without paying for the result mux).  `eq/lt/ltu` are the comparison
// flags used by conditional branches (the decoder selects SUB for branches).
// =============================================================================
`include "gust_defs.vh"

module gust_alu #(
    parameter EXT_B       = 1,
    parameter OPERAND_ISO = 1
) (
    input  wire [5:0]  op,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] result,
    output wire [31:0] add,
    output wire        eq,
    output wire        lt,
    output wire        ltu
);
    wire has_b = (EXT_B != 0);

    // ------------------------------------------------------------------ adder
    wire is_sub = (op == `GUST_ALU_SUB)  | (op == `GUST_ALU_SLT)  |
                  (op == `GUST_ALU_SLTU) |
                  (has_b & ((op == `GUST_ALU_MIN) | (op == `GUST_ALU_MINU) |
                            (op == `GUST_ALU_MAX) | (op == `GUST_ALU_MAXU)));

    wire [1:0] sh_add = !has_b                    ? 2'd0 :
                        (op == `GUST_ALU_SH1ADD) ? 2'd1 :
                        (op == `GUST_ALU_SH2ADD) ? 2'd2 :
                        (op == `GUST_ALU_SH3ADD) ? 2'd3 : 2'd0;
    wire [31:0] add_a = a << sh_add;
    wire [31:0] add_b = b ^ {32{is_sub}};
    wire [32:0] sum   = {1'b0, add_a} + {1'b0, add_b} + {32'b0, is_sub};

    assign add = sum[31:0];
    assign eq  = (a == b);
    assign ltu = ~sum[32];                               // valid when is_sub
    assign lt  = (a[31] ^ b[31]) ? a[31] : sum[31];      // valid when is_sub

    // --------------------------------------------------------------- shifter
    wire sh_left  = (op == `GUST_ALU_SLL) | (has_b & (op == `GUST_ALU_ROL));
    wire sh_arith = (op == `GUST_ALU_SRA);
    wire sh_rot   = has_b & ((op == `GUST_ALU_ROL) | (op == `GUST_ALU_ROR));
    wire sh_used  = sh_left | sh_arith | sh_rot | (op == `GUST_ALU_SRL) |
                    (has_b & (op == `GUST_ALU_BEXT));

    function [31:0] bitrev;
        input [31:0] v;
        integer i;
        begin
            for (i = 0; i < 32; i = i + 1) bitrev[i] = v[31 - i];
        end
    endfunction

    wire [31:0] sh_a   = (OPERAND_ISO != 0 && !sh_used) ? 32'b0 : a;
    wire [4:0]  shamt  = (OPERAND_ISO != 0 && !sh_used) ? 5'b0  : b[4:0];
    wire [31:0] sh_in  = sh_left ? bitrev(sh_a) : sh_a;
    wire [31:0] sh_hi  = sh_rot ? sh_in : {32{sh_arith & sh_in[31]}};
    wire [63:0] sh_ext = {sh_hi, sh_in} >> shamt;
    wire [31:0] sh_r   = sh_ext[31:0];
    wire [31:0] sh_out = sh_left ? bitrev(sh_r) : sh_r;

    // ------------------------------------------------------------ logic ops
    wire        inv_b   = has_b & ((op == `GUST_ALU_ANDN) | (op == `GUST_ALU_ORN) |
                                   (op == `GUST_ALU_XNOR));
    wire [31:0] lb      = b ^ {32{inv_b}};
    wire [31:0] and_res = a & lb;
    wire [31:0] or_res  = a | lb;
    wire [31:0] xor_res = a ^ lb;

    // ------------------------------------------- bit counting (clz/ctz/cpop)
    wire cnt_used = has_b & ((op == `GUST_ALU_CLZ) | (op == `GUST_ALU_CTZ) |
                             (op == `GUST_ALU_CPOP));
    wire [31:0] cnt_a = (OPERAND_ISO != 0 && !cnt_used) ? 32'b0 : a;

    // count trailing zeros with a 5-level binary search; clz = ctz(bitrev)
    wire [31:0] tz_in = (op == `GUST_ALU_CLZ) ? bitrev(cnt_a) : cnt_a;
    wire        tz16  = ~|tz_in[15:0];
    wire [15:0] tz_v16 = tz16 ? tz_in[31:16] : tz_in[15:0];
    wire        tz8   = ~|tz_v16[7:0];
    wire [7:0]  tz_v8 = tz8 ? tz_v16[15:8] : tz_v16[7:0];
    wire        tz4   = ~|tz_v8[3:0];
    wire [3:0]  tz_v4 = tz4 ? tz_v8[7:4] : tz_v8[3:0];
    wire        tz2   = ~|tz_v4[1:0];
    wire [1:0]  tz_v2 = tz2 ? tz_v4[3:2] : tz_v4[1:0];
    wire        tz1   = ~tz_v2[0];
    wire        tz_all = ~|tz_in;
    wire [5:0]  tz_cnt = tz_all ? 6'd32 : {1'b0, tz16, tz8, tz4, tz2, tz1};

    // population count as a balanced adder tree
    wire [1:0] pc2 [0:15];
    wire [2:0] pc3 [0:7];
    wire [3:0] pc4 [0:3];
    wire [4:0] pc5 [0:1];
    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : g_pc2
            assign pc2[g] = {1'b0, cnt_a[2*g]} + {1'b0, cnt_a[2*g+1]};
        end
        for (g = 0; g < 8; g = g + 1) begin : g_pc3
            assign pc3[g] = {1'b0, pc2[2*g]} + {1'b0, pc2[2*g+1]};
        end
        for (g = 0; g < 4; g = g + 1) begin : g_pc4
            assign pc4[g] = {1'b0, pc3[2*g]} + {1'b0, pc3[2*g+1]};
        end
        for (g = 0; g < 2; g = g + 1) begin : g_pc5
            assign pc5[g] = {1'b0, pc4[2*g]} + {1'b0, pc4[2*g+1]};
        end
    endgenerate
    wire [5:0] pop_cnt = {1'b0, pc5[0]} + {1'b0, pc5[1]};

    // ------------------------------------------------------- single-bit ops
    wire [31:0] onehot = 32'b1 << b[4:0];

    // ------------------------------------------------------------ min / max
    wire        a_less  = ((op == `GUST_ALU_MINU) | (op == `GUST_ALU_MAXU)) ? ltu : lt;
    wire        pick_a  = ((op == `GUST_ALU_MIN) | (op == `GUST_ALU_MINU)) ? a_less : ~a_less;
    wire [31:0] minmax  = pick_a ? a : b;

    // ----------------------------------------------------------- result mux
    always @* begin
        case (op)
        `GUST_ALU_ADD,
        `GUST_ALU_SUB:    result = sum[31:0];
        `GUST_ALU_SLT:    result = {31'b0, lt};
        `GUST_ALU_SLTU:   result = {31'b0, ltu};
        `GUST_ALU_XOR:    result = xor_res;
        `GUST_ALU_OR:     result = or_res;
        `GUST_ALU_AND:    result = and_res;
        `GUST_ALU_SLL,
        `GUST_ALU_SRL,
        `GUST_ALU_SRA:    result = sh_out;
        default:          result = sum[31:0];
        endcase
        if (has_b) begin
            case (op)
            `GUST_ALU_SH1ADD,
            `GUST_ALU_SH2ADD,
            `GUST_ALU_SH3ADD: result = sum[31:0];
            `GUST_ALU_ANDN:   result = and_res;
            `GUST_ALU_ORN:    result = or_res;
            `GUST_ALU_XNOR:   result = xor_res;
            `GUST_ALU_MIN,
            `GUST_ALU_MINU,
            `GUST_ALU_MAX,
            `GUST_ALU_MAXU:   result = minmax;
            `GUST_ALU_ROL,
            `GUST_ALU_ROR:    result = sh_out;
            `GUST_ALU_CLZ,
            `GUST_ALU_CTZ:    result = {26'b0, tz_cnt};
            `GUST_ALU_CPOP:   result = {26'b0, pop_cnt};
            `GUST_ALU_SEXTB:  result = {{24{a[7]}}, a[7:0]};
            `GUST_ALU_SEXTH:  result = {{16{a[15]}}, a[15:0]};
            `GUST_ALU_ZEXTH:  result = {16'b0, a[15:0]};
            `GUST_ALU_ORCB:   result = {{8{|a[31:24]}}, {8{|a[23:16]}},
                                        {8{|a[15:8]}},  {8{|a[7:0]}}};
            `GUST_ALU_REV8:   result = {a[7:0], a[15:8], a[23:16], a[31:24]};
            `GUST_ALU_BSET:   result = a | onehot;
            `GUST_ALU_BCLR:   result = a & ~onehot;
            `GUST_ALU_BINV:   result = a ^ onehot;
            `GUST_ALU_BEXT:   result = {31'b0, sh_r[0]};
            default: ;
            endcase
        end
    end
endmodule
