// =============================================================================
// Gust v1.0 - fetch stage
//
//  * Requests 32-bit words on the instruction bus, at most one outstanding,
//    back-to-back (a new request can go out in the cycle a response arrives).
//  * A 5-parcel (80-bit) elastic buffer re-aligns the 16/32-bit instruction
//    stream.  Five parcels is the minimum that sustains one instruction per
//    cycle for every alignment (worst case: 1 leftover parcel + 2 arriving
//    + 2 already requested).
//  * The head of the buffer is expanded (RVC -> 32 bit) and pre-decoded.
//    Static prediction: JAL always taken, conditional branches taken when the
//    offset is negative (loops).  A predicted-taken instruction redirects the
//    fetch address in the same cycle it is handed to the X stage, so a
//    correctly predicted loop branch or a direct jump costs zero bubbles.
//  * One adder computes both "next sequential PC" and "predicted target".
//
// Bus protocol (both Gust buses): req/gnt address phase, then exactly one
// rvalid pulse per granted request, at least one cycle later.
// =============================================================================
module gust_fetch #(
    parameter [31:0] RESET_VECTOR = 32'h0000_0000,
    parameter        BRANCH_PRED  = 1,
    parameter        EXT_B        = 1,
    parameter        EXT_ZCB      = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        fetch_en,     // 0: issue no new requests (sleep/debug)
    input  wire        flush,        // redirect from the core (has priority)
    input  wire [31:0] flush_pc,

    output wire        ibus_req,
    output wire [31:0] ibus_addr,
    input  wire        ibus_gnt,
    input  wire        ibus_rvalid,
    input  wire [31:0] ibus_rdata,
    input  wire        ibus_err,

    output wire        f_valid,      // head instruction available
    input  wire        f_ready,      // X stage accepts it this cycle
    output wire [31:0] f_instr,      // always a 32-bit encoding
    output wire [31:0] f_pc,
    output wire        f_compressed,
    output wire        f_illegal,    // reserved compressed encoding
    output wire        f_fetch_err,  // bus error on (part of) the instruction
    output wire        f_pred_taken,

    output wire        busy          // a response is still outstanding
);
    localparam CAP = 5;

    reg  [15:0]     buf_q [0:CAP-1];
    reg  [CAP-1:0]  err_q;
    reg  [2:0]      cnt_q;        // valid parcels in buf_q
    reg  [31:0]     pc_q;         // PC of the head instruction
    reg  [29:0]     faddr_q;      // next word address to request
    reg             pending_q;    // request granted, response not yet seen
    reg             discard_q;    // ... and that response must be dropped
    reg             skip_q;       // next accepted word: drop the low parcel

    // -------------------------------------------------- incoming response
    wire        resp_good = ibus_rvalid & pending_q & ~discard_q;
    wire [1:0]  in_n      = skip_q ? 2'd1 : 2'd2;
    wire [15:0] in0       = skip_q ? ibus_rdata[31:16] : ibus_rdata[15:0];
    wire [15:0] in1       = ibus_rdata[31:16];

    wire [3:0]  vcnt = {1'b0, cnt_q} + (resp_good ? {2'b0, in_n} : 4'd0);

    // "view": buffer contents followed by the parcels arriving this cycle
    wire [15:0] v    [0:CAP+1];
    wire        verr [0:CAP+1];
    genvar gi;
    generate
        for (gi = 0; gi < CAP + 2; gi = gi + 1) begin : g_view
            if (gi < CAP) begin : g_in_buf
                wire from_buf = (cnt_q > gi);
                assign v[gi]    = from_buf ? buf_q[gi] :
                                  ((cnt_q == gi) ? in0 : in1);
                assign verr[gi] = from_buf ? err_q[gi] : ibus_err;
            end else begin : g_past_buf
                assign v[gi]    = (cnt_q == gi) ? in0 : in1;
                assign verr[gi] = ibus_err;
            end
        end
    endgenerate

    // ---------------------------------------------------- head instruction
    wire [15:0] h0       = v[0];
    wire [15:0] h1       = v[1];
    wire        h0_valid = (vcnt >= 4'd1);
    wire        h1_valid = (vcnt >= 4'd2);
    wire        is_c     = (h0[1:0] != 2'b11);
    wire        head_ok  = h0_valid & (is_c | h1_valid | verr[0]);
    wire        head_err = verr[0] | (~is_c & h1_valid & verr[1]);

    wire [31:0] c_instr;
    wire        c_illegal;
    gust_rvc #(.EXT_ZCB(EXT_ZCB), .EXT_B(EXT_B)) u_rvc (
        .c(h0), .instr(c_instr), .illegal(c_illegal)
    );

    assign f_valid      = head_ok;
    assign f_instr      = is_c ? c_instr : {h1, h0};
    assign f_pc         = pc_q;
    assign f_compressed = is_c;
    assign f_illegal    = is_c & c_illegal;
    assign f_fetch_err  = head_err;

    // ------------------------------------------------- static prediction
    wire is_jal = (f_instr[6:0] == 7'b1101111);
    wire is_br  = (f_instr[6:0] == 7'b1100011);
    wire [31:0] imm_j = {{12{f_instr[31]}}, f_instr[19:12], f_instr[20],
                         f_instr[30:21], 1'b0};
    wire [31:0] imm_b = {{20{f_instr[31]}}, f_instr[7], f_instr[30:25],
                         f_instr[11:8], 1'b0};
    wire pred = (BRANCH_PRED != 0) & ~head_err & ~f_illegal &
                (is_jal | (is_br & f_instr[31]));
    assign f_pred_taken = pred;

    wire [31:0] pc_step = pred ? (is_jal ? imm_j : imm_b)
                               : (is_c ? 32'd2 : 32'd4);
    wire [31:0] pc_next = pc_q + pc_step;

    wire f_take        = head_ok & f_ready & ~flush;
    wire pred_redirect = f_take & pred;
    wire redirect      = flush | pred_redirect;
    wire [31:0] target = flush ? flush_pc : pc_next;

    // ------------------------------------------------------------ requests
    // Space check uses registered state only (worst case: nothing consumed),
    // so ibus_req never depends on the X stage's stall logic.
    wire       can_req  = ~pending_q | ibus_rvalid;
    wire [3:0] worst    = {1'b0, cnt_q} +
                          ((pending_q & ~discard_q) ? 4'd2 : 4'd0) + 4'd2;
    wire       space_ok = (worst <= CAP);

    assign ibus_req  = fetch_en & can_req & (redirect | space_ok);
    assign ibus_addr = redirect ? {target[31:2], 2'b00} : {faddr_q, 2'b00};
    assign busy      = pending_q;

    wire granted = ibus_req & ibus_gnt;

    // ------------------------------------------------------- state update
    // parcels consumed by the X stage this cycle (never more than we have)
    wire [1:0] k = !f_take                 ? 2'd0 :
                   (is_c | ~h1_valid)      ? 2'd1 : 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_q     <= 3'd0;
            pending_q <= 1'b0;
            discard_q <= 1'b0;
            skip_q    <= RESET_VECTOR[1];
            pc_q      <= RESET_VECTOR;
            faddr_q   <= RESET_VECTOR[31:2];
        end else begin
            if (granted)          pending_q <= 1'b1;
            else if (ibus_rvalid) pending_q <= 1'b0;

            if (redirect)         discard_q <= pending_q & ~ibus_rvalid;
            else if (ibus_rvalid) discard_q <= 1'b0;

            if (redirect)         faddr_q <= target[31:2] + {29'b0, granted};
            else if (granted)     faddr_q <= faddr_q + 30'd1;

            if (redirect)         skip_q <= target[1];
            else if (resp_good)   skip_q <= 1'b0;

            if (flush)            pc_q <= {flush_pc[31:1], 1'b0};
            else if (f_take)      pc_q <= pc_next;

            if (redirect)         cnt_q <= 3'd0;
            else                  cnt_q <= vcnt[2:0] - {1'b0, k};
        end
    end

    // buffer data: only clocked when something moves
    wire buf_we = (resp_good | f_take) & ~redirect;
    generate
        for (gi = 0; gi < CAP; gi = gi + 1) begin : g_buf
            always @(posedge clk) begin
                if (buf_we) begin
                    buf_q[gi] <= (k == 2'd0) ? v[gi] :
                                 (k == 2'd1) ? v[gi+1] : v[gi+2];
                    err_q[gi] <= (k == 2'd0) ? verr[gi] :
                                 (k == 2'd1) ? verr[gi+1] : verr[gi+2];
                end
            end
        end
    endgenerate
endmodule
