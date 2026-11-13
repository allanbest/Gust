// =============================================================================
// Gust v1.0 - control and status registers, trap state, interrupt selection
//
// Machine mode only (plus debug mode).  Implements:
//   mstatus(MIE,MPIE) misa mie mip mtvec(direct+vectored) mscratch mepc
//   mcause mtval mcountinhibit mcycle[h] minstret[h] (+ read-only cycle /
//   instret aliases), mvendorid marchid mimpid mhartid mconfigptr mstatush,
//   hpm counters/events as read-only zero, tselect/tdata1/tdata2 as read-only
//   zero ("no triggers"), dcsr and dpc (debug-mode access only).
//
// Interrupts: MSI(3), MTI(7), MEI(11) and 16 fast local lines on mip[31:16].
// Priority (high to low): fast[15] .. fast[0], MEI, MSI, MTI.
// mip is a pure wire view of the interrupt inputs, so "is anything pending"
// (the WFI wake condition) is valid even while the core clock is stopped.
// =============================================================================
module gust_counter64 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        inc,
    input  wire        wr_lo,
    input  wire        wr_hi,
    input  wire [31:0] wdata,
    output wire [63:0] q
);
    // Byte-sliced counter.  Slice i only receives a clock enable when every
    // lower slice is 0xFF, so on average ~8 of the 64 flops are clocked per
    // increment instead of all 64.  Functionally identical to q <= q + 1.
    reg  [63:0] q_r;
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : g_slice
            wire wr = (i < 4) ? wr_lo : wr_hi;
            wire en;
            if (i == 0) begin : g_first
                assign en = inc;
            end else begin : g_rest
                assign en = inc & (&q_r[8*i-1:0]);   // all lower slices full
            end
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)  q_r[8*i +: 8] <= 8'd0;
                else if (wr) q_r[8*i +: 8] <= wdata[8*(i%4) +: 8];
                else if (en) q_r[8*i +: 8] <= q_r[8*i +: 8] + 8'd1;
            end
        end
    endgenerate
    assign q = q_r;
endmodule

module gust_csr #(
    parameter        EXT_B  = 1,
    parameter [31:0] MIMPID = 32'h0001_0000      // Gust v1.0
) (
    input  wire        clk,
    input  wire        rst_n,

    // register access (X stage, or the debug module while halted)
    input  wire [11:0] addr,
    input  wire [1:0]  op,          // 01 write, 10 set bits, 11 clear bits
    input  wire        wen,         // commit the write this cycle
    input  wire        wr_intent,   // the access would write (for legality)
    input  wire        dbg_access,  // access comes from debug mode
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        illegal,

    // trap entry / return
    input  wire        trap_en,
    input  wire        trap_is_irq,
    input  wire [4:0]  trap_cause,
    input  wire [31:0] trap_pc,
    input  wire [31:0] trap_tval,
    input  wire        mret_en,
    output wire [31:0] trap_vector,
    output wire [31:0] mepc,

    // debug mode entry
    input  wire        dbg_enter,
    input  wire [2:0]  dbg_cause,
    input  wire [31:0] dbg_pc,
    output wire [31:0] dpc,
    output wire        dcsr_step,
    output wire        dcsr_ebreakm,

    // interrupts
    input  wire        irq_software,
    input  wire        irq_timer,
    input  wire        irq_external,
    input  wire [15:0] irq_fast,
    output wire        irq_any,     // some enabled interrupt is pending (wake)
    output wire        irq_valid,   // ... and mstatus.MIE is set
    output reg  [4:0]  irq_cause,

    // counters
    input  wire        retire
);
    localparam [11:0] A_MSTATUS = 12'h300, A_MISA = 12'h301, A_MIE = 12'h304,
                      A_MTVEC = 12'h305, A_MSTATUSH = 12'h310,
                      A_MCOUNTINH = 12'h320, A_MSCRATCH = 12'h340,
                      A_MEPC = 12'h341, A_MCAUSE = 12'h342, A_MTVAL = 12'h343,
                      A_MIP = 12'h344, A_MCYCLE = 12'hB00, A_MINSTRET = 12'hB02,
                      A_MCYCLEH = 12'hB80, A_MINSTRETH = 12'hB82,
                      A_CYCLE = 12'hC00, A_INSTRET = 12'hC02,
                      A_CYCLEH = 12'hC80, A_INSTRETH = 12'hC82,
                      A_MVENDORID = 12'hF11, A_MARCHID = 12'hF12,
                      A_MIMPID = 12'hF13, A_MHARTID = 12'hF14,
                      A_MCONFIGPTR = 12'hF15, A_TSELECT = 12'h7A0,
                      A_TDATA1 = 12'h7A1, A_TDATA2 = 12'h7A2,
                      A_DCSR = 12'h7B0, A_DPC = 12'h7B1;

    localparam [31:0] MIE_MASK = 32'hFFFF_0888;
    localparam [31:0] MISA_VAL = 32'h4000_0000 | (32'h1 << 8) | (32'h1 << 12) |
                                 (32'h1 << 2) | ((EXT_B != 0) ? 32'h2 : 32'h0);

    // ------------------------------------------------------------ registers
    reg        mstatus_mie, mstatus_mpie;
    reg [31:0] mie_q;
    reg [29:0] mtvec_base;
    reg        mtvec_mode;
    reg [31:0] mscratch_q;
    reg [30:0] mepc_q;
    reg        mcause_irq;
    reg [4:0]  mcause_code;
    reg [31:0] mtval_q;
    reg        inh_cy, inh_ir;
    reg        dcsr_ebreakm_q, dcsr_step_q;
    reg [2:0]  dcsr_cause_q;
    reg [30:0] dpc_q;
    reg        ir_skip_q;

    wire [63:0] mcycle_q, minstret_q;

    wire [31:0] mip = {irq_fast, 4'b0, irq_external, 3'b0, irq_timer, 3'b0,
                       irq_software, 3'b0};
    wire [31:0] pend = mip & mie_q;

    assign irq_any   = |pend;
    assign irq_valid = irq_any & mstatus_mie;

    integer k;
    always @* begin
        irq_cause = 5'd7;
        if (pend[3])  irq_cause = 5'd3;
        if (pend[11]) irq_cause = 5'd11;
        for (k = 16; k < 32; k = k + 1)
            if (pend[k]) irq_cause = k[4:0];
    end

    assign mepc         = {mepc_q, 1'b0};
    assign dpc          = {dpc_q, 1'b0};
    assign dcsr_step    = dcsr_step_q;
    assign dcsr_ebreakm = dcsr_ebreakm_q;

    // vectored mode: interrupts enter at BASE + 4*cause, exceptions at BASE
    wire [29:0] vec_off = (mtvec_mode & trap_is_irq) ? {25'b0, trap_cause} : 30'b0;
    assign trap_vector  = {mtvec_base + vec_off, 2'b00};

    // ----------------------------------------------------------------- read
    wire hpm_range = ((addr[11:5] == 7'b1011_000) |      // mhpmcounter3..31
                      (addr[11:5] == 7'b1011_100) |      // mhpmcounter3h..31h
                      (addr[11:5] == 7'b0011_001)) &     // mhpmevent3..31
                     (addr[4:0] >= 5'd3);
    reg exists;

    always @* begin
        rdata  = 32'b0;
        exists = 1'b1;
        case (addr)
        A_MSTATUS:    rdata = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0,
                               mstatus_mie, 3'b0};
        A_MISA:       rdata = MISA_VAL;
        A_MIE:        rdata = mie_q;
        A_MTVEC:      rdata = {mtvec_base, 1'b0, mtvec_mode};
        A_MSTATUSH:   rdata = 32'b0;
        A_MCOUNTINH:  rdata = {29'b0, inh_ir, 1'b0, inh_cy};
        A_MSCRATCH:   rdata = mscratch_q;
        A_MEPC:       rdata = {mepc_q, 1'b0};
        A_MCAUSE:     rdata = {mcause_irq, 26'b0, mcause_code};
        A_MTVAL:      rdata = mtval_q;
        A_MIP:        rdata = mip;
        A_MCYCLE,
        A_CYCLE:      rdata = mcycle_q[31:0];
        A_MCYCLEH,
        A_CYCLEH:     rdata = mcycle_q[63:32];
        A_MINSTRET,
        A_INSTRET:    rdata = minstret_q[31:0];
        A_MINSTRETH,
        A_INSTRETH:   rdata = minstret_q[63:32];
        A_MVENDORID,
        A_MARCHID,
        A_MHARTID,
        A_MCONFIGPTR: rdata = 32'b0;
        A_MIMPID:     rdata = MIMPID;
        A_TSELECT,
        A_TDATA1,
        A_TDATA2:     rdata = 32'b0;
        A_DCSR: begin
            rdata  = {4'd4, 12'b0, dcsr_ebreakm_q, 6'b0, dcsr_cause_q, 3'b0,
                      dcsr_step_q, 2'b11};
            exists = dbg_access;
        end
        A_DPC: begin
            rdata  = {dpc_q, 1'b0};
            exists = dbg_access;
        end
        default:      exists = hpm_range;
        endcase
    end

    assign illegal = ~exists | (wr_intent & (addr[11:10] == 2'b11));

    // ---------------------------------------------------------------- write
    wire [31:0] wval = (op == 2'b10) ? (rdata | wdata) :
                       (op == 2'b11) ? (rdata & ~wdata) : wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstatus_mie    <= 1'b0;
            mstatus_mpie   <= 1'b0;
            mie_q          <= 32'b0;
            mtvec_base     <= 30'b0;
            mtvec_mode     <= 1'b0;
            mcause_irq     <= 1'b0;
            mcause_code    <= 5'b0;
            inh_cy         <= 1'b0;
            inh_ir         <= 1'b0;
            dcsr_ebreakm_q <= 1'b0;
            dcsr_step_q    <= 1'b0;
            dcsr_cause_q   <= 3'b0;
        end else begin
            if (trap_en) begin
                mstatus_mpie <= mstatus_mie;
                mstatus_mie  <= 1'b0;
                mcause_irq   <= trap_is_irq;
                mcause_code  <= trap_cause;
            end else if (mret_en) begin
                mstatus_mie  <= mstatus_mpie;
                mstatus_mpie <= 1'b1;
            end else if (wen) begin
                case (addr)
                A_MSTATUS: begin
                    mstatus_mie  <= wval[3];
                    mstatus_mpie <= wval[7];
                end
                A_MIE:   mie_q <= wval & MIE_MASK;
                A_MTVEC: begin
                    mtvec_base <= wval[31:2];
                    mtvec_mode <= wval[0];
                end
                A_MCAUSE: begin
                    mcause_irq  <= wval[31];
                    mcause_code <= wval[4:0];
                end
                A_MCOUNTINH: begin
                    inh_cy <= wval[0];
                    inh_ir <= wval[2];
                end
                A_DCSR: begin
                    dcsr_ebreakm_q <= wval[15];
                    dcsr_step_q    <= wval[2];
                end
                default: ;
                endcase
            end
            if (dbg_enter) dcsr_cause_q <= dbg_cause;
        end
    end

    // wide data registers: no reset, written before use
    always @(posedge clk) begin
        if (trap_en) begin
            mepc_q  <= trap_pc[31:1];
            mtval_q <= trap_tval;
        end else if (wen) begin
            case (addr)
            A_MSCRATCH: mscratch_q <= wval;
            A_MEPC:     mepc_q     <= wval[31:1];
            A_MTVAL:    mtval_q    <= wval;
            default: ;
            endcase
        end
        if (dbg_enter)                  dpc_q <= dbg_pc[31:1];
        else if (wen && addr == A_DPC)  dpc_q <= wval[31:1];
    end

    // ------------------------------------------------------------- counters
    wire wr_cnt = wen & ~trap_en & ~mret_en;
    wire wr_ir  = wr_cnt & ((addr == A_MINSTRET) | (addr == A_MINSTRETH));

    // An instruction that writes minstret must not also increment it.  The
    // retire pulse for that instruction arrives one cycle after the write.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      ir_skip_q <= 1'b0;
        else if (wr_ir)  ir_skip_q <= 1'b1;
        else if (retire) ir_skip_q <= 1'b0;
    end

    gust_counter64 u_mcycle (
        .clk(clk), .rst_n(rst_n), .inc(~inh_cy),
        .wr_lo(wr_cnt & (addr == A_MCYCLE)), .wr_hi(wr_cnt & (addr == A_MCYCLEH)),
        .wdata(wval), .q(mcycle_q)
    );
    gust_counter64 u_minstret (
        .clk(clk), .rst_n(rst_n), .inc(retire & ~inh_ir & ~ir_skip_q),
        .wr_lo(wr_cnt & (addr == A_MINSTRET)), .wr_hi(wr_cnt & (addr == A_MINSTRETH)),
        .wdata(wval), .q(minstret_q)
    );
endmodule
