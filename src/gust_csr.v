// =============================================================================
// Gust N1 - control and status registers: machine + supervisor + user modes
//
// Extends the Gust v1.0 CSR block (machine mode only) with:
//   * a privilege register (M = 3, S = 1, U = 0) and MPP / SPP stacking
//   * mstatus: SIE SPIE SPP MPRV SUM MXR TVM TW TSR;  sstatus view
//   * medeleg / mideleg trap delegation, stvec sscratch sepc scause stval
//   * sie / sip views, software-writable SSIP STIP SEIP
//   * satp (Sv32, no ASIDs), mcounteren / scounteren
//   * privilege checks on every CSR access
// Unchanged from Gust: counters, debug CSRs, interrupt inputs.
//
// Interrupt priority (high to low): fast[15..0], MEI, MSI, MTI, SEI, SSI, STI.
// Machine-level interrupts (not delegated) win over supervisor-level ones.
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
    // Byte-sliced: slice i is only enabled when all lower slices are 0xFF.
    reg  [63:0] q_r;
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : g_slice
            wire wr = (i < 4) ? wr_lo : wr_hi;
            wire en;
            if (i == 0) begin : g_first
                assign en = inc;
            end else begin : g_rest
                assign en = inc & (&q_r[8*i-1:0]);
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
    parameter [31:0] MIMPID = 32'h4E31_0001      // "N1", stage 1
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [11:0] addr,
    input  wire [1:0]  op,          // 01 write, 10 set bits, 11 clear bits
    input  wire        wen,
    input  wire        wr_intent,
    input  wire        dbg_access,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        illegal,

    input  wire        trap_en,
    input  wire        trap_is_irq,
    input  wire [4:0]  trap_cause,
    input  wire [31:0] trap_pc,
    input  wire [31:0] trap_tval,
    input  wire        mret_en,
    input  wire        sret_en,
    output wire [31:0] trap_vector,
    output wire [31:0] mepc,
    output wire [31:0] sepc,

    input  wire        dbg_enter,
    input  wire [2:0]  dbg_cause,
    input  wire [31:0] dbg_pc,
    input  wire        dbg_resume,
    output wire [31:0] dpc,
    output wire        dcsr_step,
    output wire        dcsr_ebreak,   // ebreak enters debug at the current privilege

    input  wire        irq_software,
    input  wire        irq_timer,
    input  wire        irq_external,
    input  wire [15:0] irq_fast,
    output wire        irq_any,
    output wire        irq_valid,
    output reg  [4:0]  irq_cause,

    input  wire        retire,

    output wire [1:0]  priv,
    output wire        mstatus_mprv,
    output wire [1:0]  mstatus_mpp,
    output wire        mstatus_sum,
    output wire        mstatus_mxr,
    output wire        mstatus_tvm,
    output wire        mstatus_tw,
    output wire        mstatus_tsr,
    output wire        satp_mode,
    output wire [19:0] satp_ppn
);
    localparam [1:0] PRV_U = 2'b00, PRV_S = 2'b01, PRV_M = 2'b11;

    localparam [11:0] A_SSTATUS = 12'h100, A_SIE = 12'h104, A_STVEC = 12'h105,
                      A_SCOUNTEREN = 12'h106, A_SSCRATCH = 12'h140,
                      A_SEPC = 12'h141, A_SCAUSE = 12'h142, A_STVAL = 12'h143,
                      A_SIP = 12'h144, A_SATP = 12'h180,
                      A_MSTATUS = 12'h300, A_MISA = 12'h301, A_MEDELEG = 12'h302,
                      A_MIDELEG = 12'h303, A_MIE = 12'h304, A_MTVEC = 12'h305,
                      A_MCOUNTEREN = 12'h306, A_MSTATUSH = 12'h310,
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

    localparam [31:0] MIE_MASK     = 32'hFFFF_0AAA;   // fast, MEI SEI MTI STI MSI SSI
    localparam [31:0] MIDELEG_MASK = 32'hFFFF_0222;   // fast and supervisor interrupts
    localparam [15:0] MEDELEG_MASK = 16'hB3FF;        // all but ecall-from-M and reserved
    localparam [31:0] MISA_VAL = 32'h4014_1104 | ((EXT_B != 0) ? 32'h2 : 32'h0); // RV32 IMC S U

    // ------------------------------------------------------------ registers
    reg [1:0]  priv_q, mpp_q, dprv_q;
    reg        mie_b, mpie_b, sie_b, spie_b, spp_b;
    reg        mprv_b, sum_b, mxr_b, tvm_b, tw_b, tsr_b;
    reg [31:0] mie_q, mideleg_q;
    reg [15:0] medeleg_q;
    reg        ssip_b, stip_b, seip_b;
    reg [29:0] mtvec_base, stvec_base;
    reg        mtvec_mode, stvec_mode;
    reg [31:0] mscratch_q, sscratch_q, mtval_q, stval_q;
    reg [30:0] mepc_q, sepc_q, dpc_q;
    reg        mcause_irq, scause_irq;
    reg [4:0]  mcause_code, scause_code;
    reg        satp_mode_q;
    reg [21:0] satp_ppn_q;
    reg [2:0]  mcounteren_q, scounteren_q;
    reg        inh_cy, inh_ir;
    reg        dcsr_ebreakm_q, dcsr_ebreaks_q, dcsr_ebreaku_q, dcsr_step_q;
    reg [2:0]  dcsr_cause_q;
    reg        ir_skip_q;
    wire [63:0] mcycle_q, minstret_q;

    assign priv         = priv_q;
    assign mstatus_mprv = mprv_b;
    assign mstatus_mpp  = mpp_q;
    assign mstatus_sum  = sum_b;
    assign mstatus_mxr  = mxr_b;
    assign mstatus_tvm  = tvm_b;
    assign mstatus_tw   = tw_b;
    assign mstatus_tsr  = tsr_b;
    assign satp_mode    = satp_mode_q;
    assign satp_ppn     = satp_ppn_q[19:0];
    assign mepc         = {mepc_q, 1'b0};
    assign sepc         = {sepc_q, 1'b0};
    assign dpc          = {dpc_q, 1'b0};
    assign dcsr_step    = dcsr_step_q;
    assign dcsr_ebreak  = (priv_q == PRV_M) ? dcsr_ebreakm_q :
                          (priv_q == PRV_S) ? dcsr_ebreaks_q : dcsr_ebreaku_q;

    // ----------------------------------------------------------- interrupts
    wire [31:0] mip = {irq_fast, 4'b0, irq_external, 1'b0, seip_b, 1'b0,
                       irq_timer, 1'b0, stip_b, 1'b0, irq_software, 1'b0,
                       ssip_b, 1'b0};
    wire [31:0] pend   = mip & mie_q;
    wire [31:0] m_pend = pend & ~mideleg_q;
    wire [31:0] s_pend = pend &  mideleg_q;
    wire m_take = (|m_pend) & ((priv_q != PRV_M) | mie_b);
    wire s_take = (|s_pend) & ((priv_q == PRV_U) | ((priv_q == PRV_S) & sie_b));
    wire [31:0] sel = m_take ? m_pend : s_pend;

    assign irq_any   = |pend;
    assign irq_valid = m_take | s_take;

    integer k;
    always @* begin
        irq_cause = 5'd5;
        if (sel[1])  irq_cause = 5'd1;
        if (sel[9])  irq_cause = 5'd9;
        if (sel[7])  irq_cause = 5'd7;
        if (sel[3])  irq_cause = 5'd3;
        if (sel[11]) irq_cause = 5'd11;
        for (k = 16; k < 32; k = k + 1)
            if (sel[k]) irq_cause = k[4:0];
    end

    // -------------------------------------------------------- trap routing
    // An interrupt goes where it was selected; an exception is delegated to
    // S-mode if medeleg says so and we are not in M-mode.
    wire exc_deleg = (trap_cause < 5'd16) & medeleg_q[trap_cause[3:0]] & (priv_q != PRV_M);
    wire trap_to_s = trap_is_irq ? ~m_take : exc_deleg;

    wire [29:0] tv_base = trap_to_s ? stvec_base : mtvec_base;
    wire        tv_mode = trap_to_s ? stvec_mode : mtvec_mode;
    wire [29:0] vec_off = (tv_mode & trap_is_irq) ? {25'b0, trap_cause} : 30'b0;
    assign trap_vector  = {tv_base + vec_off, 2'b00};

    // ----------------------------------------------------------------- read
    wire [1:0] eff_priv = dbg_access ? PRV_M : priv_q;
    wire hpm_range = ((addr[11:5] == 7'b1011_000) | (addr[11:5] == 7'b1011_100) |
                      (addr[11:5] == 7'b0011_001)) & (addr[4:0] >= 5'd3);
    wire [31:0] mstatus_rd = {9'b0, tsr_b, tw_b, tvm_b, mxr_b, sum_b, mprv_b,
                              4'b0, mpp_q, 2'b0, spp_b, mpie_b, 1'b0, spie_b,
                              1'b0, mie_b, 1'b0, sie_b, 1'b0};
    localparam [31:0] SSTATUS_MASK = 32'h000C_0122;   // MXR SUM SPP SPIE SIE
    reg exists;

    always @* begin
        rdata  = 32'b0;
        exists = 1'b1;
        case (addr)
        A_SSTATUS:    rdata = mstatus_rd & SSTATUS_MASK;
        A_SIE:        rdata = mie_q & mideleg_q;
        A_STVEC:      rdata = {stvec_base, 1'b0, stvec_mode};
        A_SCOUNTEREN: rdata = {29'b0, scounteren_q};
        A_SSCRATCH:   rdata = sscratch_q;
        A_SEPC:       rdata = {sepc_q, 1'b0};
        A_SCAUSE:     rdata = {scause_irq, 26'b0, scause_code};
        A_STVAL:      rdata = stval_q;
        A_SIP:        rdata = mip & mideleg_q;
        A_SATP:       rdata = {satp_mode_q, 9'b0, satp_ppn_q};
        A_MSTATUS:    rdata = mstatus_rd;
        A_MISA:       rdata = MISA_VAL;
        A_MEDELEG:    rdata = {16'b0, medeleg_q};
        A_MIDELEG:    rdata = mideleg_q;
        A_MIE:        rdata = mie_q;
        A_MTVEC:      rdata = {mtvec_base, 1'b0, mtvec_mode};
        A_MCOUNTEREN: rdata = {29'b0, mcounteren_q};
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
            rdata  = {4'd4, 12'b0, dcsr_ebreakm_q, 1'b0, dcsr_ebreaks_q,
                      dcsr_ebreaku_q, 3'b0, dcsr_cause_q, 3'b0, dcsr_step_q, dprv_q};
            exists = dbg_access;
        end
        A_DPC: begin
            rdata  = {dpc_q, 1'b0};
            exists = dbg_access;
        end
        default:      exists = hpm_range;
        endcase
    end

    // privilege checks
    wire priv_ok   = (eff_priv >= addr[9:8]);
    wire tvm_block = (addr == A_SATP) & (eff_priv == PRV_S) & tvm_b;
    wire is_ucnt   = (addr[11:8] == 4'hC) & (addr[6:2] == 5'd0) & ~addr[0];
    wire [2:0] cnt_bit = 3'b001 << addr[1:0];
    wire cnt_ok    = (eff_priv == PRV_M) |
                     ((|(mcounteren_q & cnt_bit)) &
                      ((eff_priv == PRV_S) | (|(scounteren_q & cnt_bit))));
    assign illegal = ~exists | ~priv_ok | tvm_block | (is_ucnt & ~cnt_ok) |
                     (wr_intent & (addr[11:10] == 2'b11));

    // ---------------------------------------------------------------- write
    wire [31:0] wval = (op == 2'b10) ? (rdata | wdata) :
                       (op == 2'b11) ? (rdata & ~wdata) : wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            priv_q <= PRV_M; mpp_q <= PRV_U; dprv_q <= PRV_M;
            mie_b <= 1'b0; mpie_b <= 1'b0; sie_b <= 1'b0; spie_b <= 1'b0; spp_b <= 1'b0;
            mprv_b <= 1'b0; sum_b <= 1'b0; mxr_b <= 1'b0; tvm_b <= 1'b0; tw_b <= 1'b0; tsr_b <= 1'b0;
            mie_q <= 32'b0; mideleg_q <= 32'b0; medeleg_q <= 16'b0;
            ssip_b <= 1'b0; stip_b <= 1'b0; seip_b <= 1'b0;
            mtvec_base <= 30'b0; mtvec_mode <= 1'b0; stvec_base <= 30'b0; stvec_mode <= 1'b0;
            mcause_irq <= 1'b0; mcause_code <= 5'b0; scause_irq <= 1'b0; scause_code <= 5'b0;
            satp_mode_q <= 1'b0; satp_ppn_q <= 22'b0;
            mcounteren_q <= 3'b0; scounteren_q <= 3'b0;
            inh_cy <= 1'b0; inh_ir <= 1'b0;
            dcsr_ebreakm_q <= 1'b0; dcsr_ebreaks_q <= 1'b0; dcsr_ebreaku_q <= 1'b0;
            dcsr_step_q <= 1'b0; dcsr_cause_q <= 3'b0;
        end else begin
            if (trap_en) begin
                if (trap_to_s) begin
                    spie_b      <= sie_b;
                    sie_b       <= 1'b0;
                    spp_b       <= priv_q[0];
                    priv_q      <= PRV_S;
                    scause_irq  <= trap_is_irq;
                    scause_code <= trap_cause;
                end else begin
                    mpie_b      <= mie_b;
                    mie_b       <= 1'b0;
                    mpp_q       <= priv_q;
                    priv_q      <= PRV_M;
                    mcause_irq  <= trap_is_irq;
                    mcause_code <= trap_cause;
                end
            end else if (mret_en) begin
                mie_b  <= mpie_b;
                mpie_b <= 1'b1;
                priv_q <= mpp_q;
                mpp_q  <= PRV_U;
                if (mpp_q != PRV_M) mprv_b <= 1'b0;
            end else if (sret_en) begin
                sie_b  <= spie_b;
                spie_b <= 1'b1;
                priv_q <= {1'b0, spp_b};
                spp_b  <= 1'b0;
                mprv_b <= 1'b0;
            end else if (wen) begin
                case (addr)
                A_MSTATUS: begin
                    sie_b <= wval[1]; mie_b <= wval[3]; spie_b <= wval[5];
                    mpie_b <= wval[7]; spp_b <= wval[8];
                    if (wval[12:11] != 2'b10) mpp_q <= wval[12:11];
                    mprv_b <= wval[17]; sum_b <= wval[18]; mxr_b <= wval[19];
                    tvm_b <= wval[20]; tw_b <= wval[21]; tsr_b <= wval[22];
                end
                A_SSTATUS: begin
                    sie_b <= wval[1]; spie_b <= wval[5]; spp_b <= wval[8];
                    sum_b <= wval[18]; mxr_b <= wval[19];
                end
                A_MIE:     mie_q <= wval & MIE_MASK;
                A_SIE:     mie_q <= (mie_q & ~mideleg_q) | (wval & mideleg_q & MIE_MASK);
                A_MIP: begin
                    ssip_b <= wval[1]; stip_b <= wval[5]; seip_b <= wval[9];
                end
                A_SIP:     if (mideleg_q[1]) ssip_b <= wval[1];
                A_MEDELEG: medeleg_q <= wval[15:0] & MEDELEG_MASK;
                A_MIDELEG: mideleg_q <= wval & MIDELEG_MASK;
                A_MTVEC: begin
                    mtvec_base <= wval[31:2]; mtvec_mode <= wval[0];
                end
                A_STVEC: begin
                    stvec_base <= wval[31:2]; stvec_mode <= wval[0];
                end
                A_MCAUSE: begin
                    mcause_irq <= wval[31]; mcause_code <= wval[4:0];
                end
                A_SCAUSE: begin
                    scause_irq <= wval[31]; scause_code <= wval[4:0];
                end
                A_SATP: begin
                    satp_mode_q <= wval[31]; satp_ppn_q <= wval[21:0];
                end
                A_MCOUNTEREN: mcounteren_q <= wval[2:0] & 3'b101;
                A_SCOUNTEREN: scounteren_q <= wval[2:0] & 3'b101;
                A_MCOUNTINH: begin
                    inh_cy <= wval[0]; inh_ir <= wval[2];
                end
                A_DCSR: begin
                    dcsr_ebreakm_q <= wval[15]; dcsr_ebreaks_q <= wval[13];
                    dcsr_ebreaku_q <= wval[12]; dcsr_step_q <= wval[2];
                    if (wval[1:0] != 2'b10) dprv_q <= wval[1:0];
                end
                default: ;
                endcase
            end
            // debug mode runs with machine privilege; the interrupted
            // privilege is kept in dcsr.prv and restored on resume
            if (dbg_enter) begin
                dcsr_cause_q <= dbg_cause;
                dprv_q       <= priv_q;
                priv_q       <= PRV_M;
            end else if (dbg_resume) begin
                priv_q       <= dprv_q;
            end
        end
    end

    // wide data registers: no reset, written before use
    always @(posedge clk) begin
        if (trap_en) begin
            if (trap_to_s) begin
                sepc_q  <= trap_pc[31:1];
                stval_q <= trap_tval;
            end else begin
                mepc_q  <= trap_pc[31:1];
                mtval_q <= trap_tval;
            end
        end else if (wen) begin
            case (addr)
            A_MSCRATCH: mscratch_q <= wval;
            A_SSCRATCH: sscratch_q <= wval;
            A_MEPC:     mepc_q     <= wval[31:1];
            A_SEPC:     sepc_q     <= wval[31:1];
            A_MTVAL:    mtval_q    <= wval;
            A_STVAL:    stval_q    <= wval;
            default: ;
            endcase
        end
        if (dbg_enter)                  dpc_q <= dbg_pc[31:1];
        else if (wen && addr == A_DPC)  dpc_q <= wval[31:1];
    end

    // ------------------------------------------------------------- counters
    wire wr_cnt = wen & ~trap_en & ~mret_en & ~sret_en;
    wire wr_ir  = wr_cnt & ((addr == A_MINSTRET) | (addr == A_MINSTRETH));
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
