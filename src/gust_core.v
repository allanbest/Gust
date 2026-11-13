// =============================================================================
// Gust v1.0 - CPU core
//
//   RV32IMC + Zicsr + Zifencei, optional Zba/Zbb/Zbs (EXT_B) and Zcb.
//   Machine mode + debug mode.  Three stages:
//
//     F  fetch     gust_fetch: bus request, 5-parcel aligner, RVC expansion,
//                  static branch prediction (zero-bubble loops and jumps)
//     X  execute   decode, register read, forwarding, ALU / branch / CSR,
//                  load-store address phase on the data bus, trap decisions
//     M  memory    data-bus response, load alignment, register write-back
//
//   Every architectural side effect happens in X (CSR writes, redirects, bus
//   requests) or at the end of M (register write).  X only "fires" in a cycle
//   in which M is completing, so a bus error in M is always precise.
//
//   Timing summary (single-cycle memory):
//     ALU / load / store / CSR ......... 1 cycle
//     load followed by a use ........... +1
//     JAL, predicted branch ............ 1 cycle (0 bubbles)
//     mispredicted branch, JALR, MRET .. +1
//     MUL (MUL_FAST=1) ................. 1 cycle
//     MULH*/MUL(slow) .................. 34 cycles,  DIV/REM 36 cycles
//     interrupt entry .................. 2 cycles from pending to handler in X
//
// Bus protocol (ibus and dbus): req/gnt address phase; one rvalid pulse per
// granted request at least one cycle later; err qualifies rvalid.  req, once
// raised, is held until gnt.
// =============================================================================
`include "gust_defs.vh"

module gust_core #(
    parameter [31:0] RESET_VECTOR = 32'h0000_0000,
    parameter        EXT_B        = 1,   // Zba + Zbb + Zbs
    parameter        EXT_ZCB      = 1,   // Zcb code-size extension
    parameter        MUL_FAST     = 1,   // single-cycle MUL array
    parameter        BRANCH_PRED  = 1,   // static predictor in fetch
    parameter        OPERAND_ISO  = 1,   // isolate idle datapath blocks
    parameter        CLOCK_GATE   = 0    // root + register-row clock gates (ASIC)
) (
    input  wire        clk,
    input  wire        rst_n,

    // instruction bus
    output wire        ibus_req,
    output wire [31:0] ibus_addr,
    input  wire        ibus_gnt,
    input  wire        ibus_rvalid,
    input  wire [31:0] ibus_rdata,
    input  wire        ibus_err,

    // data bus
    output wire        dbus_req,
    output wire        dbus_we,
    output wire [3:0]  dbus_be,
    output wire [31:0] dbus_addr,
    output wire [31:0] dbus_wdata,
    input  wire        dbus_gnt,
    input  wire        dbus_rvalid,
    input  wire [31:0] dbus_rdata,
    input  wire        dbus_err,

    // interrupts (level sensitive, synchronous to clk)
    input  wire        irq_software,
    input  wire        irq_timer,
    input  wire        irq_external,
    input  wire [15:0] irq_fast,

    // debug module interface
    input  wire        dbg_halt_req,     // level
    input  wire        dbg_resume_req,   // pulse while halted
    output wire        dbg_halted,
    input  wire        dbg_reg_req,      // single-cycle register access
    input  wire        dbg_reg_we,
    input  wire [15:0] dbg_reg_no,       // 0x0000-0x0fff CSR, 0x1000-0x101f GPR
    input  wire [31:0] dbg_reg_wdata,
    output wire [31:0] dbg_reg_rdata,
    output wire        dbg_reg_err,

    output wire        sleeping          // WFI sleep, core clock may be off
);
    // ------------------------------------------------------------------
    // Clocking.  `clk` is free running; `gclk` stops while the core sleeps.
    // ------------------------------------------------------------------
    wire clk_en;
    wire gclk;
    generate
        if (CLOCK_GATE != 0) begin : g_icg
            gust_icg u_icg (.clk(clk), .en(clk_en), .gclk(gclk));
        end else begin : g_no_icg
            assign gclk = clk;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Pipeline registers
    // ------------------------------------------------------------------
    // F -> X
    reg         x_valid;
    reg  [31:0] x_instr;
    reg  [30:0] x_pc_r;
    reg         x_compressed;
    reg         x_illegal_c;
    reg         x_fetch_err;
    reg         x_pred_taken;
    // X -> M
    reg         m_valid;
    reg  [4:0]  m_rd;
    reg         m_wen;
    reg         m_is_load;
    reg         m_is_store;
    reg  [2:0]  m_funct3;
    reg  [31:0] m_result;       // ALU result, or the address of a load/store
    reg  [30:0] m_pc_r;         // only kept for loads/stores (precise faults)
    // control state
    reg         sleep_q;        // on the free-running clock
    reg         debug_q;
    reg         step_armed_q;
    reg         step_pend_q;
    reg         req_hold_q;

    wire [31:0] x_pc = {x_pc_r, 1'b0};
    wire [31:0] m_pc = {m_pc_r, 1'b0};

    // ------------------------------------------------------------------
    // Fetch
    // ------------------------------------------------------------------
    wire        f_valid, f_ready, f_compressed, f_illegal, f_fetch_err;
    wire        f_pred_taken, f_busy;
    wire [31:0] f_instr, f_pc;
    wire        flush;
    wire [31:0] flush_pc;
    wire        fetch_en = ~sleep_q & ~debug_q;

    gust_fetch #(
        .RESET_VECTOR(RESET_VECTOR), .BRANCH_PRED(BRANCH_PRED),
        .EXT_B(EXT_B), .EXT_ZCB(EXT_ZCB)
    ) u_fetch (
        .clk(gclk), .rst_n(rst_n),
        .fetch_en(fetch_en), .flush(flush), .flush_pc(flush_pc),
        .ibus_req(ibus_req), .ibus_addr(ibus_addr), .ibus_gnt(ibus_gnt),
        .ibus_rvalid(ibus_rvalid), .ibus_rdata(ibus_rdata), .ibus_err(ibus_err),
        .f_valid(f_valid), .f_ready(f_ready), .f_instr(f_instr), .f_pc(f_pc),
        .f_compressed(f_compressed), .f_illegal(f_illegal),
        .f_fetch_err(f_fetch_err), .f_pred_taken(f_pred_taken),
        .busy(f_busy)
    );

    // ------------------------------------------------------------------
    // Decode
    // ------------------------------------------------------------------
    wire        d_illegal, d_rs1_used, d_rs2_used, d_rd_wen, d_op_b_imm;
    wire        d_is_branch, d_is_jal, d_is_jalr, d_is_load, d_is_store;
    wire        d_is_csr, d_is_mul, d_is_md, d_is_ecall, d_is_ebreak;
    wire        d_is_mret, d_is_wfi, d_is_fencei;
    wire [31:0] d_imm, d_imm_bj;
    wire [5:0]  d_alu_op;
    wire [1:0]  d_op_a_sel;
    wire [2:0]  d_res_sel;

    gust_decode #(.EXT_B(EXT_B), .MUL_FAST(MUL_FAST)) u_decode (
        .instr(x_instr), .illegal(d_illegal),
        .rs1_used(d_rs1_used), .rs2_used(d_rs2_used), .rd_wen(d_rd_wen),
        .imm(d_imm), .imm_bj(d_imm_bj), .alu_op(d_alu_op),
        .op_a_sel(d_op_a_sel), .op_b_imm(d_op_b_imm), .res_sel(d_res_sel),
        .is_branch(d_is_branch), .is_jal(d_is_jal), .is_jalr(d_is_jalr),
        .is_load(d_is_load), .is_store(d_is_store), .is_csr(d_is_csr),
        .is_mul(d_is_mul), .is_md(d_is_md), .is_ecall(d_is_ecall),
        .is_ebreak(d_is_ebreak), .is_mret(d_is_mret), .is_wfi(d_is_wfi),
        .is_fencei(d_is_fencei)
    );

    wire [4:0] x_rs1 = x_instr[19:15];
    wire [4:0] x_rs2 = x_instr[24:20];
    wire [4:0] x_rd  = x_instr[11:7];
    wire [2:0] x_f3  = x_instr[14:12];
    wire       d_is_mem = d_is_load | d_is_store;

    // ------------------------------------------------------------------
    // M stage: data-bus response and load alignment
    // ------------------------------------------------------------------
    wire m_mem  = m_valid & (m_is_load | m_is_store);
    wire m_done = ~m_mem | dbus_rvalid;
    wire m_trap = m_mem & dbus_rvalid & dbus_err;

    wire [31:0] ld_sh = dbus_rdata >> {m_result[1:0], 3'b000};
    reg  [31:0] load_data;
    always @* begin
        case (m_funct3)
        3'b000:  load_data = {{24{ld_sh[7]}},  ld_sh[7:0]};
        3'b001:  load_data = {{16{ld_sh[15]}}, ld_sh[15:0]};
        3'b100:  load_data = {24'b0, ld_sh[7:0]};
        3'b101:  load_data = {16'b0, ld_sh[15:0]};
        default: load_data = dbus_rdata;
        endcase
    end

    // ------------------------------------------------------------------
    // Register file (shared with the debug module while halted)
    // ------------------------------------------------------------------
    wire        dbg_acc    = debug_q & dbg_reg_req;
    wire        dbg_is_gpr = (dbg_reg_no[15:5] == 11'b0001_0000_000);
    wire        dbg_is_csr = (dbg_reg_no[15:12] == 4'h0);
    wire        dbg_gpr_wr = dbg_acc & dbg_reg_we & dbg_is_gpr;

    wire [31:0] rf_rdata1, rf_rdata2;
    wire        m_wr       = m_valid & m_wen & m_done & ~m_trap;
    wire        rf_we      = debug_q ? dbg_gpr_wr : m_wr;
    wire [4:0]  rf_waddr   = debug_q ? dbg_reg_no[4:0] : m_rd;
    wire [31:0] rf_wdata   = debug_q ? dbg_reg_wdata :
                             (m_is_load ? load_data : m_result);
    wire [4:0]  rf_raddr1  = debug_q ? dbg_reg_no[4:0] : x_rs1;

    gust_regfile #(.CLOCK_GATE(CLOCK_GATE)) u_regfile (
        .clk(gclk), .we(rf_we), .waddr(rf_waddr), .wdata(rf_wdata),
        .raddr1(rf_raddr1), .raddr2(x_rs2),
        .rdata1(rf_rdata1), .rdata2(rf_rdata2)
    );

    // ------------------------------------------------------------------
    // Forwarding (single source: the M stage) and load-use detection
    // ------------------------------------------------------------------
    wire fwd1 = m_valid & m_wen & (m_rd == x_rs1);
    wire fwd2 = m_valid & m_wen & (m_rd == x_rs2);
    wire load_use = m_is_load & ((d_rs1_used & fwd1) | (d_rs2_used & fwd2));

    wire [31:0] rs1_val = fwd1 ? m_result : rf_rdata1;
    wire [31:0] rs2_val = fwd2 ? m_result : rf_rdata2;

    // ------------------------------------------------------------------
    // Execute datapath
    // ------------------------------------------------------------------
    wire [31:0] op_a = (d_op_a_sel == `GUST_OPA_PC)   ? x_pc :
                       (d_op_a_sel == `GUST_OPA_ZERO) ? 32'b0 : rs1_val;
    wire [31:0] op_b = d_op_b_imm ? d_imm : rs2_val;

    wire [31:0] alu_result, alu_add;
    wire        alu_eq, alu_lt, alu_ltu;
    gust_alu #(.EXT_B(EXT_B), .OPERAND_ISO(OPERAND_ISO)) u_alu (
        .op(d_alu_op), .a(op_a), .b(op_b),
        .result(alu_result), .add(alu_add),
        .eq(alu_eq), .lt(alu_lt), .ltu(alu_ltu)
    );

    // single-cycle multiplier with operand isolation
    wire [31:0] mul_res;
    generate
        if (MUL_FAST != 0) begin : g_mul
            wire        mul_act = (OPERAND_ISO == 0) | (x_valid & d_is_mul);
            wire [31:0] mul_a   = mul_act ? rs1_val : 32'b0;
            wire [31:0] mul_b   = mul_act ? rs2_val : 32'b0;
            gust_mul u_mul (.a(mul_a), .b(mul_b), .p(mul_res));
        end else begin : g_no_mul
            assign mul_res = 32'b0;
        end
    endgenerate

    // sequential multiply / divide
    wire        md_start, md_abort, md_ack, md_busy, md_done;
    wire [31:0] md_result;
    gust_muldiv u_muldiv (
        .clk(gclk), .rst_n(rst_n),
        .start(md_start), .abort(md_abort), .ack(md_ack),
        .op(x_f3), .a(rs1_val), .b(rs2_val),
        .busy(md_busy), .done(md_done), .result(md_result)
    );

    // branch / jump targets
    wire [31:0] x_pc_next = x_pc + (x_compressed ? 32'd2 : 32'd4);
    wire [31:0] btarget   = x_pc + d_imm_bj;
    reg         br_taken;
    always @* begin
        case (x_f3)
        3'b000:  br_taken = alu_eq;
        3'b001:  br_taken = ~alu_eq;
        3'b100:  br_taken = alu_lt;
        3'b101:  br_taken = ~alu_lt;
        3'b110:  br_taken = alu_ltu;
        default: br_taken = ~alu_ltu;
        endcase
    end

    // load / store address phase
    wire [31:0] mem_addr = alu_add;
    wire        mem_misaligned = d_is_mem &
                    (((x_f3[1:0] == 2'b10) & (|mem_addr[1:0])) |
                     ((x_f3[1:0] == 2'b01) & mem_addr[0]));
    reg  [3:0]  mem_be;
    reg  [31:0] mem_wdata;
    always @* begin
        case (x_f3[1:0])
        2'b00: begin
            mem_be    = 4'b0001 << mem_addr[1:0];
            mem_wdata = {4{rs2_val[7:0]}};
        end
        2'b01: begin
            mem_be    = mem_addr[1] ? 4'b1100 : 4'b0011;
            mem_wdata = {2{rs2_val[15:0]}};
        end
        default: begin
            mem_be    = 4'b1111;
            mem_wdata = rs2_val;
        end
        endcase
    end

    // ------------------------------------------------------------------
    // CSRs
    // ------------------------------------------------------------------
    wire        x_csr_wr_intent = (x_f3[1:0] == 2'b01) | (|x_rs1);
    wire [31:0] x_csr_wdata     = x_f3[2] ? {27'b0, x_rs1} : rs1_val;

    // minstret counts at the end of M, but CSRs are read and written in X.
    // An access to (m)instret[h] therefore waits one cycle for the M stage to
    // drain, so it sees exactly the instructions that precede it.  (Found by
    // rv32mi-p-instret_overflow once memory became truly single-cycle.)
    wire        x_instret_csr = d_is_csr & (x_instr[26:20] == 7'h02) &
                                ((x_instr[31:28] == 4'hB) | (x_instr[31:28] == 4'hC));
    wire        x_csr_wait    = x_instret_csr & m_valid;

    wire [31:0] csr_rdata, csr_trap_vector, csr_mepc, csr_dpc;
    wire        csr_illegal, csr_dcsr_step, csr_dcsr_ebreakm;
    wire        irq_any, irq_valid;
    wire [4:0]  irq_cause;

    wire        x_fire;
    wire        trap_en, trap_is_irq, dbg_enter;
    wire [4:0]  trap_cause;
    wire [31:0] trap_pc, trap_tval;
    wire [2:0]  dbg_cause;
    wire        retire = m_valid & m_done & ~m_trap;

    wire [11:0] csr_addr   = debug_q ? dbg_reg_no[11:0] : x_instr[31:20];
    wire [1:0]  csr_op     = debug_q ? 2'b01 : x_f3[1:0];
    wire        csr_intent = debug_q ? dbg_reg_we : x_csr_wr_intent;
    wire [31:0] csr_wdata  = debug_q ? dbg_reg_wdata : x_csr_wdata;
    wire        csr_wen    = debug_q
                           ? (dbg_acc & dbg_reg_we & dbg_is_csr & ~csr_illegal)
                           : (x_fire & d_is_csr & x_csr_wr_intent);

    gust_csr #(.EXT_B(EXT_B)) u_csr (
        .clk(gclk), .rst_n(rst_n),
        .addr(csr_addr), .op(csr_op), .wen(csr_wen), .wr_intent(csr_intent),
        .dbg_access(debug_q), .wdata(csr_wdata), .rdata(csr_rdata),
        .illegal(csr_illegal),
        .trap_en(trap_en), .trap_is_irq(trap_is_irq), .trap_cause(trap_cause),
        .trap_pc(trap_pc), .trap_tval(trap_tval),
        .mret_en(x_fire & d_is_mret),
        .trap_vector(csr_trap_vector), .mepc(csr_mepc),
        .dbg_enter(dbg_enter), .dbg_cause(dbg_cause), .dbg_pc(x_pc),
        .dpc(csr_dpc), .dcsr_step(csr_dcsr_step), .dcsr_ebreakm(csr_dcsr_ebreakm),
        .irq_software(irq_software), .irq_timer(irq_timer),
        .irq_external(irq_external), .irq_fast(irq_fast),
        .irq_any(irq_any), .irq_valid(irq_valid), .irq_cause(irq_cause),
        .retire(retire)
    );

    assign dbg_reg_rdata = dbg_is_gpr ? rf_rdata1 : csr_rdata;
    assign dbg_reg_err   = ~(dbg_is_gpr | (dbg_is_csr & ~csr_illegal));
    assign dbg_halted    = debug_q;

    // ------------------------------------------------------------------
    // X-stage exceptions
    // ------------------------------------------------------------------
    wire x_illegal   = d_illegal | x_illegal_c | (d_is_csr & csr_illegal);
    wire x_exc_fetch = x_fetch_err;
    wire x_exc_ill   = ~x_fetch_err & x_illegal;
    wire x_dec_ok    = ~x_fetch_err & ~x_illegal;
    wire x_exc_ecall = x_dec_ok & d_is_ecall;
    wire x_exc_ebrk  = x_dec_ok & d_is_ebreak;
    wire x_exc_mal   = x_dec_ok & mem_misaligned;
    wire x_exc       = x_exc_fetch | x_exc_ill | x_exc_ecall | x_exc_ebrk |
                       x_exc_mal;

    reg  [4:0]  exc_cause;
    reg  [31:0] exc_tval;
    always @* begin
        exc_tval = 32'b0;
        if (x_exc_fetch) begin
            exc_cause = `GUST_EXC_IFAULT;
            exc_tval  = x_pc;
        end else if (x_exc_ill) begin
            exc_cause = `GUST_EXC_ILLEGAL;
        end else if (x_exc_ebrk) begin
            exc_cause = `GUST_EXC_BREAKPOINT;
        end else if (x_exc_ecall) begin
            exc_cause = `GUST_EXC_ECALL_M;
        end else begin
            exc_cause = d_is_store ? `GUST_EXC_SMISALIGN : `GUST_EXC_LMISALIGN;
            exc_tval  = mem_addr;
        end
    end

    // ------------------------------------------------------------------
    // Events taken at the X-stage instruction boundary
    //   priority: debug halt > interrupt > exception / ebreak-to-debug
    // Nothing is taken while a data-bus request is waiting for its grant.
    // ------------------------------------------------------------------
    wire halt_now  = (dbg_halt_req | step_pend_q) & ~debug_q & ~req_hold_q;
    wire irq_now   = irq_valid & ~debug_q & ~step_armed_q & ~req_hold_q &
                     ~halt_now;
    wire evt_ready = x_valid & m_done & ~m_trap;

    wire ebrk_dbg      = x_exc_ebrk & csr_dcsr_ebreakm;
    wire take_halt     = evt_ready & halt_now;
    wire take_irq      = evt_ready & irq_now;
    wire take_exc      = evt_ready & ~halt_now & ~irq_now & x_exc &
                         ~load_use & ~ebrk_dbg;
    wire take_ebrk_dbg = evt_ready & ~halt_now & ~irq_now & ebrk_dbg;

    wire x_kill = halt_now | irq_now | x_exc;

    // ------------------------------------------------------------------
    // Normal execution
    // ------------------------------------------------------------------
    wire x_exec_ok = x_valid & ~x_kill & ~load_use & ~x_csr_wait & m_done &
                     ~m_trap & ~sleep_q & ~debug_q;

    assign dbus_req   = x_exec_ok & d_is_mem;
    assign dbus_we    = d_is_store;
    assign dbus_be    = mem_be;
    assign dbus_addr  = mem_addr;
    assign dbus_wdata = mem_wdata;

    assign md_start = x_exec_ok & d_is_md & ~md_busy & ~md_done;
    wire   mem_ok   = ~d_is_mem | dbus_gnt;
    wire   md_ok    = ~d_is_md | md_done;

    assign x_fire = x_exec_ok & mem_ok & md_ok;
    assign md_ack = x_fire & d_is_md;

    // ------------------------------------------------------------------
    // Redirects
    // ------------------------------------------------------------------
    wire br_mispred = d_is_branch & (br_taken != x_pred_taken);
    wire x_redirect = x_fire & (br_mispred | (d_is_jal & ~x_pred_taken) |
                                d_is_jalr | d_is_mret | d_is_fencei);
    wire [31:0] x_redir_pc =
        d_is_jalr                 ? {alu_add[31:1], 1'b0} :
        d_is_mret                 ? csr_mepc :
        d_is_fencei               ? x_pc_next :
        (d_is_jal | br_taken)     ? btarget : x_pc_next;

    wire trap_x     = take_irq | take_exc;
    wire dbg_resume = debug_q & dbg_resume_req;

    assign trap_en     = m_trap | trap_x;
    assign trap_is_irq = ~m_trap & take_irq;
    assign trap_cause  = m_trap   ? (m_is_store ? `GUST_EXC_SFAULT : `GUST_EXC_LFAULT) :
                         take_irq ? irq_cause : exc_cause;
    assign trap_pc     = m_trap ? m_pc : x_pc;
    assign trap_tval   = m_trap ? m_result : (take_irq ? 32'b0 : exc_tval);

    assign dbg_enter = take_halt | take_ebrk_dbg;
    assign dbg_cause = take_ebrk_dbg ? 3'd1 : (step_pend_q ? 3'd4 : 3'd3);

    assign flush    = x_redirect | trap_en | dbg_resume;
    assign flush_pc = trap_en    ? csr_trap_vector :
                      dbg_resume ? csr_dpc : x_redir_pc;

    // ------------------------------------------------------------------
    // X-stage result
    // ------------------------------------------------------------------
    reg [31:0] x_result;
    always @* begin
        case (d_res_sel)
        `GUST_RES_LINK: x_result = x_pc_next;
        `GUST_RES_CSR:  x_result = csr_rdata;
        `GUST_RES_MUL:  x_result = mul_res;
        `GUST_RES_MD:   x_result = md_result;
        default:        x_result = alu_result;
        endcase
    end

    // ------------------------------------------------------------------
    // Pipeline advance
    // ------------------------------------------------------------------
    wire x_clear = flush | dbg_enter;
    assign md_abort = x_clear;
    assign f_ready  = (~x_valid | x_fire) & ~sleep_q & ~debug_q;
    wire   f_take   = f_valid & f_ready & ~flush;

    always @(posedge gclk or negedge rst_n) begin
        if (!rst_n)       x_valid <= 1'b0;
        else if (x_clear) x_valid <= 1'b0;
        else if (f_ready) x_valid <= f_valid;
    end

    always @(posedge gclk) begin
        if (f_take) begin
            x_instr      <= f_instr;
            x_pc_r       <= f_pc[31:1];
            x_compressed <= f_compressed;
            x_illegal_c  <= f_illegal;
            x_fetch_err  <= f_fetch_err;
            x_pred_taken <= f_pred_taken;
        end
    end

    always @(posedge gclk or negedge rst_n) begin
        if (!rst_n)      m_valid <= 1'b0;
        else if (m_done) m_valid <= x_fire;
    end

    always @(posedge gclk) begin
        if (x_fire) begin
            m_rd       <= x_rd;
            m_wen      <= d_rd_wen & (|x_rd);
            m_is_load  <= d_is_load;
            m_is_store <= d_is_store;
            m_funct3   <= x_f3;
        end
        if (x_fire & (d_rd_wen | d_is_mem)) m_result <= x_result;
        if (x_fire & d_is_mem)              m_pc_r   <= x_pc_r;
    end

    // ------------------------------------------------------------------
    // Debug / single-step state, bus request hold
    // ------------------------------------------------------------------
    wire step_done = step_armed_q & (x_fire | trap_en | take_ebrk_dbg);

    always @(posedge gclk or negedge rst_n) begin
        if (!rst_n) begin
            debug_q      <= 1'b0;
            step_armed_q <= 1'b0;
            step_pend_q  <= 1'b0;
            req_hold_q   <= 1'b0;
        end else begin
            if (dbg_enter)       debug_q <= 1'b1;
            else if (dbg_resume) debug_q <= 1'b0;

            if (dbg_resume)      step_armed_q <= csr_dcsr_step;
            else if (step_done)  step_armed_q <= 1'b0;

            if (dbg_enter)       step_pend_q <= 1'b0;
            else if (step_done & ~take_ebrk_dbg) step_pend_q <= 1'b1;

            req_hold_q <= dbus_req & ~dbus_gnt;
        end
    end

    // ------------------------------------------------------------------
    // Sleep (WFI).  sleep_q lives on the free-running clock so that it can
    // be cleared while gclk is stopped.  The core clock is only stopped once
    // nothing is in flight.
    // ------------------------------------------------------------------
    wire wake     = irq_any | dbg_halt_req | step_pend_q;
    wire wfi_fire = x_fire & d_is_wfi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        sleep_q <= 1'b0;
        else if (wake)     sleep_q <= 1'b0;
        else if (wfi_fire) sleep_q <= 1'b1;
    end

    assign clk_en   = ~sleep_q | wake | f_busy | m_valid | req_hold_q;
    assign sleeping = sleep_q;
endmodule
