// =============================================================================
// Gust N1 - processor core (stage 1)
//
//   n1_core = gust_core (Gust v1.0 core with M/S/U privilege added)
//           + gust_mmu  (Sv32 translation)
//
// Same ports as gust_core, so it drops into the Gust SoC and test benches.
// The two buses carry PHYSICAL addresses.
// =============================================================================
module n1_core #(
    parameter [31:0] RESET_VECTOR = 32'h0000_0000,
    parameter        EXT_B        = 1,
    parameter        EXT_ZCB      = 1,
    parameter        MUL_FAST     = 1,
    parameter        BRANCH_PRED  = 1,
    parameter        OPERAND_ISO  = 1,
    parameter        CLOCK_GATE   = 0,
    parameter        TLB_ENTRIES  = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    output wire        ibus_req,
    output wire [31:0] ibus_addr,
    input  wire        ibus_gnt,
    input  wire        ibus_rvalid,
    input  wire [31:0] ibus_rdata,
    input  wire        ibus_err,
    output wire        dbus_req,
    output wire        dbus_we,
    output wire [3:0]  dbus_be,
    output wire [31:0] dbus_addr,
    output wire [31:0] dbus_wdata,
    input  wire        dbus_gnt,
    input  wire        dbus_rvalid,
    input  wire [31:0] dbus_rdata,
    input  wire        dbus_err,
    input  wire        irq_software,
    input  wire        irq_timer,
    input  wire        irq_external,
    input  wire [15:0] irq_fast,
    input  wire        dbg_halt_req,
    input  wire        dbg_resume_req,
    output wire        dbg_halted,
    input  wire        dbg_reg_req,
    input  wire        dbg_reg_we,
    input  wire [15:0] dbg_reg_no,
    input  wire [31:0] dbg_reg_wdata,
    output wire [31:0] dbg_reg_rdata,
    output wire        dbg_reg_err,
    output wire        sleeping
);
    wire        ci_req, ci_gnt, ci_rvalid, ci_err, ci_pf;
    wire [31:0] ci_addr, ci_rdata;
    wire        cd_req, cd_we, cd_gnt, cd_rvalid, cd_err, cd_pf;
    wire [3:0]  cd_be;
    wire [31:0] cd_addr, cd_wdata, cd_rdata;
    wire [1:0]  priv_i, priv_d;
    wire        satp_mode, sum, mxr, tlb_flush;
    wire [19:0] satp_ppn;

    gust_core #(
        .RESET_VECTOR(RESET_VECTOR), .EXT_B(EXT_B), .EXT_ZCB(EXT_ZCB),
        .MUL_FAST(MUL_FAST), .BRANCH_PRED(BRANCH_PRED),
        .OPERAND_ISO(OPERAND_ISO), .CLOCK_GATE(CLOCK_GATE)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .ibus_req(ci_req), .ibus_addr(ci_addr), .ibus_gnt(ci_gnt),
        .ibus_rvalid(ci_rvalid), .ibus_rdata(ci_rdata), .ibus_err(ci_err), .ibus_pf(ci_pf),
        .dbus_req(cd_req), .dbus_we(cd_we), .dbus_be(cd_be), .dbus_addr(cd_addr),
        .dbus_wdata(cd_wdata), .dbus_gnt(cd_gnt), .dbus_rvalid(cd_rvalid),
        .dbus_rdata(cd_rdata), .dbus_err(cd_err), .dbus_pf(cd_pf),
        .irq_software(irq_software), .irq_timer(irq_timer),
        .irq_external(irq_external), .irq_fast(irq_fast),
        .dbg_halt_req(dbg_halt_req), .dbg_resume_req(dbg_resume_req),
        .dbg_halted(dbg_halted), .dbg_reg_req(dbg_reg_req), .dbg_reg_we(dbg_reg_we),
        .dbg_reg_no(dbg_reg_no), .dbg_reg_wdata(dbg_reg_wdata),
        .dbg_reg_rdata(dbg_reg_rdata), .dbg_reg_err(dbg_reg_err), .sleeping(sleeping),
        .mmu_priv_i(priv_i), .mmu_priv_d(priv_d), .mmu_satp_mode(satp_mode),
        .mmu_satp_ppn(satp_ppn), .mmu_sum(sum), .mmu_mxr(mxr), .mmu_tlb_flush(tlb_flush)
    );

    // The MMU runs on the free-running clock: a page-table walk must be able
    // to finish even if the core clock gate closes.
    gust_mmu #(.ENTRIES(TLB_ENTRIES)) u_mmu (
        .clk(clk), .rst_n(rst_n),
        .priv_i(priv_i), .priv_d(priv_d), .satp_mode(satp_mode), .satp_ppn(satp_ppn),
        .sum(sum), .mxr(mxr), .tlb_flush(tlb_flush),
        .c_ibus_req(ci_req), .c_ibus_addr(ci_addr), .c_ibus_gnt(ci_gnt),
        .c_ibus_rvalid(ci_rvalid), .c_ibus_rdata(ci_rdata), .c_ibus_err(ci_err), .c_ibus_pf(ci_pf),
        .c_dbus_req(cd_req), .c_dbus_we(cd_we), .c_dbus_be(cd_be), .c_dbus_addr(cd_addr),
        .c_dbus_wdata(cd_wdata), .c_dbus_gnt(cd_gnt), .c_dbus_rvalid(cd_rvalid),
        .c_dbus_rdata(cd_rdata), .c_dbus_err(cd_err), .c_dbus_pf(cd_pf),
        .ibus_req(ibus_req), .ibus_addr(ibus_addr), .ibus_gnt(ibus_gnt),
        .ibus_rvalid(ibus_rvalid), .ibus_rdata(ibus_rdata), .ibus_err(ibus_err),
        .dbus_req(dbus_req), .dbus_we(dbus_we), .dbus_be(dbus_be), .dbus_addr(dbus_addr),
        .dbus_wdata(dbus_wdata), .dbus_gnt(dbus_gnt), .dbus_rvalid(dbus_rvalid),
        .dbus_rdata(dbus_rdata), .dbus_err(dbus_err)
    );
endmodule
