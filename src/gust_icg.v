// =============================================================================
// Gust v1.0 - integrated clock gate wrapper
//
// gclk follows clk while `en` is high.  `en` is sampled while clk is low, so
// gclk never glitches.
//
//   default            : behavioural latch + AND (simulation, generic synth)
//   GUST_ICG_SKY130    : SkyWater sky130_fd_sc_hd integrated clock gate
//   GUST_ICG_GF180     : GF180MCU 7-track integrated clock gate
//
// FPGA builds do not instantiate this module at all (gust_core CLOCK_GATE=0);
// FPGAs should use clock enables, never gated clocks.
// =============================================================================
module gust_icg (
    input  wire clk,
    input  wire en,
    output wire gclk
);
`ifdef GUST_ICG_SKY130
    sky130_fd_sc_hd__dlclkp_1 u_icg (.CLK(clk), .GATE(en), .GCLK(gclk));
`elsif GUST_ICG_GF180
    gf180mcu_fd_sc_mcu7t5v0__icgtp_1 u_icg (.CLK(clk), .E(en), .TE(1'b0), .Q(gclk));
`else
    reg en_lat;
    /* verilator lint_off LATCH */
    /* verilator lint_off COMBDLY */
    always @(clk or en) begin
        if (!clk) en_lat = en;
    end
    /* verilator lint_on COMBDLY */
    /* verilator lint_on LATCH */
    assign gclk = clk & en_lat;
`endif
endmodule
