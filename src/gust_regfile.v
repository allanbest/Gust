// =============================================================================
// Gust v1.0 - integer register file (x1..x31, x0 hard-wired to zero)
//
// One synchronous write port, two combinational read ports.
//
//   CLOCK_GATE = 0 (FPGA / simulation default)
//       A plain memory array with a write enable.  Yosys maps it to
//       distributed (LUT) RAM on ECP5 and Gowin.
//
//   CLOCK_GATE = 1 (ASIC)
//       31 rows of 32 flip-flops, each row behind its own integrated clock
//       gate.  A row's clock pins toggle only in the cycle that row is
//       written, instead of 992 clock pins toggling every cycle.  The rows
//       also lose their 992 enable muxes, so this version is *smaller* than
//       the enable-flop version as well as lower power.
// =============================================================================
module gust_regfile #(
    parameter CLOCK_GATE = 0
) (
    input  wire        clk,
    input  wire        we,
    input  wire [4:0]  waddr,
    input  wire [31:0] wdata,
    input  wire [4:0]  raddr1,
    input  wire [4:0]  raddr2,
    output wire [31:0] rdata1,
    output wire [31:0] rdata2
);
    generate
    if (CLOCK_GATE != 0) begin : g_gated
        wire [31:0] row [0:31];
        assign row[0] = 32'b0;
        genvar r;
        for (r = 1; r < 32; r = r + 1) begin : g_row
            wire       row_en = we & (waddr == r);
            wire       row_clk;
            reg [31:0] q;
            gust_icg u_icg (.clk(clk), .en(row_en), .gclk(row_clk));
            always @(posedge row_clk) q <= wdata;
            assign row[r] = q;
`ifdef GUST_SIM
            initial q = 32'b0;
`endif
        end
        assign rdata1 = row[raddr1];
        assign rdata2 = row[raddr2];
    end else begin : g_plain
        reg [31:0] rf [1:31];
        always @(posedge clk) begin
            if (we && (waddr != 5'd0)) rf[waddr] <= wdata;
        end
        assign rdata1 = (raddr1 == 5'd0) ? 32'b0 : rf[raddr1];
        assign rdata2 = (raddr2 == 5'd0) ? 32'b0 : rf[raddr2];
`ifdef GUST_SIM
        // Simulation only: start from zeros instead of X so that programs
        // which read a register before writing it behave deterministically.
        // Real silicon powers up with arbitrary values.
        integer i;
        initial begin
            for (i = 1; i < 32; i = i + 1) rf[i] = 32'b0;
        end
`endif
    end
    endgenerate
endmodule
