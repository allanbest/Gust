/*
 * Tiny Tapeout wrapper for the Gust v1.0 RV32 core.
 * SPDX-License-Identifier: Apache-2.0
 *
 * The core's instruction and data buses are bridged to a byte-serial
 * external-memory protocol, so an outside microcontroller acts as memory.
 *
 * Pins
 *   uo_out[7:0]  byte from Gust to host
 *   ui_in[7:0]   byte from host to Gust
 *   uio[0] out   OUT_TGL : toggles when a new uo_out byte is valid
 *   uio[1] in    ACK     : host sets ACK = OUT_TGL after taking the byte
 *   uio[2] in    IN_TGL  : host toggles after putting a byte on ui_in
 *   uio[3] out   SLEEP   : core is in WFI
 *   uio[4] out   BUSY    : a bus transaction is in progress
 *   uio[5] in    IRQ     : interrupt request (fast interrupt 0, cause 16)
 *   uio[6..7]    unused inputs
 *
 * One transaction (data bus has priority over instruction bus):
 *   Gust -> host : CMD, ADDR[31:24], ADDR[23:16], ADDR[15:8], ADDR[7:0]
 *                  CMD = {DATA_BUS, WRITE, 2'b00, BYTE_EN[3:0]}
 *   write        : Gust -> host WDATA bytes 0..3 (LSB first), then done
 *   read         : host -> Gust RDATA bytes 0..3 (LSB first), then done
 * All handshake inputs pass through two-flop synchronisers, so the host
 * may run at any speed.  The host must keep ui_in stable, and must not toggle
 * IN_TGL again, for at least 4 Gust clock cycles after each IN_TGL toggle.
 */
`default_nettype none

module tt_um_gust (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ---------------- synchronisers ----------------
    reg [1:0] ack_sync, in_sync, irq_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_sync <= 2'b00; in_sync <= 2'b00; irq_sync <= 2'b00;
        end else begin
            ack_sync <= {ack_sync[0], uio_in[1]};
            in_sync  <= {in_sync[0],  uio_in[2]};
            irq_sync <= {irq_sync[0], uio_in[5]};
        end
    end
    wire ack_s = ack_sync[1];
    wire in_s  = in_sync[1];

    // ---------------- core ----------------
    wire        ibus_req, dbus_req, dbus_we, sleeping;
    wire [31:0] ibus_addr, dbus_addr, dbus_wdata;
    wire [3:0]  dbus_be;
    wire        ibus_gnt, dbus_gnt;
    reg         ibus_rvalid, dbus_rvalid;
    reg  [31:0] rdata;

    gust_core #(
        .RESET_VECTOR(32'h0000_0000),
        .EXT_B       (0),
        .MUL_FAST    (0),
        .CLOCK_GATE  (1)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .ibus_req(ibus_req), .ibus_addr(ibus_addr), .ibus_gnt(ibus_gnt),
        .ibus_rvalid(ibus_rvalid), .ibus_rdata(rdata), .ibus_err(1'b0),
        .dbus_req(dbus_req), .dbus_we(dbus_we), .dbus_be(dbus_be), .dbus_addr(dbus_addr),
        .dbus_wdata(dbus_wdata), .dbus_gnt(dbus_gnt), .dbus_rvalid(dbus_rvalid),
        .dbus_rdata(rdata), .dbus_err(1'b0),
        .irq_software(1'b0), .irq_timer(1'b0), .irq_external(1'b0),
        .irq_fast({15'b0, irq_sync[1]}),
        .dbg_halt_req(1'b0), .dbg_resume_req(1'b0), .dbg_halted(),
        .dbg_reg_req(1'b0), .dbg_reg_we(1'b0), .dbg_reg_no(16'h0),
        .dbg_reg_wdata(32'h0), .dbg_reg_rdata(), .dbg_reg_err(),
        .sleeping(sleeping)
    );

    // ---------------- bridge FSM ----------------
    localparam IDLE = 2'd0, SEND = 2'd1, RECV = 2'd2, DONE = 2'd3;
    reg  [1:0]  state;
    reg  [3:0]  idx;          // byte index within the current phase
    reg         sel_d, we;
    reg  [3:0]  be;
    reg  [31:0] addr, wdata;
    reg  [7:0]  out_byte;
    reg         out_tgl, waiting_ack, in_seen;

    // grants are given only while idle; data bus wins
    assign dbus_gnt = (state == IDLE) && dbus_req;
    assign ibus_gnt = (state == IDLE) && ibus_req && !dbus_req;

    wire [3:0] n_send = we ? 4'd9 : 4'd5;

    reg [7:0] next_byte;
    always @(*) begin
        case (idx)
            4'd0: next_byte = {sel_d, we, 2'b00, be};
            4'd1: next_byte = addr[31:24];
            4'd2: next_byte = addr[23:16];
            4'd3: next_byte = addr[15:8];
            4'd4: next_byte = addr[7:0];
            4'd5: next_byte = wdata[7:0];
            4'd6: next_byte = wdata[15:8];
            4'd7: next_byte = wdata[23:16];
            default: next_byte = wdata[31:24];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; idx <= 4'd0; sel_d <= 1'b0; we <= 1'b0; be <= 4'h0;
            addr <= 32'h0; wdata <= 32'h0; out_byte <= 8'h0; out_tgl <= 1'b0;
            waiting_ack <= 1'b0; in_seen <= 1'b0; rdata <= 32'h0;
            ibus_rvalid <= 1'b0; dbus_rvalid <= 1'b0;
        end else begin
            ibus_rvalid <= 1'b0;
            dbus_rvalid <= 1'b0;
            case (state)
                IDLE: begin
                    if (dbus_gnt) begin
                        sel_d <= 1'b1; we <= dbus_we; be <= dbus_be;
                        addr <= dbus_addr; wdata <= dbus_wdata;
                        idx <= 4'd0; waiting_ack <= 1'b0; state <= SEND;
                    end else if (ibus_gnt) begin
                        sel_d <= 1'b0; we <= 1'b0; be <= 4'hf;
                        addr <= ibus_addr; idx <= 4'd0; waiting_ack <= 1'b0; state <= SEND;
                    end
                end
                SEND: begin
                    if (!waiting_ack) begin
                        out_byte <= next_byte;
                        out_tgl <= ~out_tgl;
                        waiting_ack <= 1'b1;
                    end else if (ack_s == out_tgl) begin
                        waiting_ack <= 1'b0;
                        if (idx == n_send - 4'd1) begin
                            idx <= 4'd0;
                            state <= we ? DONE : RECV;
                        end else idx <= idx + 4'd1;
                    end
                end
                RECV: begin
                    if (in_s != in_seen) begin
                        in_seen <= in_s;
                        rdata <= {ui_in, rdata[31:8]};     // LSB first
                        if (idx == 4'd3) state <= DONE;
                        idx <= idx + 4'd1;
                    end
                end
                DONE: begin
                    if (sel_d) dbus_rvalid <= 1'b1; else ibus_rvalid <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

    assign uo_out  = out_byte;
    assign uio_out = {3'b000, (state != IDLE), sleeping, 2'b00, out_tgl};
    assign uio_oe  = 8'b0001_1001;

    wire _unused = &{ena, uio_in[7:6], uio_in[4:3], uio_in[0], 1'b0};
endmodule
