/*
 * Tiny Tapeout wrapper for the Gust v1.0 ALU (one block of the Gust RV32 core).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pins
 *   ui_in[7:0]   data byte to load
 *   uio[0]  in   LOAD   : on a rising edge, ui_in is loaded into the target below
 *   uio[2:1] in  TARGET : 0 = operand A, 1 = operand B (shifted in LSB byte first,
 *                         four loads per operand), 2 = OP (ui_in[5:0])
 *   uio[4:3] in  RSEL   : which byte of the registered result appears on uo_out
 *   uio[6]  out  EQ     : A == B
 *   uio[7]  out  LT     : A < B (signed)
 *   uo_out[7:0]  selected result byte
 * All 34 Gust ALU operations (RV32I + Zba/Zbb/Zbs) are available.
 */
`default_nettype none

module tt_um_gust_alu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    reg  [31:0] a, b, result_q;
    reg  [5:0]  op;
    reg         load_q, eq_q, lt_q;
    wire [31:0] result, add;
    wire        eq, lt, ltu;

    gust_alu #(.EXT_B(1), .OPERAND_ISO(0)) u_alu (
        .op(op), .a(a), .b(b), .result(result), .add(add), .eq(eq), .lt(lt), .ltu(ltu)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a <= 32'h0; b <= 32'h0; op <= 6'h0; load_q <= 1'b0;
            result_q <= 32'h0; eq_q <= 1'b0; lt_q <= 1'b0;
        end else begin
            load_q <= uio_in[0];
            if (uio_in[0] && !load_q) begin
                case (uio_in[2:1])
                    2'd0: a  <= {ui_in, a[31:8]};
                    2'd1: b  <= {ui_in, b[31:8]};
                    2'd2: op <= ui_in[5:0];
                    default: ;
                endcase
            end
            result_q <= result;
            eq_q     <= eq;
            lt_q     <= lt;
        end
    end

    assign uo_out  = result_q[8*uio_in[4:3] +: 8];
    assign uio_out = {lt_q, eq_q, 6'b0};
    assign uio_oe  = 8'b1100_0000;

    wire _unused = &{ena, uio_in[7:5], add, ltu, 1'b0};
endmodule
