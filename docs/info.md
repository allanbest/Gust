## How it works

This is the ALU (arithmetic logic unit) of Gust v1.0, a 32-bit RISC-V microcontroller core designed by Claude Fable 5.2. It performs all 34 operations Gust uses: add/subtract, compares, logic, shifts, plus the Zba/Zbb/Zbs bit-manipulation set (shift-add, min/max, rotates, count leading/trailing zeros, popcount, byte reverse, sign/zero extend, single-bit set/clear/invert/extract).

Two 32-bit operand registers (A, B) and a 6-bit operation register are loaded over the pins; the result is registered and read back one byte at a time.

## How to test

1. Set `TARGET` (uio[2:1]) to 0, put a byte on `ui_in`, and pulse `LOAD` (uio[0]) high. Repeat four times to shift in operand A, least significant byte first.
2. Do the same with `TARGET` = 1 for operand B.
3. Load the operation code with `TARGET` = 2. Examples: 0 = add, 1 = sub, 22 = count leading zeros, 24 = popcount, 29 = byte reverse.
4. Two clocks later, set `RSEL` (uio[4:3]) to 0–3 to read result bytes 0–3 on `uo_out`. `EQ` (uio[6]) and `LT` (uio[7]) show A == B and signed A < B.

## External hardware

None. The demo board's microcontroller drives the pins.

Credits: Gust architecture and ALU RTL by Claude Fable 5.2. Tiny Tapeout wrapper and tests by Claude.
