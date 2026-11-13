## How it works

Gust v1.0 is a 32-bit RISC-V microcontroller core (RV32IMC + Zicsr + Zifencei + Zba/Zbb/Zbs + Zcb, machine mode) with a three-stage pipeline: fetch, execute, memory/write-back. It has a single-cycle multiplier, a compressed-instruction expander and static branch prediction with zero-bubble taken branches. On-chip RAM measured about 2.93 CoreMark/MHz in simulation.

A Tiny Tapeout tile has no room for RAM, so this project wraps the core in a bridge that turns its instruction and data buses into a byte-serial protocol on the pins. An external microcontroller acts as the memory.

Each bus transaction (the data bus has priority over the instruction bus):

1. Gust sends 5 bytes on `uo_out`: `CMD`, then `ADDR[31:24]`, `ADDR[23:16]`, `ADDR[15:8]`, `ADDR[7:0]`.
   `CMD = {DATA_BUS, WRITE, 0, 0, BYTE_EN[3:0]}`.
2. For a write, Gust then sends 4 data bytes, least significant first.
3. For a read, the host sends 4 data bytes on `ui_in`, least significant first.

Handshake:

- Gust toggles `OUT_TGL` (uio[0]) when a new byte is on `uo_out`. The host takes it and sets `ACK` (uio[1]) equal to `OUT_TGL`.
- The host puts a byte on `ui_in` and then toggles `IN_TGL` (uio[2]). It must hold `ui_in` stable, and must not toggle `IN_TGL` again, for at least 4 Gust clock cycles.

All handshake inputs pass through two-flop synchronisers, so the host may run at any speed. The core starts executing at address 0 after reset. `IRQ` (uio[5]) is fast interrupt 0 (mcause 16). `SLEEP` shows the core is in `wfi`, and `BUSY` shows a transaction is in progress.

## How to test

The RP2040/RP2350 on the Tiny Tapeout demo board can act as the memory host:

1. Load a program built with `-march=rv32imc_zba_zbb_zbs_zcb -mabi=ilp32` and linked at address 0 into the host's RAM.
2. Release reset and serve each transaction as described above.

The cocotb test in `test/` does this for a small program that sums 1 to 10 and writes the result (55) to address `0x8000_0000`.

## External hardware

None besides the demo board's microcontroller acting as memory.

Credits: the Gust core architecture and RTL were designed by Claude Fable 5.2. The Tiny Tapeout bridge and tests were written by Claude.
