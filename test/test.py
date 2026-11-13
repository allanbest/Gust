# SPDX-License-Identifier: Apache-2.0
# Test for tt_um_gust: the testbench acts as external memory over the pins
# and runs a small RISC-V program (sum of 1..10, result written to 0x8000_0000).

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# sum.S assembled for RV32IMC at address 0:
#   li a0,0; li t0,1; li t1,11; loop: add a0,a0,t0; addi t0,t0,1; bne t0,t1,loop
#   li t2,0x80000000; sw a0,0(t2); j .
PROGRAM = [0x42854501, 0x9516432D, 0x9EE30285, 0x03B7FE62, 0xA0238000, 0xA00100A3]

OUT_TGL, ACK, IN_TGL = 0, 1, 2


class Host:
    def __init__(self, dut):
        self.dut = dut
        self.ack = 0
        self.in_tgl = 0
        self.mem = {i * 4: w for i, w in enumerate(PROGRAM)}

    def _drive_uio(self):
        self.dut.uio_in.value = (self.in_tgl << IN_TGL) | (self.ack << ACK)

    async def get_byte(self, limit=2000):
        for _ in range(limit):
            if ((int(self.dut.uio_out.value) >> OUT_TGL) & 1) != self.ack:
                b = int(self.dut.uo_out.value)
                self.ack ^= 1
                self._drive_uio()
                await RisingEdge(self.dut.clk)
                return b
            await RisingEdge(self.dut.clk)
        raise AssertionError("timeout waiting for a byte from Gust")

    async def put_byte(self, b):
        self.dut.ui_in.value = b
        await RisingEdge(self.dut.clk)
        self.in_tgl ^= 1
        self._drive_uio()
        await ClockCycles(self.dut.clk, 4)   # protocol: hold >= 4 cycles

    async def transaction(self):
        cmd = await self.get_byte()
        addr = 0
        for _ in range(4):
            addr = (addr << 8) | await self.get_byte()
        if cmd & 0x40:  # write
            data = 0
            for k in range(4):
                data |= (await self.get_byte()) << (8 * k)
            return ("w", addr, data)
        word = self.mem.get(addr & ~3, 0)
        for k in range(4):
            await self.put_byte((word >> (8 * k)) & 0xFF)
        return ("r", addr, word)


@cocotb.test()
async def test_runs_program(dut):
    clock = Clock(dut.clk, 20, unit="ns")    # 50 MHz
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    host = Host(dut)
    first = None
    for _ in range(400):
        kind, addr, data = await host.transaction()
        if first is None:
            first = (kind, addr)
            assert first == ("r", 0), f"first access should be a fetch from 0, got {first}"
        if kind == "w" and addr == 0x80000000:
            dut._log.info(f"Gust wrote result {data}")
            assert data == 55, f"expected 55, got {data}"
            return
    raise AssertionError("program did not finish")
