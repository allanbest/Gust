# SPDX-License-Identifier: Apache-2.0
# Test for tt_um_gust_alu: load operands over the pins, compare every ALU
# operation against a Python reference model.

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

M = 0xFFFFFFFF


def s32(x):
    return x - (1 << 32) if x & 0x80000000 else x


def ref(op, a, b):
    sh = b & 31
    rol = lambda x, n: ((x << n) | (x >> (32 - n))) & M if n else x
    return {
        0: (a + b) & M, 1: (a - b) & M, 2: int(s32(a) < s32(b)), 3: int(a < b),
        4: a ^ b, 5: a | b, 6: a & b, 7: (a << sh) & M, 8: a >> sh, 9: (s32(a) >> sh) & M,
        10: ((a << 1) + b) & M, 11: ((a << 2) + b) & M, 12: ((a << 3) + b) & M,
        13: a & ~b & M, 14: (a | ~b) & M, 15: ~(a ^ b) & M,
        16: a if s32(a) < s32(b) else b, 17: min(a, b), 18: a if s32(a) > s32(b) else b, 19: max(a, b),
        20: rol(a, sh), 21: rol(a, (32 - sh) & 31),
        22: 32 - a.bit_length(), 23: ((a & -a).bit_length() - 1) if a else 32, 24: bin(a).count("1"),
        25: (a & 0xFF) | (0xFFFFFF00 if a & 0x80 else 0), 26: (a & 0xFFFF) | (0xFFFF0000 if a & 0x8000 else 0),
        27: a & 0xFFFF,
        28: sum((0xFF << (8 * i)) for i in range(4) if (a >> (8 * i)) & 0xFF),
        29: int.from_bytes(a.to_bytes(4, "little"), "big"),
        30: a | (1 << sh), 31: a & ~(1 << sh) & M, 32: a ^ (1 << sh), 33: (a >> sh) & 1,
    }[op]


async def load(dut, target, byte):
    dut.ui_in.value = byte
    dut.uio_in.value = target << 1
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = (target << 1) | 1     # LOAD rising edge
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = target << 1
    await ClockCycles(dut.clk, 1)


async def run_op(dut, op, a, b):
    for k in range(4):
        await load(dut, 0, (a >> (8 * k)) & 0xFF)
    for k in range(4):
        await load(dut, 1, (b >> (8 * k)) & 0xFF)
    await load(dut, 2, op)
    await ClockCycles(dut.clk, 2)
    r = 0
    for k in range(4):
        dut.uio_in.value = k << 3
        await ClockCycles(dut.clk, 1)
        r |= int(dut.uo_out.value) << (8 * k)
    return r


@cocotb.test()
async def test_alu_ops(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    rng = random.Random(1)
    edge = [0, 1, 0x7FFFFFFF, 0x80000000, M, 0x80, 0x8000, 31, 32]
    checked = 0
    for op in range(34):
        for _ in range(4):
            a = rng.choice(edge + [rng.getrandbits(32)] * 3)
            b = rng.choice(edge + [rng.getrandbits(32)] * 3)
            got = await run_op(dut, op, a, b)
            want = ref(op, a, b)
            assert got == want, f"op {op}: a={a:08x} b={b:08x} got {got:08x} want {want:08x}"
            checked += 1
    dut._log.info(f"{checked} ALU operations matched the reference model")
