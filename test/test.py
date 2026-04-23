# SPDX-License-Identifier: Apache-2.0
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

OP_ADD = 0b000
OP_XOR = 0b001
OP_AND = 0b010
OP_COMPARE = 0b011
OP_SWAP = 0b100
OP_PARITY = 0b101
OP_SHIFT = 0b110
OP_REVERSE = 0b111


def pack_ui(execute: int, operand: int, opcode: int) -> int:
    return ((execute & 0x1) << 7) | ((operand & 0xF) << 3) | (opcode & 0x7)


def result_nibble(dut) -> int:
    return int(dut.uo_out.value) & 0xF


def flag_z(dut) -> int:
    return (int(dut.uo_out.value) >> 4) & 0x1


def flag_c(dut) -> int:
    return (int(dut.uo_out.value) >> 5) & 0x1


async def apply_reset(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)


async def load_acc(dut, value: int):
    dut.ui_in.value = pack_ui(0, value, 0)
    await ClockCycles(dut.clk, 1)


async def execute_op(dut, opcode: int, operand: int):
    dut.ui_in.value = pack_ui(1, operand, opcode)
    await ClockCycles(dut.clk, 1)


@cocotb.test()
async def test_rqpu_demo_flow(dut):
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    await apply_reset(dut)

    assert int(dut.uio_out.value) == 0
    assert int(dut.uio_oe.value) == 0

    # LOAD 0x6
    await load_acc(dut, 0x6)
    assert result_nibble(dut) == 0x6
    assert flag_z(dut) == 0
    assert flag_c(dut) == 0

    # ADD 0x3 => 0x9
    await execute_op(dut, OP_ADD, 0x3)
    assert result_nibble(dut) == 0x9
    assert flag_z(dut) == 0
    assert flag_c(dut) == 0

    # REVERSE should go back to 0x6
    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0x6

    # REVERSE again should replay the ADD and return to 0x9
    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0x9

    # XOR 0x5 => 0xC
    await execute_op(dut, OP_XOR, 0x5)
    assert result_nibble(dut) == 0xC

    # REVERSE returns to 0x9
    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0x9

    # COMPARE 0xC against 0xC => result {0,gt,eq,lt} = 0b0010
    await load_acc(dut, 0xC)
    await execute_op(dut, OP_COMPARE, 0xC)
    assert result_nibble(dut) == 0x2
    assert flag_z(dut) == 1  # equal
    assert flag_c(dut) == 0  # not greater

    # REVERSE restores the loaded 0xC display state
    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0xC

    # SWAP accumulator halves of 0xC (1100) when operand[0]=1 => 0x3
    await execute_op(dut, OP_SWAP, 0x1)
    assert result_nibble(dut) == 0x3
    assert flag_c(dut) == 1

    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0xC

    # SHIFT left 0xC (1100) => 0x9 (1001), shifted_out = 1
    await execute_op(dut, OP_SHIFT, 0x0)
    assert result_nibble(dut) == 0x9
    assert flag_c(dut) == 1

    await execute_op(dut, OP_REVERSE, 0x0)
    assert result_nibble(dut) == 0xC

    # PARITY demo: acc = 0x7, operand = 0x1
    # combined parity is even => parity_bit = 0, hamming_bit(acc)=1 => result 0b0010
    await load_acc(dut, 0x7)
    await execute_op(dut, OP_PARITY, 0x1)
    assert result_nibble(dut) == 0x2
    assert flag_z(dut) == 1
    assert flag_c(dut) == 1
