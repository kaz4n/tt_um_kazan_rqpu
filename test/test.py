import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

OP_ADD = 0b000
OP_REVERSE = 0b111

def pack_ui(execute, operand, opcode):
    return ((execute & 1) << 7) | ((operand & 0xF) << 3) | (opcode & 0x7)

@cocotb.test()
async def test_rqpu_basic(dut):
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)

    # load accumulator with 0x6
    dut.ui_in.value = pack_ui(0, 0x6, 0)
    await ClockCycles(dut.clk, 1)
    assert (int(dut.uo_out.value) & 0xF) == 0x6

    # add 0x3 -> expect 0x9
    dut.ui_in.value = pack_ui(1, 0x3, OP_ADD)
    await ClockCycles(dut.clk, 1)
    assert (int(dut.uo_out.value) & 0xF) == 0x9

    # reverse -> expect back to 0x6
    dut.ui_in.value = pack_ui(1, 0x0, OP_REVERSE)
    await ClockCycles(dut.clk, 1)
    assert (int(dut.uo_out.value) & 0xF) == 0x6
