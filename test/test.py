import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, ReadOnly

OP_ADD = 0b000
OP_REVERSE = 0b111

def pack_ui(execute, operand, opcode):
    return ((execute & 1) << 7) | ((operand & 0xF) << 3) | (opcode & 0x7)

async def apply_and_sample(dut, ui_value):
    await FallingEdge(dut.clk)
    dut.ui_in.value = ui_value
    await RisingEdge(dut.clk)
    await ReadOnly()
    dut._log.info(f"ui_in  = {int(dut.ui_in.value):08b}")
    dut._log.info(f"uo_out = {int(dut.uo_out.value):08b}")
    return int(dut.uo_out.value)

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

    # let reset release settle and let one clean cycle happen
    await RisingEdge(dut.clk)
    await ReadOnly()

    # load accumulator with 0x6
    out = await apply_and_sample(dut, pack_ui(0, 0x6, 0))
    assert (out & 0xF) == 0x6, f"load failed: uo_out={out:08b}"

    # add 0x3 -> expect 0x9
    out = await apply_and_sample(dut, pack_ui(1, 0x3, OP_ADD))
    assert (out & 0xF) == 0x9, f"add failed: uo_out={out:08b}"

    # reverse -> expect back to 0x6
    out = await apply_and_sample(dut, pack_ui(1, 0x0, OP_REVERSE))
    assert (out & 0xF) == 0x6, f"reverse failed: uo_out={out:08b}"