import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

CLS_ALU   = 0b000
CLS_PERM  = 0b001
CLS_MEM   = 0b010
CLS_REV   = 0b011
CLS_CTRL  = 0b100
CLS_RFALU = 0b101
CLS_RFIO  = 0b110

FUNC0 = 0x0
FUNC1 = 0x1
FUNC2 = 0x2
FUNC3 = 0x3


def pack_instr(cls: int, mode: int, func: int) -> int:
    return ((cls & 0x7) << 5) | ((mode & 0x1) << 4) | (func & 0xF)


def pack_oper(a: int, b: int) -> int:
    return ((a & 0xF) << 4) | (b & 0xF)


def decode_uo(raw: int) -> dict:
    return {
        "raw": raw & 0xFF,
        "phase": (raw >> 6) & 0x3,
        "data": (raw >> 2) & 0xF,
        "z": (raw >> 1) & 0x1,
        "c": raw & 0x1,
    }


async def edge_settle() -> None:
    await Timer(2, unit="ns")
    await ReadOnly()


async def advance_and_check(dut, expected_phase: int) -> dict:
    await RisingEdge(dut.clk)
    await edge_settle()
    state = decode_uo(int(dut.uo_out.value))
    assert state["phase"] == expected_phase, (
        f"expected phase {expected_phase:02b}, got {state['phase']:02b} "
        f"(uo_out={state['raw']:08b})"
    )
    return state


async def sync_p0_writable(dut, max_edges: int = 8) -> None:
    for _ in range(max_edges):
        await Timer(1, unit="ns")
        state = decode_uo(int(dut.uo_out.value))
        if state["phase"] == P0_INSTR:
            return
        await advance_and_check(dut, (state["phase"] + 1) & 0x3)
    raise AssertionError("failed to synchronize to phase 00")


async def reset_dut(dut) -> None:
    dut.ena.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)
    await Timer(1, unit="ns")

    state = decode_uo(int(dut.uo_out.value))
    assert state["phase"] == P0_INSTR, f"reset phase mismatch: {state}"
    assert int(dut.uio_out.value) == 0, f"uio_out should be 0, got {int(dut.uio_out.value):08b}"
    assert int(dut.uio_oe.value) == 0, f"uio_oe should be 0, got {int(dut.uio_oe.value):08b}"

    dut.ena.value = 1
    await Timer(1, unit="ns")


async def issue(dut, cls: int, mode: int, func: int, a: int = 0, b: int = 0) -> dict:
    await sync_p0_writable(dut)

    dut.ui_in.value = pack_instr(cls, mode, func)
    await advance_and_check(dut, P1_OPERAND)

    await Timer(1, unit="ns")
    dut.ui_in.value = pack_oper(a, b)
    await advance_and_check(dut, P2_EXECUTE)

    s3 = await advance_and_check(dut, P3_OUTPUT)
    await advance_and_check(dut, P0_INSTR)

    await Timer(1, unit="ns")
    dut.ui_in.value = 0
    return s3


@cocotb.test()
async def test_rqpu_with_4x4_register_file(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    # SETACC + immediate ALU path
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x6)  # SETACC 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_ALU, 0, FUNC0, 0x0, 0x3)   # ADD 3
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    # Load standalone register file entries: r0=5, r1=3
    out = await issue(dut, CLS_RFIO, 0, FUNC0, 0x0, 0x5)  # RFLOADI r0, 5
    assert out["data"] == 0x5 and out["z"] == 0, out
    out = await issue(dut, CLS_RFIO, 0, FUNC0, 0x1, 0x3)  # RFLOADI r1, 3
    assert out["data"] == 0x3 and out["z"] == 0, out

    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x0, 0x0)  # RFREAD r0
    assert out["data"] == 0x5 and out["z"] == 0, out
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x1, 0x0)  # RFREAD r1
    assert out["data"] == 0x3 and out["z"] == 0, out

    # r0 + r1 -> r2
    a = ((0 & 0x3) << 2) | (1 & 0x3)
    b = ((2 & 0x3) << 2)
    out = await issue(dut, CLS_RFALU, 0, FUNC0, a, b)     # ADD
    assert out["data"] == 0x8 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x2, 0x0)  # RFREAD r2
    assert out["data"] == 0x8 and out["z"] == 0, out

    # r0 - r1 -> r3
    a = ((0 & 0x3) << 2) | (1 & 0x3)
    b = ((3 & 0x3) << 2)
    out = await issue(dut, CLS_RFALU, 1, FUNC0, a, b)     # SUB
    assert out["data"] == 0x2 and out["z"] == 0 and out["c"] == 1, out
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x3, 0x0)  # RFREAD r3
    assert out["data"] == 0x2 and out["z"] == 0, out

    # Move r2 into ACC, then store ACC into r1
    out = await issue(dut, CLS_RFIO, 0, FUNC2, 0x2, 0x0)  # RFTOACC r2
    assert out["data"] == 0x8 and out["z"] == 0, out
    out = await issue(dut, CLS_RFIO, 0, FUNC3, 0x1, 0x0)  # ACCTORF r1
    assert out["data"] == 0x8 and out["z"] == 0, out
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x1, 0x0)  # RFREAD r1
    assert out["data"] == 0x8 and out["z"] == 0, out

    # REVERSE after RF register write should restore the old value of r1 (=3)
    _ = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)     # REVERSE
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x1, 0x0)  # RFREAD r1
    assert out["data"] == 0x3 and out["z"] == 0, out

    # REVERSE after RFALU write should restore r2 old value (currently 8 -> overwrite with XOR -> restore 8)
    a = ((0 & 0x3) << 2) | (1 & 0x3)
    b = ((2 & 0x3) << 2)
    out = await issue(dut, CLS_RFALU, 0, FUNC1, a, b)     # XOR r0 ^ r1 -> r2 = 6
    assert out["data"] == 0x6 and out["z"] == 0, out
    _ = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)     # REVERSE
    out = await issue(dut, CLS_RFIO, 0, FUNC1, 0x2, 0x0)  # RFREAD r2
    assert out["data"] == 0x8 and out["z"] == 0, out
