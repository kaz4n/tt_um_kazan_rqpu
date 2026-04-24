import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

CLS_ARITH = 0b000
CLS_MEM = 0b100
CLS_REV = 0b110
CLS_CTRL = 0b111

FUNC_ADD = 0x0
MEM_READ = 0x0
MEM_WRITE = 0x1
MEM_LOADACC = 0x2
REV_REVERSE = 0x1
CTRL_SET_ACC_IMM = 0xA

SYS_TMP0 = 0xC


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
    assert state["data"] == 0, f"reset data mismatch: {state}"
    assert int(dut.uio_out.value) == 0, f"uio_out should be 0, got {int(dut.uio_out.value):08b}"
    assert int(dut.uio_oe.value) == 0, f"uio_oe should be 0, got {int(dut.uio_oe.value):08b}"

    dut.ena.value = 1
    await Timer(1, unit="ns")


async def issue(dut, cls: int, mode: int, func: int, a: int = 0, b: int = 0) -> dict:
    await sync_p0_writable(dut)

    dut.ui_in.value = pack_instr(cls, mode, func)
    s1 = await advance_and_check(dut, P1_OPERAND)
    dut._log.info(f"P1 ui_in={int(dut.ui_in.value):08b} uo_out={s1['raw']:08b}")

    await Timer(1, unit="ns")
    dut.ui_in.value = pack_oper(a, b)
    s2 = await advance_and_check(dut, P2_EXECUTE)
    dut._log.info(f"P2 ui_in={int(dut.ui_in.value):08b} uo_out={s2['raw']:08b}")

    s3 = await advance_and_check(dut, P3_OUTPUT)
    dut._log.info(f"P3 ui_in={int(dut.ui_in.value):08b} uo_out={s3['raw']:08b}")

    s0 = await advance_and_check(dut, P0_INSTR)
    dut._log.info(f"P0 ui_in={int(dut.ui_in.value):08b} uo_out={s0['raw']:08b}")

    await Timer(1, unit="ns")
    dut.ui_in.value = 0
    return s3


@cocotb.test()
async def test_rqpu_v2_protocol_and_memory(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    out = await issue(dut, CLS_CTRL, 0, CTRL_SET_ACC_IMM, 0x0, 0x6)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ARITH, 0, FUNC_ADD, 0x0, 0x3)
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_WRITE, 0x3, 0xC)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_READ, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, MEM_WRITE, SYS_TMP0, 0x7)
    assert out["data"] == 0x7 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, MEM_READ, SYS_TMP0, 0x0)
    assert out["data"] == 0x7 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_WRITE, 0x3, 0xA)
    assert out["data"] == 0xA and out["z"] == 0, out

    _ = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)

    out = await issue(dut, CLS_MEM, 0, MEM_READ, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_LOADACC, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out
