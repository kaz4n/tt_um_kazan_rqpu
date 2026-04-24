import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

CLS_ALU = 0b000
CLS_PERM = 0b001
CLS_MEM = 0b010
CLS_REV = 0b011
CLS_CTRL = 0b100

ALU_ADD = 0x0
PERM_SWAP2 = 0x2
MEM_READ = 0x0
MEM_WRITE = 0x1
MEM_LOADACC = 0x2
REV_SAVE = 0x0
REV_REVERSE = 0x1
REV_SWAP_ACC_SHD = 0x2
REV_PARITY = 0x3
CTRL_SETACC = 0x0
CTRL_SETSHD = 0x1
CTRL_CMP = 0x2
CTRL_CLEAR = 0x3

SYS_ACC = 0x0
SYS_SHD = 0x1
SYS_FLAGS = 0x2
SYS_OUT = 0x3


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
    _ = await advance_and_check(dut, P1_OPERAND)

    await Timer(1, unit="ns")
    dut.ui_in.value = pack_oper(a, b)
    _ = await advance_and_check(dut, P2_EXECUTE)

    s3 = await advance_and_check(dut, P3_OUTPUT)
    _ = await advance_and_check(dut, P0_INSTR)

    await Timer(1, unit="ns")
    dut.ui_in.value = 0
    return s3


@cocotb.test()
async def test_rqpu_v2_fit(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    out = await issue(dut, CLS_CTRL, 0, CTRL_SETACC, 0x0, 0x6)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ALU, 0, ALU_ADD, 0x0, 0x3)
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # Single-step reverse is consumed; a second reverse is a no-op.
    out = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, CTRL_SETSHD, 0x0, 0x5)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, MEM_READ, SYS_SHD, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_SWAP_ACC_SHD, 0x0, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, MEM_READ, SYS_ACC, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, MEM_READ, SYS_ACC, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_WRITE, 0x3, 0xC)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_READ, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_LOADACC, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_PERM, 0, PERM_SWAP2, 0x0, 0x0)
    assert out["data"] == 0x3 and out["z"] == 0, out

    out = await issue(dut, CLS_REV, 0, REV_PARITY, 0x0, 0x0)
    assert out["data"] in (0x0, 0x1), out

    out = await issue(dut, CLS_CTRL, 0, CTRL_CMP, 0x0, 0x3)
    assert out["z"] == 1 and out["c"] == 1, out

    out = await issue(dut, CLS_MEM, 0, MEM_WRITE, 0x3, 0xA)
    assert out["data"] == 0xA and out["z"] == 0, out

    _ = await issue(dut, CLS_REV, 0, REV_REVERSE, 0x0, 0x0)

    out = await issue(dut, CLS_MEM, 0, MEM_READ, 0x3, 0x0)
    assert out["data"] == 0xC and out["z"] == 0, out

    # RAM is 4x4, so RAM addresses decode with low 2 bits only.
    out = await issue(dut, CLS_MEM, 0, MEM_WRITE, 0x7, 0x5)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, MEM_READ, 0x3, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0, out
