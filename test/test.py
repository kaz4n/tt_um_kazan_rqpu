import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

CLS_ALU  = 0b000
CLS_PERM = 0b001
CLS_MEM  = 0b010
CLS_REV  = 0b011
CLS_CTRL = 0b100

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
    assert state["data"] == 0, f"reset data mismatch: {state}"
    assert int(dut.uio_out.value) == 0
    assert int(dut.uio_oe.value) == 0

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
async def test_rqpu_v2_fit_plus(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    # SETACC 0x6
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x6)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # ADD 3 => 9
    out = await issue(dut, CLS_ALU, 0, FUNC0, 0x0, 0x3)
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    # REVERSE => back to 6
    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # SUB 2 => 4, no borrow => C=1
    out = await issue(dut, CLS_ALU, 1, FUNC0, 0x0, 0x2)
    assert out["data"] == 0x4 and out["z"] == 0 and out["c"] == 1, out

    # XNOR 5 => ~(4^5)=E
    out = await issue(dut, CLS_ALU, 1, FUNC1, 0x0, 0x5)
    assert out["data"] == 0xE and out["z"] == 0 and out["c"] == 0, out

    # NAND F => ~(E&F)=1
    out = await issue(dut, CLS_ALU, 1, FUNC2, 0x0, 0xF)
    assert out["data"] == 0x1 and out["z"] == 0 and out["c"] == 0, out

    # NOR 0 => ~(1|0)=E
    out = await issue(dut, CLS_ALU, 1, FUNC3, 0x0, 0x0)
    assert out["data"] == 0xE and out["z"] == 0 and out["c"] == 0, out

    # SHL E => C
    out = await issue(dut, CLS_PERM, 1, FUNC0, 0x0, 0x0)
    assert out["data"] == 0xC and out["z"] == 0 and out["c"] == 1, out

    # SHR C => 6
    out = await issue(dut, CLS_PERM, 1, FUNC1, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # GRAYENC 6 => 5
    out = await issue(dut, CLS_PERM, 1, FUNC2, 0x0, 0x0)
    assert out["data"] == 0x5 and out["z"] == 0 and out["c"] == 0, out

    # GRAYDEC 5 => 6
    out = await issue(dut, CLS_PERM, 1, FUNC3, 0x0, 0x0)
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # CMP 6 vs 3 => {0,GT,EQ,LT}=0100, Z=0, C=1
    out = await issue(dut, CLS_CTRL, 0, FUNC2, 0x0, 0x3)
    assert out["data"] == 0x4 and out["z"] == 0 and out["c"] == 1, out

    # CMP 6 vs 6 => 0010, Z=1, C=0
    out = await issue(dut, CLS_CTRL, 0, FUNC2, 0x0, 0x6)
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    # SETACC B, then ECC nibble should be 3 for current ecc4() definition
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0xB)
    assert out["data"] == 0xB and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 0, FUNC3, 0x0, 0x0)
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 1, out

    # RAM write/read/loadacc using 2-bit address decode
    out = await issue(dut, CLS_MEM, 0, FUNC1, 0x2, 0xA)
    assert out["data"] == 0xA and out["z"] == 0, out
    out = await issue(dut, CLS_MEM, 0, FUNC0, 0x2, 0x0)
    assert out["data"] == 0xA and out["z"] == 0, out
    out = await issue(dut, CLS_MEM, 0, FUNC2, 0x2, 0x0)
    assert out["data"] == 0xA and out["z"] == 0, out

    # REVERSE should undo last LOADACC, restoring prior ACC (=B from earlier)
    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)
    assert out["data"] == 0xA and out["z"] == 0, out
