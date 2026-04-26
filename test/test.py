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
async def test_rqpu_v2_fit_plus_breglite(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    # Base datapath and reverse
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x6)  # SETACC 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ALU, 0, FUNC0, 0x0, 0x3)  # ADD imm 3 => 9
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)  # REVERSE => 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # Hidden BREG-lite: set and use as ALU rhs source.
    out = await issue(dut, CLS_CTRL, 1, FUNC0, 0x0, 0x5)  # SETBREG 5
    assert out["data"] == 0x5 and out["z"] == 0 and out["c"] == 0, out

    # A[0]=1 selects BREG as rhs, B nibble is ignored.
    out = await issue(dut, CLS_ALU, 0, FUNC0, 0x1, 0x0)  # ACC(6) + BREG(5) => B
    assert out["data"] == 0xB and out["z"] == 0 and out["c"] == 0, out

    # Extended ALU pack via immediate rhs.
    out = await issue(dut, CLS_ALU, 1, FUNC0, 0x0, 0x2)  # SUB 2 => 9, no borrow
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 1, out

    out = await issue(dut, CLS_ALU, 1, FUNC1, 0x0, 0x5)  # XNOR 5 => ~(9^5)=3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ALU, 1, FUNC2, 0x0, 0xF)  # NAND F => C
    assert out["data"] == 0xC and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ALU, 1, FUNC3, 0x0, 0x0)  # NOR 0 => 3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out

    # Permute/encoding extension pack.
    out = await issue(dut, CLS_PERM, 1, FUNC0, 0x0, 0x0)  # SHL 3 => 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_PERM, 1, FUNC1, 0x0, 0x0)  # SHR 6 => 3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_PERM, 1, FUNC2, 0x0, 0x0)  # GRAYENC 3 => 2
    assert out["data"] == 0x2 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_PERM, 1, FUNC3, 0x0, 0x0)  # GRAYDEC 2 => 3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out

    # Compare visibility.
    out = await issue(dut, CLS_CTRL, 0, FUNC2, 0x0, 0x3)  # CMP 3 vs 3
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x6)  # SETACC 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, FUNC2, 0x0, 0x3)  # CMP 6 vs 3 => GT
    assert out["data"] == 0x4 and out["z"] == 0 and out["c"] == 1, out

    # ECC nibble.
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0xB)  # SETACC B
    assert out["data"] == 0xB and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 0, FUNC3, 0x0, 0x0)
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 1, out

    # RAM write/read/loadacc.
    out = await issue(dut, CLS_MEM, 0, FUNC1, 0x2, 0xA)  # WRITE RAM[2] = A
    assert out["data"] == 0xA and out["z"] == 0, out
    out = await issue(dut, CLS_MEM, 0, FUNC0, 0x2, 0x0)  # READ RAM[2]
    assert out["data"] == 0xA and out["z"] == 0, out
    out = await issue(dut, CLS_MEM, 0, FUNC2, 0x2, 0x0)  # LOADACC RAM[2]
    assert out["data"] == 0xA and out["z"] == 0, out

    # REVERSE after LOADACC restores prior ACC and BREG state,
    # but visible OUT restores prev_out from the checkpointed state.
    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)
    assert out["data"] == 0xA and out["z"] == 0, out

    # Verify REVERSE after SETBREG truly restores hidden BREG.
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x1)  # SETACC 1
    assert out["data"] == 0x1 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_CTRL, 1, FUNC0, 0x0, 0x5)  # SETBREG 5
    assert out["data"] == 0x5 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_CTRL, 1, FUNC0, 0x0, 0x7)  # SETBREG 7
    assert out["data"] == 0x7 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)  # REVERSE => prev_out=5, BREG=5
    assert out["data"] == 0x5 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_ALU, 0, FUNC0, 0x1, 0x0)  # ACC(1) + BREG(5) => 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # Verify ACC <-> BREG uses BREG as the swapped partner.
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0xA)  # SETACC A
    assert out["data"] == 0xA and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_CTRL, 1, FUNC0, 0x0, 0x3)  # SETBREG 3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 1, FUNC2, 0x0, 0x0)  # ACC <-> BREG => ACC=3, BREG=A
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_ALU, 1, FUNC0, 0x1, 0x0)  # SUB using BREG => 3 - A = 9, borrow
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    # Verify REVERSE directly after ACC <-> BREG restores hidden BREG.
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0xA)  # SETACC A
    assert out["data"] == 0xA and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_CTRL, 1, FUNC0, 0x0, 0x3)  # SETBREG 3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 1, FUNC2, 0x0, 0x0)  # ACC <-> BREG
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)  # REVERSE => restores ACC=A, BREG=3
    assert out["data"] == 0x3 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, CLS_ALU, 1, FUNC0, 0x1, 0x0)  # SUB using restored BREG => A - 3 = 7, no borrow
    assert out["data"] == 0x7 and out["z"] == 0 and out["c"] == 1, out
