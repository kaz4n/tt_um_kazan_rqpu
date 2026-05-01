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
    assert state["data"] == 0, f"reset data mismatch: {state}"
    assert int(dut.uio_out.value) == 0
    assert int(dut.uio_oe.value) == 0

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

    out = await advance_and_check(dut, P3_OUTPUT)
    dut._log.info(f"P3 ui_in={int(dut.ui_in.value):08b} uo_out={out['raw']:08b}")

    s0 = await advance_and_check(dut, P0_INSTR)
    dut._log.info(f"P0 ui_in={int(dut.ui_in.value):08b} uo_out={s0['raw']:08b}")

    await Timer(1, unit="ns")
    dut.ui_in.value = 0
    return out


async def rf_loadi(dut, idx: int, value: int) -> dict:
    return await issue(dut, CLS_RFIO, 0, FUNC0, idx & 0x3, value & 0xF)


async def rf_read(dut, idx: int) -> dict:
    return await issue(dut, CLS_RFIO, 0, FUNC1, idx & 0x3, 0)


async def rf_to_acc(dut, idx: int) -> dict:
    return await issue(dut, CLS_RFIO, 0, FUNC2, idx & 0x3, 0)


async def acc_to_rf(dut, idx: int) -> dict:
    return await issue(dut, CLS_RFIO, 0, FUNC3, idx & 0x3, 0)


async def rf_alu(dut, mode: int, func: int, rs: int, rt: int, rd: int) -> dict:
    a = ((rs & 0x3) << 2) | (rt & 0x3)
    b = ((rd & 0x3) << 2)
    return await issue(dut, CLS_RFALU, mode, func, a, b)


@cocotb.test()
async def test_rqpu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    # Core working regs according to current RTL:
    # CTRL func0=SETACC, func1=SETSHD, func2=SETBREG, func3=CMP
    out = await issue(dut, CLS_CTRL, 0, FUNC0, 0x0, 0x6)  # ACC=6
    assert out["data"] == 0x6 and out["z"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, FUNC1, 0x0, 0x9)  # SHD=9
    assert out["data"] == 0x9 and out["z"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, FUNC2, 0x0, 0xA)  # BREG=A
    assert out["data"] == 0xA and out["z"] == 0, out

    # Standalone register file: write r1=5 and r2=3
    out = await rf_loadi(dut, 1, 0x5)
    assert out["data"] == 0x5 and out["z"] == 0, out

    out = await rf_loadi(dut, 2, 0x3)
    assert out["data"] == 0x3 and out["z"] == 0, out

    # Read back r1 and r2 explicitly so failures localize before RFALU.
    out = await rf_read(dut, 1)
    assert out["data"] == 0x5 and out["z"] == 0, f"RFREAD r1 failed: {out}"

    out = await rf_read(dut, 2)
    assert out["data"] == 0x3 and out["z"] == 0, f"RFREAD r2 failed: {out}"

    # r3 = r1 + r2 = 8
    out = await rf_alu(dut, 0, FUNC0, 1, 2, 3)
    assert out["data"] == 0x8 and out["z"] == 0 and out["c"] == 0, (
        "RFALU ADD mismatch; if this returns the previous output instead of 8, "
        "the RTL being compiled likely does not include the RFALU implementation. "
        f"Observed: {out}"
    )

    out = await rf_read(dut, 3)
    assert out["data"] == 0x8 and out["z"] == 0, f"RFREAD r3 failed: {out}"

    # r0 = XNOR(r1, r2) = ~(5 ^ 3) = 9
    out = await rf_alu(dut, 1, FUNC1, 1, 2, 0)
    assert out["data"] == 0x9 and out["z"] == 0, out

    # move r0 -> ACC and verify round-trip
    out = await rf_to_acc(dut, 0)
    assert out["data"] == 0x9 and out["z"] == 0, out

    # ECC / parity nibble from ACC should be non-zero for 9 under current code
    out = await issue(dut, CLS_REV, 0, FUNC3, 0x0, 0x0)
    assert out["data"] != 0x0, out

    # Compare low 3 bits: ACC=9 => 001 vs imm=1 => EQ
    out = await issue(dut, CLS_CTRL, 1, FUNC3, 0x0, 0x1)
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    # Gray encode/decode round trip for ACC
    out = await issue(dut, CLS_PERM, 1, FUNC2, 0x0, 0x0)
    gray = out["data"]
    out = await issue(dut, CLS_PERM, 1, FUNC3, 0x0, 0x0)
    assert out["data"] == 0x9, (gray, out)

    # RAM write then reverse restore
    out = await issue(dut, CLS_MEM, 0, FUNC1, 0x2, 0xC)
    assert out["data"] == 0xC, out
    out = await issue(dut, CLS_MEM, 0, FUNC1, 0x2, 0xA)
    assert out["data"] == 0xA, out
    _ = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)
    out = await issue(dut, CLS_MEM, 0, FUNC0, 0x2, 0x0)
    assert out["data"] == 0xC, out

    # RF write then reverse restore
    out = await rf_loadi(dut, 1, 0xE)
    assert out["data"] == 0xE, out
    _ = await issue(dut, CLS_REV, 0, FUNC1, 0x0, 0x0)
    out = await rf_read(dut, 1)
    assert out["data"] == 0x5, out
