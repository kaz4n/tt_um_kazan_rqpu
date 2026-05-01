import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

# Compact / RF-oriented class map
C_CLS_ALU   = 0b000
C_CLS_PERM  = 0b001
C_CLS_MEM   = 0b010
C_CLS_REV   = 0b011
C_CLS_CTRL  = 0b100
C_CLS_RFALU = 0b101
C_CLS_RFIO  = 0b110

# Broad-v2 class map
B_CLS_ARITH = 0b000
B_CLS_LOGIC = 0b001
B_CLS_SHIFT = 0b010
B_CLS_CMP   = 0b011
B_CLS_MEM   = 0b100
B_CLS_SYS   = 0b101
B_CLS_REV   = 0b110
B_CLS_CTRL  = 0b111

# Broad-v2 system addresses
SYS_ACC   = 0x0
SYS_BREG  = 0x1
SYS_SHD   = 0x2
SYS_FLAGS = 0x3
SYS_MAR   = 0x8
SYS_MDR   = 0x9
SYS_OUT   = 0xA
SYS_PHS   = 0xB
SYS_TMP0  = 0xC
SYS_TMP1  = 0xD
SYS_EXT0  = 0xE
SYS_EXT1  = 0xF


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


async def issue(dut, cls: int, mode: int, func: int, a: int = 0, b: int = 0, label: str = "") -> dict:
    await sync_p0_writable(dut)

    dut.ui_in.value = pack_instr(cls, mode, func)
    s1 = await advance_and_check(dut, P1_OPERAND)
    if label:
        dut._log.info(f"{label} P1 ui_in={int(dut.ui_in.value):08b} uo_out={s1['raw']:08b}")

    await Timer(1, unit="ns")
    dut.ui_in.value = pack_oper(a, b)
    s2 = await advance_and_check(dut, P2_EXECUTE)
    if label:
        dut._log.info(f"{label} P2 ui_in={int(dut.ui_in.value):08b} uo_out={s2['raw']:08b}")

    s3 = await advance_and_check(dut, P3_OUTPUT)
    if label:
        dut._log.info(f"{label} P3 ui_in={int(dut.ui_in.value):08b} uo_out={s3['raw']:08b}")

    s0 = await advance_and_check(dut, P0_INSTR)
    if label:
        dut._log.info(f"{label} P0 ui_in={int(dut.ui_in.value):08b} uo_out={s0['raw']:08b}")

    await Timer(1, unit="ns")
    dut.ui_in.value = 0
    return s3


async def detect_isa(dut) -> str:
    # Probe compact/RF design first: CLS_CTRL=100, func0 => SETACC immediate
    out = await issue(dut, C_CLS_CTRL, 0, 0x0, 0x0, 0x6, label="probe-compact")
    if out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0:
        return "compact"

    # Re-reset before probing broad-v2
    await reset_dut(dut)
    out = await issue(dut, B_CLS_CTRL, 0, 0xA, 0x0, 0x6, label="probe-broad")
    if out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0:
        return "broad"

    raise AssertionError(
        "Could not identify ISA flavor. "
        f"compact probe returned {out}. The compiled RTL may not match the checked-in test assumptions."
    )


async def run_compact_tests(dut) -> None:
    # Working regs
    out = await issue(dut, C_CLS_CTRL, 0, 0x0, 0x0, 0x6, label="compact-setacc")
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out
    out = await issue(dut, C_CLS_CTRL, 0, 0x1, 0x0, 0x9, label="compact-setshd")
    assert out["data"] == 0x9 and out["z"] == 0, out
    out = await issue(dut, C_CLS_CTRL, 0, 0x2, 0x0, 0xA, label="compact-setbreg")
    assert out["data"] == 0xA and out["z"] == 0, out

    # RFLOADI r1=5, r2=3
    out = await issue(dut, C_CLS_RFIO, 0, 0x0, 0x1, 0x5, label="compact-rfloadi-r1")
    assert out["data"] == 0x5 and out["z"] == 0, out
    out = await issue(dut, C_CLS_RFIO, 0, 0x0, 0x2, 0x3, label="compact-rfloadi-r2")
    assert out["data"] == 0x3 and out["z"] == 0, out

    # r3 = r1 + r2 = 8 (A=rs|rt = 01_10, B=rd|00 = 11_00)
    out = await issue(dut, C_CLS_RFALU, 0, 0x0, 0b0110, 0b1100, label="compact-rfalu-add")
    assert out["data"] == 0x8 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, C_CLS_RFIO, 0, 0x1, 0x3, 0x0, label="compact-rfread-r3")
    assert out["data"] == 0x8 and out["z"] == 0, out

    # ALU/perm/ctrl/rev/mem smoke tests
    out = await issue(dut, C_CLS_ALU, 1, 0x1, 0x0, 0x5, label="compact-xnor")
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, C_CLS_PERM, 1, 0x2, 0x0, 0x0, label="compact-grayenc")
    # grayenc(0xC)=0xA
    assert out["data"] == 0xA and out["z"] == 0, out

    out = await issue(dut, C_CLS_CTRL, 1, 0x3, 0x0, 0xA, label="compact-cmp3")
    # 3-bit compare low bits of ACC=0xA(010) vs 0xA(010) => {0,0,1,0}=2
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    out = await issue(dut, C_CLS_REV, 0, 0x3, 0x0, 0x0, label="compact-parityecc")
    assert out["phase"] == P3_OUTPUT, out

    out = await issue(dut, C_CLS_MEM, 0, 0x1, 0x2, 0xA, label="compact-ram-write")
    assert out["data"] == 0xA, out
    out = await issue(dut, C_CLS_MEM, 0, 0x0, 0x2, 0x0, label="compact-ram-read")
    assert out["data"] == 0xA, out


async def run_broad_tests(dut) -> None:
    out = await issue(dut, B_CLS_CTRL, 0, 0xA, 0x0, 0x6, label="broad-setacc")
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, B_CLS_CTRL, 0, 0x4, 0x0, 0x3, label="broad-setbreg")
    assert out["data"] == 0x3 and out["z"] == 0, out

    out = await issue(dut, B_CLS_CTRL, 0, 0x5, 0x0, 0x9, label="broad-setshd")
    assert out["data"] == 0x9 and out["z"] == 0, out

    out = await issue(dut, B_CLS_ARITH, 0, 0x0, 0x0, 0x3, label="broad-add")
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, B_CLS_ARITH, 1, 0x2, SYS_BREG, 0x0, label="broad-sub-breg")
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 1, out

    out = await issue(dut, B_CLS_LOGIC, 0, 0x6, 0x0, 0x3, label="broad-xnor")
    assert out["data"] == 0xA and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, B_CLS_CTRL, 0, 0xA, 0x0, 0x9, label="broad-setacc-9")
    assert out["data"] == 0x9, out
    out = await issue(dut, B_CLS_SHIFT, 0, 0x0, 0x0, 0x0, label="broad-shl")
    assert out["data"] == 0x2 and out["c"] == 1, out
    out = await issue(dut, B_CLS_SHIFT, 0, 0x5, 0x0, 0x0, label="broad-bitrev")
    assert out["data"] == 0x4 and out["z"] == 0, out

    out = await issue(dut, B_CLS_CTRL, 0, 0xA, 0x0, 0x5, label="broad-setacc-5")
    assert out["data"] == 0x5, out
    out = await issue(dut, B_CLS_CMP, 0, 0x0, 0x0, 0x5, label="broad-cmp")
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    out = await issue(dut, B_CLS_MEM, 0, 0x1, 0x2, 0xC, label="broad-ram-write")
    assert out["data"] == 0xC and out["z"] == 0, out
    out = await issue(dut, B_CLS_MEM, 0, 0x0, 0x2, 0x0, label="broad-ram-read")
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, B_CLS_SYS, 0, 0x3, SYS_TMP0, 0x7, label="broad-sys-loadimm")
    assert out["data"] == 0x7 and out["z"] == 0, out
    out = await issue(dut, B_CLS_SYS, 0, 0x0, SYS_TMP0, SYS_TMP1, label="broad-sys-mov")
    assert out["data"] == 0x7 and out["z"] == 0, out
    out = await issue(dut, B_CLS_MEM, 1, 0x0, SYS_TMP1, 0x0, label="broad-sys-read")
    assert out["data"] == 0x7 and out["z"] == 0, out

    out = await issue(dut, B_CLS_CTRL, 0, 0xA, 0x0, 0x4, label="broad-setacc-4")
    assert out["data"] == 0x4, out
    out = await issue(dut, B_CLS_CTRL, 0, 0x4, 0x0, 0xA, label="broad-setbreg-A")
    assert out["data"] == 0xA, out
    out = await issue(dut, B_CLS_REV, 0, 0x3, 0x0, 0x0, label="broad-acc-breg-swap")
    assert out["data"] == 0xA and out["z"] == 0, out
    _ = await issue(dut, B_CLS_REV, 0, 0x1, 0x0, 0x0, label="broad-reverse")
    out = await issue(dut, B_CLS_MEM, 1, 0x0, SYS_ACC, 0x0, label="broad-read-acc-after-reverse")
    assert out["data"] == 0x4, out

    out = await issue(dut, B_CLS_SHIFT, 0, 0x7, 0x0, 0x0, label="broad-parity")
    assert out["phase"] == P3_OUTPUT, out


@cocotb.test()
async def test_rqpu(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    flavor = await detect_isa(dut)
    dut._log.info(f"Detected ISA flavor: {flavor}")

    # Reset once more so branch-specific tests start cleanly.
    await reset_dut(dut)

    if flavor == "compact":
        await run_compact_tests(dut)
    else:
        await run_broad_tests(dut)
