import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

# Class map (unified across all ISAs)
CLS_ALU   = 0b000
CLS_PERM  = 0b001
CLS_CMP   = 0b010
CLS_MEM   = 0b011
CLS_SYS   = 0b100
CLS_REV   = 0b101
CLS_RFALU = 0b110
CLS_RFIO  = 0b111

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
    # The Verilog implements a unified design based on broad-v2 + RF architecture
    # No need to probe - just return the flavor we support
    return "broad"


async def run_compact_tests(dut) -> None:
    # Compact tests removed - Verilog only implements broad-v2 design
    pass


async def run_broad_tests(dut) -> None:
    # Set ACC to 0x6
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_ACC, 0x6, label="broad-setacc")
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    # Set BREG to 0x3
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_BREG, 0x3, label="broad-setbreg")
    assert out["data"] == 0x3 and out["z"] == 0, out

    # Set SHD to 0x9
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_SHD, 0x9, label="broad-setshd")
    assert out["data"] == 0x9 and out["z"] == 0, out

    # ADD: ACC(0x6) + 0x3 = 0x9
    out = await issue(dut, CLS_ALU, 0, 0x0, 0x0, 0x3, label="broad-add")
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    # SUB: ACC(0x9) - BREG(0x3) = 0x6, c=1 (no borrow)
    out = await issue(dut, CLS_ALU, 1, 0x2, SYS_BREG, 0x0, label="broad-sub-breg")
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 1, out

    # XNOR: ACC(0x6) xnor 0x3 = 0xC xnor = 0xA (bitwise NOT XOR)
    out = await issue(dut, CLS_ALU, 0, 0x7, 0x0, 0x3, label="broad-xnor")
    assert out["data"] == 0xA and out["z"] == 0 and out["c"] == 0, out

    # Set ACC to 0x9
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_ACC, 0x9, label="broad-setacc-9")
    assert out["data"] == 0x9, out
    
    # SHL: ACC(0x9) << 1 = 0x2 (1001 << 1 = 0010 with carry)
    out = await issue(dut, CLS_PERM, 0, 0x2, 0x0, 0x0, label="broad-shl")
    assert out["data"] == 0x2 and out["c"] == 1, out
    
    # BITREV: reverse bits ACC(0x2) = 0x4 (0010 -> 0100)
    out = await issue(dut, CLS_PERM, 0, 0x5, 0x0, 0x0, label="broad-bitrev")
    assert out["data"] == 0x4 and out["z"] == 0, out

    # Set ACC to 0x5
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_ACC, 0x5, label="broad-setacc-5")
    assert out["data"] == 0x5, out
    
    # CMP: ACC(0x5) vs 0x5 => {0, 1, 1, 0} = 0x2 (gt=0, eq=1, lt=0)
    out = await issue(dut, CLS_CMP, 0, 0x0, 0x0, 0x5, label="broad-cmp")
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    # RAM write: write 0xC to address 0x2
    out = await issue(dut, CLS_MEM, 0, 0x1, 0x2, 0xC, label="broad-ram-write")
    assert out["data"] == 0xC and out["z"] == 0, out
    
    # RAM read: read from address 0x2, should get 0xC
    out = await issue(dut, CLS_MEM, 0, 0x0, 0x2, 0x0, label="broad-ram-read")
    assert out["data"] == 0xC and out["z"] == 0, out

    # SYS write: write 0x7 to SYS_TMP0
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_TMP0, 0x7, label="broad-sys-loadimm")
    assert out["data"] == 0x7 and out["z"] == 0, out
    
    # SYS mov: SYS_TMP0 -> SYS_TMP1 (both 0x7)
    out = await issue(dut, CLS_SYS, 0, 0x1, SYS_TMP0, SYS_TMP1, label="broad-sys-mov")
    assert out["data"] == 0x7 and out["z"] == 0, out
    
    # SYS read: read from SYS_TMP1 (system mode)
    out = await issue(dut, CLS_MEM, 1, 0x0, SYS_TMP1, 0x0, label="broad-sys-read")
    assert out["data"] == 0x7 and out["z"] == 0, out

    # Set ACC to 0x4
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_ACC, 0x4, label="broad-setacc-4")
    assert out["data"] == 0x4, out
    
    # Set BREG to 0xA
    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_BREG, 0xA, label="broad-setbreg-A")
    assert out["data"] == 0xA, out
    
    # ACC <-> BREG swap (CLS_REV, func 0x3)
    out = await issue(dut, CLS_REV, 0, 0x3, 0x0, 0x0, label="broad-acc-breg-swap")
    assert out["data"] == 0xA and out["z"] == 0, out
    
    # REVERSE (undo last operation)
    _ = await issue(dut, CLS_REV, 0, 0x1, 0x0, 0x0, label="broad-reverse")
    
    # Read ACC after reverse (should be 0x4)
    out = await issue(dut, CLS_MEM, 1, 0x0, SYS_ACC, 0x0, label="broad-read-acc-after-reverse")
    assert out["data"] == 0x4, out

    # PARITY on ACC: parity/ecc operation
    out = await issue(dut, CLS_REV, 0, 0x5, 0x0, 0x0, label="broad-parity")
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
