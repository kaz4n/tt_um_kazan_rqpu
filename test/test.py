import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly, Timer

P0_INSTR = 0b00
P1_OPERAND = 0b01
P2_EXECUTE = 0b10
P3_OUTPUT = 0b11

CLS_ARITH = 0b000
CLS_LOGIC = 0b001
CLS_SHIFT = 0b010
CLS_CMP   = 0b011
CLS_MEM   = 0b100
CLS_SYS   = 0b101
CLS_REV   = 0b110
CLS_CTRL  = 0b111

# System-register addresses used by the current broad-v2 RTL.
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
    # Let HDL updates settle before sampling outputs.
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
    assert int(dut.uio_out.value) == 0, f"uio_out should be 0"
    assert int(dut.uio_oe.value) == 0, f"uio_oe should be 0"

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
async def test_rqpu_broad_v2(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    # ------------------------------------------------------------------
    # Direct control/setup path
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_CTRL, 0, 0xA, 0x0, 0x6)  # ACC <= 6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, 0x4, 0x0, 0x3)  # BREG <= 3
    assert out["data"] == 0x3 and out["z"] == 0, out

    out = await issue(dut, CLS_CTRL, 0, 0x5, 0x0, 0x9)  # SHD <= 9
    assert out["data"] == 0x9 and out["z"] == 0, out

    # ------------------------------------------------------------------
    # Arithmetic family: immediate source and system-register source
    # mode=0 => immediate B, mode=1 => sys_read(A)
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_ARITH, 0, 0x0, 0x0, 0x3)  # ADD imm 3 => 6+3=9
    assert out["data"] == 0x9 and out["z"] == 0 and out["c"] == 0, out

    out = await issue(dut, CLS_ARITH, 1, 0x2, SYS_BREG, 0x0)  # SUB BREG(3) => 9-3=6
    assert out["data"] == 0x6 and out["z"] == 0 and out["c"] == 1, out

    # ------------------------------------------------------------------
    # Logic family
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_LOGIC, 0, 0x6, 0x0, 0x3)  # XNOR imm 3 => ~(6^3)=A
    assert out["data"] == 0xA and out["z"] == 0 and out["c"] == 0, out

    # ------------------------------------------------------------------
    # Shift / permute family
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_CTRL, 0, 0xA, 0x0, 0x9)  # ACC <= 9
    assert out["data"] == 0x9, out

    out = await issue(dut, CLS_SHIFT, 0, 0x0, 0x0, 0x0)  # SHL: 9 -> 2, carrylike=1
    assert out["data"] == 0x2 and out["c"] == 1, out

    out = await issue(dut, CLS_SHIFT, 0, 0x5, 0x0, 0x0)  # BITREV: 2 -> 4
    assert out["data"] == 0x4 and out["z"] == 0, out

    # ------------------------------------------------------------------
    # Compare family
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_CTRL, 0, 0xA, 0x0, 0x5)  # ACC <= 5
    assert out["data"] == 0x5, out

    out = await issue(dut, CLS_CMP, 0, 0x0, 0x0, 0x5)  # CMP imm 5 => {0,0,1,0}=2
    assert out["data"] == 0x2 and out["z"] == 1 and out["c"] == 0, out

    # ------------------------------------------------------------------
    # Memory family: RAM read/write/loadacc
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_MEM, 0, 0x1, 0x2, 0xC)  # RAM[2] <= C
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, 0x0, 0x2, 0x0)  # READ RAM[2]
    assert out["data"] == 0xC and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 0, 0x2, 0x2, 0x0)  # LOADACC RAM[2]
    assert out["data"] == 0xC and out["z"] == 0, out

    # ------------------------------------------------------------------
    # SYS family + MEM system-space access
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_SYS, 0, 0x3, SYS_TMP0, 0x7)  # LOADIMM TMP0 <= 7
    assert out["data"] == 0x7 and out["z"] == 0, out

    out = await issue(dut, CLS_SYS, 0, 0x0, SYS_TMP0, SYS_TMP1)  # MOV TMP1 <= TMP0
    assert out["data"] == 0x7 and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, 0x0, SYS_TMP1, 0x0)  # READ system TMP1
    assert out["data"] == 0x7 and out["z"] == 0, out

    # ------------------------------------------------------------------
    # REV family: ACC <-> BREG and REVERSE
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_CTRL, 0, 0xA, 0x0, 0x4)  # ACC <= 4
    assert out["data"] == 0x4, out

    out = await issue(dut, CLS_CTRL, 0, 0x4, 0x0, 0xA)  # BREG <= A
    assert out["data"] == 0xA, out

    out = await issue(dut, CLS_REV, 0, 0x3, 0x0, 0x0)  # ACC <-> BREG
    assert out["data"] == 0xA and out["z"] == 0, out

    out = await issue(dut, CLS_MEM, 1, 0x0, SYS_ACC, 0x0)  # READ ACC => A
    assert out["data"] == 0xA, out

    _ = await issue(dut, CLS_REV, 0, 0x1, 0x0, 0x0)  # REVERSE
    out = await issue(dut, CLS_MEM, 1, 0x0, SYS_ACC, 0x0)  # READ ACC => 4
    assert out["data"] == 0x4, out

    # ------------------------------------------------------------------
    # SHIFT default path = parity nibble output from ACC
    # Set ACC to 5 => parity bit 0 (two ones)
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_CTRL, 0, 0xA, 0x0, 0x5)
    assert out["data"] == 0x5, out
    out = await issue(dut, CLS_SHIFT, 0, 0x7, 0x0, 0x0)  # default/PARITY
    assert out["data"] == 0x0 and out["z"] == 1, out

    # ------------------------------------------------------------------
    # Tracked RAM write undo via REVERSE
    # ------------------------------------------------------------------
    out = await issue(dut, CLS_MEM, 0, 0x1, 0x1, 0xA)  # RAM[1] <= A
    assert out["data"] == 0xA, out

    out = await issue(dut, CLS_MEM, 0, 0x1, 0x1, 0x3)  # RAM[1] <= 3
    assert out["data"] == 0x3, out

    _ = await issue(dut, CLS_REV, 0, 0x1, 0x0, 0x0)    # REVERSE last write
    out = await issue(dut, CLS_MEM, 0, 0x0, 0x1, 0x0)  # READ RAM[1] => A restored
    assert out["data"] == 0xA, out
