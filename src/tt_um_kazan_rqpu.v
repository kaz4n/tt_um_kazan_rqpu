/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RQPU v2: Reversible-computing-inspired 4-bit processor for Tiny Tapeout.
 *
 * Public bus contract:
 *   uo_out[7:6] = phase[1:0]
 *   uo_out[5:2] = result/data nibble
 *   uo_out[1]   = Z flag
 *   uo_out[0]   = C flag
 *
 * Input protocol:
 *   Phase 00: ui_in[7:5] = class, ui_in[4] = mode, ui_in[3:0] = func
 *   Phase 01: ui_in[7:4] = A,     ui_in[3:0] = B
 *   Phase 10: internal execute / commit preparation
 *   Phase 11: outputs valid
 *
 * Architectural highlights:
 * - 4-phase deterministic controller protocol
 * - 16x4 scratchpad RAM implemented with registers
 * - Memory-mapped internal registers
 * - Undo/redo style REVERSE operation using current/previous architectural
 *   state swapping
 * - uio_* intentionally unused
 */
`default_nettype none

module tt_um_kazan_rqpu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path (unused)
    output wire [7:0] uio_out,  // IOs: Output path (unused)
    output wire [7:0] uio_oe,   // IOs: Enable path (unused)
    input  wire       ena,      // Design enable
    input  wire       clk,      // Clock
    input  wire       rst_n     // Active-low reset
);
    // ---------------------------------------------------------------------
    // Phase encoding
    // ---------------------------------------------------------------------
    localparam [1:0] P0_INSTR   = 2'b00;
    localparam [1:0] P1_OPERAND = 2'b01;
    localparam [1:0] P2_EXECUTE = 2'b10;
    localparam [1:0] P3_OUTPUT  = 2'b11;

    // ---------------------------------------------------------------------
    // Instruction families
    // ---------------------------------------------------------------------
    localparam [2:0] CLS_ARITH = 3'b000;
    localparam [2:0] CLS_LOGIC = 3'b001;
    localparam [2:0] CLS_SHIFT = 3'b010;
    localparam [2:0] CLS_CMP   = 3'b011;
    localparam [2:0] CLS_MEM   = 3'b100;
    localparam [2:0] CLS_SYS   = 3'b101;
    localparam [2:0] CLS_REV   = 3'b110;
    localparam [2:0] CLS_CTRL  = 3'b111;

    // ---------------------------------------------------------------------
    // System-register map (mode = 1)
    // ---------------------------------------------------------------------
    localparam [3:0] SYS_ACC   = 4'h0;
    localparam [3:0] SYS_BREG  = 4'h1;
    localparam [3:0] SYS_SHD   = 4'h2;
    localparam [3:0] SYS_FLAGS = 4'h3; // {UNDO_VALID, N, C, Z}
    localparam [3:0] SYS_PACC  = 4'h4;
    localparam [3:0] SYS_PB    = 4'h5;
    localparam [3:0] SYS_PSHD  = 4'h6;
    localparam [3:0] SYS_PFLG  = 4'h7;
    localparam [3:0] SYS_MAR   = 4'h8;
    localparam [3:0] SYS_MDR   = 4'h9;
    localparam [3:0] SYS_OUT   = 4'hA;
    localparam [3:0] SYS_PHS   = 4'hB;
    localparam [3:0] SYS_TMP0  = 4'hC;
    localparam [3:0] SYS_TMP1  = 4'hD;
    localparam [3:0] SYS_EXT0  = 4'hE;
    localparam [3:0] SYS_EXT1  = 4'hF;

    // ---------------------------------------------------------------------
    // Architectural state
    // ---------------------------------------------------------------------
    reg [1:0] phase_q;

    reg [2:0] ir_class_q;
    reg       ir_mode_q;
    reg [3:0] ir_func_q;
    reg [3:0] arg_a_q;
    reg [3:0] arg_b_q;

    reg [3:0] acc_q;
    reg [3:0] breg_q;
    reg [3:0] shd_q;
    reg [2:0] flags_q;      // {N, C, Z}
    reg       undo_valid_q; // mirrored into FLAGS[3]
    reg [3:0] mar_q;
    reg [3:0] mdr_q;
    reg [3:0] out_q;
    reg [3:0] tmp0_q;
    reg [3:0] tmp1_q;
    reg [3:0] ext0_q;
    reg [3:0] ext1_q;
    reg [3:0] ram_q [0:15];

    reg [3:0] prev_acc_q;
    reg [3:0] prev_breg_q;
    reg [3:0] prev_shd_q;
    reg [2:0] prev_flags_q;
    reg [3:0] prev_mar_q;
    reg [3:0] prev_mdr_q;
    reg [3:0] prev_out_q;
    reg [3:0] prev_tmp0_q;
    reg [3:0] prev_tmp1_q;
    reg [3:0] prev_ext0_q;
    reg [3:0] prev_ext1_q;
    reg [3:0] prev_ram_q [0:15];

    integer i;

    // ---------------------------------------------------------------------
    // Utility functions
    // ---------------------------------------------------------------------
    function [2:0] flags_from_result;
        input [3:0] value;
        input       carry_like;
        begin
            flags_from_result = {value[3], carry_like, (value == 4'h0)};
        end
    endfunction

    function [3:0] popcount4;
        input [3:0] value;
        begin
            case (value)
                4'b0000: popcount4 = 4'd0;
                4'b0001,
                4'b0010,
                4'b0100,
                4'b1000: popcount4 = 4'd1;
                4'b0011,
                4'b0101,
                4'b0110,
                4'b1001,
                4'b1010,
                4'b1100: popcount4 = 4'd2;
                4'b0111,
                4'b1011,
                4'b1101,
                4'b1110: popcount4 = 4'd3;
                default: popcount4 = 4'd4;
            endcase
        end
    endfunction

    function [3:0] bitrev4;
        input [3:0] value;
        begin
            bitrev4 = {value[0], value[1], value[2], value[3]};
        end
    endfunction

    function [3:0] swap2_4;
        input [3:0] value;
        begin
            swap2_4 = {value[1:0], value[3:2]};
        end
    endfunction

    function        is_gp_sys_addr;
        input [3:0] addr;
        begin
            case (addr)
                SYS_ACC,
                SYS_BREG,
                SYS_SHD,
                SYS_MAR,
                SYS_MDR,
                SYS_TMP0,
                SYS_TMP1,
                SYS_EXT0,
                SYS_EXT1: is_gp_sys_addr = 1'b1;
                default:   is_gp_sys_addr = 1'b0;
            endcase
        end
    endfunction

    function        is_writable_sys_addr;
        input [3:0] addr;
        begin
            case (addr)
                SYS_ACC,
                SYS_BREG,
                SYS_SHD,
                SYS_FLAGS,
                SYS_MAR,
                SYS_MDR,
                SYS_OUT,
                SYS_TMP0,
                SYS_TMP1,
                SYS_EXT0,
                SYS_EXT1: is_writable_sys_addr = 1'b1;
                default:   is_writable_sys_addr = 1'b0;
            endcase
        end
    endfunction

    function [3:0] sys_read;
        input [3:0] addr;
        begin
            case (addr)
                SYS_ACC:   sys_read = acc_q;
                SYS_BREG:  sys_read = breg_q;
                SYS_SHD:   sys_read = shd_q;
                SYS_FLAGS: sys_read = {undo_valid_q, flags_q};
                SYS_PACC:  sys_read = prev_acc_q;
                SYS_PB:    sys_read = prev_breg_q;
                SYS_PSHD:  sys_read = prev_shd_q;
                SYS_PFLG:  sys_read = {1'b0, prev_flags_q};
                SYS_MAR:   sys_read = mar_q;
                SYS_MDR:   sys_read = mdr_q;
                SYS_OUT:   sys_read = out_q;
                SYS_PHS:   sys_read = {phase_q, (phase_q != P0_INSTR), (phase_q == P3_OUTPUT)};
                SYS_TMP0:  sys_read = tmp0_q;
                SYS_TMP1:  sys_read = tmp1_q;
                SYS_EXT0:  sys_read = ext0_q;
                SYS_EXT1:  sys_read = ext1_q;
                default:   sys_read = 4'h0;
            endcase
        end
    endfunction

    function [3:0] mapped_read;
        input       mode;
        input [3:0] addr;
        begin
            if (mode) begin
                mapped_read = sys_read(addr);
            end else begin
                mapped_read = ram_q[addr];
            end
        end
    endfunction

    wire [3:0] src_val_w    = ir_mode_q ? sys_read(arg_a_q) : arg_b_q;
    wire [3:0] mapped_val_w = mapped_read(ir_mode_q, arg_a_q);
    wire [3:0] sys_a_val_w  = sys_read(arg_a_q);
    wire [3:0] sys_b_val_w  = sys_read(arg_b_q);

    // ---------------------------------------------------------------------
    // Snapshot helpers for REVERSE
    // ---------------------------------------------------------------------
    task snapshot_current_to_prev;
        integer idx;
        begin
            prev_acc_q   <= acc_q;
            prev_breg_q  <= breg_q;
            prev_shd_q   <= shd_q;
            prev_flags_q <= flags_q;
            prev_mar_q   <= mar_q;
            prev_mdr_q   <= mdr_q;
            prev_out_q   <= out_q;
            prev_tmp0_q  <= tmp0_q;
            prev_tmp1_q  <= tmp1_q;
            prev_ext0_q  <= ext0_q;
            prev_ext1_q  <= ext1_q;
            for (idx = 0; idx < 16; idx = idx + 1) begin
                prev_ram_q[idx] <= ram_q[idx];
            end
        end
    endtask

    task clear_prev_snapshot;
        integer idx;
        begin
            prev_acc_q   <= 4'h0;
            prev_breg_q  <= 4'h0;
            prev_shd_q   <= 4'h0;
            prev_flags_q <= 3'b000;
            prev_mar_q   <= 4'h0;
            prev_mdr_q   <= 4'h0;
            prev_out_q   <= 4'h0;
            prev_tmp0_q  <= 4'h0;
            prev_tmp1_q  <= 4'h0;
            prev_ext0_q  <= 4'h0;
            prev_ext1_q  <= 4'h0;
            for (idx = 0; idx < 16; idx = idx + 1) begin
                prev_ram_q[idx] <= 4'h0;
            end
        end
    endtask

    task swap_current_and_prev;
        integer idx;
        begin
            acc_q        <= prev_acc_q;
            breg_q       <= prev_breg_q;
            shd_q        <= prev_shd_q;
            flags_q      <= prev_flags_q;
            mar_q        <= prev_mar_q;
            mdr_q        <= prev_mdr_q;
            out_q        <= prev_out_q;
            tmp0_q       <= prev_tmp0_q;
            tmp1_q       <= prev_tmp1_q;
            ext0_q       <= prev_ext0_q;
            ext1_q       <= prev_ext1_q;

            prev_acc_q   <= acc_q;
            prev_breg_q  <= breg_q;
            prev_shd_q   <= shd_q;
            prev_flags_q <= flags_q;
            prev_mar_q   <= mar_q;
            prev_mdr_q   <= mdr_q;
            prev_out_q   <= out_q;
            prev_tmp0_q  <= tmp0_q;
            prev_tmp1_q  <= tmp1_q;
            prev_ext0_q  <= ext0_q;
            prev_ext1_q  <= ext1_q;

            for (idx = 0; idx < 16; idx = idx + 1) begin
                ram_q[idx]      <= prev_ram_q[idx];
                prev_ram_q[idx] <= ram_q[idx];
            end
        end
    endtask

    task clear_current_state;
        integer idx;
        begin
            acc_q   <= 4'h0;
            breg_q  <= 4'h0;
            shd_q   <= 4'h0;
            flags_q <= 3'b000;
            mar_q   <= 4'h0;
            mdr_q   <= 4'h0;
            out_q   <= 4'h0;
            tmp0_q  <= 4'h0;
            tmp1_q  <= 4'h0;
            ext0_q  <= 4'h0;
            ext1_q  <= 4'h0;
            for (idx = 0; idx < 16; idx = idx + 1) begin
                ram_q[idx] <= 4'h0;
            end
        end
    endtask

    task write_current_sys_reg;
        input [3:0] addr;
        input [3:0] data;
        begin
            case (addr)
                SYS_ACC:   acc_q   <= data;
                SYS_BREG:  breg_q  <= data;
                SYS_SHD:   shd_q   <= data;
                SYS_FLAGS: flags_q <= data[2:0];
                SYS_MAR:   mar_q   <= data;
                SYS_MDR:   mdr_q   <= data;
                SYS_OUT:   out_q   <= data;
                SYS_TMP0:  tmp0_q  <= data;
                SYS_TMP1:  tmp1_q  <= data;
                SYS_EXT0:  ext0_q  <= data;
                SYS_EXT1:  ext1_q  <= data;
                default: begin end
            endcase
        end
    endtask

    // ---------------------------------------------------------------------
    // Main phase-driven controller
    // Commit architectural state in phase 10 so that outputs are valid during
    // phase 11.
    // ---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        reg [4:0] ext5;
        reg [3:0] tmp_data;
        reg [3:0] tmp_data_b;
        if (!rst_n) begin
            phase_q      <= P0_INSTR;
            ir_class_q   <= 3'b000;
            ir_mode_q    <= 1'b0;
            ir_func_q    <= 4'h0;
            arg_a_q      <= 4'h0;
            arg_b_q      <= 4'h0;

            acc_q        <= 4'h0;
            breg_q       <= 4'h0;
            shd_q        <= 4'h0;
            flags_q      <= 3'b000;
            undo_valid_q <= 1'b0;
            mar_q        <= 4'h0;
            mdr_q        <= 4'h0;
            out_q        <= 4'h0;
            tmp0_q       <= 4'h0;
            tmp1_q       <= 4'h0;
            ext0_q       <= 4'h0;
            ext1_q       <= 4'h0;

            prev_acc_q   <= 4'h0;
            prev_breg_q  <= 4'h0;
            prev_shd_q   <= 4'h0;
            prev_flags_q <= 3'b000;
            prev_mar_q   <= 4'h0;
            prev_mdr_q   <= 4'h0;
            prev_out_q   <= 4'h0;
            prev_tmp0_q  <= 4'h0;
            prev_tmp1_q  <= 4'h0;
            prev_ext0_q  <= 4'h0;
            prev_ext1_q  <= 4'h0;
            for (i = 0; i < 16; i = i + 1) begin
                ram_q[i]      <= 4'h0;
                prev_ram_q[i] <= 4'h0;
            end
        end else if (ena) begin
            case (phase_q)
                P0_INSTR: begin
                    ir_class_q <= ui_in[7:5];
                    ir_mode_q  <= ui_in[4];
                    ir_func_q  <= ui_in[3:0];
                    phase_q    <= P1_OPERAND;
                end

                P1_OPERAND: begin
                    arg_a_q <= ui_in[7:4];
                    arg_b_q <= ui_in[3:0];
                    phase_q <= P2_EXECUTE;
                end

                P2_EXECUTE: begin
                    phase_q <= P3_OUTPUT;

                    case (ir_class_q)
                        CLS_ARITH: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[2:0])
                                3'd0: begin // ADD
                                    ext5    = {1'b0, acc_q} + {1'b0, src_val_w};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= flags_from_result(ext5[3:0], ext5[4]);
                                end
                                3'd1: begin // ADC
                                    ext5    = {1'b0, acc_q} + {1'b0, src_val_w} + {4'b0000, flags_q[1]};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= flags_from_result(ext5[3:0], ext5[4]);
                                end
                                3'd2: begin // SUB
                                    ext5    = {1'b0, acc_q} - {1'b0, src_val_w};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= {ext5[3], (acc_q >= src_val_w), (ext5[3:0] == 4'h0)};
                                end
                                3'd3: begin // SBC
                                    ext5    = {1'b0, acc_q} - {1'b0, src_val_w} - {4'b0000, ~flags_q[1]};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= {ext5[3], (acc_q >= (src_val_w + {3'b000, ~flags_q[1]})), (ext5[3:0] == 4'h0)};
                                end
                                3'd4: begin // INC
                                    ext5    = {1'b0, acc_q} + 5'd1;
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= flags_from_result(ext5[3:0], ext5[4]);
                                end
                                3'd5: begin // DEC
                                    ext5    = {1'b0, acc_q} - 5'd1;
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= {ext5[3], (acc_q != 4'h0), (ext5[3:0] == 4'h0)};
                                end
                                3'd6: begin // NEG
                                    ext5    = 5'd0 - {1'b0, acc_q};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= flags_from_result(ext5[3:0], (acc_q != 4'h0));
                                end
                                default: begin // PASS
                                    acc_q   <= src_val_w;
                                    out_q   <= src_val_w;
                                    flags_q <= flags_from_result(src_val_w, 1'b0);
                                end
                            endcase
                        end

                        CLS_LOGIC: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[2:0])
                                3'd0: begin // AND
                                    tmp_data = acc_q & src_val_w;
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd1: begin // OR
                                    tmp_data = acc_q | src_val_w;
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd2: begin // XOR
                                    tmp_data = acc_q ^ src_val_w;
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd3: begin // NAND
                                    tmp_data = ~(acc_q & src_val_w);
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd4: begin // NOT
                                    tmp_data = ~acc_q;
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd5: begin // BIC
                                    tmp_data = acc_q & ~src_val_w;
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd6: begin // XNOR
                                    tmp_data = ~(acc_q ^ src_val_w);
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                default: begin // TEST (flags/output only)
                                    tmp_data = acc_q & src_val_w;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                            endcase
                        end

                        CLS_SHIFT: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[2:0])
                                3'd0: begin // SHL
                                    tmp_data = {acc_q[2:0], 1'b0};
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, acc_q[3]);
                                end
                                3'd1: begin // SHR
                                    tmp_data = {1'b0, acc_q[3:1]};
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, acc_q[0]);
                                end
                                3'd2: begin // ROL
                                    tmp_data = {acc_q[2:0], acc_q[3]};
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, acc_q[3]);
                                end
                                3'd3: begin // ROR
                                    tmp_data = {acc_q[0], acc_q[3:1]};
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, acc_q[0]);
                                end
                                3'd4: begin // SWAP2
                                    tmp_data = swap2_4(acc_q);
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd5: begin // BITREV
                                    tmp_data = bitrev4(acc_q);
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd6: begin // ASR
                                    tmp_data = {acc_q[3], acc_q[3:1]};
                                    acc_q   <= tmp_data;
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, acc_q[0]);
                                end
                                default: begin // PARITY
                                    tmp_data = {3'b000, ^acc_q};
                                    out_q   <= tmp_data;
                                    flags_q <= flags_from_result(tmp_data, ^acc_q);
                                end
                            endcase
                        end

                        CLS_CMP: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[2:0])
                                3'd0: begin // CMP flags-only
                                    tmp_data = {1'b0, (acc_q > src_val_w), (acc_q == src_val_w), (acc_q < src_val_w)};
                                    out_q    <= tmp_data;
                                    flags_q  <= {(acc_q < src_val_w), (acc_q > src_val_w), (acc_q == src_val_w)};
                                end
                                3'd1: begin // EQ -> 0/1 in ACC
                                    tmp_data = {3'b000, (acc_q == src_val_w)};
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd2: begin // GT -> 0/1 in ACC
                                    tmp_data = {3'b000, (acc_q > src_val_w)};
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd3: begin // LT -> 0/1 in ACC
                                    tmp_data = {3'b000, (acc_q < src_val_w)};
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd4: begin // MIN
                                    tmp_data = (acc_q <= src_val_w) ? acc_q : src_val_w;
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd5: begin // MAX
                                    tmp_data = (acc_q >= src_val_w) ? acc_q : src_val_w;
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                3'd6: begin // POPCOUNT
                                    tmp_data = popcount4(acc_q);
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                                default: begin // ZERO? of ACC
                                    tmp_data = {3'b000, (acc_q == 4'h0)};
                                    acc_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end
                            endcase
                        end

                        CLS_MEM: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[1:0])
                                2'd0: begin // READ
                                    tmp_data = mapped_val_w;
                                    mdr_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end

                                2'd1: begin // WRITE
                                    if (ir_mode_q) begin
                                        if (is_writable_sys_addr(arg_a_q)) begin
                                            write_current_sys_reg(arg_a_q, arg_b_q);
                                        end
                                        if (arg_a_q == SYS_FLAGS) begin
                                            mdr_q   <= {1'b0, arg_b_q[2:0]};
                                            out_q   <= {1'b0, arg_b_q[2:0]};
                                            flags_q <= arg_b_q[2:0];
                                        end else begin
                                            mdr_q   <= arg_b_q;
                                            out_q   <= arg_b_q;
                                            flags_q <= flags_from_result(arg_b_q, 1'b0);
                                        end
                                    end else begin
                                        ram_q[arg_a_q] <= arg_b_q;
                                        mdr_q          <= arg_b_q;
                                        out_q          <= arg_b_q;
                                        flags_q        <= flags_from_result(arg_b_q, 1'b0);
                                    end
                                end

                                2'd2: begin // LOADACC
                                    tmp_data = mapped_val_w;
                                    acc_q    <= tmp_data;
                                    mdr_q    <= tmp_data;
                                    out_q    <= tmp_data;
                                    flags_q  <= flags_from_result(tmp_data, 1'b0);
                                end

                                default: begin // SWAPACC
                                    if (!ir_mode_q) begin
                                        tmp_data      = ram_q[arg_a_q];
                                        ram_q[arg_a_q] <= acc_q;
                                        acc_q         <= tmp_data;
                                        mdr_q         <= tmp_data;
                                        out_q         <= tmp_data;
                                        flags_q       <= flags_from_result(tmp_data, 1'b0);
                                    end else if (is_gp_sys_addr(arg_a_q)) begin
                                        tmp_data = sys_read(arg_a_q);
                                        write_current_sys_reg(arg_a_q, acc_q);
                                        acc_q    <= tmp_data;
                                        mdr_q    <= tmp_data;
                                        out_q    <= tmp_data;
                                        flags_q  <= flags_from_result(tmp_data, 1'b0);
                                    end else begin
                                        out_q   <= acc_q;
                                        flags_q <= flags_from_result(acc_q, 1'b0);
                                    end
                                end
                            endcase
                        end

                        CLS_SYS: begin
                            snapshot_current_to_prev();
                            undo_valid_q <= 1'b1;
                            case (ir_func_q[1:0])
                                2'd0: begin // MOV sys[B] <= sys[A]
                                    if (is_gp_sys_addr(arg_b_q)) begin
                                        tmp_data = sys_a_val_w;
                                        write_current_sys_reg(arg_b_q, tmp_data);
                                        out_q   <= tmp_data;
                                        flags_q <= flags_from_result(tmp_data, 1'b0);
                                    end
                                end

                                2'd1: begin // SWAP sys[A] <-> sys[B]
                                    if (is_gp_sys_addr(arg_a_q) && is_gp_sys_addr(arg_b_q)) begin
                                        tmp_data   = sys_a_val_w;
                                        tmp_data_b = sys_b_val_w;
                                        write_current_sys_reg(arg_a_q, tmp_data_b);
                                        write_current_sys_reg(arg_b_q, tmp_data);
                                        out_q   <= tmp_data_b;
                                        flags_q <= flags_from_result(tmp_data_b, 1'b0);
                                    end
                                end

                                2'd2: begin // CLEAR sys[A]
                                    if (is_gp_sys_addr(arg_a_q) || (arg_a_q == SYS_OUT)) begin
                                        write_current_sys_reg(arg_a_q, 4'h0);
                                        out_q   <= 4'h0;
                                        flags_q <= flags_from_result(4'h0, 1'b0);
                                    end
                                end

                                default: begin // LOADIMM sys[A] <= B
                                    if (is_gp_sys_addr(arg_a_q) || (arg_a_q == SYS_OUT)) begin
                                        write_current_sys_reg(arg_a_q, arg_b_q);
                                        out_q   <= arg_b_q;
                                        flags_q <= flags_from_result(arg_b_q, 1'b0);
                                    end
                                end
                            endcase
                        end

                        CLS_REV: begin
                            case (ir_func_q)
                                4'h0: begin // SAVE checkpoint
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    out_q        <= acc_q;
                                end

                                4'h1: begin // REVERSE (undo/redo swap)
                                    if (undo_valid_q) begin
                                        swap_current_and_prev();
                                    end
                                end

                                4'h2: begin // SWAP ACC <-> SHD
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    acc_q        <= shd_q;
                                    shd_q        <= acc_q;
                                    out_q        <= shd_q;
                                    flags_q      <= flags_from_result(shd_q, 1'b0);
                                end

                                4'h3: begin // SWAP ACC <-> BREG
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    acc_q        <= breg_q;
                                    breg_q       <= acc_q;
                                    out_q        <= breg_q;
                                    flags_q      <= flags_from_result(breg_q, 1'b0);
                                end

                                default: begin // CLEAR undo state
                                    clear_prev_snapshot();
                                    undo_valid_q <= 1'b0;
                                    out_q        <= acc_q;
                                end
                            endcase
                        end

                        default: begin // CLS_CTRL
                            case (ir_func_q)
                                4'h0: begin // NOP
                                    out_q <= out_q;
                                end

                                4'h1: begin // CLR_ACC
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    acc_q        <= 4'h0;
                                    out_q        <= 4'h0;
                                    flags_q      <= flags_from_result(4'h0, 1'b0);
                                end

                                4'h2: begin // CLR_ALL current state
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    clear_current_state();
                                end

                                4'h3: begin // MAR <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    mar_q        <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h4: begin // BREG <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    breg_q       <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h5: begin // SHD <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    shd_q        <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h6: begin // TMP0 <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    tmp0_q       <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h7: begin // TMP1 <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    tmp1_q       <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h8: begin // EXT0 <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    ext0_q       <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'h9: begin // EXT1 <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    ext1_q       <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'hA: begin // ACC <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    acc_q        <= arg_b_q;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'hB: begin // OUT <= B
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    out_q        <= arg_b_q;
                                    flags_q      <= flags_from_result(arg_b_q, 1'b0);
                                end

                                4'hC: begin // FLAGS <= B[2:0]
                                    snapshot_current_to_prev();
                                    undo_valid_q <= 1'b1;
                                    flags_q      <= arg_b_q[2:0];
                                    out_q        <= {1'b0, arg_b_q[2:0]};
                                end

                                default: begin // reserved -> NOP-like
                                    out_q <= out_q;
                                end
                            endcase
                        end
                    endcase
                end

                default: begin // P3_OUTPUT
                    phase_q <= P0_INSTR;
                end
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Public bus mapping
    // ---------------------------------------------------------------------
    assign uo_out[7:6] = phase_q;
    assign uo_out[5:2] = out_q;
    assign uo_out[1]   = flags_q[0];
    assign uo_out[0]   = flags_q[1];

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    wire _unused = &{1'b0, uio_in};

endmodule

`default_nettype wire
