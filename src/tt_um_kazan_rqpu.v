/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RQPU v2-rf: phase-driven 4-bit execution core for Tiny Tapeout.
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
 *   Phase 10: internal execute / commit
 *   Phase 11: outputs valid
 *
 * ISA (compact RF-oriented extension):
 *   000 ALU   : mode=0 ADD/XOR/AND/OR, mode=1 SUB/XNOR/NAND/NOR
 *   001 PERM  : mode=0 ROL/ROR/SWAP2/BITREV, mode=1 SHL/SHR/GRAYENC/GRAYDEC
 *   010 MEM   : READ/WRITE/LOADACC/SWAPACC, mode=0 RAM, mode=1 SYS(ACC/SHD/BREG/FLAGS)
 *   011 REV   : SAVE/REVERSE/ACC<->SHD/PARITY-ECC
 *   100 CTRL  : SETACC/SETSHD/SETBREG/CMP (mode=1 makes CMP a 3-bit comparator)
 *   101 RFALU : r[rd] = r[rs] op r[rt], ALU op from mode/func[1:0]
 *   110 RFIO  : RFLOADI/RFREAD/RFTOACC/ACCTORF
 *   111 RSVD  : reserved / future MUL
 */
`default_nettype none

module tt_um_kazan_rqpu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ------------------------------------------------------------------
    // Phases
    // ------------------------------------------------------------------
    localparam [1:0] P0_INSTR   = 2'b00;
    localparam [1:0] P1_OPERAND = 2'b01;
    localparam [1:0] P2_EXECUTE = 2'b10;
    localparam [1:0] P3_OUTPUT  = 2'b11;

    // ------------------------------------------------------------------
    // Classes
    // ------------------------------------------------------------------
    localparam [2:0] CLS_ALU   = 3'b000;
    localparam [2:0] CLS_PERM  = 3'b001;
    localparam [2:0] CLS_MEM   = 3'b010;
    localparam [2:0] CLS_REV   = 3'b011;
    localparam [2:0] CLS_CTRL  = 3'b100;
    localparam [2:0] CLS_RFALU = 3'b101;
    localparam [2:0] CLS_RFIO  = 3'b110;
    localparam [2:0] CLS_RSVD  = 3'b111;

    // ------------------------------------------------------------------
    // Small system-space map used by CLS_MEM, mode=1
    // ------------------------------------------------------------------
    localparam [1:0] SYS_ACC   = 2'b00;
    localparam [1:0] SYS_SHD   = 2'b01;
    localparam [1:0] SYS_BREG  = 2'b10;
    localparam [1:0] SYS_FLAGS = 2'b11; // {0, N, C, Z}

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    reg [1:0] phase_q;

    reg [2:0] ir_class_q;
    reg       ir_mode_q;
    reg [3:0] ir_func_q;
    reg [3:0] arg_a_q;
    reg [3:0] arg_b_q;

    reg [3:0] acc_q;
    reg [3:0] breg_q;
    reg [3:0] shd_q;
    reg [3:0] out_q;
    reg [2:0] flags_q; // {N, C, Z}

    reg [3:0] ram_q [0:3];
    reg [3:0] rf_q  [0:3];

    // Single-step REVERSE snapshots / undo metadata
    reg [3:0] prev_acc_q;
    reg [3:0] prev_breg_q;
    reg [3:0] prev_shd_q;
    reg [3:0] prev_out_q;
    reg [2:0] prev_flags_q;
    reg       undo_valid_q;

    reg       undo_write_valid_q;
    reg       undo_write_is_rf_q; // 0: RAM, 1: RF
    reg [1:0] undo_write_addr_q;
    reg [3:0] undo_write_olddata_q;

    integer i;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    function [2:0] flags_from_result;
        input [3:0] value;
        input       carry_like;
        begin
            flags_from_result = {value[3], carry_like, (value == 4'h0)};
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

    function [3:0] grayenc4;
        input [3:0] value;
        begin
            grayenc4 = {value[3], value[3]^value[2], value[2]^value[1], value[1]^value[0]};
        end
    endfunction

    function [3:0] graydec4;
        input [3:0] gray;
        reg [3:0] bin;
        begin
            bin[3] = gray[3];
            bin[2] = bin[3] ^ gray[2];
            bin[1] = bin[2] ^ gray[1];
            bin[0] = bin[1] ^ gray[0];
            graydec4 = bin;
        end
    endfunction

    function [3:0] parity_ecc4;
        input [3:0] d;
        reg p1, p2, p4, p0;
        begin
            // Hamming-style check bits for data positions {d3,d2,d1,d0}
            p1 = d[0] ^ d[1] ^ d[3];
            p2 = d[0] ^ d[2] ^ d[3];
            p4 = d[1] ^ d[2] ^ d[3];
            p0 = d[0] ^ d[1] ^ d[2] ^ d[3] ^ p1 ^ p2 ^ p4;
            parity_ecc4 = {p4, p2, p1, p0};
        end
    endfunction

    function [3:0] sys_read;
        input [1:0] addr;
        begin
            case (addr)
                SYS_ACC:   sys_read = acc_q;
                SYS_SHD:   sys_read = shd_q;
                SYS_BREG:  sys_read = breg_q;
                default:   sys_read = {1'b0, flags_q};
            endcase
        end
    endfunction

    function [3:0] rf_read;
        input [1:0] idx;
        begin
            case (idx)
                2'b00: rf_read = rf_q[0];
                2'b01: rf_read = rf_q[1];
                2'b10: rf_read = rf_q[2];
                default: rf_read = rf_q[3];
            endcase
        end
    endfunction

    wire [1:0] rf_rs_sel_w = arg_a_q[3:2];
    wire [1:0] rf_rt_sel_w = arg_a_q[1:0];
    wire [1:0] rf_rd_sel_w = arg_b_q[3:2];
    wire [3:0] rf_rs_data_w = rf_read(rf_rs_sel_w);
    wire [3:0] rf_rt_data_w = rf_read(rf_rt_sel_w);

    task checkpoint_no_write;
        begin
            prev_acc_q         <= acc_q;
            prev_breg_q        <= breg_q;
            prev_shd_q         <= shd_q;
            prev_out_q         <= out_q;
            prev_flags_q       <= flags_q;
            undo_write_valid_q <= 1'b0;
        end
    endtask

    task checkpoint_ram_write;
        input [1:0] addr;
        input [3:0] olddata;
        begin
            prev_acc_q          <= acc_q;
            prev_breg_q         <= breg_q;
            prev_shd_q          <= shd_q;
            prev_out_q          <= out_q;
            prev_flags_q        <= flags_q;
            undo_write_valid_q  <= 1'b1;
            undo_write_is_rf_q  <= 1'b0;
            undo_write_addr_q   <= addr;
            undo_write_olddata_q <= olddata;
        end
    endtask

    task checkpoint_rf_write;
        input [1:0] addr;
        input [3:0] olddata;
        begin
            prev_acc_q          <= acc_q;
            prev_breg_q         <= breg_q;
            prev_shd_q          <= shd_q;
            prev_out_q          <= out_q;
            prev_flags_q        <= flags_q;
            undo_write_valid_q  <= 1'b1;
            undo_write_is_rf_q  <= 1'b1;
            undo_write_addr_q   <= addr;
            undo_write_olddata_q <= olddata;
        end
    endtask

    task sys_write;
        input [1:0] addr;
        input [3:0] data;
        begin
            case (addr)
                SYS_ACC:   acc_q   <= data;
                SYS_SHD:   shd_q   <= data;
                SYS_BREG:  breg_q  <= data;
                default:   flags_q <= data[2:0];
            endcase
        end
    endtask

    task rf_write;
        input [1:0] idx;
        input [3:0] data;
        begin
            case (idx)
                2'b00: rf_q[0] <= data;
                2'b01: rf_q[1] <= data;
                2'b10: rf_q[2] <= data;
                default: rf_q[3] <= data;
            endcase
        end
    endtask

    // ------------------------------------------------------------------
    // Main controller
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        reg [4:0] ext5;
        reg [3:0] lhs;
        reg [3:0] rhs;
        reg [3:0] tmp;
        reg [3:0] ecc;
        reg       gt_v;
        reg       eq_v;
        reg       lt_v;
        if (!rst_n) begin
            phase_q <= P0_INSTR;
            ir_class_q <= 3'b000;
            ir_mode_q  <= 1'b0;
            ir_func_q  <= 4'h0;
            arg_a_q    <= 4'h0;
            arg_b_q    <= 4'h0;

            acc_q <= 4'h0;
            breg_q <= 4'h0;
            shd_q <= 4'h0;
            out_q <= 4'h0;
            flags_q <= 3'b000;

            prev_acc_q <= 4'h0;
            prev_breg_q <= 4'h0;
            prev_shd_q <= 4'h0;
            prev_out_q <= 4'h0;
            prev_flags_q <= 3'b000;
            undo_valid_q <= 1'b0;
            undo_write_valid_q <= 1'b0;
            undo_write_is_rf_q <= 1'b0;
            undo_write_addr_q <= 2'b00;
            undo_write_olddata_q <= 4'h0;

            for (i = 0; i < 4; i = i + 1) begin
                ram_q[i] <= 4'h0;
                rf_q[i]  <= 4'h0;
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
                        CLS_ALU: begin
                            checkpoint_no_write();
                            undo_valid_q <= 1'b1;
                            lhs = acc_q;
                            rhs = arg_b_q;
                            case ({ir_mode_q, ir_func_q[1:0]})
                                3'b000: begin // ADD
                                    ext5 = {1'b0, lhs} + {1'b0, rhs};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= flags_from_result(ext5[3:0], ext5[4]);
                                end
                                3'b001: begin // XOR
                                    tmp     = lhs ^ rhs;
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b010: begin // AND
                                    tmp     = lhs & rhs;
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b011: begin // OR
                                    tmp     = lhs | rhs;
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b100: begin // SUB
                                    ext5    = {1'b0, lhs} - {1'b0, rhs};
                                    acc_q   <= ext5[3:0];
                                    out_q   <= ext5[3:0];
                                    flags_q <= {ext5[3], (lhs >= rhs), (ext5[3:0] == 4'h0)};
                                end
                                3'b101: begin // XNOR
                                    tmp     = ~(lhs ^ rhs);
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b110: begin // NAND
                                    tmp     = ~(lhs & rhs);
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                default: begin // NOR
                                    tmp     = ~(lhs | rhs);
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                            endcase
                        end

                        CLS_PERM: begin
                            checkpoint_no_write();
                            undo_valid_q <= 1'b1;
                            case ({ir_mode_q, ir_func_q[1:0]})
                                3'b000: tmp = {acc_q[2:0], acc_q[3]};         // ROL
                                3'b001: tmp = {acc_q[0], acc_q[3:1]};         // ROR
                                3'b010: tmp = swap2_4(acc_q);                 // SWAP2
                                3'b011: tmp = bitrev4(acc_q);                 // BITREV
                                3'b100: tmp = {acc_q[2:0], 1'b0};             // SHL
                                3'b101: tmp = {1'b0, acc_q[3:1]};             // SHR
                                3'b110: tmp = grayenc4(acc_q);                // GRAYENC
                                default: tmp = graydec4(acc_q);               // GRAYDEC
                            endcase
                            acc_q   <= tmp;
                            out_q   <= tmp;
                            flags_q <= flags_from_result(tmp, (ir_mode_q && (ir_func_q[1:0] == 2'b00)) ? acc_q[3] : ((ir_mode_q && (ir_func_q[1:0] == 2'b01)) ? acc_q[0] : 1'b0));
                        end

                        CLS_MEM: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // READ
                                    if (!ir_mode_q) begin
                                        tmp = ram_q[arg_a_q[1:0]];
                                    end else begin
                                        tmp = sys_read(arg_a_q[1:0]);
                                    end
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                2'b01: begin // WRITE
                                    if (!ir_mode_q) begin
                                        checkpoint_ram_write(arg_a_q[1:0], ram_q[arg_a_q[1:0]]);
                                        undo_valid_q <= 1'b1;
                                        ram_q[arg_a_q[1:0]] <= arg_b_q;
                                        out_q   <= arg_b_q;
                                        flags_q <= flags_from_result(arg_b_q, 1'b0);
                                    end else begin
                                        checkpoint_no_write();
                                        undo_valid_q <= 1'b1;
                                        sys_write(arg_a_q[1:0], arg_b_q);
                                        out_q   <= arg_b_q;
                                        flags_q <= flags_from_result(arg_b_q, 1'b0);
                                    end
                                end
                                2'b10: begin // LOADACC
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    if (!ir_mode_q) begin
                                        tmp = ram_q[arg_a_q[1:0]];
                                    end else begin
                                        tmp = sys_read(arg_a_q[1:0]);
                                    end
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                default: begin // SWAPACC
                                    if (!ir_mode_q) begin
                                        checkpoint_ram_write(arg_a_q[1:0], ram_q[arg_a_q[1:0]]);
                                        undo_valid_q <= 1'b1;
                                        tmp = ram_q[arg_a_q[1:0]];
                                        ram_q[arg_a_q[1:0]] <= acc_q;
                                        acc_q <= tmp;
                                        out_q <= tmp;
                                        flags_q <= flags_from_result(tmp, 1'b0);
                                    end else begin
                                        checkpoint_no_write();
                                        undo_valid_q <= 1'b1;
                                        tmp = sys_read(arg_a_q[1:0]);
                                        sys_write(arg_a_q[1:0], acc_q);
                                        acc_q <= tmp;
                                        out_q <= tmp;
                                        flags_q <= flags_from_result(tmp, 1'b0);
                                    end
                                end
                            endcase
                        end

                        CLS_REV: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // SAVE
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    out_q        <= acc_q;
                                end
                                2'b01: begin // REVERSE
                                    if (undo_valid_q) begin
                                        acc_q   <= prev_acc_q;
                                        breg_q  <= prev_breg_q;
                                        shd_q   <= prev_shd_q;
                                        out_q   <= prev_out_q;
                                        flags_q <= prev_flags_q;
                                        if (undo_write_valid_q) begin
                                            if (undo_write_is_rf_q) begin
                                                rf_q[undo_write_addr_q] <= undo_write_olddata_q;
                                            end else begin
                                                ram_q[undo_write_addr_q] <= undo_write_olddata_q;
                                            end
                                        end
                                        undo_valid_q <= 1'b0;
                                        undo_write_valid_q <= 1'b0;
                                    end
                                end
                                2'b10: begin // ACC <-> SHD
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    acc_q <= shd_q;
                                    shd_q <= acc_q;
                                    out_q <= shd_q;
                                    flags_q <= flags_from_result(shd_q, 1'b0);
                                end
                                default: begin // PARITY / ECC nibble
                                    ecc = parity_ecc4(acc_q);
                                    out_q   <= ecc;
                                    flags_q <= flags_from_result(ecc, ecc[0]);
                                end
                            endcase
                        end

                        CLS_CTRL: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // SETACC immediate
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    acc_q   <= arg_b_q;
                                    out_q   <= arg_b_q;
                                    flags_q <= flags_from_result(arg_b_q, 1'b0);
                                end
                                2'b01: begin // SETSHD immediate
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    shd_q   <= arg_b_q;
                                    out_q   <= arg_b_q;
                                    flags_q <= flags_from_result(arg_b_q, 1'b0);
                                end
                                2'b10: begin // SETBREG immediate
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    breg_q  <= arg_b_q;
                                    out_q   <= arg_b_q;
                                    flags_q <= flags_from_result(arg_b_q, 1'b0);
                                end
                                default: begin // CMP (mode=1 => compare low 3 bits only)
                                    if (ir_mode_q) begin
                                        gt_v = (acc_q[2:0] > arg_b_q[2:0]);
                                        eq_v = (acc_q[2:0] == arg_b_q[2:0]);
                                        lt_v = (acc_q[2:0] < arg_b_q[2:0]);
                                    end else begin
                                        gt_v = (acc_q > arg_b_q);
                                        eq_v = (acc_q == arg_b_q);
                                        lt_v = (acc_q < arg_b_q);
                                    end
                                    out_q   <= {1'b0, gt_v, eq_v, lt_v};
                                    flags_q <= {lt_v, gt_v, eq_v};
                                end
                            endcase
                        end

                        CLS_RFALU: begin
                            checkpoint_rf_write(rf_rd_sel_w, rf_q[rf_rd_sel_w]);
                            undo_valid_q <= 1'b1;
                            lhs = rf_rs_data_w;
                            rhs = rf_rt_data_w;
                            case ({ir_mode_q, ir_func_q[1:0]})
                                3'b000: begin // ADD
                                    ext5 = {1'b0, lhs} + {1'b0, rhs};
                                    tmp = ext5[3:0];
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, ext5[4]);
                                end
                                3'b001: begin // XOR
                                    tmp = lhs ^ rhs;
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b010: begin // AND
                                    tmp = lhs & rhs;
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b011: begin // OR
                                    tmp = lhs | rhs;
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b100: begin // SUB
                                    ext5 = {1'b0, lhs} - {1'b0, rhs};
                                    tmp  = ext5[3:0];
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= {tmp[3], (lhs >= rhs), (tmp == 4'h0)};
                                end
                                3'b101: begin // XNOR
                                    tmp = ~(lhs ^ rhs);
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                3'b110: begin // NAND
                                    tmp = ~(lhs & rhs);
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                default: begin // NOR
                                    tmp = ~(lhs | rhs);
                                    rf_write(rf_rd_sel_w, tmp);
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                            endcase
                        end

                        CLS_RFIO: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // RFLOADI: rf[A[1:0]] <= B
                                    checkpoint_rf_write(arg_a_q[1:0], rf_q[arg_a_q[1:0]]);
                                    undo_valid_q <= 1'b1;
                                    rf_write(arg_a_q[1:0], arg_b_q);
                                    out_q   <= arg_b_q;
                                    flags_q <= flags_from_result(arg_b_q, 1'b0);
                                end
                                2'b01: begin // RFREAD: OUT <= rf[A[1:0]]
                                    tmp = rf_q[arg_a_q[1:0]];
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                2'b10: begin // RFTOACC: ACC <= rf[A[1:0]]
                                    checkpoint_no_write();
                                    undo_valid_q <= 1'b1;
                                    tmp = rf_q[arg_a_q[1:0]];
                                    acc_q   <= tmp;
                                    out_q   <= tmp;
                                    flags_q <= flags_from_result(tmp, 1'b0);
                                end
                                default: begin // ACCTORF: rf[A[1:0]] <= ACC
                                    checkpoint_rf_write(arg_a_q[1:0], rf_q[arg_a_q[1:0]]);
                                    undo_valid_q <= 1'b1;
                                    rf_write(arg_a_q[1:0], acc_q);
                                    out_q   <= acc_q;
                                    flags_q <= flags_from_result(acc_q, 1'b0);
                                end
                            endcase
                        end

                        default: begin // Reserved / future MUL
                            out_q <= out_q;
                        end
                    endcase
                end

                default: begin // P3_OUTPUT
                    phase_q <= P0_INSTR;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Public outputs
    // ------------------------------------------------------------------
    assign uo_out[7:6] = phase_q;
    assign uo_out[5:2] = out_q;
    assign uo_out[1]   = flags_q[0];
    assign uo_out[0]   = flags_q[1];

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    wire _unused = &{1'b0, uio_in};

endmodule

`default_nettype wire
