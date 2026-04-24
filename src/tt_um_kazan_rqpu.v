/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RQPU v2-fit: area-reduced reversible-computing-inspired 4-bit core for Tiny Tapeout.
 *
 * Public bus contract:
 *   uo_out[7:6] = phase[1:0]
 *   uo_out[5:2] = result/data nibble
 *   uo_out[1]   = Z flag
 *   uo_out[0]   = C flag
 *
 * Phase protocol:
 *   P0 (00): latch instruction      ui_in[7:5]=class, ui_in[4]=mode, ui_in[3:0]=func
 *   P1 (01): latch operand/data     ui_in[7:4]=A,     ui_in[3:0]=B
 *   P2 (10): execute / commit state
 *   P3 (11): outputs valid
 *
 * Area-reduced architectural state:
 *   - ACC, SHD, OUT, Z, C
 *   - 4x4 scratchpad RAM
 *   - single-step checkpoint / undo
 *   - no uio usage
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
    // ---------------------------------------------------------------------
    // Phases
    // ---------------------------------------------------------------------
    localparam [1:0] P0_INSTR   = 2'b00;
    localparam [1:0] P1_OPERAND = 2'b01;
    localparam [1:0] P2_EXECUTE = 2'b10;
    localparam [1:0] P3_OUTPUT  = 2'b11;

    // ---------------------------------------------------------------------
    // Reduced class map (v2-fit)
    // ---------------------------------------------------------------------
    localparam [2:0] CLS_ALU   = 3'b000; // ADD/XOR/AND/OR
    localparam [2:0] CLS_PERM  = 3'b001; // ROL/ROR/SWAP2/BITREV
    localparam [2:0] CLS_MEM   = 3'b010; // READ/WRITE/LOADACC/SWAPACC
    localparam [2:0] CLS_REV   = 3'b011; // SAVE/REVERSE/ACC<->SHD/PARITY
    localparam [2:0] CLS_CTRL  = 3'b100; // SETACC/SETSHD/CMP/CLEAR
    // 101/110/111 reserved as NOPs

    // ---------------------------------------------------------------------
    // 2-bit system-space map (mode = 1)
    // ---------------------------------------------------------------------
    localparam [1:0] SYS_ACC   = 2'b00;
    localparam [1:0] SYS_SHD   = 2'b01;
    localparam [1:0] SYS_FLAGS = 2'b10; // {2'b00, Z, C}
    localparam [1:0] SYS_OUT   = 2'b11;

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
    reg [3:0] shd_q;
    reg [3:0] out_q;
    reg       z_q;
    reg       c_q;

    reg [3:0] ram_q [0:3];

    // Undo / checkpoint state
    reg [3:0] prev_acc_q;
    reg [3:0] prev_shd_q;
    reg [3:0] prev_out_q;
    reg       prev_z_q;
    reg       prev_c_q;
    reg       undo_valid_q;
    reg       undo_write_valid_q;
    reg       undo_write_mode_q;    // 0 = RAM, 1 = SYS
    reg [1:0] undo_write_addr_q;
    reg [3:0] undo_write_olddata_q;

    integer i;

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------
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

    function [3:0] parity4;
        input [3:0] value;
        begin
            parity4 = {3'b000, ^value};
        end
    endfunction

    function [3:0] flags_pack;
        input z_in;
        input c_in;
        begin
            flags_pack = {2'b00, z_in, c_in};
        end
    endfunction

    function [3:0] sys_read;
        input [1:0] addr;
        begin
            case (addr)
                SYS_ACC:   sys_read = acc_q;
                SYS_SHD:   sys_read = shd_q;
                SYS_FLAGS: sys_read = flags_pack(z_q, c_q);
                default:   sys_read = out_q; // SYS_OUT
            endcase
        end
    endfunction

    function [3:0] mapped_read;
        input       mode;
        input [1:0] addr;
        begin
            if (mode) begin
                mapped_read = sys_read(addr);
            end else begin
                mapped_read = ram_q[addr];
            end
        end
    endfunction

    task checkpoint_no_write;
        begin
            prev_acc_q        <= acc_q;
            prev_shd_q        <= shd_q;
            prev_out_q        <= out_q;
            prev_z_q          <= z_q;
            prev_c_q          <= c_q;
            undo_valid_q      <= 1'b1;
            undo_write_valid_q<= 1'b0;
            undo_write_mode_q <= 1'b0;
            undo_write_addr_q <= 2'b00;
            undo_write_olddata_q <= 4'h0;
        end
    endtask

    task checkpoint_ram_write;
        input [1:0] addr;
        input [3:0] olddata;
        begin
            prev_acc_q        <= acc_q;
            prev_shd_q        <= shd_q;
            prev_out_q        <= out_q;
            prev_z_q          <= z_q;
            prev_c_q          <= c_q;
            undo_valid_q      <= 1'b1;
            undo_write_valid_q<= 1'b1;
            undo_write_mode_q <= 1'b0;
            undo_write_addr_q <= addr;
            undo_write_olddata_q <= olddata;
        end
    endtask

    task write_sys;
        input [1:0] addr;
        input [3:0] data;
        begin
            case (addr)
                SYS_ACC: begin
                    acc_q <= data;
                end
                SYS_SHD: begin
                    shd_q <= data;
                end
                SYS_FLAGS: begin
                    z_q <= data[1];
                    c_q <= data[0];
                end
                default: begin
                    out_q <= data; // SYS_OUT
                end
            endcase
        end
    endtask

    // ---------------------------------------------------------------------
    // Main controller
    // State updates happen in P2 so the committed result is visible in P3.
    // ---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        reg [4:0] ext5;
        reg [3:0] tmp_data;
        reg [3:0] old_mapped;
        reg [1:0] a2;
        if (!rst_n) begin
            phase_q <= P0_INSTR;
            ir_class_q <= 3'b000;
            ir_mode_q  <= 1'b0;
            ir_func_q  <= 4'h0;
            arg_a_q    <= 4'h0;
            arg_b_q    <= 4'h0;

            acc_q <= 4'h0;
            shd_q <= 4'h0;
            out_q <= 4'h0;
            z_q   <= 1'b0;
            c_q   <= 1'b0;

            prev_acc_q <= 4'h0;
            prev_shd_q <= 4'h0;
            prev_out_q <= 4'h0;
            prev_z_q   <= 1'b0;
            prev_c_q   <= 1'b0;
            undo_valid_q <= 1'b0;
            undo_write_valid_q <= 1'b0;
            undo_write_mode_q  <= 1'b0;
            undo_write_addr_q  <= 2'b00;
            undo_write_olddata_q <= 4'h0;

            for (i = 0; i < 4; i = i + 1) begin
                ram_q[i] <= 4'h0;
            end
        end else if (ena) begin
            a2 = arg_a_q[1:0];
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
                        // -------------------------------------------------
                        // 000: ALU family (use arg_b_q as immediate source)
                        // -------------------------------------------------
                        CLS_ALU: begin
                            checkpoint_no_write();
                            case (ir_func_q[1:0])
                                2'b00: begin // ADD
                                    ext5 = {1'b0, acc_q} + {1'b0, arg_b_q};
                                    acc_q <= ext5[3:0];
                                    out_q <= ext5[3:0];
                                    z_q   <= (ext5[3:0] == 4'h0);
                                    c_q   <= ext5[4];
                                end
                                2'b01: begin // XOR
                                    tmp_data = acc_q ^ arg_b_q;
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                                2'b10: begin // AND
                                    tmp_data = acc_q & arg_b_q;
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                                default: begin // OR
                                    tmp_data = acc_q | arg_b_q;
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                            endcase
                        end

                        // -------------------------------------------------
                        // 001: Permute family
                        // -------------------------------------------------
                        CLS_PERM: begin
                            checkpoint_no_write();
                            case (ir_func_q[1:0])
                                2'b00: begin // ROL
                                    tmp_data = {acc_q[2:0], acc_q[3]};
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= acc_q[3];
                                end
                                2'b01: begin // ROR
                                    tmp_data = {acc_q[0], acc_q[3:1]};
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= acc_q[0];
                                end
                                2'b10: begin // SWAP2
                                    tmp_data = swap2_4(acc_q);
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                                default: begin // BITREV
                                    tmp_data = bitrev4(acc_q);
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                            endcase
                        end

                        // -------------------------------------------------
                        // 010: Memory family
                        // mode=0 RAM[0:3], mode=1 SYS{ACC,SHD,FLAGS,OUT}
                        // -------------------------------------------------
                        CLS_MEM: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // READ (non-mutating)
                                    tmp_data = mapped_read(ir_mode_q, a2);
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end

                                2'b01: begin // WRITE
                                    if (ir_mode_q) begin
                                        checkpoint_no_write();
                                        write_sys(a2, arg_b_q);
                                        out_q <= arg_b_q;
                                        if (a2 != SYS_FLAGS) begin
                                            z_q <= (arg_b_q == 4'h0);
                                            c_q <= 1'b0;
                                        end
                                    end else begin
                                        checkpoint_ram_write(a2, ram_q[a2]);
                                        ram_q[a2] <= arg_b_q;
                                        out_q <= arg_b_q;
                                        z_q   <= (arg_b_q == 4'h0);
                                        c_q   <= 1'b0;
                                    end
                                end

                                2'b10: begin // LOADACC
                                    checkpoint_no_write();
                                    tmp_data = mapped_read(ir_mode_q, a2);
                                    acc_q <= tmp_data;
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end

                                default: begin // SWAPACC
                                    if (ir_mode_q) begin
                                        checkpoint_no_write();
                                        old_mapped = sys_read(a2);
                                        write_sys(a2, acc_q);
                                        acc_q <= old_mapped;
                                        out_q <= old_mapped;
                                        z_q   <= (old_mapped == 4'h0);
                                        c_q   <= 1'b0;
                                    end else begin
                                        checkpoint_ram_write(a2, ram_q[a2]);
                                        tmp_data = ram_q[a2];
                                        ram_q[a2] <= acc_q;
                                        acc_q <= tmp_data;
                                        out_q <= tmp_data;
                                        z_q   <= (tmp_data == 4'h0);
                                        c_q   <= 1'b0;
                                    end
                                end
                            endcase
                        end

                        // -------------------------------------------------
                        // 011: Reversible family
                        // -------------------------------------------------
                        CLS_REV: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // SAVE
                                    checkpoint_no_write();
                                    out_q <= acc_q;
                                    z_q   <= (acc_q == 4'h0);
                                    c_q   <= 1'b0;
                                end

                                2'b01: begin // REVERSE (single-step restore)
                                    if (undo_valid_q) begin
                                        if (undo_write_valid_q && !undo_write_mode_q) begin
                                            ram_q[undo_write_addr_q] <= undo_write_olddata_q;
                                        end
                                        acc_q <= prev_acc_q;
                                        shd_q <= prev_shd_q;
                                        out_q <= prev_out_q;
                                        z_q   <= prev_z_q;
                                        c_q   <= prev_c_q;
                                        undo_valid_q <= 1'b0;
                                        undo_write_valid_q <= 1'b0;
                                    end else begin
                                        out_q <= acc_q;
                                        z_q   <= z_q;
                                        c_q   <= c_q;
                                    end
                                end

                                2'b10: begin // ACC <-> SHD
                                    checkpoint_no_write();
                                    acc_q <= shd_q;
                                    shd_q <= acc_q;
                                    out_q <= shd_q;
                                    z_q   <= (shd_q == 4'h0);
                                    c_q   <= 1'b0;
                                end

                                default: begin // PARITY
                                    tmp_data = parity4(acc_q);
                                    out_q <= tmp_data;
                                    z_q   <= (tmp_data == 4'h0);
                                    c_q   <= 1'b0;
                                end
                            endcase
                        end

                        // -------------------------------------------------
                        // 100: Control / flags family
                        // -------------------------------------------------
                        CLS_CTRL: begin
                            case (ir_func_q[1:0])
                                2'b00: begin // SETACC immediate
                                    checkpoint_no_write();
                                    acc_q <= arg_b_q;
                                    out_q <= arg_b_q;
                                    z_q   <= (arg_b_q == 4'h0);
                                    c_q   <= 1'b0;
                                end
                                2'b01: begin // SETSHD immediate
                                    checkpoint_no_write();
                                    shd_q <= arg_b_q;
                                    out_q <= arg_b_q;
                                    z_q   <= (arg_b_q == 4'h0);
                                    c_q   <= 1'b0;
                                end
                                2'b10: begin // CMP flags
                                    out_q <= {2'b00, (acc_q > arg_b_q), (acc_q == arg_b_q)};
                                    z_q   <= (acc_q == arg_b_q);
                                    c_q   <= (acc_q >= arg_b_q);
                                end
                                default: begin // CLEAR working state (not RAM)
                                    checkpoint_no_write();
                                    acc_q <= 4'h0;
                                    shd_q <= 4'h0;
                                    out_q <= 4'h0;
                                    z_q   <= 1'b1;
                                    c_q   <= 1'b0;
                                end
                            endcase
                        end

                        default: begin // reserved NOP
                            out_q <= acc_q;
                            z_q   <= (acc_q == 4'h0);
                            c_q   <= c_q;
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
    // Public bus packing
    // ---------------------------------------------------------------------
    assign uo_out = {phase_q, out_q, z_q, c_q};

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    wire _unused = &{1'b0, uio_in};
endmodule

`default_nettype wire
