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
    // ------------------------------------------------------------------------
    // 4-phase externally-driven execution core.
    //
    // Phase 00: latch instruction word  {class, mode, func[3:0]}
    // Phase 01: latch operand/data word {A[3:0], B[3:0]}
    // Phase 10: execute and commit state
    // Phase 11: output-valid
    // ------------------------------------------------------------------------

    localparam [1:0] P0_INSTR   = 2'b00;
    localparam [1:0] P1_OPERAND = 2'b01;
    localparam [1:0] P2_EXECUTE = 2'b10;
    localparam [1:0] P3_OUTPUT  = 2'b11;

    // Class map: merged broad-v2 + RF design.
    localparam [2:0] CLS_ALU   = 3'b000; // arithmetic + boolean
    localparam [2:0] CLS_PERM  = 3'b001; // shift / permute / gray / ecc
    localparam [2:0] CLS_CMP   = 3'b010; // compare / reduce
    localparam [2:0] CLS_MEM   = 3'b011; // RAM / system access
    localparam [2:0] CLS_SYS   = 3'b100; // generic system register ops
    localparam [2:0] CLS_REV   = 3'b101; // save / reverse / swaps / parity-ecc
    localparam [2:0] CLS_RFALU = 3'b110; // r[rd] = r[rs] op r[rt]
    localparam [2:0] CLS_RFIO  = 3'b111; // register-file utilities

    // System-space map (16 entries).
    localparam [3:0] SYS_ACC   = 4'h0;
    localparam [3:0] SYS_BREG  = 4'h1;
    localparam [3:0] SYS_SHD   = 4'h2;
    localparam [3:0] SYS_FLAGS = 4'h3; // {undo_valid, N, C, Z}
    localparam [3:0] SYS_MAR   = 4'h4;
    localparam [3:0] SYS_MDR   = 4'h5;
    localparam [3:0] SYS_OUT   = 4'h6;
    localparam [3:0] SYS_PHS   = 4'h7; // {phase[1:0], busy, valid}
    localparam [3:0] SYS_TMP0  = 4'h8;
    localparam [3:0] SYS_TMP1  = 4'h9;
    localparam [3:0] SYS_EXT0  = 4'hA;
    localparam [3:0] SYS_EXT1  = 4'hB;
    localparam [3:0] SYS_RF0   = 4'hC;
    localparam [3:0] SYS_RF1   = 4'hD;
    localparam [3:0] SYS_RF2   = 4'hE;
    localparam [3:0] SYS_RF3   = 4'hF;

    // Latched instruction/operand fields.
    reg [1:0] phase_q;
    reg [2:0] ir_class_q;
    reg       ir_mode_q;
    reg [3:0] ir_func_q;
    reg [3:0] arg_a_q;
    reg [3:0] arg_b_q;

    // Architectural working state.
    reg [3:0] acc_q,  prev_acc_q;
    reg [3:0] breg_q, prev_breg_q;
    reg [3:0] shd_q,  prev_shd_q;
    reg [3:0] mar_q,  prev_mar_q;
    reg [3:0] mdr_q,  prev_mdr_q;
    reg [3:0] out_q,  prev_out_q;
    reg [3:0] tmp0_q, prev_tmp0_q;
    reg [3:0] tmp1_q, prev_tmp1_q;
    reg [3:0] ext0_q, prev_ext0_q;
    reg [3:0] ext1_q, prev_ext1_q;

    reg       z_q, prev_z_q;
    reg       c_q, prev_c_q;
    reg       n_q, prev_n_q;
    reg       undo_valid_q, prev_undo_valid_q;

    // Memories.
    reg [3:0] ram_q      [0:3];
    reg [3:0] prev_ram_q [0:3];
    reg [3:0] rf_q       [0:3];
    reg [3:0] prev_rf_q  [0:3];

    // Helper temporary registers used inside sequential execute block.
    reg [3:0] lhs4, rhs4, src4, tmp4, tmp4b, res4;
    reg [4:0] wide5;
    reg       gt_b, eq_b, lt_b;
    integer   i;

    // ------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------

    function automatic [3:0] bitrev4;
        input [3:0] x;
        begin
            bitrev4 = {x[0], x[1], x[2], x[3]};
        end
    endfunction

    function automatic [3:0] swap2_4;
        input [3:0] x;
        begin
            swap2_4 = {x[1:0], x[3:2]};
        end
    endfunction

    function automatic [3:0] grayenc4;
        input [3:0] x;
        begin
            grayenc4 = {x[3], x[3]^x[2], x[2]^x[1], x[1]^x[0]};
        end
    endfunction

    function automatic [3:0] graydec4;
        input [3:0] g;
        reg [3:0] b;
        begin
            b[3] = g[3];
            b[2] = b[3] ^ g[2];
            b[1] = b[2] ^ g[1];
            b[0] = b[1] ^ g[0];
            graydec4 = b;
        end
    endfunction

    // Compact parity/Hamming-style nibble {p4,p2,p1,p0}
    function automatic [3:0] ecc4;
        input [3:0] x;
        reg p0, p1, p2, p4;
        begin
            p0 = ^x;
            p1 = x[0] ^ x[1] ^ x[3];
            p2 = x[0] ^ x[2] ^ x[3];
            p4 = x[1] ^ x[2] ^ x[3];
            ecc4 = {p4, p2, p1, p0};
        end
    endfunction

    function automatic [2:0] flags_from_result;
        input [3:0] result;
        input       carry_or_gt;
        begin
            flags_from_result = {result[3], carry_or_gt, (result == 4'h0)};
        end
    endfunction

    function automatic [3:0] rf_read;
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

    function automatic [3:0] sys_read;
        input [3:0] addr;
        begin
            case (addr)
                SYS_ACC:   sys_read = acc_q;
                SYS_BREG:  sys_read = breg_q;
                SYS_SHD:   sys_read = shd_q;
                SYS_FLAGS: sys_read = {undo_valid_q, n_q, c_q, z_q};
                SYS_MAR:   sys_read = mar_q;
                SYS_MDR:   sys_read = mdr_q;
                SYS_OUT:   sys_read = out_q;
                SYS_PHS:   sys_read = {phase_q, (phase_q != P0_INSTR), 1'b1};
                SYS_TMP0:  sys_read = tmp0_q;
                SYS_TMP1:  sys_read = tmp1_q;
                SYS_EXT0:  sys_read = ext0_q;
                SYS_EXT1:  sys_read = ext1_q;
                SYS_RF0:   sys_read = rf_q[0];
                SYS_RF1:   sys_read = rf_q[1];
                SYS_RF2:   sys_read = rf_q[2];
                default:   sys_read = rf_q[3];
            endcase
        end
    endfunction

    function automatic [3:0] mem_read;
        input       mode;
        input [3:0] addr;
        begin
            if (mode) begin
                mem_read = sys_read(addr);
            end else begin
                mem_read = ram_q[addr[1:0]];
            end
        end
    endfunction

    task automatic sys_write;
        input [3:0] addr;
        input [3:0] data;
        begin
            case (addr)
                SYS_ACC:   acc_q  <= data;
                SYS_BREG:  breg_q <= data;
                SYS_SHD:   shd_q  <= data;
                SYS_FLAGS: begin
                    n_q <= data[2];
                    c_q <= data[1];
                    z_q <= data[0];
                end
                SYS_MAR:   mar_q  <= data;
                SYS_MDR:   mdr_q  <= data;
                SYS_OUT:   out_q  <= data;
                SYS_TMP0:  tmp0_q <= data;
                SYS_TMP1:  tmp1_q <= data;
                SYS_EXT0:  ext0_q <= data;
                SYS_EXT1:  ext1_q <= data;
                SYS_RF0:   rf_q[0] <= data;
                SYS_RF1:   rf_q[1] <= data;
                SYS_RF2:   rf_q[2] <= data;
                SYS_RF3:   rf_q[3] <= data;
                default: ; // SYS_PHS read-only
            endcase
        end
    endtask

    task automatic snapshot_current_to_prev;
        begin
            prev_acc_q        <= acc_q;
            prev_breg_q       <= breg_q;
            prev_shd_q        <= shd_q;
            prev_mar_q        <= mar_q;
            prev_mdr_q        <= mdr_q;
            prev_out_q        <= out_q;
            prev_tmp0_q       <= tmp0_q;
            prev_tmp1_q       <= tmp1_q;
            prev_ext0_q       <= ext0_q;
            prev_ext1_q       <= ext1_q;
            prev_z_q          <= z_q;
            prev_c_q          <= c_q;
            prev_n_q          <= n_q;
            prev_undo_valid_q <= undo_valid_q;
            for (i = 0; i < 4; i = i + 1) begin
                prev_ram_q[i] <= ram_q[i];
                prev_rf_q[i]  <= rf_q[i];
            end
            undo_valid_q <= 1'b1;
        end
    endtask

    task automatic reverse_swap_all;
        begin
            tmp4 = acc_q;        acc_q  <= prev_acc_q;        prev_acc_q  <= tmp4;
            tmp4 = breg_q;       breg_q <= prev_breg_q;       prev_breg_q <= tmp4;
            tmp4 = shd_q;        shd_q  <= prev_shd_q;        prev_shd_q  <= tmp4;
            tmp4 = mar_q;        mar_q  <= prev_mar_q;        prev_mar_q  <= tmp4;
            tmp4 = mdr_q;        mdr_q  <= prev_mdr_q;        prev_mdr_q  <= tmp4;
            tmp4 = out_q;        out_q  <= prev_out_q;        prev_out_q  <= tmp4;
            tmp4 = tmp0_q;       tmp0_q <= prev_tmp0_q;       prev_tmp0_q <= tmp4;
            tmp4 = tmp1_q;       tmp1_q <= prev_tmp1_q;       prev_tmp1_q <= tmp4;
            tmp4 = ext0_q;       ext0_q <= prev_ext0_q;       prev_ext0_q <= tmp4;
            tmp4 = ext1_q;       ext1_q <= prev_ext1_q;       prev_ext1_q <= tmp4;

            gt_b = z_q;          z_q    <= prev_z_q;          prev_z_q    <= gt_b;
            gt_b = c_q;          c_q    <= prev_c_q;          prev_c_q    <= gt_b;
            gt_b = n_q;          n_q    <= prev_n_q;          prev_n_q    <= gt_b;
            gt_b = undo_valid_q; undo_valid_q <= prev_undo_valid_q; prev_undo_valid_q <= gt_b;

            for (i = 0; i < 4; i = i + 1) begin
                tmp4 = ram_q[i]; ram_q[i] <= prev_ram_q[i]; prev_ram_q[i] <= tmp4;
                tmp4 = rf_q[i];  rf_q[i]  <= prev_rf_q[i];  prev_rf_q[i]  <= tmp4;
            end
        end
    endtask

    task automatic clear_all_current;
        begin
            acc_q  <= 4'h0;
            breg_q <= 4'h0;
            shd_q  <= 4'h0;
            mar_q  <= 4'h0;
            mdr_q  <= 4'h0;
            out_q  <= 4'h0;
            tmp0_q <= 4'h0;
            tmp1_q <= 4'h0;
            ext0_q <= 4'h0;
            ext1_q <= 4'h0;
            z_q    <= 1'b1;
            c_q    <= 1'b0;
            n_q    <= 1'b0;
            for (i = 0; i < 4; i = i + 1) begin
                ram_q[i] <= 4'h0;
                rf_q[i]  <= 4'h0;
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Main sequential controller
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_q           <= P0_INSTR;
            ir_class_q        <= 3'b000;
            ir_mode_q         <= 1'b0;
            ir_func_q         <= 4'h0;
            arg_a_q           <= 4'h0;
            arg_b_q           <= 4'h0;

            acc_q             <= 4'h0;
            breg_q            <= 4'h0;
            shd_q             <= 4'h0;
            mar_q             <= 4'h0;
            mdr_q             <= 4'h0;
            out_q             <= 4'h0;
            tmp0_q            <= 4'h0;
            tmp1_q            <= 4'h0;
            ext0_q            <= 4'h0;
            ext1_q            <= 4'h0;
            z_q               <= 1'b1;
            c_q               <= 1'b0;
            n_q               <= 1'b0;
            undo_valid_q      <= 1'b0;

            prev_acc_q        <= 4'h0;
            prev_breg_q       <= 4'h0;
            prev_shd_q        <= 4'h0;
            prev_mar_q        <= 4'h0;
            prev_mdr_q        <= 4'h0;
            prev_out_q        <= 4'h0;
            prev_tmp0_q       <= 4'h0;
            prev_tmp1_q       <= 4'h0;
            prev_ext0_q       <= 4'h0;
            prev_ext1_q       <= 4'h0;
            prev_z_q          <= 1'b1;
            prev_c_q          <= 1'b0;
            prev_n_q          <= 1'b0;
            prev_undo_valid_q <= 1'b0;

            for (i = 0; i < 4; i = i + 1) begin
                ram_q[i]      <= 4'h0;
                prev_ram_q[i] <= 4'h0;
                rf_q[i]       <= 4'h0;
                prev_rf_q[i]  <= 4'h0;
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
                    // Defaults for temporaries
                    lhs4 = acc_q;
                    rhs4 = ir_mode_q ? sys_read(arg_a_q) : arg_b_q;
                    src4 = mem_read(ir_mode_q, arg_a_q);
                    res4 = out_q;
                    phase_q <= P3_OUTPUT;

                    case (ir_class_q)
                        CLS_ALU: begin
                            snapshot_current_to_prev();
                            case (ir_func_q)
                                4'h0: begin // ADD
                                    wide5 = {1'b0, acc_q} + {1'b0, rhs4};
                                    res4 = wide5[3:0];
                                    acc_q <= res4;
                                    out_q <= res4;
                                    n_q   <= res4[3];
                                    c_q   <= wide5[4];
                                    z_q   <= (res4 == 4'h0);
                                end
                                4'h1: begin // ADC
                                    wide5 = {1'b0, acc_q} + {1'b0, rhs4} + {4'b0000, c_q};
                                    res4 = wide5[3:0];
                                    acc_q <= res4;
                                    out_q <= res4;
                                    n_q   <= res4[3];
                                    c_q   <= wide5[4];
                                    z_q   <= (res4 == 4'h0);
                                end
                                4'h2: begin // SUB
                                    wide5 = {1'b0, acc_q} - {1'b0, rhs4};
                                    res4 = wide5[3:0];
                                    acc_q <= res4;
                                    out_q <= res4;
                                    n_q   <= res4[3];
                                    c_q   <= ~wide5[4]; // no-borrow indicator
                                    z_q   <= (res4 == 4'h0);
                                end
                                4'h3: begin // SBC
                                    wide5 = {1'b0, acc_q} - {1'b0, rhs4} - {4'b0000, ~c_q};
                                    res4 = wide5[3:0];
                                    acc_q <= res4;
                                    out_q <= res4;
                                    n_q   <= res4[3];
                                    c_q   <= ~wide5[4];
                                    z_q   <= (res4 == 4'h0);
                                end
                                4'h4: begin // AND
                                    res4 = acc_q & rhs4;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'h5: begin // OR
                                    res4 = acc_q | rhs4;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'h6: begin // XOR
                                    res4 = acc_q ^ rhs4;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'h7: begin // XNOR
                                    res4 = ~(acc_q ^ rhs4);
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'h8: begin // NAND
                                    res4 = ~(acc_q & rhs4);
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'h9: begin // NOR
                                    res4 = ~(acc_q | rhs4);
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'hA: begin // NOT ACC
                                    res4 = ~acc_q;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'hB: begin // BIC = ACC & ~rhs
                                    res4 = acc_q & ~rhs4;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'hC: begin // PASS rhs
                                    res4 = rhs4;
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= 1'b0; z_q <= (res4 == 4'h0);
                                end
                                4'hD: begin // INC
                                    wide5 = {1'b0, acc_q} + 5'd1;
                                    res4 = wide5[3:0];
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= wide5[4]; z_q <= (res4 == 4'h0);
                                end
                                4'hE: begin // DEC
                                    wide5 = {1'b0, acc_q} - 5'd1;
                                    res4 = wide5[3:0];
                                    acc_q <= res4; out_q <= res4; n_q <= res4[3]; c_q <= ~wide5[4]; z_q <= (res4 == 4'h0);
                                end
                                default: begin // NOP/pass-through
                                    out_q <= acc_q;
                                end
                            endcase
                        end

                        CLS_PERM: begin
                            snapshot_current_to_prev();
                            src4 = ir_mode_q ? sys_read(arg_a_q) : acc_q;
                            case (ir_func_q)
                                4'h0: res4 = {src4[2:0], src4[3]};          // ROL
                                4'h1: res4 = {src4[0], src4[3:1]};          // ROR
                                4'h2: res4 = {src4[2:0], 1'b0};             // SHL
                                4'h3: res4 = {1'b0, src4[3:1]};             // SHR
                                4'h4: res4 = swap2_4(src4);                 // SWAP2
                                4'h5: res4 = bitrev4(src4);                 // BITREV
                                4'h6: res4 = grayenc4(src4);                // GRAYENC
                                4'h7: res4 = graydec4(src4);                // GRAYDEC
                                4'h8: res4 = {src4[3], src4[3:1]};          // ASR
                                4'h9: res4 = ecc4(src4);                    // ECC/Parity nibble
                                default: res4 = src4;
                            endcase
                            acc_q <= res4;
                            out_q <= res4;
                            n_q   <= res4[3];
                            c_q   <= (ir_func_q == 4'h1) ? src4[0] : ((ir_func_q == 4'h0 || ir_func_q == 4'h2) ? src4[3] : 1'b0);
                            z_q   <= (res4 == 4'h0);
                        end

                        CLS_CMP: begin
                            snapshot_current_to_prev();
                            rhs4 = ir_mode_q ? sys_read(arg_a_q) : arg_b_q;
                            if (ir_mode_q) begin
                                gt_b = (acc_q[2:0] > rhs4[2:0]);
                                eq_b = (acc_q[2:0] == rhs4[2:0]);
                                lt_b = (acc_q[2:0] < rhs4[2:0]);
                            end else begin
                                gt_b = (acc_q > rhs4);
                                eq_b = (acc_q == rhs4);
                                lt_b = (acc_q < rhs4);
                            end
                            case (ir_func_q)
                                4'h0: res4 = {1'b0, gt_b, eq_b, lt_b};      // compare nibble
                                4'h1: res4 = {3'b000, eq_b};                // EQ
                                4'h2: res4 = {3'b000, gt_b};                // GT
                                4'h3: res4 = {3'b000, lt_b};                // LT
                                4'h4: res4 = (acc_q < rhs4) ? acc_q : rhs4; // MIN
                                4'h5: res4 = (acc_q > rhs4) ? acc_q : rhs4; // MAX
                                4'h6: res4 = {{1{1'b0}}, acc_q[0]+acc_q[1]+acc_q[2]+acc_q[3]}; // POPCOUNT
                                4'h7: res4 = {3'b000, (acc_q == 4'h0)};     // ZERO?
                                default: res4 = {1'b0, gt_b, eq_b, lt_b};
                            endcase
                            out_q <= res4;
                            n_q   <= res4[3];
                            c_q   <= gt_b;
                            z_q   <= eq_b;
                        end

                        CLS_MEM: begin
                            snapshot_current_to_prev();
                            src4 = mem_read(ir_mode_q, arg_a_q);
                            case (ir_func_q)
                                4'h0: begin // READ
                                    mdr_q <= src4;
                                    out_q <= src4;
                                    n_q   <= src4[3];
                                    c_q   <= 1'b0;
                                    z_q   <= (src4 == 4'h0);
                                end
                                4'h1: begin // WRITE
                                    if (ir_mode_q)
                                        sys_write(arg_a_q, arg_b_q);
                                    else
                                        ram_q[arg_a_q[1:0]] <= arg_b_q;
                                    mdr_q <= arg_b_q;
                                    out_q <= arg_b_q;
                                    n_q   <= arg_b_q[3];
                                    c_q   <= 1'b0;
                                    z_q   <= (arg_b_q == 4'h0);
                                end
                                4'h2: begin // LOADACC
                                    acc_q <= src4;
                                    mdr_q <= src4;
                                    out_q <= src4;
                                    n_q   <= src4[3];
                                    c_q   <= 1'b0;
                                    z_q   <= (src4 == 4'h0);
                                end
                                4'h3: begin // SWAPACC
                                    if (ir_mode_q) begin
                                        tmp4 = sys_read(arg_a_q);
                                        sys_write(arg_a_q, acc_q);
                                        acc_q <= tmp4;
                                        mdr_q <= tmp4;
                                        out_q <= tmp4;
                                        n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                    end else begin
                                        tmp4 = ram_q[arg_a_q[1:0]];
                                        ram_q[arg_a_q[1:0]] <= acc_q;
                                        acc_q <= tmp4;
                                        mdr_q <= tmp4;
                                        out_q <= tmp4;
                                        n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                    end
                                end
                                default: begin
                                    out_q <= out_q;
                                end
                            endcase
                        end

                        CLS_SYS: begin
                            snapshot_current_to_prev();
                            case (ir_func_q[1:0])
                                2'd0: begin // LOADIMM: sys[A] <= B
                                    sys_write(arg_a_q, arg_b_q);
                                    out_q <= arg_b_q;
                                    n_q   <= arg_b_q[3]; c_q <= 1'b0; z_q <= (arg_b_q == 4'h0);
                                end
                                2'd1: begin // MOV: sys[A] <= sys[B]
                                    tmp4 = sys_read(arg_b_q);
                                    sys_write(arg_a_q, tmp4);
                                    out_q <= tmp4;
                                    n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                end
                                2'd2: begin // SWAP: sys[A] <-> sys[B]
                                    tmp4  = sys_read(arg_a_q);
                                    tmp4b = sys_read(arg_b_q);
                                    sys_write(arg_a_q, tmp4b);
                                    sys_write(arg_b_q, tmp4);
                                    out_q <= tmp4b;
                                    n_q   <= tmp4b[3]; c_q <= 1'b0; z_q <= (tmp4b == 4'h0);
                                end
                                default: begin // CLEAR sys[A]
                                    sys_write(arg_a_q, 4'h0);
                                    out_q <= 4'h0;
                                    n_q   <= 1'b0; c_q <= 1'b0; z_q <= 1'b1;
                                end
                            endcase
                        end

                        CLS_REV: begin
                            case (ir_func_q)
                                4'h0: begin // SAVE
                                    snapshot_current_to_prev();
                                    out_q <= out_q;
                                end
                                4'h1: begin // REVERSE current/previous swap
                                    if (undo_valid_q)
                                        reverse_swap_all();
                                end
                                4'h2: begin // ACC <-> SHD
                                    snapshot_current_to_prev();
                                    tmp4  = acc_q;
                                    acc_q <= shd_q;
                                    shd_q <= tmp4;
                                    out_q <= shd_q;
                                    n_q   <= shd_q[3]; c_q <= 1'b0; z_q <= (shd_q == 4'h0);
                                end
                                4'h3: begin // ACC <-> BREG
                                    snapshot_current_to_prev();
                                    tmp4   = acc_q;
                                    acc_q  <= breg_q;
                                    breg_q <= tmp4;
                                    out_q  <= breg_q;
                                    n_q    <= breg_q[3]; c_q <= 1'b0; z_q <= (breg_q == 4'h0);
                                end
                                4'h4: begin // clear undo state
                                    undo_valid_q <= 1'b0;
                                end
                                4'h5: begin // parity/ecc on ACC
                                    snapshot_current_to_prev();
                                    res4 = ecc4(acc_q);
                                    out_q <= res4;
                                    n_q   <= res4[3]; c_q <= res4[0]; z_q <= (res4 == 4'h0);
                                end
                                default: begin
                                    out_q <= out_q;
                                end
                            endcase
                        end

                        CLS_RFALU: begin
                            snapshot_current_to_prev();
                            lhs4 = rf_read(arg_a_q[3:2]);
                            rhs4 = rf_read(arg_a_q[1:0]);
                            case (ir_func_q)
                                4'h0: begin wide5 = {1'b0,lhs4}+{1'b0,rhs4}; res4 = wide5[3:0]; c_q <= wide5[4]; end // ADD
                                4'h1: begin res4 = lhs4 ^ rhs4; c_q <= 1'b0; end // XOR
                                4'h2: begin res4 = lhs4 & rhs4; c_q <= 1'b0; end // AND
                                4'h3: begin res4 = lhs4 | rhs4; c_q <= 1'b0; end // OR
                                4'h4: begin wide5 = {1'b0,lhs4}-{1'b0,rhs4}; res4 = wide5[3:0]; c_q <= ~wide5[4]; end // SUB
                                4'h5: begin res4 = ~(lhs4 ^ rhs4); c_q <= 1'b0; end // XNOR
                                4'h6: begin res4 = ~(lhs4 & rhs4); c_q <= 1'b0; end // NAND
                                4'h7: begin res4 = ~(lhs4 | rhs4); c_q <= 1'b0; end // NOR
                                default: begin res4 = lhs4; c_q <= 1'b0; end
                            endcase
                            rf_q[arg_b_q[3:2]] <= res4;
                            out_q <= res4;
                            n_q   <= res4[3];
                            z_q   <= (res4 == 4'h0);
                        end

                        CLS_RFIO: begin
                            snapshot_current_to_prev();
                            case (ir_func_q)
                                4'h0: begin // RFLOADI rf[A[1:0]] <= B
                                    rf_q[arg_a_q[1:0]] <= arg_b_q;
                                    out_q <= arg_b_q;
                                    n_q   <= arg_b_q[3]; c_q <= 1'b0; z_q <= (arg_b_q == 4'h0);
                                end
                                4'h1: begin // RFREAD
                                    tmp4 = rf_read(arg_a_q[1:0]);
                                    out_q <= tmp4;
                                    n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                end
                                4'h2: begin // RFTOACC
                                    tmp4 = rf_read(arg_a_q[1:0]);
                                    acc_q <= tmp4;
                                    out_q <= tmp4;
                                    n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                end
                                4'h3: begin // ACCTORF
                                    rf_q[arg_a_q[1:0]] <= acc_q;
                                    out_q <= acc_q;
                                    n_q   <= acc_q[3]; c_q <= 1'b0; z_q <= (acc_q == 4'h0);
                                end
                                4'h4: begin // RFMOVE A[3:2]=src, A[1:0]=dst
                                    tmp4 = rf_read(arg_a_q[3:2]);
                                    rf_q[arg_a_q[1:0]] <= tmp4;
                                    out_q <= tmp4;
                                    n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                end
                                4'h5: begin // RFSWAP A[3:2]=r1, A[1:0]=r2
                                    tmp4  = rf_read(arg_a_q[3:2]);
                                    tmp4b = rf_read(arg_a_q[1:0]);
                                    rf_q[arg_a_q[3:2]] <= tmp4b;
                                    rf_q[arg_a_q[1:0]] <= tmp4;
                                    out_q <= tmp4b;
                                    n_q   <= tmp4b[3]; c_q <= 1'b0; z_q <= (tmp4b == 4'h0);
                                end
                                4'h6: begin // RFTOBREG
                                    tmp4 = rf_read(arg_a_q[1:0]);
                                    breg_q <= tmp4;
                                    out_q <= tmp4;
                                    n_q   <= tmp4[3]; c_q <= 1'b0; z_q <= (tmp4 == 4'h0);
                                end
                                4'h7: begin // BREGTORF
                                    rf_q[arg_a_q[1:0]] <= breg_q;
                                    out_q <= breg_q;
                                    n_q   <= breg_q[3]; c_q <= 1'b0; z_q <= (breg_q == 4'h0);
                                end
                                default: begin
                                    out_q <= out_q;
                                end
                            endcase
                        end

                        default: begin
                            // NOP
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

    // ------------------------------------------------------------------------
    // Public outputs
    // ------------------------------------------------------------------------
    assign uo_out[7:6] = phase_q;
    assign uo_out[5:2] = out_q;
    assign uo_out[1]   = z_q;
    assign uo_out[0]   = c_q;

    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // Unused input suppression
    wire _unused = &{1'b0, uio_in};

endmodule

`default_nettype wire
