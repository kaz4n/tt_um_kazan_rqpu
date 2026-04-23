/*
 * RQPU: Reversible-logic-inspired 4-bit processing unit for Tiny Tapeout.
 *
 * NOTE:
 * - The top module name is `tt_um_kazan_rqpu`.
 * - This combined file includes all submodules: primitives, operators, core.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

// ============================================================================
// Reversible-logic-inspired primitive library
// ============================================================================

module rqpu_cnot (
    input  wire control,
    input  wire target,
    output wire control_o,
    output wire target_o
);
    assign control_o = control;
    assign target_o  = control ^ target;
endmodule

module rqpu_toffoli (
    input  wire a,
    input  wire b,
    input  wire c,
    output wire a_o,
    output wire b_o,
    output wire c_o
);
    assign a_o = a;
    assign b_o = b;
    assign c_o = c ^ (a & b);
endmodule

module rqpu_fredkin (
    input  wire sel,
    input  wire a,
    input  wire b,
    output wire sel_o,
    output wire a_o,
    output wire b_o
);
    assign sel_o = sel;
    assign a_o   = sel ? b : a;
    assign b_o   = sel ? a : b;
endmodule

// ============================================================================
// Reversible-logic-inspired operator blocks
// ============================================================================

module rqpu_full_adder_ri (
    input  wire       a,
    input  wire       b,
    input  wire       cin,
    output wire       sum,
    output wire       cout,
    output wire [1:0] garbage
);
    wire ab;
    wire ac;
    wire bc;

    rqpu_toffoli u_ab (
        .a(a),
        .b(b),
        .c(1'b0),
        .a_o(),
        .b_o(),
        .c_o(ab)
    );

    rqpu_toffoli u_ac (
        .a(a),
        .b(cin),
        .c(1'b0),
        .a_o(),
        .b_o(),
        .c_o(ac)
    );

    rqpu_toffoli u_bc (
        .a(b),
        .b(cin),
        .c(1'b0),
        .a_o(),
        .b_o(),
        .c_o(bc)
    );

    assign sum     = a ^ b ^ cin;
    assign cout    = ab | ac | bc;
    assign garbage = {ab, bc};
endmodule

module rqpu_adder4 (
    input  wire [3:0] a,
    input  wire [3:0] b,
    output wire [3:0] sum,
    output wire       carry_out,
    output wire       carry_dbg
);
    wire c1;
    wire c2;
    wire c3;
    wire c4;
    wire [1:0] g0;
    wire [1:0] g1;
    wire [1:0] g2;
    wire [1:0] g3;

    rqpu_full_adder_ri fa0 (
        .a(a[0]),
        .b(b[0]),
        .cin(1'b0),
        .sum(sum[0]),
        .cout(c1),
        .garbage(g0)
    );

    rqpu_full_adder_ri fa1 (
        .a(a[1]),
        .b(b[1]),
        .cin(c1),
        .sum(sum[1]),
        .cout(c2),
        .garbage(g1)
    );

    rqpu_full_adder_ri fa2 (
        .a(a[2]),
        .b(b[2]),
        .cin(c2),
        .sum(sum[2]),
        .cout(c3),
        .garbage(g2)
    );

    rqpu_full_adder_ri fa3 (
        .a(a[3]),
        .b(b[3]),
        .cin(c3),
        .sum(sum[3]),
        .cout(c4),
        .garbage(g3)
    );

    assign carry_out = c4;
    assign carry_dbg = c3;

    wire _unused = &{g0, g1, g2, g3, 1'b0};
endmodule

module rqpu_xor4 (
    input  wire [3:0] a,
    input  wire [3:0] b,
    output wire [3:0] y
);
    wire [3:0] passthrough;

    rqpu_cnot x0 (.control(a[0]), .target(b[0]), .control_o(passthrough[0]), .target_o(y[0]));
    rqpu_cnot x1 (.control(a[1]), .target(b[1]), .control_o(passthrough[1]), .target_o(y[1]));
    rqpu_cnot x2 (.control(a[2]), .target(b[2]), .control_o(passthrough[2]), .target_o(y[2]));
    rqpu_cnot x3 (.control(a[3]), .target(b[3]), .control_o(passthrough[3]), .target_o(y[3]));

    wire _unused = &{passthrough, 1'b0};
endmodule

module rqpu_and4 (
    input  wire [3:0] a,
    input  wire [3:0] b,
    output wire [3:0] y
);
    wire [3:0] a_passthrough;
    wire [3:0] b_passthrough;

    rqpu_toffoli t0 (.a(a[0]), .b(b[0]), .c(1'b0), .a_o(a_passthrough[0]), .b_o(b_passthrough[0]), .c_o(y[0]));
    rqpu_toffoli t1 (.a(a[1]), .b(b[1]), .c(1'b0), .a_o(a_passthrough[1]), .b_o(b_passthrough[1]), .c_o(y[1]));
    rqpu_toffoli t2 (.a(a[2]), .b(b[2]), .c(1'b0), .a_o(a_passthrough[2]), .b_o(b_passthrough[2]), .c_o(y[2]));
    rqpu_toffoli t3 (.a(a[3]), .b(b[3]), .c(1'b0), .a_o(a_passthrough[3]), .b_o(b_passthrough[3]), .c_o(y[3]));

    wire _unused = &{a_passthrough, b_passthrough, 1'b0};
endmodule

module rqpu_compare4 (
    input  wire [3:0] a,
    input  wire [3:0] b,
    output wire       eq,
    output wire       gt,
    output wire       lt
);
    assign eq = (a == b);
    assign gt = (a > b);
    assign lt = (a < b);
endmodule

module rqpu_swap_halves4 (
    input  wire       control,
    input  wire [3:0] a,
    output wire [3:0] y
);
    wire s0;
    wire s1;

    rqpu_fredkin f0 (
        .sel(control),
        .a(a[0]),
        .b(a[2]),
        .sel_o(s0),
        .a_o(y[0]),
        .b_o(y[2])
    );

    rqpu_fredkin f1 (
        .sel(control),
        .a(a[1]),
        .b(a[3]),
        .sel_o(s1),
        .a_o(y[1]),
        .b_o(y[3])
    );

    wire _unused = &{s0, s1, 1'b0};
endmodule

module rqpu_parity4 (
    input  wire [3:0] data_a,
    input  wire [3:0] data_b,
    output wire       parity_bit,
    output wire       hamming_bit
);
    assign parity_bit  = ^{data_a, data_b};
    assign hamming_bit = ^data_a;
endmodule

module rqpu_rotate4 (
    input  wire       dir_right,
    input  wire [3:0] a,
    output wire [3:0] y,
    output wire       shifted_out
);
    wire [3:0] rot_left;
    wire [3:0] rot_right;

    assign rot_left    = {a[2:0], a[3]};
    assign rot_right   = {a[0], a[3:1]};
    assign y           = dir_right ? rot_right : rot_left;
    assign shifted_out = dir_right ? a[0] : a[3];
endmodule

// ============================================================================
// Combinational opcode decoder and datapath
// ============================================================================

module rqpu_core (
    input  wire [2:0] opcode,
    input  wire [3:0] acc_in,
    input  wire [3:0] operand_in,
    output reg  [3:0] acc_out,
    output reg  [3:0] result_out,
    output reg        flag_z_out,
    output reg        flag_c_out,
    output reg  [1:0] garbage_out
);
    wire [3:0] add_sum;
    wire       add_carry;
    wire       add_carry_dbg;
    wire [3:0] xor_y;
    wire [3:0] and_y;
    wire       cmp_eq;
    wire       cmp_gt;
    wire       cmp_lt;
    wire [3:0] swap_y;
    wire       parity_bit;
    wire       hamming_bit;
    wire [3:0] shift_y;
    wire       shift_out_bit;

    rqpu_adder4 u_add (
        .a(acc_in),
        .b(operand_in),
        .sum(add_sum),
        .carry_out(add_carry),
        .carry_dbg(add_carry_dbg)
    );

    rqpu_xor4 u_xor (
        .a(acc_in),
        .b(operand_in),
        .y(xor_y)
    );

    rqpu_and4 u_and (
        .a(acc_in),
        .b(operand_in),
        .y(and_y)
    );

    rqpu_compare4 u_compare (
        .a(acc_in),
        .b(operand_in),
        .eq(cmp_eq),
        .gt(cmp_gt),
        .lt(cmp_lt)
    );

    rqpu_swap_halves4 u_swap (
        .control(operand_in[0]),
        .a(acc_in),
        .y(swap_y)
    );

    rqpu_parity4 u_parity (
        .data_a(acc_in),
        .data_b(operand_in),
        .parity_bit(parity_bit),
        .hamming_bit(hamming_bit)
    );

    rqpu_rotate4 u_rotate (
        .dir_right(operand_in[0]),
        .a(acc_in),
        .y(shift_y),
        .shifted_out(shift_out_bit)
    );

    always @* begin
        acc_out     = acc_in;
        result_out  = acc_in;
        flag_z_out  = (acc_in == 4'b0000);
        flag_c_out  = 1'b0;
        garbage_out = 2'b00;

        case (opcode)
            3'b000: begin
                acc_out     = add_sum;
                result_out  = add_sum;
                flag_z_out  = (add_sum == 4'b0000);
                flag_c_out  = add_carry;
                garbage_out = {add_carry_dbg, add_carry};
            end

            3'b001: begin
                acc_out     = xor_y;
                result_out  = xor_y;
                flag_z_out  = (xor_y == 4'b0000);
                flag_c_out  = 1'b0;
                garbage_out = operand_in[1:0];
            end

            3'b010: begin
                acc_out     = and_y;
                result_out  = and_y;
                flag_z_out  = (and_y == 4'b0000);
                flag_c_out  = 1'b0;
                garbage_out = and_y[1:0];
            end

            3'b011: begin
                acc_out     = acc_in;
                result_out  = {1'b0, cmp_gt, cmp_eq, cmp_lt};
                flag_z_out  = cmp_eq;
                flag_c_out  = cmp_gt;
                garbage_out = {cmp_gt, cmp_lt};
            end

            3'b100: begin
                acc_out     = swap_y;
                result_out  = swap_y;
                flag_z_out  = (swap_y == 4'b0000);
                flag_c_out  = operand_in[0];
                garbage_out = acc_in[3:2];
            end

            3'b101: begin
                acc_out     = acc_in;
                result_out  = {2'b00, hamming_bit, parity_bit};
                flag_z_out  = ~parity_bit;
                flag_c_out  = hamming_bit;
                garbage_out = operand_in[1:0];
            end

            3'b110: begin
                acc_out     = shift_y;
                result_out  = shift_y;
                flag_z_out  = (shift_y == 4'b0000);
                flag_c_out  = shift_out_bit;
                garbage_out = {operand_in[0], shift_out_bit};
            end

            default: begin
                acc_out     = acc_in;
                result_out  = acc_in;
                flag_z_out  = (acc_in == 4'b0000);
                flag_c_out  = 1'b0;
                garbage_out = 2'b00;
            end
        endcase
    end
endmodule

// ============================================================================
// Top-level Tiny Tapeout module
// ============================================================================

module tt_um_kazan_rqpu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high)
    input  wire       ena,      // Design enable
    input  wire       clk,      // Clock
    input  wire       rst_n     // Active-low reset
);
    localparam [2:0] OP_REVERSE = 3'b111;

    // ui_in mapping:
    // ui_in[2:0] = opcode
    // ui_in[6:3] = 4-bit operand
    // ui_in[7]   = execute/load control (1=execute, 0=load)
    wire [2:0] opcode  = ui_in[2:0];
    wire [3:0] operand = ui_in[6:3];
    wire       execute = ui_in[7];

    reg [3:0] acc_q;
    reg [3:0] result_q;
    reg       flag_z_q;
    reg       flag_c_q;
    reg [1:0] garbage_q;

    reg [3:0] prev_acc_q;
    reg [3:0] prev_result_q;
    reg       prev_flag_z_q;
    reg       prev_flag_c_q;
    reg [1:0] prev_garbage_q;

    wire [3:0] acc_d;
    wire [3:0] result_d;
    wire       flag_z_d;
    wire       flag_c_d;
    wire [1:0] garbage_d;

    rqpu_core u_core (
        .opcode(opcode),
        .acc_in(acc_q),
        .operand_in(operand),
        .acc_out(acc_d),
        .result_out(result_d),
        .flag_z_out(flag_z_d),
        .flag_c_out(flag_c_d),
        .garbage_out(garbage_d)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_q          <= 4'b0000;
            result_q       <= 4'b0000;
            flag_z_q       <= 1'b0;
            flag_c_q       <= 1'b0;
            garbage_q      <= 2'b00;
            prev_acc_q     <= 4'b0000;
            prev_result_q  <= 4'b0000;
            prev_flag_z_q  <= 1'b0;
            prev_flag_c_q  <= 1'b0;
            prev_garbage_q <= 2'b00;
        end else if (ena) begin
            if (!execute) begin
                // LOAD mode: snapshot current state, then load the accumulator.
                prev_acc_q     <= acc_q;
                prev_result_q  <= result_q;
                prev_flag_z_q  <= flag_z_q;
                prev_flag_c_q  <= flag_c_q;
                prev_garbage_q <= garbage_q;

                acc_q          <= operand;
                result_q       <= operand;
                flag_z_q       <= (operand == 4'b0000);
                flag_c_q       <= 1'b0;
                garbage_q      <= 2'b00;
            end else if (opcode == OP_REVERSE) begin
                // REVERSE mode: swap current and previous architectural state.
                acc_q          <= prev_acc_q;
                result_q       <= prev_result_q;
                flag_z_q       <= prev_flag_z_q;
                flag_c_q       <= prev_flag_c_q;
                garbage_q      <= prev_garbage_q;

                prev_acc_q     <= acc_q;
                prev_result_q  <= result_q;
                prev_flag_z_q  <= flag_z_q;
                prev_flag_c_q  <= flag_c_q;
                prev_garbage_q <= garbage_q;
            end else begin
                // Forward execution modes.
                prev_acc_q     <= acc_q;
                prev_result_q  <= result_q;
                prev_flag_z_q  <= flag_z_q;
                prev_flag_c_q  <= flag_c_q;
                prev_garbage_q <= garbage_q;

                acc_q          <= acc_d;
                result_q       <= result_d;
                flag_z_q       <= flag_z_d;
                flag_c_q       <= flag_c_d;
                garbage_q      <= garbage_d;
            end
        end
    end

    // uo_out mapping:
    // uo_out[3:0] = result nibble
    // uo_out[4]   = flag_z (zero / equal)
    // uo_out[5]   = flag_c (carry / greater-than in compare mode)
    // uo_out[7:6] = observable garbage/debug bits
    assign uo_out[3:0] = result_q;
    assign uo_out[4]   = flag_z_q;
    assign uo_out[5]   = flag_c_q;
    assign uo_out[7:6] = garbage_q;

    // No bidirectional GPIO is used in this version.
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // Prevent unused-input warnings.
    wire _unused = &{1'b0, uio_in};
endmodule

`default_nettype wire
