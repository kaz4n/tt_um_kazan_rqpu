/*
 * Combinational opcode decoder and datapath for the Tiny Tapeout RQPU demo.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

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

`default_nettype wire
