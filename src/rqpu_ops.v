/*
 * Reversible-logic-inspired operator blocks for the Tiny Tapeout RQPU demo.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

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

`default_nettype wire
