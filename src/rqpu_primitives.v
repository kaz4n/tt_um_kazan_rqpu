/*
 * Reversible-logic-inspired primitive library for the Tiny Tapeout RQPU demo.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

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

`default_nettype wire
