/*
 * RQPU: Reversible-logic-inspired 4-bit processing unit for Tiny Tapeout.
 *
 * NOTE:
 * - The top module name intentionally uses the placeholder `yourname`.
 * - Before final submission, rename it to `tt_um_<githubusername>_rqpu`
 *   and make the same replacement in info.yaml, test/Makefile and test/tb.v.
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

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
