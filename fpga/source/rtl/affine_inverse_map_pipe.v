`timescale 1ns/1ps
// Two-stage output-to-source affine coordinate mapper.
// Matrix coefficients and center arithmetic are signed Q2.14; coordinates at
// the interface are integer pixels in and signed Q16.16 pixels out. This mapper
// feeds the optional full-frame Brown+affine DDR path. The default display path
// continues to use its separate one-axis correction pipeline.
module affine_inverse_map_pipe #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire [11:0] x_out,
    input wire [11:0] y_out,
    input wire [11:0] center_x,
    input wire [11:0] center_y,
    input wire signed [15:0] a_q14,
    input wire signed [15:0] b_q14,
    input wire signed [15:0] c_q14,
    input wire signed [15:0] d_q14,
    output reg valid_out = 1'b0,
    output reg signed [31:0] src_x_q16 = 0,
    output reg signed [31:0] src_y_q16 = 0,
    output reg inside_image = 1'b0
);
    localparam signed [31:0] X_MAX_Q16 = (IMAGE_WIDTH - 1) << 16;
    localparam signed [31:0] Y_MAX_Q16 = (IMAGE_HEIGHT - 1) << 16;

    wire signed [12:0] dx = $signed({1'b0, x_out}) - $signed({1'b0, center_x});
    wire signed [12:0] dy = $signed({1'b0, y_out}) - $signed({1'b0, center_y});
    wire signed [28:0] dx_a = dx * a_q14;
    wire signed [28:0] dy_b = dy * b_q14;
    wire signed [28:0] dx_c = dx * c_q14;
    wire signed [28:0] dy_d = dy * d_q14;

    reg signed [29:0] delta_x_q14_0 = 0;
    reg signed [29:0] delta_y_q14_0 = 0;
    reg [11:0] center_x_0 = 0;
    reg [11:0] center_y_0 = 0;
    reg valid_0 = 0;

    wire signed [31:0] delta_x_q14_32 = {{2{delta_x_q14_0[29]}}, delta_x_q14_0};
    wire signed [31:0] delta_y_q14_32 = {{2{delta_y_q14_0[29]}}, delta_y_q14_0};
    wire signed [31:0] center_x_q14 = $signed({6'b0, center_x_0, 14'b0});
    wire signed [31:0] center_y_q14 = $signed({6'b0, center_y_0, 14'b0});
    wire signed [31:0] src_x_q14_next = delta_x_q14_32 + center_x_q14;
    wire signed [31:0] src_y_q14_next = delta_y_q14_32 + center_y_q14;
    wire signed [31:0] src_x_q16_next = src_x_q14_next <<< 2;
    wire signed [31:0] src_y_q16_next = src_y_q14_next <<< 2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            delta_x_q14_0 <= 0;
            delta_y_q14_0 <= 0;
            center_x_0 <= 0;
            center_y_0 <= 0;
            valid_0 <= 1'b0;
            valid_out <= 1'b0;
            src_x_q16 <= 0;
            src_y_q16 <= 0;
            inside_image <= 1'b0;
        end else begin
            delta_x_q14_0 <= $signed(dx_a) + $signed(dy_b);
            delta_y_q14_0 <= $signed(dx_c) + $signed(dy_d);
            center_x_0 <= center_x;
            center_y_0 <= center_y;
            valid_0 <= valid_in;
            valid_out <= valid_0;
            if (valid_0) begin
                src_x_q16 <= src_x_q16_next;
                src_y_q16 <= src_y_q16_next;
                inside_image <= (src_x_q16_next >= 0) && (src_y_q16_next >= 0) &&
                                (src_x_q16_next <= X_MAX_Q16) &&
                                (src_y_q16_next <= Y_MAX_Q16);
            end else begin
                inside_image <= 1'b0;
            end
        end
    end
endmodule
