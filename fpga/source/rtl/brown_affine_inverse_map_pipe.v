`timescale 1ns/1ps
// Subpixel coordinate composition for an output-to-input 2D affine transform
// followed by the inverse Brown-Conrady lens map. This is a coordinate stage;
// random-access pixel fetch, tile scheduling, and output-frame writing remain
// separate blocks.
module brown_affine_inverse_map_pipe #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter PIPELINE_FINAL_SCALE = 1'b0,
    parameter PIPELINE_FINAL_SCALE_SPLIT = 1'b0,
    parameter PIPELINE_RADIAL_SUM = 1'b0,
    parameter PIPELINE_RADIAL_PRODUCT = 1'b0,
    parameter PIPELINE_NORMALIZATION = 1'b0,
    parameter PIPELINE_SQUARE = 1'b0,
    parameter PIPELINE_R4 = 1'b0,
    parameter PIPELINE_K2_PRODUCT = 1'b0,
    parameter PIPELINE_P1XY_PRODUCT = 1'b0,
    parameter PIPELINE_P1Y_PRODUCT = 1'b0,
    parameter PIPELINE_P2XY_PRODUCT = 1'b0,
    parameter PIPELINE_P2X_PRODUCT = 1'b0
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire brown_bypass,
    input wire [11:0] x_out,
    input wire [11:0] y_out,
    input wire [11:0] center_x,
    input wire [11:0] center_y,
    input wire signed [15:0] a_q14,
    input wire signed [15:0] b_q14,
    input wire signed [15:0] c_q14,
    input wire signed [15:0] d_q14,
    input wire signed [31:0] fx_q16,
    input wire signed [31:0] fy_q16,
    input wire signed [31:0] inv_fx_q30,
    input wire signed [31:0] inv_fy_q30,
    input wire signed [31:0] k1_q24,
    input wire signed [31:0] k2_q24,
    input wire signed [31:0] p1_q24,
    input wire signed [31:0] p2_q24,
    output wire valid_out,
    output wire signed [31:0] src_x_q16,
    output wire signed [31:0] src_y_q16,
    output wire inside_image
);
    wire affine_valid;
    wire signed [31:0] affine_x_q16;
    wire signed [31:0] affine_y_q16;
    wire affine_inside;

    affine_inverse_map_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT)
    ) affine_map (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
        .x_out(x_out), .y_out(y_out), .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .valid_out(affine_valid), .src_x_q16(affine_x_q16),
        .src_y_q16(affine_y_q16), .inside_image(affine_inside)
    );

    wire brown_valid;
    wire signed [31:0] brown_x_q16;
    wire signed [31:0] brown_y_q16;
    wire brown_inside;

    brown_conrady_map_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .PIPELINE_FINAL_SCALE(PIPELINE_FINAL_SCALE),
        .PIPELINE_FINAL_SCALE_SPLIT(PIPELINE_FINAL_SCALE_SPLIT),
        .PIPELINE_RADIAL_SUM(PIPELINE_RADIAL_SUM),
        .PIPELINE_RADIAL_PRODUCT(PIPELINE_RADIAL_PRODUCT),
        .PIPELINE_NORMALIZATION(PIPELINE_NORMALIZATION),
        .PIPELINE_SQUARE(PIPELINE_SQUARE), .PIPELINE_R4(PIPELINE_R4),
        .PIPELINE_K2_PRODUCT(PIPELINE_K2_PRODUCT),
        .PIPELINE_P1XY_PRODUCT(PIPELINE_P1XY_PRODUCT),
        .PIPELINE_P1Y_PRODUCT(PIPELINE_P1Y_PRODUCT),
        .PIPELINE_P2XY_PRODUCT(PIPELINE_P2XY_PRODUCT),
        .PIPELINE_P2X_PRODUCT(PIPELINE_P2X_PRODUCT)
    ) brown_map (
        .clk(clk), .rst_n(rst_n), .valid_in(affine_valid), .bypass(brown_bypass),
        .coord_subpixel_enable(1'b1), .x_out(12'd0), .y_out(12'd0),
        .x_out_q16(affine_x_q16), .y_out_q16(affine_y_q16),
        .cx(center_x), .cy(center_y),
        .fx_q16(fx_q16), .fy_q16(fy_q16),
        .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .valid_out(brown_valid), .src_x_q16(brown_x_q16), .src_y_q16(brown_y_q16),
        .inside_image(brown_inside)
    );

    // Align affine bounds with the seven-stage Brown pipeline. The affine
    // mapper output is sampled by Brown one cycle after it becomes valid.
    // The top-level in-bounds flag follows the same data latency as the mapper;
    // the split square contributes three added cycles, while registered r^4
    // contributes two (including its tuple alignment stage).
    localparam INSIDE_PIPE_INDEX = 6 + PIPELINE_NORMALIZATION + PIPELINE_FINAL_SCALE +
        PIPELINE_FINAL_SCALE_SPLIT +
        PIPELINE_RADIAL_SUM + PIPELINE_RADIAL_PRODUCT + (PIPELINE_SQUARE ? 3 : 0) +
        (PIPELINE_R4 ? 2 : 0) + ((PIPELINE_K2_PRODUCT || PIPELINE_P1XY_PRODUCT || PIPELINE_P1Y_PRODUCT || PIPELINE_P2XY_PRODUCT || PIPELINE_P2X_PRODUCT) ? 2 : 0);
    reg [20:0] affine_inside_pipe = 21'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            affine_inside_pipe <= 21'b0;
        else
            affine_inside_pipe <= {affine_inside_pipe[19:0], affine_inside};
    end

    assign valid_out = brown_valid;
    assign src_x_q16 = brown_x_q16;
    assign src_y_q16 = brown_y_q16;
    assign inside_image = brown_valid && brown_inside &&
                          affine_inside_pipe[INSIDE_PIPE_INDEX];
endmodule
