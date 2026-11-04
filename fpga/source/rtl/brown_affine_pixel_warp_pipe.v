`timescale 1ns/1ps
// Pixel-level composition of Q2.14 affine mapping, subpixel Brown-Conrady
// mapping, tile-cache four-neighbor reads, and RGB565 bilinear interpolation.
// DDR tile loading and output-frame writing remain outside this module.
module brown_affine_pixel_warp_pipe #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720
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
    output wire cache_rd_en,
    output wire [11:0] cache_x0,
    output wire [11:0] cache_y0,
    output wire [11:0] cache_x1,
    output wire [11:0] cache_y1,
    input wire cache_rd_valid,
    input wire [15:0] cache_p00,
    input wire [15:0] cache_p10,
    input wire [15:0] cache_p01,
    input wire [15:0] cache_p11,
    output wire pixel_valid,
    output wire [15:0] pixel_out,
    output wire cache_miss
);
    wire mapped_valid;
    wire mapped_inside;
    wire signed [31:0] mapped_x_q16;
    wire signed [31:0] mapped_y_q16;

    brown_affine_inverse_map_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .PIPELINE_FINAL_SCALE(1'b1), .PIPELINE_RADIAL_SUM(1'b1),
        .PIPELINE_RADIAL_PRODUCT(1'b1), .PIPELINE_NORMALIZATION(2),
        .PIPELINE_SQUARE(1'b1), .PIPELINE_P1XY_PRODUCT(1'b1),
        .PIPELINE_P2XY_PRODUCT(1'b1),
        .PIPELINE_P2X_PRODUCT(1'b1)
    ) coordinate_map (
        .clk(clk), .rst_n(rst_n), .valid_in(valid_in), .brown_bypass(brown_bypass),
        .x_out(x_out), .y_out(y_out), .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .fx_q16(fx_q16), .fy_q16(fy_q16), .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .valid_out(mapped_valid), .src_x_q16(mapped_x_q16), .src_y_q16(mapped_y_q16),
        .inside_image(mapped_inside)
    );

    brown_affine_tile_sampler #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT)
    ) source_sampler (
        .clk(clk), .rst_n(rst_n), .coord_valid(mapped_valid), .coord_inside(mapped_inside),
        .src_x_q16(mapped_x_q16), .src_y_q16(mapped_y_q16),
        .cache_rd_en(cache_rd_en), .cache_x0(cache_x0), .cache_y0(cache_y0),
        .cache_x1(cache_x1), .cache_y1(cache_y1), .cache_rd_valid(cache_rd_valid),
        .cache_p00(cache_p00), .cache_p10(cache_p10), .cache_p01(cache_p01), .cache_p11(cache_p11),
        .pixel_valid(pixel_valid), .pixel_out(pixel_out), .cache_miss(cache_miss)
    );
endmodule
