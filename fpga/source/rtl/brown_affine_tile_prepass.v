`timescale 1ns/1ps
// Maps the selected prepass samples for one output tile and reports a source
// ROI before DDR read. The engine can scan every point or use conservative
// affine corners / margin-expanded Brown samples for throughput.
module brown_affine_tile_prepass #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter MAP_LATENCY = 24,
    parameter BOUND_MARGIN = 0
)(
    input wire clk,
    input wire rst_n,
    input wire scan_start,
    input wire valid_in,
    input wire last_in,
    input wire brown_bypass,
    input wire include_outside_bounds,
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
    output wire bbox_valid,
    output wire has_intersection,
    output wire [11:0] src_x_min,
    output wire [11:0] src_x_max,
    output wire [11:0] src_y_min,
    output wire [11:0] src_y_max,
    output wire error_pulse
);
    wire mapped_valid;
    wire mapped_inside;
    wire signed [31:0] mapped_x_q16;
    wire signed [31:0] mapped_y_q16;

    reg [MAP_LATENCY-1:0] last_pipe = {MAP_LATENCY{1'b0}};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            last_pipe <= {MAP_LATENCY{1'b0}};
        else
            last_pipe <= {last_pipe[MAP_LATENCY-2:0], last_in};
    end

    brown_affine_inverse_map_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .PIPELINE_FINAL_SCALE(1'b1), .PIPELINE_FINAL_SCALE_SPLIT(1'b1),
        .PIPELINE_RADIAL_SUM(1'b1),
        .PIPELINE_RADIAL_PRODUCT(1'b1), .PIPELINE_NORMALIZATION(2),
        .PIPELINE_SQUARE(1'b1), .PIPELINE_R4(1'b1), .PIPELINE_K2_PRODUCT(1'b1),
        .PIPELINE_P1XY_PRODUCT(1'b1), .PIPELINE_P1Y_PRODUCT(1'b1),
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

    affine_mapped_tile_bounds #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT), .BOUND_MARGIN(BOUND_MARGIN)
    ) source_roi (
        .clk(clk), .rst_n(rst_n), .scan_start(scan_start),
        .include_outside_bounds(include_outside_bounds),
        .mapped_valid(mapped_valid), .mapped_inside(mapped_inside),
        .src_x_q16(mapped_x_q16), .src_y_q16(mapped_y_q16), .scan_last(last_pipe[MAP_LATENCY-1]),
        .bbox_valid(bbox_valid), .has_intersection(has_intersection),
        .src_x_min(src_x_min), .src_x_max(src_x_max),
        .src_y_min(src_y_min), .src_y_max(src_y_max), .error_pulse(error_pulse)
    );
endmodule
