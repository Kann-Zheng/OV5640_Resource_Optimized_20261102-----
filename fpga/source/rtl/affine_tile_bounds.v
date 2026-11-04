`timescale 1ns/1ps
// One-cycle tile source-window estimator for an output-to-source affine map.
// The four transformed tile corners bound every affine sample in the tile.
// The resulting source rectangle is expanded by one pixel on its far edge
// for bilinear interpolation and clipped to the source frame. This is a
// building block for the full-frame/tile DDR reader; it is not connected to
// the current sequential 16-row display cache.
module affine_tile_bounds #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire [11:0] tile_x,
    input wire [11:0] tile_y,
    input wire [5:0] tile_width,
    input wire [5:0] tile_height,
    input wire [11:0] center_x,
    input wire [11:0] center_y,
    input wire signed [15:0] a_q14,
    input wire signed [15:0] b_q14,
    input wire signed [15:0] c_q14,
    input wire signed [15:0] d_q14,
    output reg valid_out = 1'b0,
    output reg has_intersection = 1'b0,
    output reg [11:0] src_x_min = 0,
    output reg [11:0] src_x_max = 0,
    output reg [11:0] src_y_min = 0,
    output reg [11:0] src_y_max = 0
);
    localparam signed [31:0] X_MAX = IMAGE_WIDTH - 1;
    localparam signed [31:0] Y_MAX = IMAGE_HEIGHT - 1;

    wire [12:0] tile_x_end_unclipped = {1'b0, tile_x} + tile_width - 1'b1;
    wire [12:0] tile_y_end_unclipped = {1'b0, tile_y} + tile_height - 1'b1;
    wire [11:0] tile_x_end = (tile_x_end_unclipped >= IMAGE_WIDTH) ? X_MAX[11:0] : tile_x_end_unclipped[11:0];
    wire [11:0] tile_y_end = (tile_y_end_unclipped >= IMAGE_HEIGHT) ? Y_MAX[11:0] : tile_y_end_unclipped[11:0];

    wire signed [13:0] dx0 = $signed({2'b00, tile_x}) - $signed({2'b00, center_x});
    wire signed [13:0] dx1 = $signed({2'b00, tile_x_end}) - $signed({2'b00, center_x});
    wire signed [13:0] dy0 = $signed({2'b00, tile_y}) - $signed({2'b00, center_y});
    wire signed [13:0] dy1 = $signed({2'b00, tile_y_end}) - $signed({2'b00, center_y});

    // Reuse the four x-offset and four y-offset products across all corners.
    wire signed [29:0] dx0_a = dx0 * a_q14;
    wire signed [29:0] dx1_a = dx1 * a_q14;
    wire signed [29:0] dx0_c = dx0 * c_q14;
    wire signed [29:0] dx1_c = dx1 * c_q14;
    wire signed [29:0] dy0_b = dy0 * b_q14;
    wire signed [29:0] dy1_b = dy1 * b_q14;
    wire signed [29:0] dy0_d = dy0 * d_q14;
    wire signed [29:0] dy1_d = dy1 * d_q14;

    wire signed [31:0] cx_q14 = $signed({6'b0, center_x, 14'b0});
    wire signed [31:0] cy_q14 = $signed({6'b0, center_y, 14'b0});
    wire signed [31:0] sx00 = $signed(dx0_a) + $signed(dy0_b) + cx_q14;
    wire signed [31:0] sx01 = $signed(dx0_a) + $signed(dy1_b) + cx_q14;
    wire signed [31:0] sx10 = $signed(dx1_a) + $signed(dy0_b) + cx_q14;
    wire signed [31:0] sx11 = $signed(dx1_a) + $signed(dy1_b) + cx_q14;
    wire signed [31:0] sy00 = $signed(dx0_c) + $signed(dy0_d) + cy_q14;
    wire signed [31:0] sy01 = $signed(dx0_c) + $signed(dy1_d) + cy_q14;
    wire signed [31:0] sy10 = $signed(dx1_c) + $signed(dy0_d) + cy_q14;
    wire signed [31:0] sy11 = $signed(dx1_c) + $signed(dy1_d) + cy_q14;

    reg signed [31:0] min_x_q14, max_x_q14, min_y_q14, max_y_q14;
    reg signed [31:0] x0_floor, x1_floor, y0_floor, y1_floor;
    reg signed [31:0] x0_clip, x1_clip, y0_clip, y1_clip;
    reg intersect_comb;
    always @* begin
        min_x_q14 = sx00; max_x_q14 = sx00;
        min_y_q14 = sy00; max_y_q14 = sy00;
        if (sx01 < min_x_q14) min_x_q14 = sx01;
        if (sx10 < min_x_q14) min_x_q14 = sx10;
        if (sx11 < min_x_q14) min_x_q14 = sx11;
        if (sx01 > max_x_q14) max_x_q14 = sx01;
        if (sx10 > max_x_q14) max_x_q14 = sx10;
        if (sx11 > max_x_q14) max_x_q14 = sx11;
        if (sy01 < min_y_q14) min_y_q14 = sy01;
        if (sy10 < min_y_q14) min_y_q14 = sy10;
        if (sy11 < min_y_q14) min_y_q14 = sy11;
        if (sy01 > max_y_q14) max_y_q14 = sy01;
        if (sy10 > max_y_q14) max_y_q14 = sy10;
        if (sy11 > max_y_q14) max_y_q14 = sy11;

        // Arithmetic shift implements floor for signed Q14 coordinates.
        x0_floor = min_x_q14 >>> 14;
        x1_floor = (max_x_q14 >>> 14) + 1;
        y0_floor = min_y_q14 >>> 14;
        y1_floor = (max_y_q14 >>> 14) + 1;
        intersect_comb = valid_in && (tile_width != 0) && (tile_height != 0) &&
                         (tile_x < IMAGE_WIDTH) && (tile_y < IMAGE_HEIGHT) &&
                         (max_x_q14 >= 0) && (min_x_q14 <= (X_MAX <<< 14)) &&
                         (max_y_q14 >= 0) && (min_y_q14 <= (Y_MAX <<< 14));

        x0_clip = (x0_floor < 0) ? 0 : x0_floor;
        y0_clip = (y0_floor < 0) ? 0 : y0_floor;
        x1_clip = (x1_floor > X_MAX) ? X_MAX : x1_floor;
        y1_clip = (y1_floor > Y_MAX) ? Y_MAX : y1_floor;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            has_intersection <= 1'b0;
            src_x_min <= 0; src_x_max <= 0;
            src_y_min <= 0; src_y_max <= 0;
        end else begin
            valid_out <= valid_in;
            has_intersection <= intersect_comb;
            if (intersect_comb) begin
                src_x_min <= x0_clip[11:0];
                src_x_max <= x1_clip[11:0];
                src_y_min <= y0_clip[11:0];
                src_y_max <= y1_clip[11:0];
            end else begin
                src_x_min <= 0; src_x_max <= 0;
                src_y_min <= 0; src_y_max <= 0;
            end
        end
    end
endmodule
