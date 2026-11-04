// Inverse Brown-Conrady coordinate mapper for undistort sampling.
// Pixel coordinates are integer. fx/fy are Q16.16 pixels; inverse focal lengths
// and k1/k2/p1/p2 are signed Q8.24. Source coordinates are signed Q16.16 pixels.
module brown_conrady_map #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720
)(
    input  wire [11:0] x_out,
    input  wire [11:0] y_out,
    input  wire [11:0] cx,
    input  wire [11:0] cy,
    input  wire signed [31:0] fx_q16,
    input  wire signed [31:0] fy_q16,
    input  wire signed [31:0] inv_fx_q30,
    input  wire signed [31:0] inv_fy_q30,
    input  wire signed [31:0] k1_q24,
    input  wire signed [31:0] k2_q24,
    input  wire signed [31:0] p1_q24,
    input  wire signed [31:0] p2_q24,
    output wire signed [31:0] src_x_q16,
    output wire signed [31:0] src_y_q16,
    output wire inside_image
);
    wire signed [12:0] dx = $signed({1'b0, x_out}) - $signed({1'b0, cx});
    wire signed [12:0] dy = $signed({1'b0, y_out}) - $signed({1'b0, cy});
    wire signed [63:0] x_norm_product = dx * inv_fx_q30;
    wire signed [63:0] y_norm_product = dy * inv_fy_q30;
    wire signed [31:0] x_norm_q24 = x_norm_product >>> 6;
    wire signed [31:0] y_norm_q24 = y_norm_product >>> 6;

    wire signed [63:0] x2_product = x_norm_q24 * x_norm_q24;
    wire signed [63:0] y2_product = y_norm_q24 * y_norm_q24;
    wire signed [31:0] x2_q24 = x2_product >>> 24;
    wire signed [31:0] y2_q24 = y2_product >>> 24;
    wire signed [31:0] r2_q24 = x2_q24 + y2_q24;
    wire signed [63:0] r4_product = r2_q24 * r2_q24;
    wire signed [31:0] r4_q24 = r4_product >>> 24;
    wire signed [63:0] xy_product = x_norm_q24 * y_norm_q24;
    wire signed [31:0] xy_q24 = xy_product >>> 24;

    wire signed [63:0] k1_product = k1_q24 * r2_q24;
    wire signed [63:0] k2_product = k2_q24 * r4_q24;
    wire signed [31:0] radial_q24 = 32'sh01000000 +
                                      (k1_product >>> 24) + (k2_product >>> 24);
    wire signed [31:0] r2_plus_2x2_q24 = r2_q24 + (x2_q24 <<< 1);
    wire signed [31:0] r2_plus_2y2_q24 = r2_q24 + (y2_q24 <<< 1);
    wire signed [63:0] p1_xy_product = p1_q24 * xy_q24;
    wire signed [63:0] p2_x_product = p2_q24 * r2_plus_2x2_q24;
    wire signed [63:0] p1_y_product = p1_q24 * r2_plus_2y2_q24;
    wire signed [63:0] p2_xy_product = p2_q24 * xy_q24;
    wire signed [31:0] tangential_x_q24 = (p1_xy_product >>> 23) +
                                           (p2_x_product >>> 24);
    wire signed [31:0] tangential_y_q24 = (p1_y_product >>> 24) +
                                           (p2_xy_product >>> 23);
    wire signed [63:0] x_radial_product = x_norm_q24 * radial_q24;
    wire signed [63:0] y_radial_product = y_norm_q24 * radial_q24;
    wire signed [31:0] x_distorted_q24 = (x_radial_product >>> 24) + tangential_x_q24;
    wire signed [31:0] y_distorted_q24 = (y_radial_product >>> 24) + tangential_y_q24;
    wire signed [63:0] x_pixel_product = x_distorted_q24 * fx_q16;
    wire signed [63:0] y_pixel_product = y_distorted_q24 * fy_q16;
    wire signed [31:0] cx_q16 = {4'b0000, cx, 16'b0};
    wire signed [31:0] cy_q16 = {4'b0000, cy, 16'b0};
    wire signed [31:0] x_max_q16 = (IMAGE_WIDTH - 1) << 16;
    wire signed [31:0] y_max_q16 = (IMAGE_HEIGHT - 1) << 16;

    assign src_x_q16 = (x_pixel_product >>> 24) + cx_q16;
    assign src_y_q16 = (y_pixel_product >>> 24) + cy_q16;
    assign inside_image = (src_x_q16 >= 0) && (src_y_q16 >= 0) &&
                          (src_x_q16 <= x_max_q16) && (src_y_q16 <= y_max_q16);
endmodule

