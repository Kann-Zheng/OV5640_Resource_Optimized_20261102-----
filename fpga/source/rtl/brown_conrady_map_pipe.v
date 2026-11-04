`timescale 1ns/1ps
// Pipelined inverse Brown-Conrady mapper. Optional stages are used by the
// 125 MHz affine frame path; the legacy raster mapper keeps its original delay.
// Arithmetic and Q formats match brown_conrady_map.v; valid_out is delayed by
// seven rising edges. The pipeline is intended for one output coordinate per
// pixel clock and keeps the multiplier depth bounded for the 720p pixel clock.
module brown_conrady_map_pipe #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    // The full-frame engine runs from the 125 MHz DDR system clock. Split the
    // final focal-length multiply from the source-coordinate add/clip stage
    // there; the legacy raster path retains its original seven-cycle latency.
    parameter PIPELINE_FINAL_SCALE = 1'b0,
    // Split the final 32x32 focal scale into a registered partial-product
    // stage and a recombination stage for the 125 MHz affine ROI prepass.
    parameter PIPELINE_FINAL_SCALE_SPLIT = 1'b0,
    // The affine frame path inserts a register after x^2+y^2 so the square
    // multiplier and radius adder do not share one 125 MHz timing stage.
    parameter PIPELINE_RADIAL_SUM = 1'b0,
    // Separate the radial multiply from the tangential-term addition in the
    // 125 MHz affine frame path.
    parameter PIPELINE_RADIAL_PRODUCT = 1'b0,
    // Register center deltas before the focal-normalization multiply. On the
    // optional affine path Q16.16 coordinates are exact Q14.14 values shifted
    // left two bits; using Q14.14 preserves the result and saves multiplier bits.
    parameter PIPELINE_NORMALIZATION = 1'b0,
    // Use registered Pango multiplier outputs for the affine frame mapper.
    // This preserves the full Q8.24 normalized-coordinate precision.
    parameter PIPELINE_SQUARE = 1'b0,
    parameter PIPELINE_R4 = 1'b0,
    // Register the wide k2*r^4 product and delay its peer terms to keep the
    // affine ROI prepass multiplier depth within the 125 MHz budget.
    parameter PIPELINE_K2_PRODUCT = 1'b0,
    // Register the full-precision tangential p1*(x*y) product on affine map paths.
    parameter PIPELINE_P1XY_PRODUCT = 1'b0,
    // Register the other p1 tangential product as well; the default legacy
    // mapper remains combinational here, while the 125 MHz ROI path opts in.
    parameter PIPELINE_P1Y_PRODUCT = 1'b0,
    parameter PIPELINE_P2XY_PRODUCT = 1'b0,
    parameter PIPELINE_P2X_PRODUCT = 1'b0
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire bypass,
    input wire coord_subpixel_enable,
    input wire [11:0] x_out,
    input wire [11:0] y_out,
    input wire signed [31:0] x_out_q16,
    input wire signed [31:0] y_out_q16,
    input wire [11:0] cx,
    input wire [11:0] cy,
    input wire signed [31:0] fx_q16,
    input wire signed [31:0] fy_q16,
    input wire signed [31:0] inv_fx_q30,
    input wire signed [31:0] inv_fy_q30,
    input wire signed [31:0] k1_q24,
    input wire signed [31:0] k2_q24,
    input wire signed [31:0] p1_q24,
    input wire signed [31:0] p2_q24,
    output reg valid_out = 1'b0,
    output reg signed [31:0] src_x_q16 = 0,
    output reg signed [31:0] src_y_q16 = 0,
    output reg inside_image = 1'b0
);
    localparam signed [31:0] X_MAX_Q16 = (IMAGE_WIDTH - 1) << 16;
    localparam signed [31:0] Y_MAX_Q16 = (IMAGE_HEIGHT - 1) << 16;

    reg [31:0] valid_pipe = 0;
    reg [31:0] bypass_pipe = 0;
    reg signed [31:0] bypass_x_q16_pipe [0:31];
    reg signed [31:0] bypass_y_q16_pipe [0:31];
    integer bypass_index;

    // Stage 0: center subtraction and pixel-to-normalized Q24 conversion.
    wire signed [12:0] dx_in = $signed({1'b0, x_out}) - $signed({1'b0, cx});
    wire signed [12:0] dy_in = $signed({1'b0, y_out}) - $signed({1'b0, cy});
    wire signed [31:0] x_coord_q16 = coord_subpixel_enable ? x_out_q16 : {4'b0, x_out, 16'b0};
    wire signed [31:0] y_coord_q16 = coord_subpixel_enable ? y_out_q16 : {4'b0, y_out, 16'b0};
    wire signed [31:0] cx_coord_q16 = {4'b0, cx, 16'b0};
    wire signed [31:0] cy_coord_q16 = {4'b0, cy, 16'b0};
    wire signed [31:0] dx_coord_q16 = x_coord_q16 - cx_coord_q16;
    wire signed [31:0] dy_coord_q16 = y_coord_q16 - cy_coord_q16;
    wire signed [31:0] x_coord_q14 = x_coord_q16 >>> 2;
    wire signed [31:0] y_coord_q14 = y_coord_q16 >>> 2;
    wire signed [31:0] cx_coord_q14 = {6'b0, cx, 14'b0};
    wire signed [31:0] cy_coord_q14 = {6'b0, cy, 14'b0};
    wire signed [31:0] dx_coord_q14 = x_coord_q14 - cx_coord_q14;
    wire signed [31:0] dy_coord_q14 = y_coord_q14 - cy_coord_q14;
    wire signed [63:0] x_norm_product = dx_coord_q16 * inv_fx_q30;
    wire signed [63:0] y_norm_product = dy_coord_q16 * inv_fy_q30;
    // Split the unsigned 24-bit inverse focal operand into two 12-bit pieces.
    // Register both 27x12 products, then add them one cycle later. The affine
    // Q14 delta is bounded by 3996 px at 1280x720 and fits signed 27 bits.
    wire [25:0] dx_q14_lo_x = {12'b0, dx_coord_q14[13:0]};
    wire [25:0] dx_q14_lo_y = {12'b0, dy_coord_q14[13:0]};
    wire signed [12:0] dx_q14_hi_x = dx_coord_q14[26:14];
    wire signed [12:0] dx_q14_hi_y = dy_coord_q14[26:14];
    wire signed [12:0] inv_fx_lo_signed = $signed({1'b0, inv_fx_q30[11:0]});
    wire signed [12:0] inv_fx_hi_signed = $signed({1'b0, inv_fx_q30[23:12]});
    wire signed [12:0] inv_fy_lo_signed = $signed({1'b0, inv_fy_q30[11:0]});
    wire signed [12:0] inv_fy_hi_signed = $signed({1'b0, inv_fy_q30[23:12]});
    wire signed [24:0] dx_q14_hi_x_ext = {{12{dx_q14_hi_x[12]}}, dx_q14_hi_x};
    wire signed [24:0] dx_q14_hi_y_ext = {{12{dx_q14_hi_y[12]}}, dx_q14_hi_y};
    wire signed [24:0] inv_fx_lo_ext = {{12{inv_fx_lo_signed[12]}}, inv_fx_lo_signed};
    wire signed [24:0] inv_fx_hi_ext = {{12{inv_fx_hi_signed[12]}}, inv_fx_hi_signed};
    wire signed [24:0] inv_fy_lo_ext = {{12{inv_fy_lo_signed[12]}}, inv_fy_lo_signed};
    wire signed [24:0] inv_fy_hi_ext = {{12{inv_fy_hi_signed[12]}}, inv_fy_hi_signed};
    wire [25:0] x_ll_wire = dx_q14_lo_x * inv_fx_q30[11:0];
    wire [25:0] x_lh_wire = dx_q14_lo_x * inv_fx_q30[23:12];
    wire signed [24:0] x_hl_wire = dx_q14_hi_x_ext * inv_fx_lo_ext;
    wire signed [24:0] x_hh_wire = dx_q14_hi_x_ext * inv_fx_hi_ext;
    wire [25:0] y_ll_wire = dx_q14_lo_y * inv_fy_q30[11:0];
    wire [25:0] y_lh_wire = dx_q14_lo_y * inv_fy_q30[23:12];
    wire signed [24:0] y_hl_wire = dx_q14_hi_y_ext * inv_fy_lo_ext;
    wire signed [24:0] y_hh_wire = dx_q14_hi_y_ext * inv_fy_hi_ext;
    reg [25:0] x_norm_ll_0, x_norm_lh_0, y_norm_ll_0, y_norm_lh_0;
    reg signed [24:0] x_norm_hl_0, x_norm_hh_0, y_norm_hl_0, y_norm_hh_0;
    reg [37:0] x_norm_low_term_1, y_norm_low_term_1;
    reg signed [36:0] x_norm_high_term_1, y_norm_high_term_1;
    wire [37:0] x_norm_low_term_wire = {x_norm_lh_0, 12'b0} + {12'b0, x_norm_ll_0};
    wire [37:0] y_norm_low_term_wire = {y_norm_lh_0, 12'b0} + {12'b0, y_norm_ll_0};
    wire signed [36:0] x_norm_high_term_wire = {x_norm_hh_0, 12'b0} +
        {{12{x_norm_hl_0[24]}}, x_norm_hl_0};
    wire signed [36:0] y_norm_high_term_wire = {y_norm_hh_0, 12'b0} +
        {{12{y_norm_hl_0[24]}}, y_norm_hl_0};
    wire signed [50:0] x_norm_product_two_stage = {x_norm_high_term_1, 14'b0} +
        {13'b0, x_norm_low_term_1};
    wire signed [50:0] y_norm_product_two_stage = {y_norm_high_term_1, 14'b0} +
        {13'b0, y_norm_low_term_1};
    wire signed [38:0] dx_norm_q14_ext = {{12{dx_coord_q14[26]}}, dx_coord_q14[26:0]};
    wire signed [38:0] dy_norm_q14_ext = {{12{dy_coord_q14[26]}}, dy_coord_q14[26:0]};
    wire signed [38:0] inv_fx_lo_13 = {26'b0, 1'b0, inv_fx_q30[11:0]};
    wire signed [38:0] inv_fx_hi_13 = {26'b0, 1'b0, inv_fx_q30[23:12]};
    wire signed [38:0] inv_fy_lo_13 = {26'b0, 1'b0, inv_fy_q30[11:0]};
    wire signed [38:0] inv_fy_hi_13 = {26'b0, 1'b0, inv_fy_q30[23:12]};
    wire signed [38:0] x_norm_low_product_wire = dx_norm_q14_ext * inv_fx_lo_13;
    wire signed [38:0] x_norm_high_product_wire = dx_norm_q14_ext * inv_fx_hi_13;
    wire signed [38:0] y_norm_low_product_wire = dy_norm_q14_ext * inv_fy_lo_13;
    wire signed [38:0] y_norm_high_product_wire = dy_norm_q14_ext * inv_fy_hi_13;
    reg signed [38:0] x_norm_low_product_0, x_norm_high_product_0;
    reg signed [38:0] y_norm_low_product_0, y_norm_high_product_0;
    wire signed [50:0] x_norm_product_one_stage = {x_norm_high_product_0, 12'b0} +
        {{12{x_norm_low_product_0[38]}}, x_norm_low_product_0};
    wire signed [50:0] y_norm_product_one_stage = {y_norm_high_product_0, 12'b0} +
        {{12{y_norm_low_product_0[38]}}, y_norm_low_product_0};
    wire signed [50:0] x_norm_product_split = x_norm_product_one_stage;
    wire signed [50:0] y_norm_product_split = y_norm_product_one_stage;
    reg signed [31:0] x_norm_0, y_norm_0;
    // Keep local registered copies for each hard-multiplier input. The
    // normalized coordinate bus also feeds tuple alignment and later stages;
    // sharing that high-fanout bus with the APM inputs caused long routed
    // setup paths on the affine prepass.
    reg signed [31:0] x_norm_square_input /* synthesis syn_keep = 1 */;
    reg signed [31:0] y_norm_square_input /* synthesis syn_keep = 1 */;
    reg signed [31:0] x_norm_xy_input /* synthesis syn_keep = 1 */;
    reg signed [31:0] y_norm_xy_input /* synthesis syn_keep = 1 */;
    reg signed [31:0] fx_0, fy_0, k1_0, k2_0, p1_0, p2_0;
    reg [11:0] cx_0, cy_0;
    // Keep the calibration tuple aligned with normalized coordinates when
    // normalization itself spans one or two extra register stages.
    reg signed [31:0] fx_norm_d1, fy_norm_d1, k1_norm_d1, k2_norm_d1;
    reg signed [31:0] p1_norm_d1, p2_norm_d1;
    reg [11:0] cx_norm_d1, cy_norm_d1;
    reg signed [31:0] fx_norm_d2, fy_norm_d2, k1_norm_d2, k2_norm_d2;
    reg signed [31:0] p1_norm_d2, p2_norm_d2;
    reg [11:0] cx_norm_d2, cy_norm_d2;
    wire signed [31:0] fx_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? fx_norm_d2 :
        (PIPELINE_NORMALIZATION ? fx_norm_d1 : fx_q16);
    wire signed [31:0] fy_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? fy_norm_d2 :
        (PIPELINE_NORMALIZATION ? fy_norm_d1 : fy_q16);
    wire signed [31:0] k1_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? k1_norm_d2 :
        (PIPELINE_NORMALIZATION ? k1_norm_d1 : k1_q24);
    wire signed [31:0] k2_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? k2_norm_d2 :
        (PIPELINE_NORMALIZATION ? k2_norm_d1 : k2_q24);
    wire signed [31:0] p1_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? p1_norm_d2 :
        (PIPELINE_NORMALIZATION ? p1_norm_d1 : p1_q24);
    wire signed [31:0] p2_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? p2_norm_d2 :
        (PIPELINE_NORMALIZATION ? p2_norm_d1 : p2_q24);
    wire [11:0] cx_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? cx_norm_d2 :
        (PIPELINE_NORMALIZATION ? cx_norm_d1 : cx);
    wire [11:0] cy_norm_aligned = (PIPELINE_NORMALIZATION == 2) ? cy_norm_d2 :
        (PIPELINE_NORMALIZATION ? cy_norm_d1 : cy);


    // Stage 1: x^2, y^2, xy, and r^2. The optional affine path uses the
    // registered 36x18 Pango multipliers and a registered partial-product
    // recombination stage; the legacy mapper keeps its original operators.
    wire signed [63:0] x2_product_0, y2_product_0, xy_product_0;
    generate
        if (PIPELINE_SQUARE) begin : gen_registered_square_products
            brown_signed_square32_pipe x2_multiplier (
                .clk(clk), .rst_n(rst_n), .a(x_norm_square_input),
                .product_out(x2_product_0)
            );
            brown_signed_square32_pipe y2_multiplier (
                .clk(clk), .rst_n(rst_n), .a(y_norm_square_input),
                .product_out(y2_product_0)
            );
            brown_signed_mul32x32_pipe xy_multiplier (
                .clk(clk), .rst_n(rst_n), .a(x_norm_xy_input), .b(y_norm_xy_input),
                .product_out(xy_product_0)
            );
        end else begin : gen_comb_square_products
            assign x2_product_0 = x_norm_0 * x_norm_0;
            assign y2_product_0 = y_norm_0 * y_norm_0;
            assign xy_product_0 = x_norm_0 * y_norm_0;
        end
    endgenerate
    wire signed [31:0] x2_value_0 = x2_product_0 >>> 24;
    wire signed [31:0] y2_value_0 = y2_product_0 >>> 24;
    wire signed [31:0] xy_value_0 = xy_product_0 >>> 24;
    reg signed [31:0] x_norm_1, y_norm_1, x2_1, y2_1, xy_1, r2_1;
    reg signed [31:0] fx_1, fy_1, k1_1, k2_1, p1_1, p2_1;
    reg [11:0] cx_1, cy_1;
    // The split-square pipeline takes one extra cycle versus the generic
    // multiplier; delay the matching stage-zero tuple by four registers.
    reg [279:0] square_tuple_d1 = 0, square_tuple_d2 = 0;
    reg [279:0] square_tuple_d3 = 0, square_tuple_d4 = 0;

    // Stage 2: r^4 and tangential polynomial terms.
    wire signed [63:0] r4_product_1 = r2_1 * r2_1;
    wire signed [31:0] r4_value_1 = r4_product_1 >>> 24;
    reg signed [31:0] x_norm_2, y_norm_2, x2_2, y2_2, xy_2, r2_2, r4_2;
    reg signed [31:0] r2_x_2, r2_y_2;
    reg signed [31:0] fx_2, fy_2, k1_2, k2_2, p1_2, p2_2;
    reg [11:0] cx_2, cy_2;
    reg signed [31:0] r2_3, r4_3, r2_x_3, r2_y_3;
    reg signed [31:0] x2_3, y2_3, xy_3;
    reg signed [31:0] fx_3, fy_3, k1_3, k2_3, p1_3, p2_3;
    reg [11:0] cx_3, cy_3;

    // Stage 3: coefficient products for radial and tangential distortion.
    wire signed [63:0] k1_product_2 = k1_2 * r2_2;
    wire signed [63:0] k2_product_2 = k2_2 * r4_2;
    wire signed [63:0] p1_xy_product_2 = p1_2 * xy_2;
    wire signed [63:0] p2_x_product_2 = p2_2 * r2_x_2;
    wire signed [63:0] p1_y_product_2 = p1_2 * r2_y_2;
    wire signed [63:0] p2_xy_product_2 = p2_2 * xy_2;
    reg signed [31:0] x_norm_3, y_norm_3;
    reg signed [31:0] k1_term_3, k2_term_3;
    reg signed [31:0] p1_xy_twice_3, p2_x_3, p1_y_3, p2_xy_twice_3;
    reg signed [31:0] k1_term_4, k2_term_4;
    reg signed [31:0] p1_xy_twice_4, p2_x_4, p1_y_4, p2_xy_twice_4;
    reg signed [31:0] k1_term_4_d1, k1_term_4_d2;
    reg signed [31:0] k2_term_4_d1, k2_term_4_d2;
    reg signed [31:0] p1_xy_twice_4_d1, p1_xy_twice_4_d2;
    reg signed [31:0] p2_x_4_d1, p2_x_4_d2;
    reg signed [31:0] p1_y_4_d1, p1_y_4_d2;
    reg signed [31:0] p2_xy_twice_4_d1, p2_xy_twice_4_d2;
    reg signed [31:0] x_norm_4_d1, x_norm_4_d2, y_norm_4_d1, y_norm_4_d2;
    reg signed [31:0] fx_4_d1, fx_4_d2, fy_4_d1, fy_4_d2;
    reg [11:0] cx_4_d1, cx_4_d2, cy_4_d1, cy_4_d2;

    // Stage 4: combine radial and tangential terms.
    reg signed [31:0] x_norm_4, y_norm_4;
    reg signed [31:0] fx_4, fy_4;
    reg [11:0] cx_4, cy_4;
    reg signed [31:0] radial_4, tangential_x_4, tangential_y_4;
    reg signed [31:0] x_norm_5, y_norm_5;
    reg signed [31:0] fx_6, fy_6;
    reg [11:0] cx_6, cy_6;
    reg signed [31:0] radial_5, tangential_x_5, tangential_y_5;
    reg signed [31:0] x_distorted_6, y_distorted_6;

    // Stage 5: apply radial scale and tangential offsets.
    wire signed [63:0] x_radial_product_4 = x_norm_4 * radial_4;
    wire signed [63:0] y_radial_product_4 = y_norm_4 * radial_4;
    reg signed [31:0] x_distorted_5, y_distorted_5;
    reg signed [31:0] fx_5, fy_5;
    reg [11:0] cx_5, cy_5;

    // Stage 6: convert back to source pixels and mark the image boundary.
    wire signed [63:0] x_pixel_product_5 = x_distorted_5 * fx_5;
    wire signed [63:0] y_pixel_product_5 = y_distorted_5 * fy_5;
    wire signed [31:0] cx_q16_5 = {4'b0000, cx_5, 16'b0};
    wire signed [31:0] cy_q16_5 = {4'b0000, cy_5, 16'b0};
    wire signed [31:0] src_x_next = (x_pixel_product_5 >>> 24) + cx_q16_5;
    wire signed [31:0] src_y_next = (y_pixel_product_5 >>> 24) + cy_q16_5;
    reg signed [63:0] x_pixel_product_6 = 0, y_pixel_product_6 = 0;
    reg signed [31:0] cx_q16_6 = 0, cy_q16_6 = 0;
    wire signed [31:0] src_x_split_next = (x_pixel_product_6 >>> 24) + cx_q16_6;
    wire signed [31:0] src_y_split_next = (y_pixel_product_6 >>> 24) + cy_q16_6;
    wire signed [63:0] r4_product_2;
    wire signed [63:0] r4_product_2_pipe;
    generate
        if (PIPELINE_R4) begin : gen_registered_r4_product
            brown_signed_mul32x32_pipe r4_multiplier (
                .clk(clk), .rst_n(rst_n), .a(r2_2), .b(r2_2),
                .product_out(r4_product_2_pipe)
            );
            assign r4_product_2 = r4_product_2_pipe;
        end else begin : gen_comb_r4_product
            assign r4_product_2 = r2_2 * r2_2;
        end
    endgenerate
    wire signed [31:0] r4_value_2 = r4_product_2 >>> 24;
    reg [407:0] r4_tuple_d1 = 0, r4_tuple_d2 = 0;
    wire signed [63:0] k1_product_3 = k1_3 * r2_3;
    wire signed [63:0] k2_product_3;
    wire signed [63:0] k2_product_3_pipe;
    generate
        if (PIPELINE_K2_PRODUCT) begin : gen_registered_k2_product
            brown_signed_mul32x32_pipe k2_multiplier (
                .clk(clk), .rst_n(rst_n), .a(k2_3), .b(r4_3),
                .product_out(k2_product_3_pipe)
            );
            assign k2_product_3 = k2_product_3_pipe;
        end else begin : gen_comb_k2_product
            assign k2_product_3 = k2_3 * r4_3;
            assign k2_product_3_pipe = 64'sd0;
        end
    endgenerate
    wire signed [63:0] p1_xy_product_3;
    wire signed [63:0] p1_xy_product_3_pipe;
    generate
        if (PIPELINE_P1XY_PRODUCT) begin : gen_registered_p1xy_product
            brown_signed_mul32x32_pipe p1xy_multiplier (
                .clk(clk), .rst_n(rst_n), .a(p1_3), .b(xy_3),
                .product_out(p1_xy_product_3_pipe)
            );
            assign p1_xy_product_3 = p1_xy_product_3_pipe;
        end else begin : gen_comb_p1xy_product
            assign p1_xy_product_3 = p1_3 * xy_3;
            assign p1_xy_product_3_pipe = 64'sd0;
        end
    endgenerate
    wire signed [63:0] p2_x_product_3;
    wire signed [63:0] p2_x_product_3_pipe;
    generate
        if (PIPELINE_P2X_PRODUCT) begin : gen_registered_p2x_product
            brown_signed_mul32x32_pipe p2x_multiplier (
                .clk(clk), .rst_n(rst_n), .a(p2_3), .b(r2_x_3),
                .product_out(p2_x_product_3_pipe)
            );
            assign p2_x_product_3 = p2_x_product_3_pipe;
        end else begin : gen_comb_p2x_product
            assign p2_x_product_3 = p2_3 * r2_x_3;
            assign p2_x_product_3_pipe = 64'sd0;
        end
    endgenerate
    wire signed [63:0] p1_y_product_3;
    wire signed [63:0] p1_y_product_3_pipe;
    generate
        if (PIPELINE_P1Y_PRODUCT) begin : gen_registered_p1y_product
            brown_signed_mul32x32_pipe p1y_multiplier (
                .clk(clk), .rst_n(rst_n), .a(p1_3), .b(r2_y_3),
                .product_out(p1_y_product_3_pipe)
            );
            assign p1_y_product_3 = p1_y_product_3_pipe;
        end else begin : gen_comb_p1y_product
            assign p1_y_product_3 = p1_3 * r2_y_3;
            assign p1_y_product_3_pipe = 64'sd0;
        end
    endgenerate
    wire signed [63:0] p2_xy_product_3;
    wire signed [63:0] p2_xy_product_3_pipe;
    generate
        if (PIPELINE_P2XY_PRODUCT) begin : gen_registered_p2xy_product
            brown_signed_mul32x32_pipe p2xy_multiplier (
                .clk(clk), .rst_n(rst_n), .a(p2_3), .b(xy_3),
                .product_out(p2_xy_product_3_pipe)
            );
            assign p2_xy_product_3 = p2_xy_product_3_pipe;
        end else begin : gen_comb_p2xy_product
            assign p2_xy_product_3 = p2_3 * xy_3;
            assign p2_xy_product_3_pipe = 64'sd0;
        end
    endgenerate
    reg signed [31:0] x_distorted_7, y_distorted_7;
    reg signed [31:0] fx_7, fy_7;
    reg [11:0] cx_7, cy_7;
    // Split the radial multiply across the existing product and sum stages.
    reg signed [47:0] x_radial_lo;
    reg signed [39:0] x_radial_hi;
    wire [55:0] x_radial_product_7 = {x_radial_hi, 16'b0} + {{8{x_radial_lo[47]}}, x_radial_lo};
    reg signed [47:0] y_radial_lo;
    reg signed [39:0] y_radial_hi;
    wire [55:0] y_radial_product_7 = {y_radial_hi, 16'b0} + {{8{y_radial_lo[47]}}, y_radial_lo};
    reg signed [31:0] tangential_x_6, tangential_y_6;
    reg signed [63:0] x_pixel_product_8 = 0, y_pixel_product_8 = 0;
    reg signed [31:0] cx_q16_8 = 0, cy_q16_8 = 0;
    reg signed [31:0] cx_q16_9 = 0, cy_q16_9 = 0;
    wire signed [63:0] x_pixel_product_9;
    wire signed [63:0] y_pixel_product_9;
    reg signed [63:0] x_pixel_product_7 = 0, y_pixel_product_7 = 0;
    reg signed [31:0] cx_q16_7 = 0, cy_q16_7 = 0;
    wire signed [63:0] x_radial_product_6 = x_distorted_6 * fx_6;
    wire signed [63:0] y_radial_product_6 = y_distorted_6 * fy_6;
    wire signed [63:0] x_radial_product_5 = x_norm_5 * radial_5;
    wire signed [63:0] y_radial_product_5 = y_norm_5 * radial_5;
    wire signed [31:0] cx_q16_6_next = {4'b0000, cx_6, 16'b0};
    wire signed [31:0] cy_q16_6_next = {4'b0000, cy_6, 16'b0};
    wire signed [31:0] src_x_radial_next = (x_radial_product_6 >>> 24) + cx_q16_6_next;
    wire signed [31:0] src_y_radial_next = (y_radial_product_6 >>> 24) + cy_q16_6_next;
    wire signed [31:0] src_x_split_radial_next = (x_pixel_product_7 >>> 24) + cx_q16_7;
    wire signed [31:0] src_y_split_radial_next = (y_pixel_product_7 >>> 24) + cy_q16_7;
    wire signed [63:0] x_pixel_product_7_wire = x_distorted_7 * fx_7;
    wire signed [63:0] y_pixel_product_7_wire = y_distorted_7 * fy_7;
    wire signed [31:0] cx_q16_7_next = {4'b0000, cx_7, 16'b0};
    wire signed [31:0] cy_q16_7_next = {4'b0000, cy_7, 16'b0};
    wire signed [31:0] src_x_radial_product_next = (x_pixel_product_7_wire >>> 24) + cx_q16_7_next;
    wire signed [31:0] src_y_radial_product_next = (y_pixel_product_7_wire >>> 24) + cy_q16_7_next;
    wire signed [31:0] src_x_split_radial_product_next = (x_pixel_product_8 >>> 24) + cx_q16_8;
    wire signed [31:0] src_y_split_radial_product_next = (y_pixel_product_8 >>> 24) + cy_q16_8;
    wire signed [31:0] src_x_split_radial_product_2_next = (x_pixel_product_9 >>> 24) + cx_q16_9;
    wire signed [31:0] src_y_split_radial_product_2_next = (y_pixel_product_9 >>> 24) + cy_q16_9;
    generate
        if (PIPELINE_FINAL_SCALE_SPLIT) begin : gen_split_final_scale
            brown_signed_mul32x32_pipe x_final_scale (
                .clk(clk), .rst_n(rst_n), .a(x_distorted_7), .b(fx_7),
                .product_out(x_pixel_product_9)
            );
            brown_signed_mul32x32_pipe y_final_scale (
                .clk(clk), .rst_n(rst_n), .a(y_distorted_7), .b(fy_7),
                .product_out(y_pixel_product_9)
            );
        end else begin : gen_unsplit_final_scale
            assign x_pixel_product_9 = 64'sd0;
            assign y_pixel_product_9 = 64'sd0;
        end
    endgenerate
    // OUTPUT_STAGE_INDEX counts the additional cycles after the base pipeline.
    // The split square itself has three registered stages relative to the
    // combinational square path; the fourth tuple register aligns its side
    // data and is not an additional output-data cycle. The optional registered
    // r^4 path contributes two extra output cycles, including its tuple aligner.
    localparam OUTPUT_STAGE_INDEX = 5 + PIPELINE_NORMALIZATION + PIPELINE_FINAL_SCALE +
        PIPELINE_FINAL_SCALE_SPLIT +
        PIPELINE_RADIAL_SUM + PIPELINE_RADIAL_PRODUCT + (PIPELINE_SQUARE ? 3 : 0) +
        (PIPELINE_R4 ? 2 : 0) + ((PIPELINE_K2_PRODUCT || PIPELINE_P1XY_PRODUCT || PIPELINE_P1Y_PRODUCT || PIPELINE_P2XY_PRODUCT || PIPELINE_P2X_PRODUCT) ? 2 : 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_pipe <= 0;
            bypass_pipe <= 0;
            for (bypass_index = 0; bypass_index < 32; bypass_index = bypass_index + 1) begin
                bypass_x_q16_pipe[bypass_index] <= 0;
                bypass_y_q16_pipe[bypass_index] <= 0;
            end
            x_norm_ll_0 <= 0; x_norm_lh_0 <= 0;
            y_norm_ll_0 <= 0; y_norm_lh_0 <= 0;
            x_norm_hl_0 <= 0; x_norm_hh_0 <= 0;
            y_norm_hl_0 <= 0; y_norm_hh_0 <= 0;
            x_norm_square_input <= 0; y_norm_square_input <= 0;
            x_norm_xy_input <= 0; y_norm_xy_input <= 0;
            x_norm_low_term_1 <= 0; y_norm_low_term_1 <= 0;
            x_norm_high_term_1 <= 0; y_norm_high_term_1 <= 0;
            x_norm_low_product_0 <= 0; x_norm_high_product_0 <= 0;
            y_norm_low_product_0 <= 0; y_norm_high_product_0 <= 0;
            fx_norm_d1 <= 0; fy_norm_d1 <= 0;
            k1_norm_d1 <= 0; k2_norm_d1 <= 0;
            p1_norm_d1 <= 0; p2_norm_d1 <= 0;
            cx_norm_d1 <= 0; cy_norm_d1 <= 0;
            fx_norm_d2 <= 0; fy_norm_d2 <= 0;
            k1_norm_d2 <= 0; k2_norm_d2 <= 0;
            p1_norm_d2 <= 0; p2_norm_d2 <= 0;
            cx_norm_d2 <= 0; cy_norm_d2 <= 0;
            square_tuple_d1 <= 0;
            square_tuple_d2 <= 0;
            square_tuple_d3 <= 0;
            square_tuple_d4 <= 0;
            r4_tuple_d1 <= 0;
            r4_tuple_d2 <= 0;
            k1_term_4_d1 <= 0; k1_term_4_d2 <= 0;
            k2_term_4_d1 <= 0; k2_term_4_d2 <= 0;
            p1_xy_twice_4_d1 <= 0; p1_xy_twice_4_d2 <= 0;
            p2_x_4_d1 <= 0; p2_x_4_d2 <= 0;
            p1_y_4_d1 <= 0; p1_y_4_d2 <= 0;
            p2_xy_twice_4_d1 <= 0; p2_xy_twice_4_d2 <= 0;
            x_norm_4_d1 <= 0; x_norm_4_d2 <= 0;
            y_norm_4_d1 <= 0; y_norm_4_d2 <= 0;
            fx_4_d1 <= 0; fx_4_d2 <= 0; fy_4_d1 <= 0; fy_4_d2 <= 0;
            cx_4_d1 <= 0; cx_4_d2 <= 0; cy_4_d1 <= 0; cy_4_d2 <= 0;

            x_pixel_product_6 <= 0;
            y_pixel_product_6 <= 0;
            cx_q16_6 <= 0;
            cy_q16_6 <= 0;
            x_pixel_product_7 <= 0;
            y_pixel_product_7 <= 0;
            cx_q16_7 <= 0;
            cy_q16_7 <= 0;
            x_pixel_product_8 <= 0;
            y_pixel_product_8 <= 0;
            cx_q16_8 <= 0;
            cy_q16_8 <= 0;
            cx_q16_9 <= 0;
            cy_q16_9 <= 0;
            valid_out <= 1'b0;
            src_x_q16 <= 0;
            src_y_q16 <= 0;
            inside_image <= 1'b0;
        end else begin
            fx_norm_d1 <= fx_q16; fy_norm_d1 <= fy_q16;
            k1_norm_d1 <= k1_q24; k2_norm_d1 <= k2_q24;
            p1_norm_d1 <= p1_q24; p2_norm_d1 <= p2_q24;
            cx_norm_d1 <= cx; cy_norm_d1 <= cy;
            fx_norm_d2 <= fx_norm_d1; fy_norm_d2 <= fy_norm_d1;
            k1_norm_d2 <= k1_norm_d1; k2_norm_d2 <= k2_norm_d1;
            p1_norm_d2 <= p1_norm_d1; p2_norm_d2 <= p2_norm_d1;
            cx_norm_d2 <= cx_norm_d1; cy_norm_d2 <= cy_norm_d1;
            valid_pipe <= {valid_pipe[30:0], valid_in};
            bypass_pipe <= {bypass_pipe[30:0], bypass};
            bypass_x_q16_pipe[0] <= x_coord_q16;
            bypass_y_q16_pipe[0] <= y_coord_q16;
            for (bypass_index = 1; bypass_index < 32; bypass_index = bypass_index + 1) begin
                bypass_x_q16_pipe[bypass_index] <= bypass_x_q16_pipe[bypass_index-1];
                bypass_y_q16_pipe[bypass_index] <= bypass_y_q16_pipe[bypass_index-1];
            end
            if (PIPELINE_K2_PRODUCT || PIPELINE_P1XY_PRODUCT || PIPELINE_P1Y_PRODUCT || PIPELINE_P2XY_PRODUCT || PIPELINE_P2X_PRODUCT) begin
                k1_term_4_d1 <= k1_term_4; k1_term_4_d2 <= k1_term_4_d1;
                k2_term_4_d1 <= k2_term_4; k2_term_4_d2 <= k2_term_4_d1;
                p1_xy_twice_4_d1 <= p1_xy_twice_4; p1_xy_twice_4_d2 <= p1_xy_twice_4_d1;
                p2_x_4_d1 <= p2_x_4; p2_x_4_d2 <= p2_x_4_d1;
                p1_y_4_d1 <= p1_y_4; p1_y_4_d2 <= p1_y_4_d1;
                p2_xy_twice_4_d1 <= p2_xy_twice_4; p2_xy_twice_4_d2 <= p2_xy_twice_4_d1;
                x_norm_4_d1 <= x_norm_4; x_norm_4_d2 <= x_norm_4_d1;
                y_norm_4_d1 <= y_norm_4; y_norm_4_d2 <= y_norm_4_d1;
                fx_4_d1 <= fx_4; fx_4_d2 <= fx_4_d1;
                fy_4_d1 <= fy_4; fy_4_d2 <= fy_4_d1;
                cx_4_d1 <= cx_4; cx_4_d2 <= cx_4_d1;
                cy_4_d1 <= cy_4; cy_4_d2 <= cy_4_d1;
                if (PIPELINE_RADIAL_SUM && PIPELINE_K2_PRODUCT)
                    k2_term_4 <= k2_product_3_pipe >>> 24;
            end
            square_tuple_d1 <= {cy_0, cx_0, p2_0, p1_0, k2_0, k1_0,
                                fy_0, fx_0, y_norm_0, x_norm_0};
            square_tuple_d2 <= square_tuple_d1;
            square_tuple_d3 <= square_tuple_d2;
            square_tuple_d4 <= square_tuple_d3;
            r4_tuple_d1 <= {cy_2, cx_2, p2_2, p1_2, k2_2, k1_2,
                            fy_2, fx_2, xy_2, y2_2, x2_2, r2_2,
                            y_norm_2, x_norm_2};
            r4_tuple_d2 <= r4_tuple_d1;

            valid_out <= valid_pipe[OUTPUT_STAGE_INDEX];
            if (PIPELINE_RADIAL_SUM) begin
                if (PIPELINE_RADIAL_PRODUCT) begin
                    x_radial_lo <= x_norm_5 * $signed({1'b0, radial_5[15:0]});
                    x_radial_hi <= x_norm_5 * $signed(radial_5[31:16]);
                    y_radial_lo <= y_norm_5 * $signed({1'b0, radial_5[15:0]});
                    y_radial_hi <= y_norm_5 * $signed(radial_5[31:16]);
                    tangential_x_6 <= tangential_x_5;
                    tangential_y_6 <= tangential_y_5;
                    fx_6 <= fx_5; fy_6 <= fy_5;
                    cx_6 <= cx_5; cy_6 <= cy_5;
                    x_distorted_7 <= (x_radial_product_7 >>> 24) + tangential_x_6;
                    y_distorted_7 <= (y_radial_product_7 >>> 24) + tangential_y_6;
                    fx_7 <= fx_6; fy_7 <= fy_6;
                    cx_7 <= cx_6; cy_7 <= cy_6;
                    if (PIPELINE_FINAL_SCALE) begin
                        if (PIPELINE_FINAL_SCALE_SPLIT) begin
                            cx_q16_8 <= cx_q16_7_next;
                            cy_q16_8 <= cy_q16_7_next;
                            cx_q16_9 <= cx_q16_8;
                            cy_q16_9 <= cy_q16_8;
                            if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                                src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                                src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                                inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                                (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                                (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                                (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                            end else begin
                                src_x_q16 <= src_x_split_radial_product_2_next;
                                src_y_q16 <= src_y_split_radial_product_2_next;
                                inside_image <= (src_x_split_radial_product_2_next >= 0) &&
                                                (src_y_split_radial_product_2_next >= 0) &&
                                                (src_x_split_radial_product_2_next <= X_MAX_Q16) &&
                                                (src_y_split_radial_product_2_next <= Y_MAX_Q16);
                            end
                        end else begin
                            x_pixel_product_8 <= x_pixel_product_7_wire;
                            y_pixel_product_8 <= y_pixel_product_7_wire;
                            cx_q16_8 <= cx_q16_7_next;
                            cy_q16_8 <= cy_q16_7_next;
                            if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                                src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                                src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                                inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                                (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                                (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                                (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                            end else begin
                                src_x_q16 <= src_x_split_radial_product_next;
                                src_y_q16 <= src_y_split_radial_product_next;
                                inside_image <= (src_x_split_radial_product_next >= 0) && (src_y_split_radial_product_next >= 0) &&
                                                (src_x_split_radial_product_next <= X_MAX_Q16) &&
                                                (src_y_split_radial_product_next <= Y_MAX_Q16);
                            end
                        end
                    end else if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                        src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                        src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                        inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                        (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                        (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                        (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                    end else begin
                        src_x_q16 <= src_x_radial_product_next;
                        src_y_q16 <= src_y_radial_product_next;
                        inside_image <= (src_x_radial_product_next >= 0) && (src_y_radial_product_next >= 0) &&
                                        (src_x_radial_product_next <= X_MAX_Q16) &&
                                        (src_y_radial_product_next <= Y_MAX_Q16);
                    end
                end else if (PIPELINE_FINAL_SCALE) begin
                    x_pixel_product_7 <= x_radial_product_6;
                    y_pixel_product_7 <= y_radial_product_6;
                    cx_q16_7 <= cx_q16_6_next;
                    cy_q16_7 <= cy_q16_6_next;
                    if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                        src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                        src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                        inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                        (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                        (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                        (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                    end else begin
                        src_x_q16 <= src_x_split_radial_next;
                        src_y_q16 <= src_y_split_radial_next;
                        inside_image <= (src_x_split_radial_next >= 0) && (src_y_split_radial_next >= 0) &&
                                        (src_x_split_radial_next <= X_MAX_Q16) &&
                                        (src_y_split_radial_next <= Y_MAX_Q16);
                    end
                end else if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                    src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                    src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                    inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                end else begin
                    src_x_q16 <= src_x_radial_next;
                    src_y_q16 <= src_y_radial_next;
                    inside_image <= (src_x_radial_next >= 0) && (src_y_radial_next >= 0) &&
                                    (src_x_radial_next <= X_MAX_Q16) &&
                                    (src_y_radial_next <= Y_MAX_Q16);
                end
            end else if (PIPELINE_FINAL_SCALE) begin
                x_pixel_product_6 <= x_pixel_product_5;
                y_pixel_product_6 <= y_pixel_product_5;
                cx_q16_6 <= cx_q16_5;
                cy_q16_6 <= cy_q16_5;
                if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                    src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                    src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                    inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                end else begin
                    src_x_q16 <= src_x_split_next;
                    src_y_q16 <= src_y_split_next;
                    inside_image <= (src_x_split_next >= 0) && (src_y_split_next >= 0) &&
                                    (src_x_split_next <= X_MAX_Q16) &&
                                    (src_y_split_next <= Y_MAX_Q16);
                end
            end else begin
                if (bypass_pipe[OUTPUT_STAGE_INDEX]) begin
                    src_x_q16 <= bypass_x_q16_pipe[OUTPUT_STAGE_INDEX];
                    src_y_q16 <= bypass_y_q16_pipe[OUTPUT_STAGE_INDEX];
                    inside_image <= (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] >= 0) &&
                                    (bypass_x_q16_pipe[OUTPUT_STAGE_INDEX] <= X_MAX_Q16) &&
                                    (bypass_y_q16_pipe[OUTPUT_STAGE_INDEX] <= Y_MAX_Q16);
                end else begin
                    src_x_q16 <= src_x_next;
                    src_y_q16 <= src_y_next;
                    inside_image <= (src_x_next >= 0) && (src_y_next >= 0) &&
                                    (src_x_next <= X_MAX_Q16) &&
                                    (src_y_next <= Y_MAX_Q16);
                end
            end

            // Q14.14 delta × unsigned Q2.30 inverse focal → Q8.24.
            // Mode 1 uses two 27x12 partial products; mode 2 further splits
            // the delta into 14+13 bits, registering product groups separately.
            if (PIPELINE_NORMALIZATION == 2) begin
                x_norm_ll_0 <= x_ll_wire; x_norm_lh_0 <= x_lh_wire;
                x_norm_hl_0 <= x_hl_wire; x_norm_hh_0 <= x_hh_wire;
                y_norm_ll_0 <= y_ll_wire; y_norm_lh_0 <= y_lh_wire;
                y_norm_hl_0 <= y_hl_wire; y_norm_hh_0 <= y_hh_wire;
                x_norm_low_term_1 <= x_norm_low_term_wire;
                y_norm_low_term_1 <= y_norm_low_term_wire;
                x_norm_high_term_1 <= x_norm_high_term_wire;
                y_norm_high_term_1 <= y_norm_high_term_wire;
                x_norm_0 <= x_norm_product_two_stage >>> 20;
                y_norm_0 <= y_norm_product_two_stage >>> 20;
                x_norm_square_input <= x_norm_product_two_stage >>> 20;
                y_norm_square_input <= y_norm_product_two_stage >>> 20;
                x_norm_xy_input <= x_norm_product_two_stage >>> 20;
                y_norm_xy_input <= y_norm_product_two_stage >>> 20;
            end else if (PIPELINE_NORMALIZATION) begin
                x_norm_low_product_0 <= x_norm_low_product_wire;
                x_norm_high_product_0 <= x_norm_high_product_wire;
                y_norm_low_product_0 <= y_norm_low_product_wire;
                y_norm_high_product_0 <= y_norm_high_product_wire;
                x_norm_0 <= x_norm_product_one_stage >>> 20;
                y_norm_0 <= y_norm_product_one_stage >>> 20;
                x_norm_square_input <= x_norm_product_one_stage >>> 20;
                y_norm_square_input <= y_norm_product_one_stage >>> 20;
                x_norm_xy_input <= x_norm_product_one_stage >>> 20;
                y_norm_xy_input <= y_norm_product_one_stage >>> 20;
            end else begin
                x_norm_0 <= x_norm_product >>> 22;
                y_norm_0 <= y_norm_product >>> 22;
                x_norm_square_input <= x_norm_product >>> 22;
                y_norm_square_input <= y_norm_product >>> 22;
                x_norm_xy_input <= x_norm_product >>> 22;
                y_norm_xy_input <= y_norm_product >>> 22;
            end
            fx_0 <= fx_norm_aligned; fy_0 <= fy_norm_aligned;
            k1_0 <= k1_norm_aligned; k2_0 <= k2_norm_aligned;
            p1_0 <= p1_norm_aligned; p2_0 <= p2_norm_aligned;
            cx_0 <= cx_norm_aligned; cy_0 <= cy_norm_aligned;

            x_norm_1 <= x_norm_0; y_norm_1 <= y_norm_0;
            x2_1 <= x2_value_0; y2_1 <= y2_value_0;
            xy_1 <= xy_value_0;
            if (PIPELINE_RADIAL_SUM)
                r2_1 <= 0;
            else
                r2_1 <= x2_value_0 + y2_value_0;
            fx_1 <= fx_0; fy_1 <= fy_0;
            k1_1 <= k1_0; k2_1 <= k2_0;
            p1_1 <= p1_0; p2_1 <= p2_0;
            cx_1 <= cx_0; cy_1 <= cy_0;

            if (PIPELINE_SQUARE) begin
                x_norm_2 <= square_tuple_d4[31:0];
                y_norm_2 <= square_tuple_d4[63:32];
                fx_2 <= square_tuple_d4[95:64];
                fy_2 <= square_tuple_d4[127:96];
                k1_2 <= square_tuple_d4[159:128];
                k2_2 <= square_tuple_d4[191:160];
                p1_2 <= square_tuple_d4[223:192];
                p2_2 <= square_tuple_d4[255:224];
                cx_2 <= square_tuple_d4[267:256];
                cy_2 <= square_tuple_d4[279:268];
            end else begin
                x_norm_2 <= x_norm_1; y_norm_2 <= y_norm_1;
                fx_2 <= fx_1; fy_2 <= fy_1;
                k1_2 <= k1_1; k2_2 <= k2_1;
                p1_2 <= p1_1; p2_2 <= p2_1;
                cx_2 <= cx_1; cy_2 <= cy_1;
            end
            x2_2 <= x2_1; y2_2 <= y2_1; xy_2 <= xy_1;
            if (PIPELINE_RADIAL_SUM) begin
                r2_2 <= x2_1 + y2_1;
                r4_2 <= 0;
                r2_x_2 <= 0;
                r2_y_2 <= 0;
            end else begin
                r2_2 <= r2_1; r4_2 <= r4_value_1;
                r2_x_2 <= r2_1 + (x2_1 <<< 1);
                r2_y_2 <= r2_1 + (y2_1 <<< 1);
            end
            x_norm_3 <= x_norm_2; y_norm_3 <= y_norm_2;
            fx_3 <= fx_2; fy_3 <= fy_2;
            cx_3 <= cx_2; cy_3 <= cy_2;
            if (PIPELINE_RADIAL_SUM) begin
                r4_3 <= r4_value_2;
                if (PIPELINE_R4) begin
                    x_norm_3 <= r4_tuple_d2[31:0];
                    y_norm_3 <= r4_tuple_d2[63:32];
                    r2_3 <= r4_tuple_d2[95:64];
                    x2_3 <= r4_tuple_d2[127:96];
                    y2_3 <= r4_tuple_d2[159:128];
                    xy_3 <= r4_tuple_d2[191:160];
                    fx_3 <= r4_tuple_d2[223:192];
                    fy_3 <= r4_tuple_d2[255:224];
                    k1_3 <= r4_tuple_d2[287:256];
                    k2_3 <= r4_tuple_d2[319:288];
                    p1_3 <= r4_tuple_d2[351:320];
                    p2_3 <= r4_tuple_d2[383:352];
                    cx_3 <= r4_tuple_d2[395:384];
                    cy_3 <= r4_tuple_d2[407:396];
                    r2_x_3 <= r4_tuple_d2[95:64] + (r4_tuple_d2[127:96] <<< 1);
                    r2_y_3 <= r4_tuple_d2[95:64] + (r4_tuple_d2[159:128] <<< 1);
                end else begin
                    x_norm_3 <= x_norm_2; y_norm_3 <= y_norm_2;
                    r2_3 <= r2_2;
                    r2_x_3 <= r2_2 + (x2_2 <<< 1);
                    r2_y_3 <= r2_2 + (y2_2 <<< 1);
                    x2_3 <= x2_2; y2_3 <= y2_2; xy_3 <= xy_2;
                    k1_3 <= k1_2; k2_3 <= k2_2; p1_3 <= p1_2; p2_3 <= p2_2;
                    fx_3 <= fx_2; fy_3 <= fy_2;
                    cx_3 <= cx_2; cy_3 <= cy_2;
                end
            end else begin
                k1_term_3 <= k1_product_2 >>> 24;
                k2_term_3 <= k2_product_2 >>> 24;
                p1_xy_twice_3 <= p1_xy_product_2 >>> 23;
                p2_x_3 <= p2_x_product_2 >>> 24;
                p1_y_3 <= p1_y_product_2 >>> 24;
                p2_xy_twice_3 <= p2_xy_product_2 >>> 23;
            end

            x_norm_4 <= x_norm_3; y_norm_4 <= y_norm_3;
            fx_4 <= fx_3; fy_4 <= fy_3;
            cx_4 <= cx_3; cy_4 <= cy_3;
            if (PIPELINE_RADIAL_SUM) begin
                k1_term_4 <= k1_product_3 >>> 24;
                if (!PIPELINE_K2_PRODUCT)
                    k2_term_4 <= k2_product_3 >>> 24;
                p1_xy_twice_4 <= p1_xy_product_3 >>> 23;
                p2_x_4 <= p2_x_product_3 >>> 24;
                p1_y_4 <= p1_y_product_3 >>> 24;
                p2_xy_twice_4 <= p2_xy_product_3 >>> 23;
            end else begin
                radial_4 <= 32'sh01000000 + k1_term_3 + k2_term_3;
                tangential_x_4 <= p1_xy_twice_3 + p2_x_3;
                tangential_y_4 <= p1_y_3 + p2_xy_twice_3;
            end

            if (PIPELINE_RADIAL_SUM) begin
                if (PIPELINE_K2_PRODUCT || PIPELINE_P1XY_PRODUCT || PIPELINE_P1Y_PRODUCT || PIPELINE_P2XY_PRODUCT || PIPELINE_P2X_PRODUCT) begin
                    radial_5 <= 32'sh01000000 + k1_term_4_d2 +
                        (PIPELINE_K2_PRODUCT ? k2_term_4 : k2_term_4_d2);
                    tangential_x_5 <= (PIPELINE_P1XY_PRODUCT ? p1_xy_twice_4 : p1_xy_twice_4_d2) +
                        (PIPELINE_P2X_PRODUCT ? p2_x_4 : p2_x_4_d2);
                    tangential_y_5 <= (PIPELINE_P1Y_PRODUCT ? p1_y_4 : p1_y_4_d2) +
                        (PIPELINE_P2XY_PRODUCT ? p2_xy_twice_4 : p2_xy_twice_4_d2);
                    x_norm_5 <= x_norm_4_d2; y_norm_5 <= y_norm_4_d2;
                    fx_5 <= fx_4_d2; fy_5 <= fy_4_d2;
                    cx_5 <= cx_4_d2; cy_5 <= cy_4_d2;
                end else begin
                    radial_5 <= 32'sh01000000 + k1_term_4 + k2_term_4;
                    tangential_x_5 <= p1_xy_twice_4 + p2_x_4;
                    tangential_y_5 <= p1_y_4 + p2_xy_twice_4;
                    x_norm_5 <= x_norm_4; y_norm_5 <= y_norm_4;
                    fx_5 <= fx_4; fy_5 <= fy_4;
                    cx_5 <= cx_4; cy_5 <= cy_4;
                end
                x_distorted_6 <= (x_radial_product_5 >>> 24) + tangential_x_5;
                y_distorted_6 <= (y_radial_product_5 >>> 24) + tangential_y_5;
                fx_6 <= fx_5; fy_6 <= fy_5;
                cx_6 <= cx_5; cy_6 <= cy_5;
            end else begin
                x_distorted_5 <= (x_radial_product_4 >>> 24) + tangential_x_4;
                y_distorted_5 <= (y_radial_product_4 >>> 24) + tangential_y_4;
                fx_5 <= fx_4; fy_5 <= fy_4;
                cx_5 <= cx_4; cy_5 <= cy_4;
            end

        end
    end
endmodule

// Full-precision signed square reconstructed from three 16x16 products.
// The carry-save stage is explicit bit logic so the mapper cannot turn the
// three aligned terms into one long cascaded APM multiply-accumulate chain.
module brown_signed_square32_pipe (
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] a,
    output reg signed [63:0] product_out = 0
);
    wire [15:0] low = a[15:0];
    wire signed [15:0] high = a[31:16];
    wire [31:0] low_square_comb = low * low;
    wire signed [31:0] high_square_comb = high * high;
    wire signed [32:0] cross_product_wide = $signed({1'b0, low}) * $signed(high);
    wire signed [31:0] cross_comb = cross_product_wide[31:0];

    reg [31:0] low_square_s1 = 0;
    reg signed [31:0] cross_s1 = 0, high_square_s1 = 0;
    wire [63:0] term_low = {32'b0, low_square_s1};
    wire signed [63:0] term_cross = {{32{cross_s1[31]}}, cross_s1} <<< 17;
    wire signed [63:0] term_high = {{32{high_square_s1[31]}}, high_square_s1} <<< 32;
    wire [63:0] csa_sum = term_low ^ term_cross ^ term_high;
    wire [63:0] csa_carry = ((term_low & term_cross) |
                             (term_low & term_high) |
                             (term_cross & term_high)) << 1;
    reg [63:0] sum_s2 = 0, carry_s2 = 0;
    wire [63:0] product_s3 = sum_s2 + carry_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            low_square_s1 <= 0;
            cross_s1 <= 0;
            high_square_s1 <= 0;
            sum_s2 <= 0;
            carry_s2 <= 0;
            product_out <= 0;
        end else begin
            low_square_s1 <= low_square_comb;
            cross_s1 <= cross_comb;
            high_square_s1 <= high_square_comb;
            sum_s2 <= csa_sum;
            carry_s2 <= csa_carry;
            product_out <= product_s3;
        end
    end
endmodule

// Full-precision signed 32x32 multiply split across the Pango APM output
// register and one fabric recombination register. The low 18-bit multiplier
// operand is unsigned; the upper 14-bit slice is sign-extended.
module brown_signed_mul32x32_pipe (
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] a,
    input wire signed [31:0] b,
    output reg signed [63:0] product_out = 0
);
    wire [35:0] a_ext = {{4{a[31]}}, a};
    wire [17:0] b_low = b[17:0];
    wire [17:0] b_high = {{4{b[31]}}, b[31:18]};
    wire [53:0] product_low;
    wire [53:0] product_high;

    brown_apm_mul36x18_low low_partial (
        .ce(1'b1), .rst(~rst_n), .clk(clk), .a(a_ext), .b(b_low), .p(product_low)
    );
    brown_apm_mul36x18_high high_partial (
        .ce(1'b1), .rst(~rst_n), .clk(clk), .a(a_ext), .b(b_high), .p(product_high)
    );

    wire signed [63:0] low_extended = {{10{product_low[53]}}, product_low};
    wire signed [63:0] high_extended = {{10{product_high[53]}}, product_high};
    wire signed [63:0] recombined_product = low_extended + (high_extended <<< 18);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            product_out <= 0;
        else
            product_out <= recombined_product;
    end
endmodule



