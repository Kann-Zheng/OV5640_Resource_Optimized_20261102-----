`timescale 1ns/1ps
// Converts subpixel source coordinates into a cache four-neighbor request and
// RGB565 bilinear output. The cache read is synchronous. Bilinear interpolation
// is separable: horizontal weighted sums are registered first, then combined
// vertically. This keeps the 125 MHz full-frame datapath from cascading four
// 2-D weights, four color products, and their final sums in one cycle.
module brown_affine_tile_sampler #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720
)(
    input wire clk,
    input wire rst_n,
    input wire coord_valid,
    input wire coord_inside,
    input wire signed [31:0] src_x_q16,
    input wire signed [31:0] src_y_q16,
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
    localparam signed [31:0] X_MAX_Q16 = (IMAGE_WIDTH - 1) << 16;
    localparam signed [31:0] Y_MAX_Q16 = (IMAGE_HEIGHT - 1) << 16;

    wire signed [31:0] x_floor = $signed(src_x_q16) >>> 16;
    wire signed [31:0] y_floor = $signed(src_y_q16) >>> 16;
    wire source_in_bounds = (src_x_q16 >= 0) && (src_y_q16 >= 0) &&
                            (src_x_q16 <= X_MAX_Q16) && (src_y_q16 <= Y_MAX_Q16);
    wire cache_request = coord_valid && coord_inside && source_in_bounds;
    wire [11:0] x0_value = x_floor[11:0];
    wire [11:0] y0_value = y_floor[11:0];
    // Integer coordinates have zero weight on the adjacent sample. Reuse the
    // same address so the mapped ROI prepass and the cache request agree.
    wire [11:0] x1_value = ((src_x_q16[15:0] == 0) || (x0_value >= IMAGE_WIDTH - 1)) ?
                          x0_value : x0_value + 1'b1;
    wire [11:0] y1_value = ((src_y_q16[15:0] == 0) || (y0_value >= IMAGE_HEIGHT - 1)) ?
                          y0_value : y0_value + 1'b1;

    reg cache_request_q = 1'b0;
    reg [11:0] cache_x0_q = 0, cache_y0_q = 0;
    reg [11:0] cache_x1_q = 0, cache_y1_q = 0;
    reg request_valid_q = 1'b0;
    reg request_inside_q = 1'b0;
    reg [7:0] request_frac_x_q = 0, request_frac_y_q = 0;

    assign cache_rd_en = cache_request_q;
    assign cache_x0 = cache_x0_q;
    assign cache_y0 = cache_y0_q;
    assign cache_x1 = cache_x1_q;
    assign cache_y1 = cache_y1_q;

    reg inside_q = 1'b0;
    reg [7:0] frac_x_q = 0;
    reg [7:0] frac_y_q = 0;
    reg valid_q = 1'b0;
    reg [13:0] red_top_q = 0, red_bottom_q = 0;
    reg [14:0] green_top_q = 0, green_bottom_q = 0;
    reg [13:0] blue_top_q = 0, blue_bottom_q = 0;
    reg [7:0] frac_y_interp_q = 0;
    reg inside_interp_q = 1'b0;
    reg cache_valid_interp_q = 1'b0;
    reg [23:0] red_top_vertical_q = 0, red_bottom_vertical_q = 0;
    reg [23:0] green_top_vertical_q = 0, green_bottom_vertical_q = 0;
    reg [23:0] blue_top_vertical_q = 0, blue_bottom_vertical_q = 0;
    reg horizontal_valid_q = 1'b0;
    reg pixel_valid_q = 1'b0;
    reg inside_vertical_q = 1'b0;
    reg cache_valid_vertical_q = 1'b0;

    wire [8:0] wx0 = 9'd256 - {1'b0, frac_x_q};
    wire [8:0] wx1 = {1'b0, frac_x_q};
    wire [13:0] red_top_next = cache_p00[15:11] * wx0 + cache_p10[15:11] * wx1;
    wire [13:0] red_bottom_next = cache_p01[15:11] * wx0 + cache_p11[15:11] * wx1;
    wire [14:0] green_top_next = cache_p00[10:5] * wx0 + cache_p10[10:5] * wx1;
    wire [14:0] green_bottom_next = cache_p01[10:5] * wx0 + cache_p11[10:5] * wx1;
    wire [13:0] blue_top_next = cache_p00[4:0] * wx0 + cache_p10[4:0] * wx1;
    wire [13:0] blue_bottom_next = cache_p01[4:0] * wx0 + cache_p11[4:0] * wx1;
    wire [8:0] wy0 = 9'd256 - {1'b0, frac_y_interp_q};
    wire [8:0] wy1 = {1'b0, frac_y_interp_q};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cache_request_q <= 1'b0;
            cache_x0_q <= 0; cache_y0_q <= 0;
            cache_x1_q <= 0; cache_y1_q <= 0;
            request_valid_q <= 1'b0;
            request_inside_q <= 1'b0;
            request_frac_x_q <= 0; request_frac_y_q <= 0;
            inside_q <= 1'b0;
            frac_x_q <= 0;
            frac_y_q <= 0;
            valid_q <= 1'b0;
            red_top_q <= 0; red_bottom_q <= 0;
            green_top_q <= 0; green_bottom_q <= 0;
            blue_top_q <= 0; blue_bottom_q <= 0;
            frac_y_interp_q <= 0;
            inside_interp_q <= 1'b0;
            cache_valid_interp_q <= 1'b0;
            red_top_vertical_q <= 0; red_bottom_vertical_q <= 0;
            green_top_vertical_q <= 0; green_bottom_vertical_q <= 0;
            blue_top_vertical_q <= 0; blue_bottom_vertical_q <= 0;
            horizontal_valid_q <= 1'b0;
            pixel_valid_q <= 1'b0;
            inside_vertical_q <= 1'b0;
            cache_valid_vertical_q <= 1'b0;
        end else begin
            // Stage 1: register the cache address and its metadata. This
            // breaks the coordinate-mapper-to-cache-valid path while keeping
            // one request per cycle. The synchronous cache response then
            // aligns with the delayed fractions below.
            cache_request_q <= cache_request;
            cache_x0_q <= cache_request ? x0_value : 12'd0;
            cache_y0_q <= cache_request ? y0_value : 12'd0;
            cache_x1_q <= cache_request ? x1_value : 12'd0;
            cache_y1_q <= cache_request ? y1_value : 12'd0;
            request_valid_q <= coord_valid;
            request_inside_q <= cache_request;
            request_frac_x_q <= src_x_q16[15:8];
            request_frac_y_q <= src_y_q16[15:8];

            // Stage 2: synchronous cache data and coordinate fractions align.
            valid_q <= request_valid_q;
            inside_q <= request_inside_q;
            frac_x_q <= request_frac_x_q;
            frac_y_q <= request_frac_y_q;

            // Stage 3: horizontal interpolation over the cache response.
            horizontal_valid_q <= valid_q;
            red_top_q <= red_top_next;
            red_bottom_q <= red_bottom_next;
            green_top_q <= green_top_next;
            green_bottom_q <= green_bottom_next;
            blue_top_q <= blue_top_next;
            blue_bottom_q <= blue_bottom_next;
            frac_y_interp_q <= frac_y_q;
            inside_interp_q <= inside_q;
            cache_valid_interp_q <= cache_rd_valid;

            // Stage 4: register the vertical products. The final add/round is
            // combinational from these registers, shortening the 125 MHz path
            // while preserving one output pixel per cycle.
            red_top_vertical_q <= red_top_q * wy0;
            red_bottom_vertical_q <= red_bottom_q * wy1;
            green_top_vertical_q <= green_top_q * wy0;
            green_bottom_vertical_q <= green_bottom_q * wy1;
            blue_top_vertical_q <= blue_top_q * wy0;
            blue_bottom_vertical_q <= blue_bottom_q * wy1;
            pixel_valid_q <= horizontal_valid_q;
            inside_vertical_q <= inside_interp_q;
            cache_valid_vertical_q <= cache_valid_interp_q;
        end
    end

    wire [23:0] red_sum = red_top_vertical_q + red_bottom_vertical_q;
    wire [23:0] green_sum = green_top_vertical_q + green_bottom_vertical_q;
    wire [23:0] blue_sum = blue_top_vertical_q + blue_bottom_vertical_q;
    wire [4:0] red = (red_sum + 24'd32768) >> 16;
    wire [5:0] green = (green_sum + 24'd32768) >> 16;
    wire [4:0] blue = (blue_sum + 24'd32768) >> 16;
    wire [15:0] bilinear_pixel = {red, green, blue};

    assign pixel_valid = pixel_valid_q;
    assign cache_miss = pixel_valid && inside_vertical_q && !cache_valid_vertical_q;
    // An out-of-frame coordinate or a cache miss is explicitly black; stale
    // cache data is never presented as a valid transformed pixel.
    assign pixel_out = (pixel_valid && inside_vertical_q && cache_valid_vertical_q) ? bilinear_pixel : 16'd0;
endmodule
