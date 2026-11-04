`timescale 1ns/1ps
// Serial full-frame scheduler for the verified single-tile Brown+affine DDR
// warp. Pixel output is tagged with its destination coordinate so a downstream
// row/tile assembler can place out-of-raster-order tiles safely.
module brown_affine_frame_warp_engine #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter OUTPUT_TILE_WIDTH = 16,
    parameter OUTPUT_TILE_HEIGHT = 16,
    parameter CACHE_WIDTH = 64,
    parameter CACHE_HEIGHT = 64,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22,
    parameter MAP_LATENCY = 24,
    parameter AFFINE_CORNER_PREPASS = 1'b0,
    parameter BROWN_PREPASS_STEP = 1,
    parameter PREPASS_BOUND_MARGIN = 0
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [1:0] source_frame_bank,
    input wire brown_bypass,
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

    input wire ddr_cmd_ready,
    output wire ddr_cmd_en,
    output wire [27:0] ddr_cmd_addr,
    output wire [31:0] ddr_cmd_len,
    input wire ddr_read_valid,
    input wire [255:0] ddr_read_data,

    output wire pixel_valid,
    output wire [11:0] pixel_x,
    output wire [11:0] pixel_y,
    output wire [15:0] pixel_data,
    output reg busy = 1'b0,
    output reg frame_done = 1'b0,
    output reg frame_error = 1'b0
);
    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_PREPASS    = 3'd1;
    localparam [2:0] ST_WAIT_TILE  = 3'd2;
    localparam [2:0] ST_PROCESS    = 3'd3;
    localparam [2:0] ST_DRAIN      = 3'd4;

    reg [2:0] state = ST_IDLE;
    reg [1:0] active_source_bank = 0;
    reg [11:0] tile_origin_x = 0, tile_origin_y = 0;
    reg [11:0] tile_width = 0, tile_height = 0;
    reg [11:0] pre_x = 0, pre_y = 0;
    reg [1:0] pre_corner = 0;
    reg [11:0] process_x_local = 0, process_y_local = 0;
    reg [11:0] output_x_local = 0, output_y_local = 0;
    reg [12:0] output_count = 0;
    reg bbox_seen = 0, bbox_hit = 0, read_started = 0;

    reg active_brown_bypass = 1'b0;
    reg [11:0] active_center_x = 0, active_center_y = 0;
    reg signed [15:0] active_a_q14 = 16'sd16384, active_b_q14 = 0;
    reg signed [15:0] active_c_q14 = 0, active_d_q14 = 16'sd16384;
    reg signed [31:0] active_fx_q16 = 0, active_fy_q16 = 0;
    reg signed [31:0] active_inv_fx_q30 = 0, active_inv_fy_q30 = 0;
    reg signed [31:0] active_k1_q24 = 0, active_k2_q24 = 0;
    reg signed [31:0] active_p1_q24 = 0, active_p2_q24 = 0;

    wire affine_corner_mode = AFFINE_CORNER_PREPASS && active_brown_bypass;
    // An affine map reaches its rectangular tile bounds at the four corners.
    // Brown mapping uses a sparse grid; its source ROI is expanded and clipped
    // by the bounds accumulator before the DDR cache request.
    wire prepass_start = (state == ST_PREPASS) &&
                         (affine_corner_mode ? (pre_corner == 0) : ((pre_x == 0) && (pre_y == 0)));
    wire prepass_valid = (state == ST_PREPASS);
    wire prepass_last = prepass_valid &&
                        (affine_corner_mode ? (pre_corner == 2'd3) :
                         ((pre_x == tile_width - 1'b1) && (pre_y == tile_height - 1'b1)));
    wire [11:0] prepass_x = tile_origin_x + (affine_corner_mode ?
        ((pre_corner[0]) ? (tile_width - 1'b1) : 12'd0) : pre_x);
    wire [11:0] prepass_y = tile_origin_y + (affine_corner_mode ?
        ((pre_corner[1]) ? (tile_height - 1'b1) : 12'd0) : pre_y);
    wire [11:0] warp_pixel_x, warp_pixel_y;
    wire [15:0] warp_pixel_data;
    wire warp_pixel_valid, warp_tile_ready, warp_tile_busy, warp_tile_error, warp_cache_miss;
    wire bbox_valid, bbox_has_intersection;
    wire [11:0] bbox_x_min, bbox_x_max, bbox_y_min, bbox_y_max;
    wire [12:0] tile_area = {1'b0, tile_width} * {1'b0, tile_height};
    wire process_last = (process_x_local == tile_width - 1'b1) &&
                        (process_y_local == tile_height - 1'b1);
    wire output_last = (output_count == tile_area - 1'b1);

    assign warp_pixel_x = tile_origin_x + output_x_local;
    assign warp_pixel_y = tile_origin_y + output_y_local;
    assign pixel_valid = warp_pixel_valid && !frame_error && !warp_cache_miss;
    assign pixel_x = warp_pixel_x;
    assign pixel_y = warp_pixel_y;
    assign pixel_data = warp_pixel_data;

    brown_affine_tile_ddr_warp #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .TILE_WIDTH(CACHE_WIDTH), .TILE_HEIGHT(CACHE_HEIGHT),
        .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS),
        .MAP_LATENCY(MAP_LATENCY), .PREPASS_BOUND_MARGIN(PREPASS_BOUND_MARGIN)
    ) tile_warp (
        .clk(clk), .rst_n(rst_n),
        .prepass_start(prepass_start), .prepass_valid(prepass_valid), .prepass_last(prepass_last),
        .brown_bypass(active_brown_bypass), .include_outside_bounds(1'b1),
        .prepass_x(prepass_x), .prepass_y(prepass_y),
        .center_x(active_center_x), .center_y(active_center_y),
        .a_q14(active_a_q14), .b_q14(active_b_q14), .c_q14(active_c_q14), .d_q14(active_d_q14),
        .fx_q16(active_fx_q16), .fy_q16(active_fy_q16),
        .inv_fx_q30(active_inv_fx_q30), .inv_fy_q30(active_inv_fy_q30),
        .k1_q24(active_k1_q24), .k2_q24(active_k2_q24), .p1_q24(active_p1_q24), .p2_q24(active_p2_q24),
        .bbox_valid(bbox_valid), .has_intersection(bbox_has_intersection),
        .src_x_min(bbox_x_min), .src_x_max(bbox_x_max), .src_y_min(bbox_y_min), .src_y_max(bbox_y_max),
        .frame_bank(active_source_bank), .ddr_cmd_ready(ddr_cmd_ready),
        .ddr_cmd_en(ddr_cmd_en), .ddr_cmd_addr(ddr_cmd_addr), .ddr_cmd_len(ddr_cmd_len),
        .ddr_read_valid(ddr_read_valid), .ddr_read_data(ddr_read_data),
        .tile_busy(warp_tile_busy), .tile_ready(warp_tile_ready), .tile_done(), .tile_error(warp_tile_error),
        .process_valid(state == ST_PROCESS),
        .process_x(tile_origin_x + process_x_local), .process_y(tile_origin_y + process_y_local),
        .pixel_valid(warp_pixel_valid), .pixel_out(warp_pixel_data), .cache_miss(warp_cache_miss)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            active_source_bank <= 0;
            tile_origin_x <= 0;
            tile_origin_y <= 0;
            tile_width <= 0;
            tile_height <= 0;
            pre_x <= 0;
            pre_y <= 0;
            pre_corner <= 0;
            process_x_local <= 0;
            process_y_local <= 0;
            output_x_local <= 0;
            output_y_local <= 0;
            output_count <= 0;
            bbox_seen <= 0;
            bbox_hit <= 0;
            read_started <= 0;
            active_brown_bypass <= 0;
            active_center_x <= 0;
            active_center_y <= 0;
            active_a_q14 <= 16'sd16384;
            active_b_q14 <= 0;
            active_c_q14 <= 0;
            active_d_q14 <= 16'sd16384;
            active_fx_q16 <= 0;
            active_fy_q16 <= 0;
            active_inv_fx_q30 <= 0;
            active_inv_fy_q30 <= 0;
            active_k1_q24 <= 0;
            active_k2_q24 <= 0;
            active_p1_q24 <= 0;
            active_p2_q24 <= 0;
            busy <= 0;
            frame_done <= 0;
            frame_error <= 0;
        end else begin
            frame_done <= 0;

            if (warp_tile_error || (warp_pixel_valid && warp_cache_miss)) begin
                frame_error <= 1;
                busy <= 0;
                state <= ST_IDLE;
            end else begin
                case (state)
                    ST_IDLE: begin
                        busy <= 0;
                        if (start) begin
                            if ((IMAGE_WIDTH <= 0) || (IMAGE_HEIGHT <= 0) ||
                                (OUTPUT_TILE_WIDTH <= 0) || (OUTPUT_TILE_HEIGHT <= 0) ||
                                (OUTPUT_TILE_WIDTH > CACHE_WIDTH) || (OUTPUT_TILE_HEIGHT > CACHE_HEIGHT)) begin
                                frame_error <= 1;
                            end else begin
                                frame_error <= 0;
                                busy <= 1;
                                active_source_bank <= source_frame_bank;
                                active_brown_bypass <= brown_bypass;
                                active_center_x <= center_x;
                                active_center_y <= center_y;
                                active_a_q14 <= a_q14;
                                active_b_q14 <= b_q14;
                                active_c_q14 <= c_q14;
                                active_d_q14 <= d_q14;
                                active_fx_q16 <= fx_q16;
                                active_fy_q16 <= fy_q16;
                                active_inv_fx_q30 <= inv_fx_q30;
                                active_inv_fy_q30 <= inv_fy_q30;
                                active_k1_q24 <= k1_q24;
                                active_k2_q24 <= k2_q24;
                                active_p1_q24 <= p1_q24;
                                active_p2_q24 <= p2_q24;
                                tile_origin_x <= 0;
                                tile_origin_y <= 0;
                                tile_width <= (IMAGE_WIDTH < OUTPUT_TILE_WIDTH) ? IMAGE_WIDTH : OUTPUT_TILE_WIDTH;
                                tile_height <= (IMAGE_HEIGHT < OUTPUT_TILE_HEIGHT) ? IMAGE_HEIGHT : OUTPUT_TILE_HEIGHT;
                                pre_x <= 0;
                                pre_y <= 0;
                                pre_corner <= 0;
                                output_count <= 0;
                                output_x_local <= 0;
                                output_y_local <= 0;
                                bbox_seen <= 0;
                                bbox_hit <= 0;
                                read_started <= 0;
                                state <= ST_PREPASS;
                            end
                        end
                    end

                    ST_PREPASS: begin
                        if (affine_corner_mode) begin
                            if (pre_corner == 2'd3) begin
                                pre_corner <= 0;
                                pre_x <= 0;
                                pre_y <= 0;
                                bbox_seen <= 0;
                                bbox_hit <= 0;
                                read_started <= 0;
                                state <= ST_WAIT_TILE;
                            end else begin
                                pre_corner <= pre_corner + 1'b1;
                            end
                        end else begin
                            if (pre_x == tile_width - 1'b1) begin
                                pre_x <= 0;
                                if (pre_y == tile_height - 1'b1) begin
                                    pre_corner <= 0;
                                    bbox_seen <= 0;
                                    bbox_hit <= 0;
                                    read_started <= 0;
                                    state <= ST_WAIT_TILE;
                                end else begin
                                    pre_y <= ((pre_y + BROWN_PREPASS_STEP) >= tile_height) ?
                                             (tile_height - 1'b1) : (pre_y + BROWN_PREPASS_STEP);
                                end
                            end else begin
                                pre_x <= ((pre_x + BROWN_PREPASS_STEP) >= tile_width) ?
                                         (tile_width - 1'b1) : (pre_x + BROWN_PREPASS_STEP);
                            end
                        end
                    end

                    ST_WAIT_TILE: begin
                        if (bbox_valid) begin
                            bbox_seen <= 1;
                            bbox_hit <= bbox_has_intersection;
                        end
                        if (bbox_seen && bbox_hit && (warp_tile_busy || !warp_tile_ready))
                            read_started <= 1;

                        if (bbox_seen && !bbox_hit && warp_tile_ready && !warp_tile_busy) begin
                            process_x_local <= 0;
                            process_y_local <= 0;
                            output_x_local <= 0;
                            output_y_local <= 0;
                            output_count <= 0;
                            state <= ST_PROCESS;
                        end else if (bbox_seen && bbox_hit && read_started && warp_tile_ready && !warp_tile_busy) begin
                            process_x_local <= 0;
                            process_y_local <= 0;
                            output_x_local <= 0;
                            output_y_local <= 0;
                            output_count <= 0;
                            state <= ST_PROCESS;
                        end
                    end

                    ST_PROCESS: begin
                        // The mapper may return pixels before the final input
                        // coordinate has been fed. Count those immediately so
                        // destination tags stay aligned with the output stream.
                        if (pixel_valid && !output_last) begin
                            output_count <= output_count + 1'b1;
                            if (output_x_local == tile_width - 1'b1) begin
                                output_x_local <= 0;
                                if (output_y_local != tile_height - 1'b1)
                                    output_y_local <= output_y_local + 1'b1;
                            end else begin
                                output_x_local <= output_x_local + 1'b1;
                            end
                        end
                        if (process_x_local == tile_width - 1'b1) begin
                            process_x_local <= 0;
                            if (process_y_local == tile_height - 1'b1) begin
                                process_y_local <= 0;
                                state <= ST_DRAIN;
                            end else begin
                                process_y_local <= process_y_local + 1'b1;
                            end
                        end else begin
                            process_x_local <= process_x_local + 1'b1;
                        end
                    end

                    ST_DRAIN: begin
                        if (pixel_valid) begin
                            if (output_last) begin
                                if (tile_origin_x + OUTPUT_TILE_WIDTH >= IMAGE_WIDTH) begin
                                    if (tile_origin_y + OUTPUT_TILE_HEIGHT >= IMAGE_HEIGHT) begin
                                        frame_done <= 1;
                                        busy <= 0;
                                        state <= ST_IDLE;
                                    end else begin
                                        tile_origin_x <= 0;
                                        tile_origin_y <= tile_origin_y + OUTPUT_TILE_HEIGHT;
                                        tile_width <= (IMAGE_WIDTH < OUTPUT_TILE_WIDTH) ? IMAGE_WIDTH : OUTPUT_TILE_WIDTH;
                                        tile_height <= ((IMAGE_HEIGHT - (tile_origin_y + OUTPUT_TILE_HEIGHT)) < OUTPUT_TILE_HEIGHT) ?
                                                       (IMAGE_HEIGHT - (tile_origin_y + OUTPUT_TILE_HEIGHT)) : OUTPUT_TILE_HEIGHT;
                                        pre_x <= 0;
                                        pre_y <= 0;
                                        pre_corner <= 0;
                                        state <= ST_PREPASS;
                                    end
                                end else begin
                                    tile_origin_x <= tile_origin_x + OUTPUT_TILE_WIDTH;
                                    tile_width <= ((IMAGE_WIDTH - (tile_origin_x + OUTPUT_TILE_WIDTH)) < OUTPUT_TILE_WIDTH) ?
                                                  (IMAGE_WIDTH - (tile_origin_x + OUTPUT_TILE_WIDTH)) : OUTPUT_TILE_WIDTH;
                                    pre_x <= 0;
                                    pre_y <= 0;
                                    pre_corner <= 0;
                                    state <= ST_PREPASS;
                                end
                                bbox_seen <= 0;
                                bbox_hit <= 0;
                                read_started <= 0;
                            end else begin
                                output_count <= output_count + 1'b1;
                                if (output_x_local == tile_width - 1'b1) begin
                                    output_x_local <= 0;
                                    if (output_y_local != tile_height - 1'b1)
                                        output_y_local <= output_y_local + 1'b1;
                                end else begin
                                    output_x_local <= output_x_local + 1'b1;
                                end
                            end
                        end
                    end

                    default: begin
                        frame_error <= 1;
                        busy <= 0;
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end
endmodule
