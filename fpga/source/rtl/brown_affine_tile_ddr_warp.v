`timescale 1ns/1ps
// Simulation/integration boundary for one output tile: scan final Brown+affine
// coordinates, load the exact source ROI through affine_tile_read_engine, then
// rescan output coordinates and bilinearly sample the cache. DDR arbitration,
// frame-bank ownership, and output-frame writes remain at the system level.
module brown_affine_tile_ddr_warp #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter TILE_WIDTH = 64,
    parameter TILE_HEIGHT = 64,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22,
    parameter MAP_LATENCY = 22,
    parameter PREPASS_BOUND_MARGIN = 0
)(
    input wire clk,
    input wire rst_n,
    input wire prepass_start,
    input wire prepass_valid,
    input wire prepass_last,
    input wire brown_bypass,
    input wire include_outside_bounds,
    input wire [11:0] prepass_x,
    input wire [11:0] prepass_y,
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
    input wire [1:0] frame_bank,
    input wire ddr_cmd_ready,
    output wire ddr_cmd_en,
    output wire [27:0] ddr_cmd_addr,
    output wire [31:0] ddr_cmd_len,
    input wire ddr_read_valid,
    input wire [255:0] ddr_read_data,
    output wire tile_busy,
    output wire tile_ready,
    output wire tile_done,
    output wire tile_error,
    input wire process_valid,
    input wire [11:0] process_x,
    input wire [11:0] process_y,
    output wire pixel_valid,
    output wire [15:0] pixel_out,
    output wire cache_miss
);
    wire [11:0] mapped_x0, mapped_y0, mapped_x1, mapped_y1;
    wire cache_rd_en, cache_rd_valid;
    wire engine_tile_ready;
    reg empty_tile_ready = 1'b0;
    wire [15:0] cache_p00, cache_p10, cache_p01, cache_p11;
    wire prepass_error;
    wire engine_error;
    wire [12:0] roi_width = {1'b0, src_x_max} - {1'b0, src_x_min} + 1'b1;
    wire [12:0] roi_height = {1'b0, src_y_max} - {1'b0, src_y_min} + 1'b1;
    wire roi_oversize = has_intersection && ((roi_width > TILE_WIDTH) || (roi_height > TILE_HEIGHT));
    reg engine_start = 1'b0;

    // A fully out-of-image output tile needs no DDR source data. Mark that
    // tile ready explicitly instead of inheriting a stale cache-ready flag.
    assign tile_ready = engine_tile_ready | empty_tile_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            empty_tile_ready <= 1'b0;
        else if (prepass_start)
            empty_tile_ready <= 1'b0;
        else if (bbox_valid && !has_intersection && !roi_oversize)
            empty_tile_ready <= 1'b1;
    end

    brown_affine_tile_prepass #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT), .MAP_LATENCY(MAP_LATENCY),
        .BOUND_MARGIN(PREPASS_BOUND_MARGIN)
    ) roi_prepass (
        .clk(clk), .rst_n(rst_n), .scan_start(prepass_start),
        .valid_in(prepass_valid), .last_in(prepass_last), .brown_bypass(brown_bypass),
        .include_outside_bounds(include_outside_bounds),
        .x_out(prepass_x), .y_out(prepass_y), .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .fx_q16(fx_q16), .fy_q16(fy_q16), .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .bbox_valid(bbox_valid), .has_intersection(has_intersection),
        .src_x_min(src_x_min), .src_x_max(src_x_max),
        .src_y_min(src_y_min), .src_y_max(src_y_max), .error_pulse(prepass_error)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            engine_start <= 1'b0;
        else
            engine_start <= bbox_valid && !roi_oversize;
    end

    affine_tile_read_engine #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .TILE_WIDTH(TILE_WIDTH), .TILE_HEIGHT(TILE_HEIGHT),
        .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS)
    ) tile_reader (
        .clk(clk), .rst_n(rst_n), .start(engine_start), .has_intersection(has_intersection),
        .src_x_min(src_x_min), .src_x_max(src_x_max), .src_y_min(src_y_min), .src_y_max(src_y_max),
        .frame_bank(frame_bank), .cmd_ready(ddr_cmd_ready),
        .cmd_en(ddr_cmd_en), .cmd_addr(ddr_cmd_addr), .cmd_len(ddr_cmd_len),
        .read_valid(ddr_read_valid), .read_data(ddr_read_data),
        .busy(tile_busy), .done_pulse(tile_done), .error_pulse(engine_error), .tile_ready(engine_tile_ready),
        .sample_en(cache_rd_en), .sample_x0(mapped_x0), .sample_y0(mapped_y0),
        .sample_x1(mapped_x1), .sample_y1(mapped_y1), .sample_valid(cache_rd_valid),
        .sample_p00(cache_p00), .sample_p10(cache_p10), .sample_p01(cache_p01), .sample_p11(cache_p11)
    );

    brown_affine_pixel_warp_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT)
    ) pixel_warp (
        .clk(clk), .rst_n(rst_n), .valid_in(process_valid && tile_ready),
        .brown_bypass(brown_bypass), .x_out(process_x), .y_out(process_y),
        .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .fx_q16(fx_q16), .fy_q16(fy_q16), .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .cache_rd_en(cache_rd_en), .cache_x0(mapped_x0), .cache_y0(mapped_y0),
        .cache_x1(mapped_x1), .cache_y1(mapped_y1), .cache_rd_valid(cache_rd_valid),
        .cache_p00(cache_p00), .cache_p10(cache_p10), .cache_p01(cache_p01), .cache_p11(cache_p11),
        .pixel_valid(pixel_valid), .pixel_out(pixel_out), .cache_miss(cache_miss)
    );

    assign tile_error = engine_error | prepass_error | (bbox_valid && roi_oversize);
endmodule
