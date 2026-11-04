`timescale 1ns/1ps
// Connect one affine tile's DDR request scheduler, return-beat parser, and
// on-chip source cache. DDR command arbitration/controller wiring remains an
// external integration point.
module affine_tile_read_engine #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter TILE_WIDTH = 64,
    parameter TILE_HEIGHT = 64,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire has_intersection,
    input wire [11:0] src_x_min,
    input wire [11:0] src_x_max,
    input wire [11:0] src_y_min,
    input wire [11:0] src_y_max,
    input wire [1:0] frame_bank,
    input wire cmd_ready,
    output wire cmd_en,
    output wire [27:0] cmd_addr,
    output wire [31:0] cmd_len,
    input wire read_valid,
    input wire [255:0] read_data,
    output reg busy = 1'b0,
    output reg done_pulse = 1'b0,
    output reg error_pulse = 1'b0,
    output wire tile_ready,
    input wire sample_en,
    input wire [11:0] sample_x0,
    input wire [11:0] sample_y0,
    input wire [11:0] sample_x1,
    input wire [11:0] sample_y1,
    output wire sample_valid,
    output wire [15:0] sample_p00,
    output wire [15:0] sample_p10,
    output wire [15:0] sample_p01,
    output wire [15:0] sample_p11
);
    reg active = 1'b0;
    wire start_accept = start && !active;
    wire [11:0] tile_width = src_x_max - src_x_min + 1'b1;
    wire [11:0] tile_height = src_y_max - src_y_min + 1'b1;
    wire scheduler_busy;
    wire scheduler_done;
    wire scheduler_error;
    wire [11:0] active_y;
    wire [3:0] active_group;
    wire filter_valid;
    wire [255:0] filter_pixels;
    wire [15:0] filter_mask;
    wire [11:0] filter_x_base;
    wire [11:0] filter_y;
    wire [3:0] filter_beat_index;
    wire filter_burst_last;
    wire filter_error;
    wire cache_error;
    reg scheduler_done_d1 = 1'b0;
    reg scheduler_done_d2 = 1'b0;
    reg scheduler_done_d3 = 1'b0;
    reg tile_ready_d = 1'b0;

    affine_tile_read_scheduler #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS)
    ) request_scheduler (
        .clk(clk), .rst_n(rst_n), .start(start_accept && has_intersection),
        .has_intersection(has_intersection), .src_x_min(src_x_min), .src_x_max(src_x_max),
        .src_y_min(src_y_min), .src_y_max(src_y_max), .frame_bank(frame_bank),
        .cmd_ready(cmd_ready), .cmd_en(cmd_en), .cmd_addr(cmd_addr), .cmd_len(cmd_len),
        .busy(scheduler_busy), .done_pulse(scheduler_done), .error_pulse(scheduler_error),
        .requests_issued(), .active_y(active_y), .active_group(active_group)
    );

    affine_tile_beat_filter #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .PIXEL0_IN_LOW_BITS(1)
    ) return_filter (
        .clk(clk), .rst_n(rst_n), .burst_start(cmd_en), .beat_valid(read_valid),
        .beat_data(read_data), .source_y(active_y), .source_group(active_group),
        .src_x_min(src_x_min), .src_x_max(src_x_max), .src_y_min(src_y_min), .src_y_max(src_y_max),
        .beat_valid_out(filter_valid), .pixel_data_out(filter_pixels), .pixel_mask_out(filter_mask),
        .pixel_x_base_out(filter_x_base), .pixel_y_out(filter_y),
        .beat_index_out(filter_beat_index), .burst_last_out(filter_burst_last),
        .error_pulse(filter_error)
    );

    affine_tile_cache #(
        .TILE_WIDTH(TILE_WIDTH), .TILE_HEIGHT(TILE_HEIGHT), .ORDERED_WRITES(1)
    ) source_cache (
        .clk(clk), .rst_n(rst_n),
        .tile_start(start_accept && has_intersection),
        .tile_x_min(src_x_min), .tile_y_min(src_y_min),
        .tile_width(tile_width), .tile_height(tile_height),
        .wr_valid(filter_valid), .wr_pixel_data(filter_pixels), .wr_pixel_mask(filter_mask),
        .wr_x_base(filter_x_base), .wr_y(filter_y), .tile_complete(scheduler_done_d3),
        .tile_ready(tile_ready), .tile_error_pulse(cache_error),
        .rd_en(sample_en), .rd_x0(sample_x0), .rd_y0(sample_y0),
        .rd_x1(sample_x1), .rd_y1(sample_y1), .rd_valid(sample_valid),
        .rd_p00(sample_p00), .rd_p10(sample_p10), .rd_p01(sample_p01), .rd_p11(sample_p11)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scheduler_done_d1 <= 1'b0;
            scheduler_done_d2 <= 1'b0;
            scheduler_done_d3 <= 1'b0;
            tile_ready_d <= 1'b0;
            active <= 1'b0;
            busy <= 1'b0;
            done_pulse <= 1'b0;
            error_pulse <= 1'b0;
        end else begin
            scheduler_done_d1 <= scheduler_done;
            scheduler_done_d2 <= scheduler_done_d1;
            scheduler_done_d3 <= scheduler_done_d2;
            tile_ready_d <= tile_ready;
            done_pulse <= 1'b0;
            error_pulse <= 1'b0;

            if (start && active)
                error_pulse <= 1'b1;
            if (start_accept && !has_intersection)
                done_pulse <= 1'b1;
            if (start_accept && has_intersection) begin
                active <= 1'b1;
                busy <= 1'b1;
            end
            if (tile_ready && !tile_ready_d) begin
                done_pulse <= 1'b1;
                active <= 1'b0;
                busy <= 1'b0;
            end
            if (scheduler_error || filter_error || cache_error) begin
                error_pulse <= 1'b1;
                active <= 1'b0;
                busy <= 1'b0;
            end
        end
    end
endmodule
