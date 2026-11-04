`timescale 1ns/1ps
// Full-frame processing shell: Brown+affine tile warp -> raster order ->
// double-line buffer -> processed-frame DDR bursts.
module brown_affine_ddr_frame_processor #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter OUTPUT_TILE_WIDTH = 16,
    parameter OUTPUT_TILE_HEIGHT = 1,
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
    input wire [1:0] output_frame_bank,
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

    output wire ddr_rd_cmd_en,
    output wire [27:0] ddr_rd_cmd_addr,
    output wire [31:0] ddr_rd_cmd_len,
    input wire ddr_rd_cmd_ready,
    input wire ddr_rd_data_valid,
    input wire [255:0] ddr_rd_data,

    output wire ddr_wr_cmd_en,
    output wire [27:0] ddr_wr_cmd_addr,
    output wire [31:0] ddr_wr_cmd_len,
    input wire ddr_wr_cmd_ready,
    input wire ddr_wr_data_req,
    output wire [255:0] ddr_wr_data,
    input wire ddr_wr_cmd_done,

    output wire busy,
    output wire frame_done,
    output wire frame_error
);
    wire [11:0] warp_pixel_x, warp_pixel_y;
    wire [15:0] warp_pixel_data;
    wire warp_pixel_valid;
    reg [11:0] pixel_x_q = 0, pixel_y_q = 0;
    reg [15:0] pixel_data_q = 0;
    reg pixel_valid_q = 1'b0;
    reg warp_done_q = 1'b0, warp_error_q = 1'b0;
    wire warp_busy, warp_done, warp_error;
    wire writer_busy, writer_done, writer_error;

    // Isolate the sampler-to-DDR row-buffer path with a one-cycle stream stage.
    // Coordinates, valid, frame-done, and errors advance together with pixels.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x_q <= 0;
            pixel_y_q <= 0;
            pixel_data_q <= 0;
            pixel_valid_q <= 1'b0;
            warp_done_q <= 1'b0;
            warp_error_q <= 1'b0;
        end else begin
            pixel_x_q <= warp_pixel_x;
            pixel_y_q <= warp_pixel_y;
            pixel_data_q <= warp_pixel_data;
            pixel_valid_q <= warp_pixel_valid;
            warp_done_q <= warp_done;
            warp_error_q <= warp_error;
        end
    end

    brown_affine_frame_warp_engine #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .OUTPUT_TILE_WIDTH(OUTPUT_TILE_WIDTH), .OUTPUT_TILE_HEIGHT(OUTPUT_TILE_HEIGHT),
        .CACHE_WIDTH(CACHE_WIDTH), .CACHE_HEIGHT(CACHE_HEIGHT),
        .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS),
        .MAP_LATENCY(MAP_LATENCY), .AFFINE_CORNER_PREPASS(AFFINE_CORNER_PREPASS),
        .BROWN_PREPASS_STEP(BROWN_PREPASS_STEP), .PREPASS_BOUND_MARGIN(PREPASS_BOUND_MARGIN)
    ) warp_engine (
        .clk(clk), .rst_n(rst_n), .start(start), .source_frame_bank(source_frame_bank),
        .brown_bypass(brown_bypass), .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .fx_q16(fx_q16), .fy_q16(fy_q16), .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .ddr_cmd_ready(ddr_rd_cmd_ready), .ddr_cmd_en(ddr_rd_cmd_en),
        .ddr_cmd_addr(ddr_rd_cmd_addr), .ddr_cmd_len(ddr_rd_cmd_len),
        .ddr_read_valid(ddr_rd_data_valid), .ddr_read_data(ddr_rd_data),
        .pixel_valid(warp_pixel_valid), .pixel_x(warp_pixel_x), .pixel_y(warp_pixel_y), .pixel_data(warp_pixel_data),
        .busy(warp_busy), .frame_done(warp_done), .frame_error(warp_error)
    );

    generate
        if (OUTPUT_TILE_HEIGHT == 1) begin : gen_raster_writer
            affine_raster_ddr_writer #(
                .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
                .ADDR_WIDTH(28), .LEN_WIDTH(32), .DATA_WIDTH(256), .PIXEL_WIDTH(16),
                .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS)
            ) output_writer (
                .clk(clk), .rst_n(rst_n), .frame_start(start), .output_bank(output_frame_bank),
                .pixel_valid(pixel_valid_q), .pixel_x(pixel_x_q), .pixel_y(pixel_y_q), .pixel_data(pixel_data_q),
                .warp_frame_done(warp_done_q), .warp_frame_error(warp_error_q),
                .ddr_cmd_en(ddr_wr_cmd_en), .ddr_cmd_addr(ddr_wr_cmd_addr), .ddr_cmd_len(ddr_wr_cmd_len),
                .ddr_cmd_ready(ddr_wr_cmd_ready), .ddr_data_req(ddr_wr_data_req), .ddr_data(ddr_wr_data),
                .ddr_cmd_done(ddr_wr_cmd_done), .busy(writer_busy), .frame_complete(writer_done),
                .frame_error(writer_error)
            );
        end else begin : gen_tile_writer
            affine_tile_ddr_writer #(
                .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
                .OUTPUT_TILE_WIDTH(OUTPUT_TILE_WIDTH), .OUTPUT_TILE_HEIGHT(OUTPUT_TILE_HEIGHT),
                .ADDR_WIDTH(28), .LEN_WIDTH(32), .DATA_WIDTH(256), .PIXEL_WIDTH(16),
                .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS)
            ) output_writer (
                .clk(clk), .rst_n(rst_n), .frame_start(start), .output_bank(output_frame_bank),
                .pixel_valid(pixel_valid_q), .pixel_x(pixel_x_q), .pixel_y(pixel_y_q), .pixel_data(pixel_data_q),
                .warp_frame_done(warp_done_q), .warp_frame_error(warp_error_q),
                .ddr_cmd_en(ddr_wr_cmd_en), .ddr_cmd_addr(ddr_wr_cmd_addr), .ddr_cmd_len(ddr_wr_cmd_len),
                .ddr_cmd_ready(ddr_wr_cmd_ready), .ddr_data_req(ddr_wr_data_req), .ddr_data(ddr_wr_data),
                .ddr_cmd_done(ddr_wr_cmd_done), .busy(writer_busy), .frame_complete(writer_done),
                .frame_error(writer_error)
            );
        end
    endgenerate

    assign busy = warp_busy | writer_busy;
    assign frame_done = writer_done;
    assign frame_error = warp_error | writer_error;
endmodule
