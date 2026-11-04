`timescale 1ns/1ps
module hdmi_snapshot_fifo #(
    parameter H_PIXELS = 1280,
    parameter V_PIXELS = 720,
    parameter FIFO_ADDR_BITS = 11,
    parameter KEY_DEBOUNCE_CYCLES = 750000
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        key4_n,
    input  wire        capture_request,
    input  wire [1:0]  capture_request_mode,
    input  wire        network_ready,
    input  wire        raw_vsync,
    input  wire        raw_de,
    input  wire [15:0] raw_rgb565,
    input  wire        hdmi_vsync,
    input  wire        hdmi_de,
    input  wire [15:0] hdmi_rgb565,
    output wire        frame_done,
    output wire        capture_error,
    output wire        capture_busy,
    output wire [31:0] frame_id,
    output wire [1:0]  capture_mode,
    input  wire        net_clk,
    input  wire        net_rst_n,
    input  wire        pixel_rd_en,
    output wire [15:0] pixel_rd_data,
    output wire        pixel_empty,
    output wire [FIFO_ADDR_BITS:0] pixel_count,
    output reg         capture_error_net
);
    wire fifo_wr_en, fifo_full;
    wire [15:0] fifo_wr_data;
    wire [FIFO_ADDR_BITS:0] unused_wr_level;
    reg fifo_empty_pix1, fifo_empty_pix2;
    reg net_ready_pix1, net_ready_pix2;
    reg capture_error_net1;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            fifo_empty_pix1 <= 1'b1;
            fifo_empty_pix2 <= 1'b1;
            net_ready_pix1 <= 1'b0;
            net_ready_pix2 <= 1'b0;
        end else begin
            fifo_empty_pix1 <= pixel_empty;
            fifo_empty_pix2 <= fifo_empty_pix1;
            net_ready_pix1 <= network_ready;
            net_ready_pix2 <= net_ready_pix1;
        end
    end

    always @(posedge net_clk or negedge net_rst_n) begin
        if (!net_rst_n) begin
            capture_error_net1 <= 1'b0;
            capture_error_net <= 1'b0;
        end else begin
            capture_error_net1 <= capture_error;
            capture_error_net <= capture_error_net1;
        end
    end

    hdmi_snapshot_capture #(
        .H_PIXELS(H_PIXELS), .V_PIXELS(V_PIXELS),
        .KEY_DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)
    ) capture_i (
        .pix_clk(pix_clk), .rst_n(pix_rst_n), .key4_n(key4_n),
        .capture_request(capture_request), .capture_request_mode(capture_request_mode),
        .network_ready(net_ready_pix2),
        .raw_vsync(raw_vsync), .raw_de(raw_de), .raw_rgb565(raw_rgb565),
        .hdmi_vsync(hdmi_vsync), .hdmi_de(hdmi_de), .hdmi_rgb565(hdmi_rgb565),
        .fifo_full(fifo_full), .fifo_empty(fifo_empty_pix2),
        .fifo_wr_en(fifo_wr_en), .fifo_wr_data(fifo_wr_data),
        .frame_done(frame_done), .capture_error(capture_error),
        .capture_busy(capture_busy), .frame_id(frame_id), .capture_mode(capture_mode)
    );

    async_fifo_16 #(.ADDR_BITS(FIFO_ADDR_BITS)) pixels_i (
        .wr_clk(pix_clk), .wr_rst_n(pix_rst_n), .wr_en(fifo_wr_en),
        .wr_data(fifo_wr_data), .wr_full(fifo_full), .wr_level(unused_wr_level),
        .rd_clk(net_clk), .rd_rst_n(net_rst_n), .rd_en(pixel_rd_en),
        .rd_data(pixel_rd_data), .rd_empty(pixel_empty), .rd_level(pixel_count)
    );
endmodule
