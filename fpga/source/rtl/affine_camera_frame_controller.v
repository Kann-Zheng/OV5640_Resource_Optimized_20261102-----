`timescale 1ns/1ps
// Owns camera/output frame banks and automatically processes the newest
// completed camera frame when an output bank is available. All control inputs
// and frame-vsync events must already be synchronized into clk's domain.
module affine_camera_frame_controller #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter OUTPUT_TILE_WIDTH = 16,
    parameter OUTPUT_TILE_HEIGHT = 1,
    parameter CACHE_WIDTH = 64,
    parameter CACHE_HEIGHT = 64,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22,
    // The affine ROI prepass gained one final-scale recombination cycle.
    parameter MAP_LATENCY = 24,
    parameter AFFINE_CORNER_PREPASS = 1'b0,
    parameter BROWN_PREPASS_STEP = 1,
    parameter PREPASS_BOUND_MARGIN = 0
)(
    input wire clk,
    input wire rst_n,
    input wire camera_frame_start,
    input wire camera_frame_done,
    input wire display_vsync,
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

    output wire camera_capture_active,
    output wire camera_frame_drop,
    output wire [1:0] camera_write_bank,
    output wire process_active,
    output wire [1:0] process_read_bank,
    output wire [1:0] process_write_bank,
    output wire process_busy,
    output wire process_frame_done,
    output wire process_frame_error,
    output wire display_valid,
    output wire [1:0] display_bank,
    output wire output_frame_valid,
    output wire [1:0] latest_output_bank,
    output wire drop_pulse,
    output wire error_pulse
);
    wire process_request;
    wire process_accept;
    wire processor_done, processor_error;
    wire processor_failure_done;
    wire pending_frame_valid;
    reg process_seen_busy = 1'b0;
    reg failure_reported = 1'b0;
    assign processor_failure_done = process_active && process_seen_busy && processor_error &&
                                   !process_busy && !failure_reported;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            process_seen_busy <= 1'b0;
            failure_reported <= 1'b0;
        end else begin
            if (process_accept) begin
                process_seen_busy <= 1'b0;
                failure_reported <= 1'b0;
            end else begin
                if (process_busy)
                    process_seen_busy <= 1'b1;
                if (processor_failure_done)
                    failure_reported <= 1'b1;
                if (processor_done) begin
                    process_seen_busy <= 1'b0;
                    failure_reported <= 1'b0;
                end
            end
        end
    end

    affine_frame_bank_manager bank_manager (
        .clk(clk), .rst_n(rst_n),
        .camera_frame_start(camera_frame_start), .camera_frame_done(camera_frame_done),
        .camera_capture_active(camera_capture_active), .camera_frame_drop(camera_frame_drop),
        .camera_write_bank(camera_write_bank),
        .process_start(process_request), .process_accept(process_accept),
        .process_active(process_active), .process_read_bank(process_read_bank),
        .process_write_bank(process_write_bank),
        .process_done(processor_done || processor_failure_done),
        .process_success(processor_done && !processor_error), .process_can_start(process_request),
        .pending_frame_valid(pending_frame_valid),
        .display_vsync(display_vsync), .display_valid(display_valid),
        .display_bank(display_bank), .output_frame_valid(output_frame_valid),
        .latest_output_bank(latest_output_bank), .drop_pulse(drop_pulse), .error_pulse(error_pulse)
    );

    brown_affine_ddr_frame_processor #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .OUTPUT_TILE_WIDTH(OUTPUT_TILE_WIDTH), .OUTPUT_TILE_HEIGHT(OUTPUT_TILE_HEIGHT),
        .CACHE_WIDTH(CACHE_WIDTH), .CACHE_HEIGHT(CACHE_HEIGHT),
        .ROW_STRIDE_UNITS(ROW_STRIDE_UNITS), .BANK_STRIDE_BITS(BANK_STRIDE_BITS),
        .MAP_LATENCY(MAP_LATENCY), .AFFINE_CORNER_PREPASS(AFFINE_CORNER_PREPASS),
        .BROWN_PREPASS_STEP(BROWN_PREPASS_STEP), .PREPASS_BOUND_MARGIN(PREPASS_BOUND_MARGIN)
    ) processor (
        .clk(clk), .rst_n(rst_n), .start(process_accept),
        .source_frame_bank(process_read_bank), .output_frame_bank(process_write_bank),
        .brown_bypass(brown_bypass), .center_x(center_x), .center_y(center_y),
        .a_q14(a_q14), .b_q14(b_q14), .c_q14(c_q14), .d_q14(d_q14),
        .fx_q16(fx_q16), .fy_q16(fy_q16), .inv_fx_q30(inv_fx_q30), .inv_fy_q30(inv_fy_q30),
        .k1_q24(k1_q24), .k2_q24(k2_q24), .p1_q24(p1_q24), .p2_q24(p2_q24),
        .ddr_rd_cmd_en(ddr_rd_cmd_en), .ddr_rd_cmd_addr(ddr_rd_cmd_addr),
        .ddr_rd_cmd_len(ddr_rd_cmd_len), .ddr_rd_cmd_ready(ddr_rd_cmd_ready),
        .ddr_rd_data_valid(ddr_rd_data_valid), .ddr_rd_data(ddr_rd_data),
        .ddr_wr_cmd_en(ddr_wr_cmd_en), .ddr_wr_cmd_addr(ddr_wr_cmd_addr),
        .ddr_wr_cmd_len(ddr_wr_cmd_len), .ddr_wr_cmd_ready(ddr_wr_cmd_ready),
        .ddr_wr_data_req(ddr_wr_data_req), .ddr_wr_data(ddr_wr_data),
        .ddr_wr_cmd_done(ddr_wr_cmd_done), .busy(process_busy),
        .frame_done(processor_done), .frame_error(processor_error)
    );

    assign process_frame_done = processor_done;
    assign process_frame_error = processor_error;
endmodule
