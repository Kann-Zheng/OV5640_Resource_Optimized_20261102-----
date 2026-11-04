`timescale 1ns/1ps
module hdmi_snapshot_capture #(
    parameter H_PIXELS = 1280,
    parameter V_PIXELS = 720,
    parameter KEY_DEBOUNCE_CYCLES = 750000
)(
    input  wire        pix_clk,
    input  wire        rst_n,
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
    input  wire        fifo_full,
    input  wire        fifo_empty,
    output wire        fifo_wr_en,
    output wire [15:0] fifo_wr_data,
    output reg         frame_done,
    output reg         capture_error,
    output reg         capture_busy,
    output reg [31:0]  frame_id,
    output reg [1:0]   capture_mode
);
    localparam TOTAL_PIXELS = H_PIXELS * V_PIXELS;
    reg key_meta, key_sync, key_candidate, key_stable, key_stable_d;
    reg [31:0] debounce_count;
    reg vsync_d;
    reg [1:0] mode_d;
    reg de_d;
    reg capture_pending;
    reg [20:0] pixel_count;
    reg capture_low_res;
    reg [10:0] x_count;
    reg [9:0] y_count;
    reg [1:0] source_mode;

    wire key_press = !key_stable && key_stable_d;
    wire selected_vsync = (source_mode == 2'd1) ? raw_vsync : hdmi_vsync;
    wire selected_de = (source_mode == 2'd1) ? raw_de : hdmi_de;
    wire [15:0] selected_rgb565 = (source_mode == 2'd1) ? raw_rgb565 : hdmi_rgb565;
    wire mode_changed = source_mode != mode_d;
    wire vsync_rise = selected_vsync && !vsync_d && !mode_changed;
    wire sample_enable = !capture_low_res || (!x_count[0] && !y_count[0]);
    wire sample_valid = capture_busy && selected_de && sample_enable;
    wire [20:0] target_pixel_count = capture_low_res ? (TOTAL_PIXELS / 4) : TOTAL_PIXELS;

    assign fifo_wr_en = sample_valid && !fifo_full;
    assign fifo_wr_data = selected_rgb565;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
            key_candidate <= 1'b1;
            key_stable <= 1'b1;
            key_stable_d <= 1'b1;
            debounce_count <= 0;
            vsync_d <= 1'b0;
            mode_d <= 0;
            source_mode <= 0;
            de_d <= 1'b0;
            capture_pending <= 1'b0;
            capture_busy <= 1'b0;
            pixel_count <= 0;
            capture_low_res <= 1'b0;
            x_count <= 0;
            y_count <= 0;
            frame_done <= 1'b0;
            capture_error <= 1'b0;
            frame_id <= 0;
            capture_mode <= 0;
        end else begin
            key_meta <= key4_n;
            key_sync <= key_meta;
            key_stable_d <= key_stable;
            vsync_d <= selected_vsync;
            mode_d <= source_mode;
            de_d <= selected_de;
            frame_done <= 1'b0;
            capture_error <= 1'b0;

            if (vsync_rise) begin
                x_count <= 0;
                y_count <= 0;
            end else if (selected_de) begin
                x_count <= x_count + 1'b1;
            end else if (de_d) begin
                x_count <= 0;
                y_count <= y_count + 1'b1;
            end

            if (key_sync != key_candidate) begin
                key_candidate <= key_sync;
                debounce_count <= 0;
            end else if (debounce_count < KEY_DEBOUNCE_CYCLES) begin
                debounce_count <= debounce_count + 1'b1;
                if (debounce_count == KEY_DEBOUNCE_CYCLES - 1)
                    key_stable <= key_candidate;
            end

            if (!capture_busy && !capture_pending) begin
                if (capture_request) begin
                    capture_pending <= 1'b1;
                    source_mode <= capture_request_mode;
                end else if (key_press) begin
                    capture_pending <= 1'b1;
                    source_mode <= 2'd0;
                end
            end

            if (capture_pending && vsync_rise && fifo_empty && network_ready) begin
                capture_pending <= 1'b0;
                capture_busy <= 1'b1;
                pixel_count <= 0;
                capture_mode <= source_mode;
                capture_low_res <= (source_mode != 2'd0);
            end

            if (sample_valid) begin
                if (fifo_full) begin
                    capture_busy <= 1'b0;
                    capture_pending <= 1'b0;
                    capture_error <= 1'b1;
                end else if (pixel_count == target_pixel_count - 1'b1) begin
                    capture_busy <= 1'b0;
                    pixel_count <= 0;
                    frame_done <= 1'b1;
                    frame_id <= frame_id + 1'b1;
                end else begin
                    pixel_count <= pixel_count + 1'b1;
                end
            end
        end
    end
endmodule

