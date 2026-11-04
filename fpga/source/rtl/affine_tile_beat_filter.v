`timescale 1ns/1ps
// Preserve one 256-bit DDR read beat and label its 16 RGB565 pixels with
// source coordinates. Pixels outside the requested source tile are masked.
// The output remains 16 pixels wide so a future tile RAM can use 16 banks and
// accept one beat per cycle without serializing DDR data.
module affine_tile_beat_filter #(
    parameter IMAGE_WIDTH = 1280,
    // 1: the first pixel in a beat is beat_data[15:0]. Set 0 if the DDR/IP
    // packing test on the target design proves the reverse word order.
    parameter PIXEL0_IN_LOW_BITS = 1
)(
    input wire clk,
    input wire rst_n,
    input wire burst_start,
    input wire beat_valid,
    input wire [255:0] beat_data,
    input wire [11:0] source_y,
    input wire [3:0] source_group,
    input wire [11:0] src_x_min,
    input wire [11:0] src_x_max,
    input wire [11:0] src_y_min,
    input wire [11:0] src_y_max,
    output reg beat_valid_out = 1'b0,
    output reg [255:0] pixel_data_out = 0,
    output reg [15:0] pixel_mask_out = 0,
    output reg [11:0] pixel_x_base_out = 0,
    output reg [11:0] pixel_y_out = 0,
    output reg [3:0] beat_index_out = 0,
    output reg burst_last_out = 1'b0,
    output reg error_pulse = 1'b0
);
    reg burst_active = 1'b0;
    reg [3:0] beat_index = 0;
    wire [3:0] accepted_index = burst_start ? 4'd0 : beat_index;
    wire [11:0] accepted_x_base = {source_group, 8'b0} + {accepted_index, 4'b0};
    reg [15:0] lane_mask_comb;
    reg [11:0] lane_x;
    integer lane;

    wire [255:0] ordered_beat_data;
    genvar pixel_lane;
    generate
        if (PIXEL0_IN_LOW_BITS) begin: first_pixel_low
            assign ordered_beat_data = beat_data;
        end else begin: first_pixel_high
            for (pixel_lane = 0; pixel_lane < 16; pixel_lane = pixel_lane + 1) begin: reverse_lane_order
                assign ordered_beat_data[pixel_lane*16 +: 16] = beat_data[(15-pixel_lane)*16 +: 16];
            end
        end
    endgenerate

    always @* begin
        lane_mask_comb = 0;
        lane_x = accepted_x_base;
        for (lane = 0; lane < 16; lane = lane + 1) begin
            lane_x = accepted_x_base + lane;
            lane_mask_comb[lane] = (source_y >= src_y_min) && (source_y <= src_y_max) &&
                                   (lane_x >= src_x_min) && (lane_x <= src_x_max) &&
                                   (lane_x < IMAGE_WIDTH);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            burst_active <= 1'b0;
            beat_index <= 0;
            beat_valid_out <= 1'b0;
            pixel_data_out <= 0;
            pixel_mask_out <= 0;
            pixel_x_base_out <= 0;
            pixel_y_out <= 0;
            beat_index_out <= 0;
            burst_last_out <= 1'b0;
            error_pulse <= 1'b0;
        end else begin
            beat_valid_out <= 1'b0;
            burst_last_out <= 1'b0;
            error_pulse <= 1'b0;

            if (burst_start) begin
                if (burst_active)
                    error_pulse <= 1'b1;
                burst_active <= 1'b1;
                beat_index <= 0;
            end

            if (beat_valid) begin
                if (burst_active || burst_start) begin
                    beat_valid_out <= 1'b1;
                    pixel_data_out <= ordered_beat_data;
                    pixel_mask_out <= lane_mask_comb;
                    pixel_x_base_out <= accepted_x_base;
                    pixel_y_out <= source_y;
                    beat_index_out <= accepted_index;
                    if (accepted_index == 4'd15) begin
                        burst_last_out <= 1'b1;
                        burst_active <= 1'b0;
                        beat_index <= 0;
                    end else begin
                        burst_active <= 1'b1;
                        beat_index <= accepted_index + 1'b1;
                    end
                end else begin
                    error_pulse <= 1'b1;
                end
            end
        end
    end
endmodule
