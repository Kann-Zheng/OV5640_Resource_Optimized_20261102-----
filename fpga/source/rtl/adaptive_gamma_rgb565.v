`timescale 1ns/1ps
// Frame-adaptive RGB565 gamma correction. A sparse luma average selects
// shadow lift, passthrough, or highlight compression for the following frame.
module adaptive_gamma_rgb565 #(
    parameter SAMPLE_DIV = 64,
    parameter DARK_THRESHOLD = 80,
    parameter BRIGHT_THRESHOLD = 180
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        grayscale_enable,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [15:0] rgb_in,
    output reg         hs_out,
    output reg         vs_out,
    output reg         de_out,
    output reg  [15:0] rgb_out,
    output reg  [1:0]  gamma_mode
);
    localparam MODE_BYPASS = 2'd0;
    localparam MODE_SHADOW = 2'd1;
    localparam MODE_HIGHLIGHT = 2'd2;

    reg vs_d;
    localparam SAMPLE_COUNTER_BITS = (SAMPLE_DIV <= 1) ? 1 : $clog2(SAMPLE_DIV);
    reg [SAMPLE_COUNTER_BITS-1:0] sample_div_count;
    reg [15:0] sample_count;
    reg [23:0] luma_accum;
    wire frame_start = vs_in && !vs_d;

    // Exact BT.601 integer weights, expanded before shifting to avoid truncation.
    // 77 = 64+8+4+1; 150 = 128+16+4+2; 29 = 32-2-1.
    function [15:0] weighted_luma;
        input [7:0] r, g, b;
        reg [15:0] rw, gw, bw;
        begin
            rw = {8'd0, r};
            gw = {8'd0, g};
            bw = {8'd0, b};
            weighted_luma = (rw << 6) + (rw << 3) + (rw << 2) + rw
                          + (gw << 7) + (gw << 4) + (gw << 2) + (gw << 1)
                          + (bw << 5) - (bw << 1) - bw;
        end
    endfunction

    wire [7:0] r8 = {rgb_in[15:11], rgb_in[15:13]};
    wire [7:0] g8 = {rgb_in[10:5], rgb_in[10:9]};
    wire [7:0] b8 = {rgb_in[4:0], rgb_in[4:2]};
    wire [15:0] luma_weighted = weighted_luma(r8, g8, b8);
    wire [7:0] pixel_luma = luma_weighted[15:8];

    wire [31:0] extended_sample_count = {16'd0, sample_count};
    wire [31:0] dark_limit;
    wire [31:0] bright_limit;
    generate
        if (DARK_THRESHOLD == 80) begin: dark_default
            assign dark_limit = (extended_sample_count << 6) + (extended_sample_count << 4);
        end else begin: dark_custom
            assign dark_limit = sample_count * DARK_THRESHOLD;
        end
        if (BRIGHT_THRESHOLD == 180) begin: bright_default
            assign bright_limit = (extended_sample_count << 7) + (extended_sample_count << 5)
                                + (extended_sample_count << 4) + (extended_sample_count << 2);
        end else begin: bright_custom
            assign bright_limit = sample_count * BRIGHT_THRESHOLD;
        end
    endgenerate

    function [5:0] gamma_lut6;
        input [5:0] value;
        input [1:0] mode;
        begin
            gamma_lut6 = value;
            if (mode == MODE_SHADOW) begin
                case (value)
            6'd0: gamma_lut6 = 6'd0;
            6'd1: gamma_lut6 = 6'd3;
            6'd2: gamma_lut6 = 6'd5;
            6'd3: gamma_lut6 = 6'd7;
            6'd4: gamma_lut6 = 6'd9;
            6'd5: gamma_lut6 = 6'd10;
            6'd6: gamma_lut6 = 6'd12;
            6'd7: gamma_lut6 = 6'd13;
            6'd8: gamma_lut6 = 6'd14;
            6'd9: gamma_lut6 = 6'd16;
            6'd10: gamma_lut6 = 6'd17;
            6'd11: gamma_lut6 = 6'd18;
            6'd12: gamma_lut6 = 6'd19;
            6'd13: gamma_lut6 = 6'd20;
            6'd14: gamma_lut6 = 6'd21;
            6'd15: gamma_lut6 = 6'd22;
            6'd16: gamma_lut6 = 6'd23;
            6'd17: gamma_lut6 = 6'd25;
            6'd18: gamma_lut6 = 6'd26;
            6'd19: gamma_lut6 = 6'd27;
            6'd20: gamma_lut6 = 6'd28;
            6'd21: gamma_lut6 = 6'd29;
            6'd22: gamma_lut6 = 6'd30;
            6'd23: gamma_lut6 = 6'd30;
            6'd24: gamma_lut6 = 6'd31;
            6'd25: gamma_lut6 = 6'd32;
            6'd26: gamma_lut6 = 6'd33;
            6'd27: gamma_lut6 = 6'd34;
            6'd28: gamma_lut6 = 6'd35;
            6'd29: gamma_lut6 = 6'd36;
            6'd30: gamma_lut6 = 6'd37;
            6'd31: gamma_lut6 = 6'd38;
            6'd32: gamma_lut6 = 6'd39;
            6'd33: gamma_lut6 = 6'd40;
            6'd34: gamma_lut6 = 6'd40;
            6'd35: gamma_lut6 = 6'd41;
            6'd36: gamma_lut6 = 6'd42;
            6'd37: gamma_lut6 = 6'd43;
            6'd38: gamma_lut6 = 6'd44;
            6'd39: gamma_lut6 = 6'd45;
            6'd40: gamma_lut6 = 6'd45;
            6'd41: gamma_lut6 = 6'd46;
            6'd42: gamma_lut6 = 6'd47;
            6'd43: gamma_lut6 = 6'd48;
            6'd44: gamma_lut6 = 6'd49;
            6'd45: gamma_lut6 = 6'd49;
            6'd46: gamma_lut6 = 6'd50;
            6'd47: gamma_lut6 = 6'd51;
            6'd48: gamma_lut6 = 6'd52;
            6'd49: gamma_lut6 = 6'd53;
            6'd50: gamma_lut6 = 6'd53;
            6'd51: gamma_lut6 = 6'd54;
            6'd52: gamma_lut6 = 6'd55;
            6'd53: gamma_lut6 = 6'd56;
            6'd54: gamma_lut6 = 6'd56;
            6'd55: gamma_lut6 = 6'd57;
            6'd56: gamma_lut6 = 6'd58;
            6'd57: gamma_lut6 = 6'd59;
            6'd58: gamma_lut6 = 6'd59;
            6'd59: gamma_lut6 = 6'd60;
            6'd60: gamma_lut6 = 6'd61;
            6'd61: gamma_lut6 = 6'd62;
            6'd62: gamma_lut6 = 6'd62;
            6'd63: gamma_lut6 = 6'd63;
                    default: gamma_lut6 = 6'd63;
                endcase
            end else if (mode == MODE_HIGHLIGHT) begin
                case (value)
            6'd0: gamma_lut6 = 6'd0;
            6'd1: gamma_lut6 = 6'd0;
            6'd2: gamma_lut6 = 6'd1;
            6'd3: gamma_lut6 = 6'd1;
            6'd4: gamma_lut6 = 6'd2;
            6'd5: gamma_lut6 = 6'd2;
            6'd6: gamma_lut6 = 6'd3;
            6'd7: gamma_lut6 = 6'd3;
            6'd8: gamma_lut6 = 6'd4;
            6'd9: gamma_lut6 = 6'd5;
            6'd10: gamma_lut6 = 6'd5;
            6'd11: gamma_lut6 = 6'd6;
            6'd12: gamma_lut6 = 6'd7;
            6'd13: gamma_lut6 = 6'd7;
            6'd14: gamma_lut6 = 6'd8;
            6'd15: gamma_lut6 = 6'd9;
            6'd16: gamma_lut6 = 6'd10;
            6'd17: gamma_lut6 = 6'd11;
            6'd18: gamma_lut6 = 6'd12;
            6'd19: gamma_lut6 = 6'd12;
            6'd20: gamma_lut6 = 6'd13;
            6'd21: gamma_lut6 = 6'd14;
            6'd22: gamma_lut6 = 6'd15;
            6'd23: gamma_lut6 = 6'd16;
            6'd24: gamma_lut6 = 6'd17;
            6'd25: gamma_lut6 = 6'd18;
            6'd26: gamma_lut6 = 6'd19;
            6'd27: gamma_lut6 = 6'd20;
            6'd28: gamma_lut6 = 6'd21;
            6'd29: gamma_lut6 = 6'd22;
            6'd30: gamma_lut6 = 6'd23;
            6'd31: gamma_lut6 = 6'd24;
            6'd32: gamma_lut6 = 6'd25;
            6'd33: gamma_lut6 = 6'd26;
            6'd34: gamma_lut6 = 6'd27;
            6'd35: gamma_lut6 = 6'd28;
            6'd36: gamma_lut6 = 6'd30;
            6'd37: gamma_lut6 = 6'd31;
            6'd38: gamma_lut6 = 6'd32;
            6'd39: gamma_lut6 = 6'd33;
            6'd40: gamma_lut6 = 6'd34;
            6'd41: gamma_lut6 = 6'd35;
            6'd42: gamma_lut6 = 6'd36;
            6'd43: gamma_lut6 = 6'd38;
            6'd44: gamma_lut6 = 6'd39;
            6'd45: gamma_lut6 = 6'd40;
            6'd46: gamma_lut6 = 6'd41;
            6'd47: gamma_lut6 = 6'd42;
            6'd48: gamma_lut6 = 6'd44;
            6'd49: gamma_lut6 = 6'd45;
            6'd50: gamma_lut6 = 6'd46;
            6'd51: gamma_lut6 = 6'd47;
            6'd52: gamma_lut6 = 6'd49;
            6'd53: gamma_lut6 = 6'd50;
            6'd54: gamma_lut6 = 6'd51;
            6'd55: gamma_lut6 = 6'd52;
            6'd56: gamma_lut6 = 6'd54;
            6'd57: gamma_lut6 = 6'd55;
            6'd58: gamma_lut6 = 6'd56;
            6'd59: gamma_lut6 = 6'd58;
            6'd60: gamma_lut6 = 6'd59;
            6'd61: gamma_lut6 = 6'd60;
            6'd62: gamma_lut6 = 6'd62;
            6'd63: gamma_lut6 = 6'd63;
                    default: gamma_lut6 = 6'd63;
                endcase
            end
        end
    endfunction

    wire [5:0] r_raw = {rgb_in[15:11], rgb_in[15]};
    wire [5:0] g_raw = rgb_in[10:5];
    wire [5:0] b_raw = {rgb_in[4:0], rgb_in[4]};
    wire [5:0] r_lut = gamma_lut6(r_raw, gamma_mode);
    wire [5:0] g_lut = gamma_lut6(g_raw, gamma_mode);
    wire [5:0] b_lut = gamma_lut6(b_raw, gamma_mode);
    wire [5:0] r_selected = enable ? r_lut : r_raw;
    wire [5:0] g_selected = enable ? g_lut : g_raw;
    wire [5:0] b_selected = enable ? b_lut : b_raw;
    wire [7:0] r8_selected = {r_selected, r_selected[5:4]};
    wire [7:0] g8_selected = {g_selected, g_selected[5:4]};
    wire [7:0] b8_selected = {b_selected, b_selected[5:4]};
    wire [15:0] gray_luma_weighted = weighted_luma(r8_selected, g8_selected, b8_selected);
    wire [7:0] gray_luma = gray_luma_weighted[15:8];
    wire [15:0] color_rgb565 = {r_selected[5:1], g_selected, b_selected[5:1]};
    wire [15:0] gray_rgb565 = {gray_luma[7:3], gray_luma[7:2], gray_luma[7:3]};
    wire [15:0] selected_rgb565 = grayscale_enable ? gray_rgb565 : color_rgb565;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            sample_div_count <= {SAMPLE_COUNTER_BITS{1'b0}};
            sample_count <= 16'd0;
            luma_accum <= 24'd0;
            gamma_mode <= MODE_BYPASS;
            hs_out <= 1'b0;
            vs_out <= 1'b0;
            de_out <= 1'b0;
            rgb_out <= 16'd0;
        end else begin
            vs_d <= vs_in;

            // All video signals pass through the same one-clock stage.
            hs_out <= hs_in;
            vs_out <= vs_in;
            de_out <= de_in;
            rgb_out <= de_in ? selected_rgb565 : rgb_in;

            if (frame_start) begin
                if (sample_count != 0) begin
                    if (luma_accum < dark_limit)
                        gamma_mode <= MODE_SHADOW;
                    else if (luma_accum > bright_limit)
                        gamma_mode <= MODE_HIGHLIGHT;
                    else
                        gamma_mode <= MODE_BYPASS;
                end
                sample_div_count <= {SAMPLE_COUNTER_BITS{1'b0}};
                sample_count <= 16'd0;
                luma_accum <= 24'd0;
            end else if (de_in) begin
                if (sample_div_count == SAMPLE_DIV - 1) begin
                    sample_div_count <= {SAMPLE_COUNTER_BITS{1'b0}};
                    if (sample_count != 16'hffff) begin
                        sample_count <= sample_count + 1'b1;
                        luma_accum <= luma_accum + pixel_luma;
                    end
                end else begin
                    sample_div_count <= sample_div_count + 1'b1;
                end
            end
        end
    end
endmodule


