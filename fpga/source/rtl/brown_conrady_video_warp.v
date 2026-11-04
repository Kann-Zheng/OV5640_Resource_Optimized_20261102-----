`timescale 1ns/1ps
// Raster-domain Brown-Conrady correction using a circular source-line store.
// The source raster leads HDMI by 20 lines. With the current 1280x720
// calibration, valid upper-left samples can be almost 9 lines behind the
// output coordinate, so up to 29 source rows may need to remain resident.
// The 30-bank cache preserves that window across a 720-line frame boundary.
module brown_conrady_video_warp #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter LINE_BANKS = 24,
    parameter FRAME_TAG_WIDTH = 16
)(
    input wire clk,
    input wire rst_n,

    input wire src_hs,
    input wire src_vs,
    input wire src_de,
    input wire [15:0] src_rgb565,

    input wire out_hs_in,
    input wire out_vs_in,
    input wire out_de_in,
    input wire [11:0] out_x,
    input wire [11:0] out_y,

    input wire brown_enable,
    input wire [11:0] cx,
    input wire [11:0] cy,
    input wire signed [31:0] fx_q16,
    input wire signed [31:0] fy_q16,
    input wire signed [31:0] inv_fx_q30,
    input wire signed [31:0] inv_fy_q30,
    input wire signed [31:0] k1_q24,
    input wire signed [31:0] k2_q24,
    input wire signed [31:0] p1_q24,
    input wire signed [31:0] p2_q24,

    output reg warp_hs = 1'b0,
    output reg warp_vs = 1'b0,
    output reg warp_de = 1'b0,
    output reg [15:0] warp_rgb565 = 16'd0
);
    localparam WORDS_PER_LINE = IMAGE_WIDTH / 2;
    localparam BANK_BITS = $clog2(LINE_BANKS);
    localparam X_BITS = $clog2(IMAGE_WIDTH);
    localparam Y_BITS = $clog2(IMAGE_HEIGHT);
    // The disabled-correction path still uses the normalized mapper. Use a
    // scale matched to the image dimensions so x/y stay within Q24 instead of
    // overflowing when a full-resolution pixel coordinate is normalized by 1.

    // Independent frame counters allow the read raster to be advanced relative
    // to display timing while retaining frame identity across the wrap point.
    reg src_vs_d = 1'b0, src_de_d = 1'b0;
    reg out_vs_d = 1'b0;
    reg [FRAME_TAG_WIDTH-1:0] src_frame_id = 0, out_frame_id = 0;
    reg [Y_BITS-1:0] src_line = 0;
    reg [X_BITS-1:0] src_x = 0;

    // For 720p, both 24 and 30 divide 720. Keep explicit low-cost bank
    // mapping for the two supported cache sizes; avoid a general modulo in
    // the per-pixel source-write path.
    function [BANK_BITS-1:0] bank_index_for_line;
        input [11:0] line_number;
        reg [4:0] high_offset;
        reg [4:0] high_mod;
        reg [5:0] bank_sum;
        begin
            if ((IMAGE_HEIGHT == 720) && (LINE_BANKS == 24)) begin
                case (line_number[9:5])
                    5'd0, 5'd3, 5'd6, 5'd9, 5'd12, 5'd15, 5'd18, 5'd21:
                        high_offset = 5'd0;
                    5'd1, 5'd4, 5'd7, 5'd10, 5'd13, 5'd16, 5'd19, 5'd22:
                        high_offset = 5'd8;
                    5'd2, 5'd5, 5'd8, 5'd11, 5'd14, 5'd17, 5'd20:
                        high_offset = 5'd16;
                    default: high_offset = 5'd0;
                endcase
                bank_sum = {1'b0, line_number[4:0]} + {1'b0, high_offset};
                if (bank_sum >= 24)
                    bank_sum = bank_sum - 24;
                bank_index_for_line = bank_sum[BANK_BITS-1:0];
            end else if ((IMAGE_HEIGHT == 720) && (LINE_BANKS == 30)) begin
                // y = 32*high + low and 32 mod 30 = 2. Reduce high modulo
                // 15, double it, then add low and subtract 30 at most once.
                high_mod = line_number[9:5];
                if (high_mod >= 15)
                    high_mod = high_mod - 15;
                high_offset = {high_mod[3:0], 1'b0};
                bank_sum = {1'b0, line_number[4:0]} + {1'b0, high_offset};
                if (bank_sum >= 30)
                    bank_sum = bank_sum - 30;
                bank_index_for_line = bank_sum[BANK_BITS-1:0];
            end else begin
                bank_index_for_line = line_number % LINE_BANKS;
            end
        end
    endfunction

    wire src_frame_start = src_vs && !src_vs_d;
    wire src_line_end = !src_de && src_de_d;
    wire out_frame_start = out_vs_in && !out_vs_d;
    wire [BANK_BITS-1:0] src_line_bank = bank_index_for_line(src_line);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_vs_d <= 1'b0;
            src_de_d <= 1'b0;
            out_vs_d <= 1'b0;
            src_frame_id <= 0;
            out_frame_id <= 0;
            src_line <= 0;
            src_x <= 0;
        end else begin
            src_vs_d <= src_vs;
            src_de_d <= src_de;
            out_vs_d <= out_vs_in;
            if (src_frame_start) begin
                src_frame_id <= src_frame_id + 1'b1;
                src_line <= 0;
                src_x <= 0;
            end else if (src_de) begin
                if (src_x < IMAGE_WIDTH-1)
                    src_x <= src_x + 1'b1;
            end else if (src_line_end) begin
                if (src_line < IMAGE_HEIGHT-1)
                    src_line <= src_line + 1'b1;
                src_x <= 0;
            end

            if (out_frame_start)
                out_frame_id <= out_frame_id + 1'b1;
        end
    end

    // Store even and odd source pixels in separate 16-bit memories. A mapped
    // odd x reads odd[word] and even[word+1] concurrently; an even x reads
    // even[word] and odd[word]. This removes the shifted packed-word layout,
    // whose hardware output produced black pixels for mapped odd x.
    reg [15:0] line_read_even [0:LINE_BANKS-1];
    reg [15:0] line_read_odd [0:LINE_BANKS-1];
    reg line_valid [0:LINE_BANKS-1];
    reg [Y_BITS-1:0] line_y_tag [0:LINE_BANKS-1];
    reg [FRAME_TAG_WIDTH-1:0] line_frame_tag [0:LINE_BANKS-1];

    wire map_valid;
    wire signed [31:0] map_src_x_q16, map_src_y_q16;
    wire map_inside;
    wire [6:0] map_valid_unused;
    reg [6:0] hs_pipe = 0, vs_pipe = 0, de_pipe = 0;
    reg [FRAME_TAG_WIDTH-1:0] frame_pipe [0:6];
    integer pipe_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_pipe <= 0;
            vs_pipe <= 0;
            de_pipe <= 0;
            for (pipe_index = 0; pipe_index < 7; pipe_index = pipe_index + 1)
                frame_pipe[pipe_index] <= 0;
        end else begin
            hs_pipe <= {hs_pipe[5:0], out_hs_in};
            vs_pipe <= {vs_pipe[5:0], out_vs_in};
            de_pipe <= {de_pipe[5:0], out_de_in};
            frame_pipe[0] <= out_frame_id;
            for (pipe_index = 1; pipe_index < 7; pipe_index = pipe_index + 1)
                frame_pipe[pipe_index] <= frame_pipe[pipe_index-1];
        end
    end

    wire [11:0] map_cx = brown_enable ? cx : 12'd0;
    wire [11:0] map_cy = brown_enable ? cy : 12'd0;
    wire signed [31:0] map_fx_q16 = brown_enable ? fx_q16 : 32'sh00010000;
    wire signed [31:0] map_fy_q16 = brown_enable ? fy_q16 : 32'sh00010000;
    wire signed [31:0] map_inv_fx_q30 = brown_enable ? inv_fx_q30 : 32'sh40000000;
    wire signed [31:0] map_inv_fy_q30 = brown_enable ? inv_fy_q30 : 32'sh40000000;
    wire signed [31:0] map_k1_q24 = brown_enable ? k1_q24 : 32'sd0;
    wire signed [31:0] map_k2_q24 = brown_enable ? k2_q24 : 32'sd0;
    wire signed [31:0] map_p1_q24 = brown_enable ? p1_q24 : 32'sd0;
    wire signed [31:0] map_p2_q24 = brown_enable ? p2_q24 : 32'sd0;

    brown_conrady_map_pipe #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT)
    ) coordinate_mapper (
        .clk(clk), .rst_n(rst_n), .valid_in(out_de_in), .bypass(~brown_enable),
        .coord_subpixel_enable(1'b0), .x_out_q16(32'sd0), .y_out_q16(32'sd0),
        .x_out(out_x), .y_out(out_y), .cx(map_cx), .cy(map_cy),
        .fx_q16(map_fx_q16), .fy_q16(map_fy_q16),
        .inv_fx_q30(map_inv_fx_q30), .inv_fy_q30(map_inv_fy_q30),
        .k1_q24(map_k1_q24), .k2_q24(map_k2_q24),
        .p1_q24(map_p1_q24), .p2_q24(map_p2_q24),
        .valid_out(map_valid), .src_x_q16(map_src_x_q16), .src_y_q16(map_src_y_q16),
        .inside_image(map_inside)
    );

    wire [Y_BITS-1:0] src_y0 = map_src_y_q16[16 +: Y_BITS];
    wire [Y_BITS-1:0] src_y1 = (src_y0 >= IMAGE_HEIGHT-1) ? src_y0 : src_y0 + 1'b1;
    wire [X_BITS-1:0] src_x0 = map_src_x_q16[16 +: X_BITS];
    wire [$clog2(WORDS_PER_LINE)-1:0] map_word = src_x0[X_BITS-1:1];
    wire [$clog2(WORDS_PER_LINE)-1:0] map_even_word =
        (src_x0[0] && (src_x0 < IMAGE_WIDTH-1)) ? map_word + 1'b1 : map_word;
    wire [BANK_BITS-1:0] row0_bank = bank_index_for_line(src_y0);
    wire [BANK_BITS-1:0] row1_bank = bank_index_for_line(src_y1);

    genvar bank;
    generate
        for (bank = 0; bank < LINE_BANKS; bank = bank + 1) begin: line_bank
            // Keep each physical line bank as its own one-dimensional memory.
            // This avoids nested unpacked arrays, which some PDS versions may
            // flatten instead of inferring as synchronous dual-port DRM RAM.
            reg [15:0] line_even_mem [0:WORDS_PER_LINE-1];
            reg [15:0] line_odd_mem [0:WORDS_PER_LINE-1];

            always @(posedge clk) begin
                // One read per lane RAM per clock; both neighbors are ready
                // together even when the mapped source coordinate is odd.
                if (map_valid && map_inside) begin
                    line_read_even[bank] <= line_even_mem[map_even_word];
                    line_read_odd[bank] <= line_odd_mem[map_word];
                end

                if (src_de && (src_line_bank == bank)) begin
                    if (src_x[0])
                        line_odd_mem[src_x[X_BITS-1:1]] <= src_rgb565;
                    else
                        line_even_mem[src_x[X_BITS-1:1]] <= src_rgb565;
                end
            end

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    line_valid[bank] <= 1'b0;
                    line_y_tag[bank] <= 0;
                    line_frame_tag[bank] <= 0;
                end else if (src_de && (src_line_bank == bank) &&
                             src_x[0] && (src_x == IMAGE_WIDTH-1)) begin
                    line_valid[bank] <= 1'b1;
                    line_y_tag[bank] <= src_line;
                    line_frame_tag[bank] <= src_frame_id;
                end
            end
        end
    endgenerate

    // Metadata stage 0 follows synchronous line RAM reads.
    reg [BANK_BITS-1:0] row0_bank_d0 = 0, row1_bank_d0 = 0;
    reg x_odd_d0 = 0;
    reg x_last_d0 = 0;
    reg [7:0] frac_x_d0 = 0, frac_y_d0 = 0;
    reg inside_d0 = 0, rows_valid_d0 = 0;
    reg hs_d0 = 0, vs_d0 = 0, de_d0 = 0;
    reg [FRAME_TAG_WIDTH-1:0] frame_d0 = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_bank_d0 <= 0; row1_bank_d0 <= 0; x_odd_d0 <= 0; x_last_d0 <= 0;
            frac_x_d0 <= 0; frac_y_d0 <= 0;
            inside_d0 <= 0; rows_valid_d0 <= 0;
            hs_d0 <= 0; vs_d0 <= 0; de_d0 <= 0; frame_d0 <= 0;
        end else if (map_valid) begin
            row0_bank_d0 <= row0_bank;
            row1_bank_d0 <= row1_bank;
            x_odd_d0 <= src_x0[0];
            x_last_d0 <= (src_x0 == IMAGE_WIDTH-1);
            frac_x_d0 <= map_src_x_q16[15:8];
            frac_y_d0 <= map_src_y_q16[15:8];
            inside_d0 <= map_inside;
            rows_valid_d0 <= map_inside &&
                line_valid[row0_bank] && (line_y_tag[row0_bank] == src_y0) &&
                (line_frame_tag[row0_bank] == frame_pipe[6]) &&
                line_valid[row1_bank] && (line_y_tag[row1_bank] == src_y1) &&
                (line_frame_tag[row1_bank] == frame_pipe[6]);
            hs_d0 <= hs_pipe[6];
            vs_d0 <= vs_pipe[6];
            de_d0 <= de_pipe[6];
            frame_d0 <= frame_pipe[6];
        end else begin
            hs_d0 <= hs_pipe[6];
            vs_d0 <= vs_pipe[6];
            de_d0 <= de_pipe[6];
        end
    end

    // Stage 1 unpacks the two adjacent source samples from each selected row.
    reg [15:0] p00_d1 = 0, p10_d1 = 0, p01_d1 = 0, p11_d1 = 0;
    reg [7:0] frac_x_d1 = 0, frac_y_d1 = 0;
    reg rows_valid_d1 = 0;
    reg hs_d1 = 0, vs_d1 = 0, de_d1 = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p00_d1 <= 0; p10_d1 <= 0; p01_d1 <= 0; p11_d1 <= 0;
            frac_x_d1 <= 0; frac_y_d1 <= 0; rows_valid_d1 <= 0;
            hs_d1 <= 0; vs_d1 <= 0; de_d1 <= 0;
        end else begin
            if (rows_valid_d0) begin
                if (x_odd_d0) begin
                    p00_d1 <= line_read_odd[row0_bank_d0];
                    p10_d1 <= x_last_d0 ? line_read_odd[row0_bank_d0] : line_read_even[row0_bank_d0];
                    p01_d1 <= line_read_odd[row1_bank_d0];
                    p11_d1 <= x_last_d0 ? line_read_odd[row1_bank_d0] : line_read_even[row1_bank_d0];
                end else begin
                    p00_d1 <= line_read_even[row0_bank_d0];
                    p10_d1 <= line_read_odd[row0_bank_d0];
                    p01_d1 <= line_read_even[row1_bank_d0];
                    p11_d1 <= line_read_odd[row1_bank_d0];
                end
            end else begin
                p00_d1 <= 0; p10_d1 <= 0; p01_d1 <= 0; p11_d1 <= 0;
            end
            frac_x_d1 <= frac_x_d0;
            frac_y_d1 <= frac_y_d0;
            rows_valid_d1 <= rows_valid_d0;
            hs_d1 <= hs_d0; vs_d1 <= vs_d0; de_d1 <= de_d0;
        end
    end

    wire [15:0] interpolated_pixel;
    bilinear_rgb565 interpolate (
        .p00(p00_d1), .p10(p10_d1), .p01(p01_d1), .p11(p11_d1),
        .frac_x(frac_x_d1), .frac_y(frac_y_d1), .pixel_out(interpolated_pixel)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            warp_hs <= 1'b0;
            warp_vs <= 1'b0;
            warp_de <= 1'b0;
            warp_rgb565 <= 0;
        end else begin
            warp_hs <= hs_d1;
            warp_vs <= vs_d1;
            warp_de <= de_d1;
            if (de_d1 && rows_valid_d1)
                warp_rgb565 <= interpolated_pixel;
            else
                warp_rgb565 <= 0;
        end
    end
endmodule
