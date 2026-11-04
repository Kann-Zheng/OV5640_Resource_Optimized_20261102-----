`timescale 1ns/1ps
// Line-buffer shear with Q14 displacement and horizontal linear interpolation.
module imu_row_shear #(
    parameter H_ACTIVE = 1280
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                enable,
    input  wire [11:0]         cx,
    input  wire [11:0]         cy,
    input  wire signed [15:0]  slope_q14,
    input  wire                hs_in,
    input  wire                vs_in,
    input  wire                de_in,
    input  wire [15:0]         rgb_in,
    output wire                hs_out,
    output wire                vs_out,
    output wire                de_out,
    output wire [15:0]         rgb_out
);
    localparam WORDS = H_ACTIVE / 2;
    localparam WORD_BITS = $clog2(WORDS);
    localparam signed [28:0] X_MAX_Q14 = (H_ACTIVE-1) << 14;
    // Separate lanes provide both horizontal neighbors with one read per RAM.
    reg [15:0] line0_even [0:WORDS-1];
    reg [15:0] line0_odd [0:WORDS-1];
    reg [15:0] line1_even [0:WORDS-1];
    reg [15:0] line1_odd [0:WORDS-1];
    reg write_bank;
    reg valid0, valid1;
    reg hs_d, vs_d, de_d;
    reg [11:0] x_count, y_count;
    reg first_active_line;
    reg signed [28:0] line_shift_q14;
    reg hs_sync_d, vs_sync_d;
    reg [15:0] shear_pixel;
    reg shear_valid;

    wire line_start = hs_in && !hs_d;
    wire frame_start = vs_in && !vs_d;
    wire de_start = de_in && !de_d;
    wire [11:0] pixel_x = de_start ? 12'd0 : x_count;
    wire [11:0] line_y = first_active_line ? 12'd0 : y_count + 1'b1;
    wire signed [12:0] row_delta = $signed({1'b0, line_y}) - $signed({1'b0, cy});
    wire signed [28:0] shear_product = row_delta * slope_q14;
    wire signed [28:0] source_x_q14 = $signed({3'b0, pixel_x, 14'b0}) - line_shift_q14;
    wire signed [14:0] source_x = source_x_q14 >>> 14;
    wire source_in_bounds = source_x_q14 >= 0 && source_x_q14 <= X_MAX_Q14;
    wire [WORD_BITS-1:0] source_word = source_x[WORD_BITS:1];
    wire source_last = source_x == H_ACTIVE-1;
    wire [WORD_BITS-1:0] even_word = (source_x[0] && !source_last) ?
        source_word + 1'b1 : source_word;
    wire [15:0] even_pixel = write_bank ? line0_even[even_word] : line1_even[even_word];
    wire [15:0] odd_pixel = write_bank ? line0_odd[source_word] : line1_odd[source_word];
    wire [15:0] pixel0 = source_x[0] ? odd_pixel : even_pixel;
    wire [15:0] pixel1 = source_x[0] ? (source_last ? odd_pixel : even_pixel) : odd_pixel;
    wire [7:0] fraction = source_x_q14[13:6];
    wire [8:0] weight0 = 9'd256 - {1'b0, fraction};
    wire [8:0] weight1 = {1'b0, fraction};
    wire [13:0] red_sum = pixel0[15:11]*weight0 + pixel1[15:11]*weight1 + 14'd128;
    wire [14:0] green_sum = pixel0[10:5]*weight0 + pixel1[10:5]*weight1 + 15'd128;
    wire [13:0] blue_sum = pixel0[4:0]*weight0 + pixel1[4:0]*weight1 + 14'd128;
    wire [15:0] source_pixel = {red_sum[12:8], green_sum[13:8], blue_sum[12:8]};
    wire source_line_valid = write_bank ? valid0 : valid1;

    assign rgb_out = enable ? shear_pixel : rgb_in;
    assign de_out = enable ? shear_valid : de_in;
    // Pixel, DE, HS and VS all leave this block on the same clock edge.
    assign hs_out = enable ? hs_sync_d : hs_in;
    assign vs_out = enable ? vs_sync_d : vs_in;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_bank <= 1'b0;
            valid0 <= 1'b0;
            valid1 <= 1'b0;
            hs_d <= 1'b0;
            vs_d <= 1'b0;
            de_d <= 1'b0;
            x_count <= 0;
            y_count <= 0;
            first_active_line <= 1'b1;
            line_shift_q14 <= 0;
            hs_sync_d <= 1'b0;
            vs_sync_d <= 1'b0;
            shear_pixel <= 0;
            shear_valid <= 0;
        end else begin
            hs_d <= hs_in;
            vs_d <= vs_in;
            de_d <= de_in;
            hs_sync_d <= hs_in;
            vs_sync_d <= vs_in;

            if (frame_start) begin
                valid0 <= 1'b0;
                valid1 <= 1'b0;
                first_active_line <= 1'b1;
                y_count <= 0;
            end
            if (line_start) begin
                write_bank <= ~write_bank;
                line_shift_q14 <= shear_product;
            end
            if (vs_d && !vs_in)
                first_active_line <= 1'b1;

            shear_valid <= de_in;
            if (de_start) begin
                if (first_active_line) begin
                    y_count <= 0;
                    first_active_line <= 1'b0;
                end else begin
                    y_count <= y_count + 1'b1;
                end
            end

            if (de_in) begin
                if (write_bank) begin
                    if (pixel_x[0]) line1_odd[pixel_x[WORD_BITS:1]] <= rgb_in;
                    else line1_even[pixel_x[WORD_BITS:1]] <= rgb_in;
                end else begin
                    if (pixel_x[0]) line0_odd[pixel_x[WORD_BITS:1]] <= rgb_in;
                    else line0_even[pixel_x[WORD_BITS:1]] <= rgb_in;
                end
                if (pixel_x == H_ACTIVE-1) begin
                    if (write_bank) valid1 <= 1'b1;
                    else valid0 <= 1'b1;
                end
                if (enable) begin
                    if (source_in_bounds && source_line_valid)
                        shear_pixel <= source_pixel;
                    else
                        shear_pixel <= 16'd0;
                end else begin
                    shear_pixel <= rgb_in;
                end
                x_count <= pixel_x + 1'b1;
            end else begin
                x_count <= 0;
                shear_pixel <= 0;
            end
        end
    end
endmodule
