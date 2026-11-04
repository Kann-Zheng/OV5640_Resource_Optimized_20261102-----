// Four-neighbor bilinear interpolation for RGB565 pixels.
// Fraction inputs are the top 8 fractional bits of Q16.16 source coordinates.
module bilinear_rgb565(
    input  wire [15:0] p00,
    input  wire [15:0] p10,
    input  wire [15:0] p01,
    input  wire [15:0] p11,
    input  wire [7:0]  frac_x,
    input  wire [7:0]  frac_y,
    output wire [15:0] pixel_out
);
    wire [8:0] wx0 = 9'd256 - {1'b0, frac_x};
    wire [8:0] wx1 = {1'b0, frac_x};
    wire [8:0] wy0 = 9'd256 - {1'b0, frac_y};
    wire [8:0] wy1 = {1'b0, frac_y};
    wire [17:0] w00 = wx0 * wy0;
    wire [17:0] w10 = wx1 * wy0;
    wire [17:0] w01 = wx0 * wy1;
    wire [17:0] w11 = wx1 * wy1;

    wire [23:0] red_sum = p00[15:11] * w00 + p10[15:11] * w10 +
                           p01[15:11] * w01 + p11[15:11] * w11;
    wire [23:0] green_sum = p00[10:5] * w00 + p10[10:5] * w10 +
                             p01[10:5] * w01 + p11[10:5] * w11;
    wire [23:0] blue_sum = p00[4:0] * w00 + p10[4:0] * w10 +
                            p01[4:0] * w01 + p11[4:0] * w11;
    wire [4:0] red = (red_sum + 24'd32768) >> 16;
    wire [5:0] green = (green_sum + 24'd32768) >> 16;
    wire [4:0] blue = (blue_sum + 24'd32768) >> 16;
    assign pixel_out = {red, green, blue};
endmodule
