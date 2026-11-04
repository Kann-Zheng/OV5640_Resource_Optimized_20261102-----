`timescale 1ns/1ps
// Registered 36x18 multiplier wrappers used by this isolated PDS candidate. The
// PDS synthesizes each multiplication into the target Logos2 APM resources.
module brown_apm_mul36x18_low (
    input wire ce,
    input wire rst,
    input wire clk,
    input wire [35:0] a,
    input wire [17:0] b,
    output reg [53:0] p
);
    wire signed [53:0] a_ext = {{18{a[35]}}, a};
    wire signed [53:0] b_ext = {36'b0, b};
    wire signed [53:0] product_comb = a_ext * b_ext;
    always @(posedge clk or posedge rst) begin
        if (rst)
            p <= 0;
        else if (ce)
            p <= product_comb;
    end
endmodule

module brown_apm_mul36x18_high (
    input wire ce,
    input wire rst,
    input wire clk,
    input wire [35:0] a,
    input wire [17:0] b,
    output reg [53:0] p
);
    wire signed [53:0] a_ext = {{18{a[35]}}, a};
    wire signed [53:0] b_ext = {{36{b[17]}}, b};
    wire signed [53:0] product_comb = a_ext * b_ext;
    always @(posedge clk or posedge rst) begin
        if (rst)
            p <= 0;
        else if (ce)
            p <= product_comb;
    end
endmodule
