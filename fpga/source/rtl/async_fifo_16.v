`timescale 1ns/1ps
module async_fifo_16 #(
    parameter ADDR_BITS = 11
)(
    input  wire             wr_clk,
    input  wire             wr_rst_n,
    input  wire             wr_en,
    input  wire [15:0]      wr_data,
    output reg              wr_full,
    output wire [ADDR_BITS:0] wr_level,
    input  wire             rd_clk,
    input  wire             rd_rst_n,
    input  wire             rd_en,
    output wire [15:0]      rd_data,
    output reg              rd_empty,
    output wire [ADDR_BITS:0] rd_level
);
    localparam DEPTH = (1 << ADDR_BITS);
    reg [15:0] mem [0:DEPTH-1];
    reg [ADDR_BITS:0] wr_bin, wr_gray, rd_bin, rd_gray;
    reg [ADDR_BITS:0] rd_gray_w1, rd_gray_w2;
    reg [ADDR_BITS:0] wr_gray_r1, wr_gray_r2;
    wire [ADDR_BITS:0] wr_bin_next = wr_bin + ((wr_en && !wr_full) ? 1'b1 : 1'b0);
    wire [ADDR_BITS:0] rd_bin_next = rd_bin + ((rd_en && !rd_empty) ? 1'b1 : 1'b0);
    wire [ADDR_BITS:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [ADDR_BITS:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;
    wire [ADDR_BITS:0] rd_bin_sync = gray_to_bin(rd_gray_w2);
    wire [ADDR_BITS:0] wr_bin_sync = gray_to_bin(wr_gray_r2);
    wire [ADDR_BITS:0] full_compare = {~rd_gray_w2[ADDR_BITS:ADDR_BITS-1], rd_gray_w2[ADDR_BITS-2:0]};

    function [ADDR_BITS:0] gray_to_bin;
        input [ADDR_BITS:0] gray;
        integer i;
        begin
            gray_to_bin[ADDR_BITS] = gray[ADDR_BITS];
            for (i = ADDR_BITS-1; i >= 0; i=i-1)
                gray_to_bin[i] = gray_to_bin[i+1] ^ gray[i];
        end
    endfunction

    assign rd_data = mem[rd_bin[ADDR_BITS-1:0]];
    assign wr_level = wr_bin - rd_bin_sync;
    assign rd_level = wr_bin_sync - rd_bin;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin <= 0;
            wr_gray <= 0;
            rd_gray_w1 <= 0;
            rd_gray_w2 <= 0;
            wr_full <= 1'b0;
        end else begin
            rd_gray_w1 <= rd_gray;
            rd_gray_w2 <= rd_gray_w1;
            if (wr_en && !wr_full)
                mem[wr_bin[ADDR_BITS-1:0]] <= wr_data;
            wr_bin <= wr_bin_next;
            wr_gray <= wr_gray_next;
            wr_full <= (wr_gray_next == full_compare);
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin <= 0;
            rd_gray <= 0;
            wr_gray_r1 <= 0;
            wr_gray_r2 <= 0;
            rd_empty <= 1'b1;
        end else begin
            wr_gray_r1 <= wr_gray;
            wr_gray_r2 <= wr_gray_r1;
            rd_bin <= rd_bin_next;
            rd_gray <= rd_gray_next;
            rd_empty <= (rd_gray_next == wr_gray_r2);
        end
    end
endmodule
