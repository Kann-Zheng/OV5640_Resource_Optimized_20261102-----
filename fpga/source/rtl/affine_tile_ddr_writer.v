`timescale 1ns/1ps
// Writes raster rows emitted tile-major by brown_affine_frame_warp_engine.
// Each completed output-tile row is queued with its final DDR address, so the
// producer can use multi-row tiles without a full-frame reorder buffer.
module affine_tile_ddr_writer #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter OUTPUT_TILE_WIDTH = 32,
    parameter OUTPUT_TILE_HEIGHT = 32,
    parameter ADDR_WIDTH = 28,
    parameter LEN_WIDTH = 32,
    parameter DATA_WIDTH = 256,
    parameter PIXEL_WIDTH = 16,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22,
    parameter FIFO_DEPTH = 8
)(
    input wire clk,
    input wire rst_n,
    input wire frame_start,
    input wire [1:0] output_bank,
    input wire pixel_valid,
    input wire [11:0] pixel_x,
    input wire [11:0] pixel_y,
    input wire [PIXEL_WIDTH-1:0] pixel_data,
    input wire warp_frame_done,
    input wire warp_frame_error,

    output wire ddr_cmd_en,
    output wire [ADDR_WIDTH-1:0] ddr_cmd_addr,
    output wire [LEN_WIDTH-1:0] ddr_cmd_len,
    input wire ddr_cmd_ready,
    input wire ddr_data_req,
    output wire [DATA_WIDTH-1:0] ddr_data,
    input wire ddr_cmd_done,

    output reg busy = 1'b0,
    output reg frame_complete = 1'b0,
    output reg frame_error = 1'b0
);
    localparam integer PIXELS_PER_BEAT = DATA_WIDTH / PIXEL_WIDTH;
    localparam integer WORDS_PER_TILE_ROW = OUTPUT_TILE_WIDTH / PIXELS_PER_BEAT;
    localparam integer ROW_COUNT_WIDTH = (OUTPUT_TILE_WIDTH < 2) ? 1 : $clog2(OUTPUT_TILE_WIDTH + 1);
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_CMD = 2'd1;
    localparam [1:0] ST_DATA = 2'd2;
    localparam [1:0] ST_WAIT_DONE = 2'd3;

    reg [OUTPUT_TILE_WIDTH*PIXEL_WIDTH-1:0] row_shift = 0;
    reg [11:0] row_start_x = 0, row_y = 0;
    reg [ROW_COUNT_WIDTH-1:0] row_pixel_count = 0;
    reg [11:0] expected_start_x = 0, expected_row_y = 0;
    reg [1:0] output_bank_q = 0;
    reg frame_active = 1'b0;
    reg input_done = 1'b0;

    reg [OUTPUT_TILE_WIDTH*PIXEL_WIDTH-1:0] fifo_data [0:FIFO_DEPTH-1];
    reg [ADDR_WIDTH-1:0] fifo_addr [0:FIFO_DEPTH-1];
    reg [2:0] fifo_wr_ptr = 0, fifo_rd_ptr = 0;
    reg [3:0] fifo_count = 0;
    reg [1:0] state = ST_IDLE;
    reg [3:0] beat_index = 0;

    wire row_first = (row_pixel_count == 0);
    wire row_last = (row_pixel_count == OUTPUT_TILE_WIDTH - 1);
    wire row_coordinate_ok = row_first ?
        ((pixel_x == expected_start_x) && (pixel_y == expected_row_y) &&
         (pixel_x < IMAGE_WIDTH) && (pixel_y < IMAGE_HEIGHT) &&
         ((pixel_x % OUTPUT_TILE_WIDTH) == 0)) :
        ((pixel_x == row_start_x + row_pixel_count) && (pixel_y == row_y) &&
         (pixel_x < IMAGE_WIDTH) && (pixel_y < IMAGE_HEIGHT));
    wire row_enqueue = pixel_valid && frame_active && !input_done &&
                       row_coordinate_ok && row_last;
    wire row_pop = (state == ST_WAIT_DONE) && ddr_cmd_done;
    wire fifo_push_ok = (fifo_count < FIFO_DEPTH) || row_pop;

    assign ddr_cmd_en = (state == ST_CMD);
    assign ddr_cmd_addr = fifo_addr[fifo_rd_ptr];
    assign ddr_cmd_len = WORDS_PER_TILE_ROW;
    assign ddr_data = fifo_data[fifo_rd_ptr][beat_index*DATA_WIDTH +: DATA_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_shift <= 0;
            row_start_x <= 0;
            row_y <= 0;
            row_pixel_count <= 0;
            expected_start_x <= 0;
            expected_row_y <= 0;
            output_bank_q <= 0;
            frame_active <= 1'b0;
            input_done <= 1'b0;
            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
            fifo_count <= 0;
            state <= ST_IDLE;
            beat_index <= 0;
            busy <= 1'b0;
            frame_complete <= 1'b0;
            frame_error <= 1'b0;
        end else begin
            frame_complete <= 1'b0;

            if (frame_start) begin
                if (frame_active || busy) begin
                    frame_error <= 1'b1;
                end else if ((IMAGE_WIDTH <= 0) || (IMAGE_HEIGHT <= 0) ||
                             (OUTPUT_TILE_WIDTH <= 0) || (OUTPUT_TILE_HEIGHT <= 0) ||
                             (OUTPUT_TILE_WIDTH > 63) ||
                             (DATA_WIDTH % PIXEL_WIDTH != 0) ||
                             (OUTPUT_TILE_WIDTH % PIXELS_PER_BEAT != 0) ||
                             (IMAGE_WIDTH % OUTPUT_TILE_WIDTH != 0) ||
                             (FIFO_DEPTH != 8) || (WORDS_PER_TILE_ROW > 16)) begin
                    frame_error <= 1'b1;
                end else begin
                    row_shift <= 0;
                    row_pixel_count <= 0;
                    expected_start_x <= 0;
                    expected_row_y <= 0;
                    output_bank_q <= output_bank;
                    frame_active <= 1'b1;
                    input_done <= 1'b0;
                    fifo_wr_ptr <= 0;
                    fifo_rd_ptr <= 0;
                    fifo_count <= 0;
                    state <= ST_IDLE;
                    beat_index <= 0;
                    busy <= 1'b1;
                    frame_error <= 1'b0;
                end
            end else begin
                if (warp_frame_error) begin
                    frame_error <= 1'b1;
                    input_done <= 1'b1;
                end
                if (warp_frame_done)
                    input_done <= 1'b1;

                if (pixel_valid) begin
                    if (!frame_active || input_done || !row_coordinate_ok) begin
                        frame_error <= 1'b1;
                        input_done <= 1'b1;
                        row_pixel_count <= 0;
                    end else begin
                        if (row_first) begin
                            row_start_x <= pixel_x;
                            row_y <= pixel_y;
                        end
                        row_shift <= {pixel_data, row_shift[OUTPUT_TILE_WIDTH*PIXEL_WIDTH-1:PIXEL_WIDTH]};
                        if (row_last) begin
                            row_pixel_count <= 0;
                            if (!fifo_push_ok) begin
                                frame_error <= 1'b1;
                                input_done <= 1'b1;
                            end else begin
                                fifo_data[fifo_wr_ptr] <=
                                    {pixel_data, row_shift[OUTPUT_TILE_WIDTH*PIXEL_WIDTH-1:PIXEL_WIDTH]};
                                fifo_addr[fifo_wr_ptr] <=
                                    ({{(ADDR_WIDTH-2){1'b0}}, output_bank_q} << BANK_STRIDE_BITS) +
                                    (pixel_y * ROW_STRIDE_UNITS) +
                                    (row_start_x * PIXEL_WIDTH / 32);
                                fifo_wr_ptr <= fifo_wr_ptr + 1'b1;

                                // The warp engine emits all rows of one output
                                // tile before advancing to the next tile in X.
                                if ((pixel_y % OUTPUT_TILE_HEIGHT) == OUTPUT_TILE_HEIGHT - 1 ||
                                    pixel_y == IMAGE_HEIGHT - 1) begin
                                    if (row_start_x + OUTPUT_TILE_WIDTH >= IMAGE_WIDTH) begin
                                        expected_start_x <= 0;
                                        if (pixel_y + 1 >= IMAGE_HEIGHT)
                                            expected_row_y <= IMAGE_HEIGHT;
                                        else
                                            expected_row_y <= ((pixel_y / OUTPUT_TILE_HEIGHT) + 1) * OUTPUT_TILE_HEIGHT;
                                    end else begin
                                        expected_start_x <= row_start_x + OUTPUT_TILE_WIDTH;
                                        expected_row_y <= (pixel_y / OUTPUT_TILE_HEIGHT) * OUTPUT_TILE_HEIGHT;
                                    end
                                end else begin
                                    expected_start_x <= row_start_x;
                                    expected_row_y <= pixel_y + 1'b1;
                                end
                            end
                        end else begin
                            row_pixel_count <= row_pixel_count + 1'b1;
                        end
                    end
                end

                case (state)
                    ST_IDLE: begin
                        if (fifo_count != 0)
                            state <= ST_CMD;
                    end
                    ST_CMD: begin
                        if (ddr_cmd_ready) begin
                            beat_index <= 0;
                            state <= ST_DATA;
                        end
                    end
                    ST_DATA: begin
                        if (ddr_data_req) begin
                            if (beat_index == WORDS_PER_TILE_ROW - 1)
                                state <= ST_WAIT_DONE;
                            else
                                beat_index <= beat_index + 1'b1;
                        end
                    end
                    ST_WAIT_DONE: begin
                        if (ddr_cmd_done)
                            state <= ST_IDLE;
                    end
                    default: begin
                        frame_error <= 1'b1;
                        input_done <= 1'b1;
                        state <= ST_IDLE;
                    end
                endcase

                if (row_pop)
                    fifo_rd_ptr <= fifo_rd_ptr + 1'b1;

                case ({row_enqueue && fifo_push_ok, row_pop})
                    2'b10: fifo_count <= fifo_count + 1'b1;
                    2'b01: fifo_count <= fifo_count - 1'b1;
                    default: fifo_count <= fifo_count;
                endcase

                if (frame_active && input_done && (row_pixel_count == 0) &&
                    (fifo_count == 0) && (state == ST_IDLE)) begin
                    frame_active <= 1'b0;
                    input_done <= 1'b0;
                    busy <= 1'b0;
                    if (!frame_error && !warp_frame_error)
                        frame_complete <= 1'b1;
                end
            end
        end
    end
endmodule
