`timescale 1ns/1ps
// Captures raster-ordered RGB565 pixels into two line buffers and writes each
// complete line as a 256-bit DDR burst. Configure the upstream frame warp with
// OUTPUT_TILE_HEIGHT=1 so its destination stream is raster ordered.
module affine_raster_ddr_writer #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter ADDR_WIDTH = 28,
    parameter LEN_WIDTH = 32,
    parameter DATA_WIDTH = 256,
    parameter PIXEL_WIDTH = 16,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22
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
    localparam PIXELS_PER_BEAT = DATA_WIDTH / PIXEL_WIDTH;
    localparam ROW_BEATS = IMAGE_WIDTH / PIXELS_PER_BEAT;
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_CMD = 2'd1;
    localparam [1:0] ST_DATA = 2'd2;
    localparam [1:0] ST_DONE = 2'd3;

    // Store 256-bit DDR words rather than pixels so the burst side performs
    // one synchronous-looking word selection instead of 16 independent reads.
    reg [DATA_WIDTH-1:0] row0_mem [0:ROW_BEATS-1];
    reg [DATA_WIDTH-1:0] row1_mem [0:ROW_BEATS-1];
    // Assemble a complete DDR word before writing the row RAM. Updating
    // individual RAM bit slices prevents block-memory inference in PDS.
    reg [DATA_WIDTH-1:0] row0_assembly = {DATA_WIDTH{1'b0}};
    reg [DATA_WIDTH-1:0] row1_assembly = {DATA_WIDTH{1'b0}};
    // Register each completed word before it reaches the inferred row RAM.
    // Besides making the write boundary explicit, this avoids a near-zero
    // hold path from the assembly flops to the RAM data input.
    reg pending_write_valid = 1'b0;
    reg pending_write_row = 1'b0;
    reg [15:0] pending_write_addr = 0;
    reg [DATA_WIDTH-1:0] pending_write_data = {DATA_WIDTH{1'b0}};
    reg row0_full = 1'b0, row1_full = 1'b0;
    reg [11:0] row0_y = 0, row1_y = 0;
    reg [1:0] state = ST_IDLE;
    reg frame_active = 1'b0;
    reg input_done = 1'b0;
    reg all_pixels_seen = 1'b0;
    reg [11:0] expected_x = 0, expected_y = 0;
    reg [1:0] output_bank_q = 0;
    reg write_row_select = 0;
    reg [11:0] write_row_y = 0;
    reg [15:0] beat_index = 0;
    reg [DATA_WIDTH-1:0] data_mux;

    always @* begin
        if (write_row_select == 1'b0)
            data_mux = row0_mem[beat_index];
        else
            data_mux = row1_mem[beat_index];
    end

    assign ddr_cmd_en = (state == ST_CMD);
    assign ddr_cmd_addr = ({{(ADDR_WIDTH-2){1'b0}}, output_bank_q} << BANK_STRIDE_BITS) +
                          (write_row_y * ROW_STRIDE_UNITS);
    assign ddr_cmd_len = ROW_BEATS;
    assign ddr_data = data_mux;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_full <= 1'b0;
            row1_full <= 1'b0;
            pending_write_valid <= 1'b0;
            pending_write_row <= 1'b0;
            pending_write_addr <= 0;
            pending_write_data <= {DATA_WIDTH{1'b0}};
            row0_y <= 0;
            row1_y <= 0;
            state <= ST_IDLE;
            frame_active <= 1'b0;
            input_done <= 1'b0;
            all_pixels_seen <= 1'b0;
            expected_x <= 0;
            expected_y <= 0;
            output_bank_q <= 0;
            write_row_select <= 0;
            write_row_y <= 0;
            beat_index <= 0;
            busy <= 1'b0;
            frame_complete <= 1'b0;
            frame_error <= 1'b0;
        end else begin
            frame_complete <= 1'b0;
            pending_write_valid <= 1'b0;

            // Commit the previous completed DDR word to the selected line
            // buffer. New words arrive only once per PIXELS_PER_BEAT pixels,
            // so this one-cycle stage cannot be overrun by the raster stream.
            if (pending_write_valid) begin
                if (pending_write_row)
                    row1_mem[pending_write_addr] <= pending_write_data;
                else
                    row0_mem[pending_write_addr] <= pending_write_data;
            end

            if (frame_start) begin
                if (state != ST_IDLE) begin
                    frame_error <= 1'b1;
                end else begin
                    row0_full <= 1'b0;
                    row1_full <= 1'b0;
                    pending_write_valid <= 1'b0;
                    row0_y <= 0;
                    row1_y <= 0;
                    frame_active <= 1'b1;
                    input_done <= 1'b0;
                    all_pixels_seen <= 1'b0;
                    expected_x <= 0;
                    expected_y <= 0;
                    output_bank_q <= output_bank;
                    write_row_select <= 0;
                    write_row_y <= 0;
                    beat_index <= 0;
                    busy <= 1'b1;
                    frame_error <= (IMAGE_WIDTH <= 0) || (IMAGE_HEIGHT <= 0) ||
                                   (DATA_WIDTH % PIXEL_WIDTH != 0) ||
                                   (IMAGE_WIDTH % PIXELS_PER_BEAT != 0) ||
                                   (IMAGE_WIDTH > 4095) || (IMAGE_HEIGHT > 4095);
                end
            end else if (frame_active) begin
                if (warp_frame_error)
                    frame_error <= 1'b1;

                if (pixel_valid && !frame_error && !warp_frame_error) begin
                    if ((pixel_x != expected_x) || (pixel_y != expected_y) ||
                        (expected_x >= IMAGE_WIDTH) || (expected_y >= IMAGE_HEIGHT)) begin
                        frame_error <= 1'b1;
                    end else if (expected_y[0] == 1'b0) begin
                        if (row0_full) begin
                            frame_error <= 1'b1;
                        end else begin
                            row0_assembly[(expected_x % PIXELS_PER_BEAT)*PIXEL_WIDTH +: PIXEL_WIDTH] <= pixel_data;
                            if ((expected_x % PIXELS_PER_BEAT) == (PIXELS_PER_BEAT - 1)) begin
                                pending_write_valid <= 1'b1;
                                pending_write_row <= 1'b0;
                                pending_write_addr <= expected_x / PIXELS_PER_BEAT;
                                pending_write_data <=
                                    {pixel_data, row0_assembly[DATA_WIDTH-PIXEL_WIDTH-1:0]};
                            end
                            if (expected_x == IMAGE_WIDTH - 1) begin
                                row0_y <= expected_y;
                                row0_full <= 1'b1;
                            end
                        end
                    end else begin
                        if (row1_full) begin
                            frame_error <= 1'b1;
                        end else begin
                            row1_assembly[(expected_x % PIXELS_PER_BEAT)*PIXEL_WIDTH +: PIXEL_WIDTH] <= pixel_data;
                            if ((expected_x % PIXELS_PER_BEAT) == (PIXELS_PER_BEAT - 1)) begin
                                pending_write_valid <= 1'b1;
                                pending_write_row <= 1'b1;
                                pending_write_addr <= expected_x / PIXELS_PER_BEAT;
                                pending_write_data <=
                                    {pixel_data, row1_assembly[DATA_WIDTH-PIXEL_WIDTH-1:0]};
                            end
                            if (expected_x == IMAGE_WIDTH - 1) begin
                                row1_y <= expected_y;
                                row1_full <= 1'b1;
                            end
                        end
                    end

                    if ((pixel_x == IMAGE_WIDTH - 1) && (pixel_y == IMAGE_HEIGHT - 1)) begin
                        all_pixels_seen <= 1'b1;
                        expected_x <= 0;
                    end else if (expected_x == IMAGE_WIDTH - 1) begin
                        expected_x <= 0;
                        expected_y <= expected_y + 1'b1;
                    end else begin
                        expected_x <= expected_x + 1'b1;
                    end
                end

                if (warp_frame_done) begin
                    if (!all_pixels_seen &&
                        !(pixel_valid && pixel_x == IMAGE_WIDTH - 1 && pixel_y == IMAGE_HEIGHT - 1))
                        frame_error <= 1'b1;
                    input_done <= 1'b1;
                end

                case (state)
                    ST_IDLE: begin
                        if (!frame_error && !warp_frame_error && (row0_full || row1_full)) begin
                            if (row0_full && (!row1_full || (row0_y <= row1_y))) begin
                                write_row_select <= 1'b0;
                                write_row_y <= row0_y;
                            end else begin
                                write_row_select <= 1'b1;
                                write_row_y <= row1_y;
                            end
                            beat_index <= 0;
                            state <= ST_CMD;
                        end
                    end
                    ST_CMD: begin
                        if (ddr_cmd_ready)
                            state <= ST_DATA;
                    end
                    ST_DATA: begin
                        if (ddr_data_req) begin
                            if (beat_index == ROW_BEATS - 1)
                                state <= ST_DONE;
                            else
                                beat_index <= beat_index + 1'b1;
                        end
                    end
                    ST_DONE: begin
                        if (ddr_cmd_done) begin
                            if (write_row_select == 1'b0)
                                row0_full <= 1'b0;
                            else
                                row1_full <= 1'b0;
                            state <= ST_IDLE;
                        end
                    end
                    default: begin
                        frame_error <= 1'b1;
                        state <= ST_IDLE;
                    end
                endcase

                if (input_done && all_pixels_seen && !frame_error && !warp_frame_error &&
                    (state == ST_IDLE) && !row0_full && !row1_full) begin
                    frame_complete <= 1'b1;
                    frame_active <= 1'b0;
                    busy <= 1'b0;
                end

                // On a bad frame, finish any already-issued burst first, then
                // release the writer and discard buffered rows so the caller
                // can start a later frame without leaking bank ownership.
                if ((frame_error || warp_frame_error) && (state == ST_IDLE)) begin
                    frame_active <= 1'b0;
                    input_done <= 1'b0;
                    all_pixels_seen <= 1'b0;
                    row0_full <= 1'b0;
                    row1_full <= 1'b0;
                    busy <= 1'b0;
                end
            end
        end
    end
endmodule
