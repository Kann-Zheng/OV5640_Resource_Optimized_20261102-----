`timescale 1ns/1ps
// Small 16-bank source-tile cache for the affine bilinear sampler.
// One filtered DDR beat writes up to 16 adjacent RGB565 pixels in parallel.
// Two physical copies per bank provide the two vertical samples required for
// each x position; x0/x1 are served from their respective banks in one cycle.
module affine_tile_cache #(
    parameter TILE_WIDTH = 64,
    parameter TILE_HEIGHT = 64,
    parameter ORDERED_WRITES = 0
)(
    input wire clk,
    input wire rst_n,
    input wire tile_start,
    input wire [11:0] tile_x_min,
    input wire [11:0] tile_y_min,
    input wire [11:0] tile_width,
    input wire [11:0] tile_height,
    input wire wr_valid,
    input wire [255:0] wr_pixel_data,
    input wire [15:0] wr_pixel_mask,
    input wire [11:0] wr_x_base,
    input wire [11:0] wr_y,
    input wire tile_complete,
    output reg tile_ready = 1'b0,
    output reg tile_error_pulse = 1'b0,
    input wire rd_en,
    input wire [11:0] rd_x0,
    input wire [11:0] rd_y0,
    input wire [11:0] rd_x1,
    input wire [11:0] rd_y1,
    output wire rd_valid,
    output wire [15:0] rd_p00,
    output wire [15:0] rd_p10,
    output wire [15:0] rd_p01,
    output wire [15:0] rd_p11
);
    localparam ROW_GROUPS = TILE_WIDTH / 16;
    localparam DEPTH = TILE_HEIGHT * ROW_GROUPS;
    localparam ADDR_BITS = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam ROW_GROUP_BITS = (ROW_GROUPS <= 1) ? 1 : $clog2(ROW_GROUPS);
    localparam TILE_Y_BITS = (TILE_HEIGHT <= 1) ? 1 : $clog2(TILE_HEIGHT);
    reg [11:0] origin_x = 0;
    reg [11:0] origin_y = 0;
    reg [11:0] active_width = 0;
    reg [11:0] active_height = 0;
    // Cache the exclusive tile bounds at tile_start. Keeping the extent add
    // out of every write-lane comparison reduces the origin-to-write-enable
    // combinational path in the 16-bank DDR beat fanout.
    reg [12:0] origin_x_end = 0;
    reg [12:0] origin_y_end = 0;
    reg [12:0] accepted_pixels = 0;
    reg [12:0] expected_pixels = 0;
    reg tile_fault = 1'b0;
    reg rd_fire_q = 1'b0;
    reg [3:0] rd_bank_x0_q = 0;
    reg [3:0] rd_bank_x1_q = 0;
    wire [255:0] bank_top_data;
    wire [255:0] bank_bottom_data;
    wire [15:0] bank_duplicate_event;

    reg [15:0] wr_lane_valid_q = 0;
    reg [4:0] accepted_count_q = 0;
    reg overflow_q = 1'b0;
    reg [15:0] wr_lane_valid_comb;
    reg [4:0] accepted_count_input_comb;
    reg overflow_input_comb;
    reg [12:0] input_rel_x_base_comb;
    reg [12:0] input_rel_y_base_comb;
    reg [12:0] lane_rel_x_comb;
    integer count_lane;

    // Register each DDR beat with its tile-relative address before the
    // 16-bank decoder. This localizes the high-fanout relative-address nets.
    reg wr_valid_q = 1'b0;
    reg [255:0] wr_pixel_data_q = 0;
    reg [12:0] wr_rel_x_base_q = 0;
    reg [12:0] wr_rel_y_base_q = 0;
    reg tile_complete_q = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_valid_q <= 1'b0;
            wr_pixel_data_q <= 0;
            wr_lane_valid_q <= 0;
            accepted_count_q <= 0;
            overflow_q <= 1'b0;
            wr_rel_x_base_q <= 0;
            wr_rel_y_base_q <= 0;
            tile_complete_q <= 1'b0;
        end else begin
            wr_valid_q <= wr_valid && !tile_start;
            wr_pixel_data_q <= wr_pixel_data;
            wr_lane_valid_q <= wr_lane_valid_comb;
            accepted_count_q <= accepted_count_input_comb;
            overflow_q <= overflow_input_comb;
            wr_rel_x_base_q <= {1'b0, wr_x_base} - {1'b0, origin_x};
            wr_rel_y_base_q <= {1'b0, wr_y} - {1'b0, origin_y};
            tile_complete_q <= tile_complete && !tile_start;
        end
    end

    // Decode lane validity one cycle before the cache write. Data, address,
    // accepted-count and overflow metadata stay aligned, while active ROI
    // comparators no longer sit on the high-fanout valid_map write enable.
    always @* begin
        wr_lane_valid_comb = 16'd0;
        accepted_count_input_comb = 5'd0;
        overflow_input_comb = 1'b0;
        input_rel_x_base_comb = {1'b0, wr_x_base} - {1'b0, origin_x};
        input_rel_y_base_comb = {1'b0, wr_y} - {1'b0, origin_y};
        lane_rel_x_comb = 13'd0;
        for (count_lane = 0; count_lane < 16; count_lane = count_lane + 1) begin
            lane_rel_x_comb = input_rel_x_base_comb + count_lane;
            if (wr_valid && !tile_start && wr_pixel_mask[count_lane]) begin
                if ((lane_rel_x_comb < {1'b0, active_width}) &&
                    (lane_rel_x_comb < TILE_WIDTH) &&
                    (input_rel_y_base_comb < {1'b0, active_height}) &&
                    (input_rel_y_base_comb < TILE_HEIGHT)) begin
                    wr_lane_valid_comb[count_lane] = 1'b1;
                    accepted_count_input_comb = accepted_count_input_comb + 1'b1;
                end else begin
                    overflow_input_comb = 1'b1;
                end
            end
        end
    end
    wire [12:0] rd_rel_x0 = {1'b0, rd_x0} - {1'b0, origin_x};
    wire [12:0] rd_rel_x1 = {1'b0, rd_x1} - {1'b0, origin_x};
    wire [12:0] rd_rel_y0 = {1'b0, rd_y0} - {1'b0, origin_y};
    wire [12:0] rd_rel_y1 = {1'b0, rd_y1} - {1'b0, origin_y};
    // Relative unsigned differences also reject coordinates below the ROI:
    // subtraction wraps to a value far above the small active extent. Using
    // these four checks avoids separate absolute lower/upper-bound compares.
    wire rd_inside = (rd_rel_x0 < {1'b0, active_width}) &&
                 (rd_rel_x1 < {1'b0, active_width}) &&
                 (rd_rel_y0 < {1'b0, active_height}) &&
                 (rd_rel_y1 < {1'b0, active_height}) &&
                 ((rd_x1 == rd_x0) || (rd_x1 == (rd_x0 + 1'b1))) &&
                 ((rd_y1 == rd_y0) || (rd_y1 == (rd_y0 + 1'b1)));
    wire rd_fire = rd_en && tile_ready && !tile_start && rd_inside;
    wire [ROW_GROUP_BITS-1:0] rd_group_x0 = rd_rel_x0 >> 4;
    wire [ROW_GROUP_BITS-1:0] rd_group_x1 = rd_rel_x1 >> 4;
    wire [TILE_Y_BITS-1:0] rd_tile_y0 = rd_rel_y0[TILE_Y_BITS-1:0];
    wire [TILE_Y_BITS-1:0] rd_tile_y1 = rd_rel_y1[TILE_Y_BITS-1:0];
    wire [ADDR_BITS-1:0] rd_addr_x0_y0 = rd_tile_y0 * ROW_GROUPS + rd_group_x0;
    wire [ADDR_BITS-1:0] rd_addr_x0_y1 = rd_tile_y1 * ROW_GROUPS + rd_group_x0;
    wire [ADDR_BITS-1:0] rd_addr_x1_y0 = rd_tile_y0 * ROW_GROUPS + rd_group_x1;
    wire [ADDR_BITS-1:0] rd_addr_x1_y1 = rd_tile_y1 * ROW_GROUPS + rd_group_x1;
    wire [3:0] rd_bank_x0 = rd_rel_x0[3:0];
    wire [3:0] rd_bank_x1 = rd_rel_x1[3:0];

    // tile_ready certifies that every pixel in the active tile arrived once.
    // rd_inside bounds all four neighbors, so a second valid_map lookup on the
    // read path is redundant and forms a coordinate-driven 256:1 mux.
    assign rd_valid = rd_fire_q;
    assign rd_p00 = bank_top_data[rd_bank_x0_q*16 +: 16];
    assign rd_p10 = bank_top_data[rd_bank_x1_q*16 +: 16];
    assign rd_p01 = bank_bottom_data[rd_bank_x0_q*16 +: 16];
    assign rd_p11 = bank_bottom_data[rd_bank_x1_q*16 +: 16];

    wire duplicate_any = |bank_duplicate_event;
    integer lane;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            origin_x <= 0;
            origin_y <= 0;
            active_width <= 0;
            active_height <= 0;
            origin_x_end <= 0;
            origin_y_end <= 0;
            accepted_pixels <= 0;
            expected_pixels <= 0;
            tile_fault <= 1'b0;
            tile_ready <= 1'b0;
            tile_error_pulse <= 1'b0;
            rd_fire_q <= 1'b0;
            rd_bank_x0_q <= 0;
            rd_bank_x1_q <= 0;
        end else begin
            tile_error_pulse <= 1'b0;
            rd_fire_q <= rd_fire;
            if (rd_fire) begin
                rd_bank_x0_q <= rd_bank_x0;
                rd_bank_x1_q <= rd_bank_x1;
            end

            if (tile_start) begin
                origin_x <= tile_x_min;
                origin_y <= tile_y_min;
                active_width <= tile_width;
                active_height <= tile_height;
                origin_x_end <= {1'b0, tile_x_min} + {1'b0, tile_width};
                origin_y_end <= {1'b0, tile_y_min} + {1'b0, tile_height};
                accepted_pixels <= 0;
                expected_pixels <= {6'd0, tile_width[6:0]} * {6'd0, tile_height[6:0]};
                tile_fault <= 1'b0;
                tile_ready <= 1'b0;
                if ((tile_width == 0) || (tile_height == 0) ||
                    (tile_width > TILE_WIDTH) || (tile_height > TILE_HEIGHT)) begin
                    tile_fault <= 1'b1;
                    tile_error_pulse <= 1'b1;
                end
            end else begin
                if (wr_valid_q)
                    accepted_pixels <= accepted_pixels + accepted_count_q;
                if (duplicate_any || overflow_q) begin
                    tile_fault <= 1'b1;
                    tile_ready <= 1'b0;
                    tile_error_pulse <= 1'b1;
                end
                if (tile_complete_q) begin
                    if (!tile_fault && !duplicate_any && !overflow_q &&
                        (accepted_pixels == expected_pixels)) begin
                        tile_ready <= 1'b1;
                    end else begin
                        tile_ready <= 1'b0;
                        tile_fault <= 1'b1;
                        tile_error_pulse <= 1'b1;
                    end
                end
            end
        end
    end

    genvar bank;
    generate
        for (bank = 0; bank < 16; bank = bank + 1) begin: tile_pixel_bank
            reg [15:0] mem_top [0:DEPTH-1];
            reg [15:0] mem_bottom [0:DEPTH-1];

            reg [15:0] top_data_q = 0;
            reg [15:0] bottom_data_q = 0;
            reg duplicate_event_q = 1'b0;
            wire [3:0] first_rel_bank = wr_rel_x_base_q[3:0];
            wire [3:0] bank_wr_lane = bank[3:0] - first_rel_bank;
            wire [8:0] bank_rel_x_high = wr_rel_x_base_q[12:4] +
                                          (bank[3:0] < first_rel_bank);
            wire [12:0] bank_rel_x = {bank_rel_x_high, bank[3:0]};
            wire [12:0] bank_rel_y = wr_rel_y_base_q;
            wire [31:0] bank_wr_addr_full = bank_rel_y * ROW_GROUPS + (bank_rel_x >> 4);
            wire [ADDR_BITS-1:0] bank_wr_addr = bank_wr_addr_full[ADDR_BITS-1:0];
            wire [15:0] bank_wr_data = wr_pixel_data_q[bank_wr_lane*16 +: 16];
            wire bank_pixel_in_tile = wr_lane_valid_q[bank_wr_lane];
            wire bank_write = bank_pixel_in_tile && !tile_start;
            wire bank_read = rd_fire &&
                ((rd_bank_x0 == bank[3:0]) || (rd_bank_x1 == bank[3:0]));
            wire [ADDR_BITS-1:0] bank_rd_top_addr = (rd_bank_x0 == bank[3:0]) ?
                rd_addr_x0_y0 : rd_addr_x1_y0;
            wire [ADDR_BITS-1:0] bank_rd_bottom_addr = (rd_bank_x0 == bank[3:0]) ?
                rd_addr_x0_y1 : rd_addr_x1_y1;
            assign bank_top_data[bank*16 +: 16] = top_data_q;
            assign bank_bottom_data[bank*16 +: 16] = bottom_data_q;
            assign bank_duplicate_event[bank] = duplicate_event_q;

            // Keep block-RAM data arrays in clock-only processes with one
            // write per bank. A 16-lane DDR beat spans each of the 16 banks
            // exactly once; expressing that fact here avoids variable multi-
            // write loops and permits PDS memory inference.
            always @(posedge clk) begin
                if (bank_write) begin
                    mem_top[bank_wr_addr] <= bank_wr_data;
                    mem_bottom[bank_wr_addr] <= bank_wr_data;
                end
                if (bank_read) begin
                    top_data_q <= mem_top[bank_rd_top_addr];
                    bottom_data_q <= mem_bottom[bank_rd_bottom_addr];
                end
            end

            if (ORDERED_WRITES) begin: ordered_duplicate_check
                reg seen = 1'b0;
                reg [ADDR_BITS-1:0] last_addr = 0;
                // The read scheduler visits increasing DDR groups within each
                // row, then advances the row. Each bank therefore writes a
                // strictly increasing tile-relative address. Reject repeated
                // and backwards addresses; accepted_pixels still checks holes.
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        seen <= 1'b0;
                        last_addr <= 0;
                        duplicate_event_q <= 1'b0;
                    end else begin
                        duplicate_event_q <= 1'b0;
                        if (tile_start) begin
                            seen <= 1'b0;
                            last_addr <= 0;
                        end else if (bank_write) begin
                            duplicate_event_q <= seen && (bank_wr_addr <= last_addr);
                            last_addr <= bank_wr_addr;
                            seen <= 1'b1;
                        end
                    end
                end
            end else begin: bitmap_duplicate_check
                reg [DEPTH-1:0] valid_map;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    valid_map <= {DEPTH{1'b0}};
                    duplicate_event_q <= 1'b0;
                end else begin
                    duplicate_event_q <= 1'b0;
                    if (tile_start) begin
                        valid_map <= {DEPTH{1'b0}};
                    end else begin
                        if (bank_write) begin
                            if (valid_map[bank_wr_addr])
                                duplicate_event_q <= 1'b1;
                            valid_map[bank_wr_addr] <= 1'b1;
                        end
                    end
                end
            end
            end
        end
    endgenerate

endmodule







