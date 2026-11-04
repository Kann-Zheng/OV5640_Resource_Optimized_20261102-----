`timescale 1ns/1ps
// Schedule fixed 16-beat DDR reads covering an affine source tile rectangle.
// The current Pango command translator starts every read with a 16-beat AXI
// burst and advances addresses by 8 controller units per beat. Therefore this
// scheduler requests 256-pixel aligned blocks (one 16x256-bit burst) for each
// source row that intersects the tile bounds. This is a request generator;
// it does not own the DDR channel or store returned pixels.
module affine_tile_read_scheduler #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter ROW_STRIDE_UNITS = 640,
    parameter BANK_STRIDE_BITS = 22,
    parameter BURST_BEATS = 16,
    parameter ADDR_UNITS_PER_BEAT = 8
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire has_intersection,
    input wire [11:0] src_x_min,
    input wire [11:0] src_x_max,
    input wire [11:0] src_y_min,
    input wire [11:0] src_y_max,
    input wire [1:0] frame_bank,
    input wire cmd_ready,
    output reg cmd_en = 1'b0,
    output wire [27:0] cmd_addr,
    output wire [31:0] cmd_len,
    output reg busy = 1'b0,
    output reg done_pulse = 1'b0,
    output reg error_pulse = 1'b0,
    output reg [15:0] requests_issued = 0,
    output wire [11:0] active_y,
    output wire [3:0] active_group
);
    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_ISSUE      = 3'd1;
    localparam [2:0] ST_WAIT_BUSY  = 3'd2;
    localparam [2:0] ST_WAIT_READY = 3'd3;

    reg [2:0] state = ST_IDLE;
    reg [1:0] bank_latched = 0;
    reg [11:0] row_index = 0;
    reg [11:0] last_row = 0;
    reg [3:0] first_group = 0;
    reg [3:0] group_index = 0;
    reg [3:0] last_group = 0;

    wire bounds_valid = has_intersection &&
        (src_x_min <= src_x_max) && (src_y_min <= src_y_max) &&
        (src_x_max < IMAGE_WIDTH) && (src_y_max < IMAGE_HEIGHT);
    wire bounds_malformed = has_intersection && !bounds_valid;

    wire [31:0] row_offset_units = row_index * ROW_STRIDE_UNITS;
    localparam [31:0] GROUP_ADDR_UNITS = BURST_BEATS * ADDR_UNITS_PER_BEAT;
    wire [31:0] group_offset_units = group_index * GROUP_ADDR_UNITS;
    wire [27:0] bank_base = {4'b0000, bank_latched, {(BANK_STRIDE_BITS){1'b0}}};
    assign cmd_addr = bank_base + row_offset_units[27:0] + group_offset_units[27:0];
    assign cmd_len = BURST_BEATS;
    // Tags remain stable from command issue through the last returned beat;
    // the scheduler advances only after cmd_ready returns high.
    assign active_y = row_index;
    assign active_group = group_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            bank_latched <= 0;
            row_index <= 0;
            last_row <= 0;
            first_group <= 0;
            group_index <= 0;
            last_group <= 0;
            cmd_en <= 1'b0;
            busy <= 1'b0;
            done_pulse <= 1'b0;
            error_pulse <= 1'b0;
            requests_issued <= 0;
        end else begin
            cmd_en <= 1'b0;
            done_pulse <= 1'b0;
            error_pulse <= 1'b0;
            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        requests_issued <= 0;
                        if (bounds_malformed) begin
                            error_pulse <= 1'b1;
                            done_pulse <= 1'b1;
                        end else if (!bounds_valid) begin
                            // An empty/clipped-out tile requires no DDR reads.
                            done_pulse <= 1'b1;
                        end else begin
                            bank_latched <= frame_bank;
                            row_index <= src_y_min;
                            last_row <= src_y_max;
                            first_group <= src_x_min[11:8];
                            group_index <= src_x_min[11:8];
                            last_group <= src_x_max[11:8];
                            busy <= 1'b1;
                            state <= ST_ISSUE;
                        end
                    end
                end

                ST_ISSUE: begin
                    busy <= 1'b1;
                    if (cmd_ready) begin
                        cmd_en <= 1'b1;
                        requests_issued <= requests_issued + 1'b1;
                        state <= ST_WAIT_BUSY;
                    end
                end

                // The downstream request translator edge-detects cmd_en and
                // drops cmd_ready after accepting it. Wait for that acceptance
                // before waiting for transaction completion.
                ST_WAIT_BUSY: begin
                    busy <= 1'b1;
                    if (!cmd_ready)
                        state <= ST_WAIT_READY;
                end

                ST_WAIT_READY: begin
                    busy <= 1'b1;
                    if (cmd_ready) begin
                        if (group_index < last_group) begin
                            group_index <= group_index + 1'b1;
                            state <= ST_ISSUE;
                        end else if (row_index < last_row) begin
                            row_index <= row_index + 1'b1;
                            group_index <= first_group;
                            state <= ST_ISSUE;
                        end else begin
                            busy <= 1'b0;
                            done_pulse <= 1'b1;
                            state <= ST_IDLE;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    busy <= 1'b0;
                    error_pulse <= 1'b1;
                    done_pulse <= 1'b1;
                end
            endcase
        end
    end
endmodule
