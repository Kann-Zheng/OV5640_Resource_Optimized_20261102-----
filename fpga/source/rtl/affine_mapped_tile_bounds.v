`timescale 1ns/1ps
// Accumulates source bounds from the mapped samples supplied by the prepass.
// BOUND_MARGIN can make sparse Brown-map sampling conservative; affine corner
// mode includes out-of-image corners, then clips the resulting ROI to the frame.
module affine_mapped_tile_bounds #(
    parameter IMAGE_WIDTH = 1280,
    parameter IMAGE_HEIGHT = 720,
    parameter BOUND_MARGIN = 0
)(
    input wire clk,
    input wire rst_n,
    input wire scan_start,
    input wire mapped_valid,
    input wire mapped_inside,
    input wire include_outside_bounds,
    input wire signed [31:0] src_x_q16,
    input wire signed [31:0] src_y_q16,
    input wire scan_last,
    output reg bbox_valid = 1'b0,
    output reg has_intersection = 1'b0,
    output reg [11:0] src_x_min = 0,
    output reg [11:0] src_x_max = 0,
    output reg [11:0] src_y_min = 0,
    output reg [11:0] src_y_max = 0,
    output reg error_pulse = 1'b0
);
    localparam signed [31:0] X_MAX_Q16 = (IMAGE_WIDTH - 1) << 16;
    localparam signed [31:0] Y_MAX_Q16 = (IMAGE_HEIGHT - 1) << 16;
    localparam [11:0] X_LAST = IMAGE_WIDTH - 1;
    localparam [11:0] Y_LAST = IMAGE_HEIGHT - 1;

    reg active = 1'b0;
    reg seen_point = 1'b0;
    reg [11:0] x_min_acc = 0, x_max_acc = 0;
    reg [11:0] y_min_acc = 0, y_max_acc = 0;
    reg raw_seen = 1'b0;
    reg signed [31:0] raw_x_min = 0, raw_x_max = 0;
    reg signed [31:0] raw_y_min = 0, raw_y_max = 0;

    wire signed [31:0] x_floor_signed = $signed(src_x_q16) >>> 16;
    wire signed [31:0] y_floor_signed = $signed(src_y_q16) >>> 16;
    wire coordinate_in_bounds = (src_x_q16 >= 0) && (src_y_q16 >= 0) &&
                                (src_x_q16 <= X_MAX_Q16) && (src_y_q16 <= Y_MAX_Q16);
    wire [11:0] x_floor = x_floor_signed[11:0];
    wire [11:0] y_floor = y_floor_signed[11:0];
    wire [11:0] x_ceil = ((src_x_q16[15:0] != 0) && (x_floor < X_LAST)) ? x_floor + 1'b1 : x_floor;
    wire [11:0] y_ceil = ((src_y_q16[15:0] != 0) && (y_floor < Y_LAST)) ? y_floor + 1'b1 : y_floor;
    wire accept_point = active && mapped_valid && mapped_inside && coordinate_in_bounds;
    wire malformed_point = active && mapped_valid && mapped_inside && !coordinate_in_bounds;

    wire [11:0] next_x_min = !accept_point ? x_min_acc :
        (!seen_point ? x_floor : ((x_floor < x_min_acc) ? x_floor : x_min_acc));
    wire [11:0] next_x_max = !accept_point ? x_max_acc :
        (!seen_point ? x_ceil : ((x_ceil > x_max_acc) ? x_ceil : x_max_acc));
    wire [11:0] next_y_min = !accept_point ? y_min_acc :
        (!seen_point ? y_floor : ((y_floor < y_min_acc) ? y_floor : y_min_acc));
    wire [11:0] next_y_max = !accept_point ? y_max_acc :
        (!seen_point ? y_ceil : ((y_ceil > y_max_acc) ? y_ceil : y_max_acc));
    wire seen_at_end = seen_point || accept_point;
    wire [12:0] margin_x_min = (next_x_min > BOUND_MARGIN) ? next_x_min - BOUND_MARGIN : 13'd0;
    wire [12:0] margin_y_min = (next_y_min > BOUND_MARGIN) ? next_y_min - BOUND_MARGIN : 13'd0;
    wire [12:0] margin_x_max = {1'b0, next_x_max} + BOUND_MARGIN;
    wire [12:0] margin_y_max = {1'b0, next_y_max} + BOUND_MARGIN;
    wire raw_accept = active && mapped_valid;
    wire raw_seen_at_end = raw_seen || raw_accept;
    wire signed [31:0] next_raw_x_min = !raw_seen ? src_x_q16 :
        (($signed(src_x_q16) < raw_x_min) ? src_x_q16 : raw_x_min);
    wire signed [31:0] next_raw_x_max = !raw_seen ? src_x_q16 :
        (($signed(src_x_q16) > raw_x_max) ? src_x_q16 : raw_x_max);
    wire signed [31:0] next_raw_y_min = !raw_seen ? src_y_q16 :
        (($signed(src_y_q16) < raw_y_min) ? src_y_q16 : raw_y_min);
    wire signed [31:0] next_raw_y_max = !raw_seen ? src_y_q16 :
        (($signed(src_y_q16) > raw_y_max) ? src_y_q16 : raw_y_max);
    wire signed [31:0] raw_x_floor = next_raw_x_min >>> 16;
    wire signed [31:0] raw_x_ceil_floor = next_raw_x_max >>> 16;
    wire signed [31:0] raw_y_floor = next_raw_y_min >>> 16;
    wire signed [31:0] raw_y_ceil_floor = next_raw_y_max >>> 16;
    wire signed [31:0] raw_x_ceil = (next_raw_x_max[15:0] != 0) ?
        (raw_x_ceil_floor + 1) : raw_x_ceil_floor;
    wire signed [31:0] raw_y_ceil = (next_raw_y_max[15:0] != 0) ?
        (raw_y_ceil_floor + 1) : raw_y_ceil_floor;
    wire signed [31:0] expanded_raw_x_min = raw_x_floor - BOUND_MARGIN;
    wire signed [31:0] expanded_raw_x_max = raw_x_ceil + BOUND_MARGIN;
    wire signed [31:0] expanded_raw_y_min = raw_y_floor - BOUND_MARGIN;
    wire signed [31:0] expanded_raw_y_max = raw_y_ceil + BOUND_MARGIN;
    wire raw_bbox_intersects = raw_seen_at_end &&
        (expanded_raw_x_min <= X_LAST) && (expanded_raw_x_max >= 0) &&
        (expanded_raw_y_min <= Y_LAST) && (expanded_raw_y_max >= 0);
    wire [11:0] clipped_x_min = (expanded_raw_x_min < 0) ? 12'd0 : expanded_raw_x_min[11:0];
    wire [11:0] clipped_x_max = (expanded_raw_x_max > X_LAST) ? X_LAST : expanded_raw_x_max[11:0];
    wire [11:0] clipped_y_min = (expanded_raw_y_min < 0) ? 12'd0 : expanded_raw_y_min[11:0];
    wire [11:0] clipped_y_max = (expanded_raw_y_max > Y_LAST) ? Y_LAST : expanded_raw_y_max[11:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            seen_point <= 1'b0;
            x_min_acc <= 0; x_max_acc <= 0;
            y_min_acc <= 0; y_max_acc <= 0;
            raw_seen <= 1'b0;
            raw_x_min <= 0; raw_x_max <= 0;
            raw_y_min <= 0; raw_y_max <= 0;
            bbox_valid <= 1'b0;
            has_intersection <= 1'b0;
            src_x_min <= 0; src_x_max <= 0;
            src_y_min <= 0; src_y_max <= 0;
            error_pulse <= 1'b0;
        end else begin
            bbox_valid <= 1'b0;
            error_pulse <= 1'b0;
            if (scan_start) begin
                if (active)
                    error_pulse <= 1'b1;
                active <= 1'b1;
                seen_point <= 1'b0;
                x_min_acc <= 0; x_max_acc <= 0;
                y_min_acc <= 0; y_max_acc <= 0;
                raw_seen <= 1'b0;
                raw_x_min <= 0; raw_x_max <= 0;
                raw_y_min <= 0; raw_y_max <= 0;
            end else if (active) begin
                if (accept_point) begin
                    x_min_acc <= next_x_min;
                    x_max_acc <= next_x_max;
                    y_min_acc <= next_y_min;
                    y_max_acc <= next_y_max;
                    seen_point <= 1'b1;
                end
                if (include_outside_bounds && raw_accept) begin
                    raw_x_min <= next_raw_x_min;
                    raw_x_max <= next_raw_x_max;
                    raw_y_min <= next_raw_y_min;
                    raw_y_max <= next_raw_y_max;
                    raw_seen <= 1'b1;
                end
                if (!include_outside_bounds && malformed_point)
                    error_pulse <= 1'b1;
                if (scan_last) begin
                    bbox_valid <= 1'b1;
                    if (include_outside_bounds) begin
                        has_intersection <= raw_bbox_intersects;
                        if (raw_bbox_intersects) begin
                            src_x_min <= clipped_x_min;
                            src_x_max <= clipped_x_max;
                            src_y_min <= clipped_y_min;
                            src_y_max <= clipped_y_max;
                        end else begin
                            src_x_min <= 0; src_x_max <= 0;
                            src_y_min <= 0; src_y_max <= 0;
                        end
                    end else if (seen_at_end) begin
                        has_intersection <= 1'b1;
                        src_x_min <= margin_x_min[11:0];
                        src_x_max <= (margin_x_max > X_LAST) ? X_LAST : margin_x_max[11:0];
                        src_y_min <= margin_y_min[11:0];
                        src_y_max <= (margin_y_max > Y_LAST) ? Y_LAST : margin_y_max[11:0];
                    end else begin
                        has_intersection <= 1'b0;
                        src_x_min <= 0; src_x_max <= 0;
                        src_y_min <= 0; src_y_max <= 0;
                    end
                    active <= 1'b0;
                end
            end else if (scan_last || mapped_valid) begin
                error_pulse <= 1'b1;
            end
        end
    end
endmodule
