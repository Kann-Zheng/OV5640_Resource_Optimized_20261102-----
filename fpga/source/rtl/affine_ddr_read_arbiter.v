`timescale 1ns/1ps
// Shares the controller's single read-command/data stream between live HDMI
// scanout and the affine tile engine. Display commands have priority once
// queued; an in-flight burst is never preempted, and its return data stays
// routed to the owner until controller ready returns after a busy interval.
module affine_ddr_read_arbiter #(
    parameter ADDR_WIDTH = 28,
    parameter LEN_WIDTH = 32,
    parameter DATA_WIDTH = 256
)(
    input wire clk,
    input wire rst_n,

    input wire display_cmd_en,
    input wire [ADDR_WIDTH-1:0] display_cmd_addr,
    input wire [LEN_WIDTH-1:0] display_cmd_len,
    output wire display_cmd_ready,
    output wire display_cmd_done,
    output wire display_read_valid,
    output wire [DATA_WIDTH-1:0] display_read_data,

    input wire affine_cmd_en,
    input wire [ADDR_WIDTH-1:0] affine_cmd_addr,
    input wire [LEN_WIDTH-1:0] affine_cmd_len,
    output wire affine_cmd_ready,
    output wire affine_cmd_done,
    output wire affine_read_valid,
    output wire [DATA_WIDTH-1:0] affine_read_data,

    output wire ctrl_cmd_en,
    output wire [ADDR_WIDTH-1:0] ctrl_cmd_addr,
    output wire [LEN_WIDTH-1:0] ctrl_cmd_len,
    input wire ctrl_cmd_ready,
    input wire ctrl_cmd_done,
    input wire ctrl_read_valid,
    input wire [DATA_WIDTH-1:0] ctrl_read_data,

    output wire busy,
    output reg error_pulse = 1'b0
);
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_DISPLAY = 2'd1;
    localparam [1:0] ST_AFFINE = 2'd2;

    reg [1:0] state = ST_IDLE;
    reg saw_busy = 1'b0;
    reg display_pending = 1'b0;
    reg affine_pending = 1'b0;
    reg [ADDR_WIDTH-1:0] display_addr_q = 0;
    reg [LEN_WIDTH-1:0] display_len_q = 0;
    reg [ADDR_WIDTH-1:0] affine_addr_q = 0;
    reg [LEN_WIDTH-1:0] affine_len_q = 0;

    reg [ADDR_WIDTH-1:0] active_addr = 0;
    reg [LEN_WIDTH-1:0] active_len = 0;
    wire issue_display = (state == ST_IDLE) && ctrl_cmd_ready && display_pending;
    wire issue_affine = (state == ST_IDLE) && ctrl_cmd_ready && !display_pending && affine_pending;
    assign ctrl_cmd_en = issue_display || issue_affine;
    assign ctrl_cmd_addr = (state != ST_IDLE) ? active_addr : (display_pending ? display_addr_q : affine_addr_q);
    assign ctrl_cmd_len = (state != ST_IDLE) ? active_len : (display_pending ? display_len_q : affine_len_q);

    assign busy = (state != ST_IDLE);
    assign display_cmd_ready =
        ((state == ST_IDLE) && ctrl_cmd_ready && !display_pending) ||
        ((state == ST_DISPLAY) && saw_busy && ctrl_cmd_ready);
    assign affine_cmd_ready =
        ((state == ST_IDLE) && ctrl_cmd_ready && !display_pending && !affine_pending &&
         !display_cmd_en && !affine_cmd_en) ||
        ((state == ST_AFFINE) && saw_busy && ctrl_cmd_ready);

    assign display_cmd_done = ctrl_cmd_done && (state == ST_DISPLAY);
    assign affine_cmd_done = ctrl_cmd_done && (state == ST_AFFINE);
    assign display_read_valid = ctrl_read_valid && (state == ST_DISPLAY);
    assign affine_read_valid = ctrl_read_valid && (state == ST_AFFINE);
    assign display_read_data = ctrl_read_data;
    assign affine_read_data = ctrl_read_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_addr <= 0; active_len <= 0;
            state <= ST_IDLE;
            saw_busy <= 1'b0;
            display_pending <= 1'b0;
            affine_pending <= 1'b0;
            display_addr_q <= 0;
            display_len_q <= 0;
            affine_addr_q <= 0;
            affine_len_q <= 0;
            error_pulse <= 1'b0;
        end else begin
            error_pulse <= 1'b0;
            if (issue_display) begin active_addr <= display_addr_q; active_len <= display_len_q; end
            else if (issue_affine) begin active_addr <= affine_addr_q; active_len <= affine_len_q; end


            // Each client has a one-entry request queue. Preserve a new pulse
            // arriving on the same cycle its older queued request is issued.
            if (display_cmd_en) begin
                if (!display_pending || issue_display) begin
                    display_pending <= 1'b1;
                    display_addr_q <= display_cmd_addr;
                    display_len_q <= display_cmd_len;
                    if (display_cmd_len == 0)
                        error_pulse <= 1'b1;
                end else begin
                    error_pulse <= 1'b1;
                end
            end else if (issue_display) begin
                display_pending <= 1'b0;
            end

            if (affine_cmd_en) begin
                if (!affine_pending || issue_affine) begin
                    affine_pending <= 1'b1;
                    affine_addr_q <= affine_cmd_addr;
                    affine_len_q <= affine_cmd_len;
                    if (affine_cmd_len == 0)
                        error_pulse <= 1'b1;
                end else begin
                    error_pulse <= 1'b1;
                end
            end else if (issue_affine) begin
                affine_pending <= 1'b0;
            end

            case (state)
                ST_IDLE: begin
                    saw_busy <= 1'b0;
                    if (issue_display)
                        state <= ST_DISPLAY;
                    else if (issue_affine)
                        state <= ST_AFFINE;
                end
                ST_DISPLAY, ST_AFFINE: begin
                    if (!ctrl_cmd_ready)
                        saw_busy <= 1'b1;
                    else if (saw_busy) begin
                        state <= ST_IDLE;
                        saw_busy <= 1'b0;
                    end
                end
                default: begin
                    state <= ST_IDLE;
                    saw_busy <= 1'b0;
                    error_pulse <= 1'b1;
                end
            endcase
        end
    end
endmodule
