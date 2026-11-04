`timescale 1ns/1ps
// Shares the single DDR write channel between camera frame capture and an
// optional processed-frame writer. A burst owner holds the channel through
// the controller's data requests and done pulse; clients keep request levels
// asserted until their first data request, so each level is queued once.
module affine_ddr_write_arbiter #(
    parameter ADDR_WIDTH = 28,
    parameter LEN_WIDTH = 32,
    parameter DATA_WIDTH = 256
)(
    input wire clk,
    input wire rst_n,
    input wire camera_cmd_en,
    input wire [ADDR_WIDTH-1:0] camera_cmd_addr,
    input wire [LEN_WIDTH-1:0] camera_cmd_len,
    input wire [DATA_WIDTH-1:0] camera_data,
    output wire camera_cmd_ready,
    output wire camera_cmd_done,
    output wire camera_data_req,
    input wire affine_cmd_en,
    input wire [ADDR_WIDTH-1:0] affine_cmd_addr,
    input wire [LEN_WIDTH-1:0] affine_cmd_len,
    input wire [DATA_WIDTH-1:0] affine_data,
    output wire affine_cmd_ready,
    output wire affine_cmd_done,
    output wire affine_data_req,
    output wire ctrl_cmd_en,
    output wire [ADDR_WIDTH-1:0] ctrl_cmd_addr,
    output wire [LEN_WIDTH-1:0] ctrl_cmd_len,
    input wire ctrl_cmd_ready,
    input wire ctrl_cmd_done,
    input wire ctrl_data_req,
    output wire [DATA_WIDTH-1:0] ctrl_data,
    output wire busy,
    output reg error_pulse = 1'b0
);
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_CAMERA = 2'd1;
    localparam [1:0] ST_AFFINE = 2'd2;

    reg [1:0] state = ST_IDLE;
    reg last_owner_affine = 1'b1;
    reg camera_pending = 1'b0, affine_pending = 1'b0;
    reg camera_req_seen = 1'b0, affine_req_seen = 1'b0;
    reg [ADDR_WIDTH-1:0] camera_addr_q = 0, affine_addr_q = 0;
    reg [LEN_WIDTH-1:0] camera_len_q = 0, affine_len_q = 0;

    reg [ADDR_WIDTH-1:0] active_addr = 0;
    reg [LEN_WIDTH-1:0] active_len = 0;
    wire choose_camera = camera_pending && (!affine_pending || last_owner_affine);
    wire choose_affine = affine_pending && !choose_camera;
    wire issue_camera = (state == ST_IDLE) && ctrl_cmd_ready && choose_camera;
    wire issue_affine = (state == ST_IDLE) && ctrl_cmd_ready && choose_affine;
    wire camera_new_req = camera_cmd_en && !camera_req_seen;
    wire affine_new_req = affine_cmd_en && !affine_req_seen;

    assign ctrl_cmd_en = issue_camera || issue_affine;
    assign ctrl_cmd_addr = (state != ST_IDLE) ? active_addr : (choose_camera ? camera_addr_q : affine_addr_q);
    assign ctrl_cmd_len = (state != ST_IDLE) ? active_len : (choose_camera ? camera_len_q : affine_len_q);
    assign camera_cmd_ready = !camera_pending || issue_camera;
    assign affine_cmd_ready = !affine_pending || issue_affine;
    assign camera_cmd_done = ctrl_cmd_done && (state == ST_CAMERA);
    assign affine_cmd_done = ctrl_cmd_done && (state == ST_AFFINE);
    assign camera_data_req = ctrl_data_req && (state == ST_CAMERA);
    assign affine_data_req = ctrl_data_req && (state == ST_AFFINE);
    assign ctrl_data = (state == ST_AFFINE) ? affine_data : camera_data;
    assign busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_addr <= 0; active_len <= 0;
            state <= ST_IDLE;
            last_owner_affine <= 1'b1;
            camera_pending <= 1'b0;
            affine_pending <= 1'b0;
            camera_req_seen <= 1'b0;
            affine_req_seen <= 1'b0;
            camera_addr_q <= 0;
            affine_addr_q <= 0;
            camera_len_q <= 0;
            affine_len_q <= 0;
            error_pulse <= 1'b0;
        end else begin
            error_pulse <= 1'b0;
            if (issue_camera) begin active_addr <= camera_addr_q; active_len <= camera_len_q; end
            else if (issue_affine) begin active_addr <= affine_addr_q; active_len <= affine_len_q; end

            if (!camera_cmd_en)
                camera_req_seen <= 1'b0;
            if (!affine_cmd_en)
                affine_req_seen <= 1'b0;

            if (camera_new_req) begin
                camera_req_seen <= 1'b1;
                if (!camera_pending || issue_camera) begin
                    camera_pending <= 1'b1;
                    camera_addr_q <= camera_cmd_addr;
                    camera_len_q <= camera_cmd_len;
                    if (camera_cmd_len == 0) error_pulse <= 1'b1;
                end else begin
                    error_pulse <= 1'b1;
                end
            end else if (issue_camera) begin
                camera_pending <= 1'b0;
            end

            if (affine_new_req) begin
                affine_req_seen <= 1'b1;
                if (!affine_pending || issue_affine) begin
                    affine_pending <= 1'b1;
                    affine_addr_q <= affine_cmd_addr;
                    affine_len_q <= affine_cmd_len;
                    if (affine_cmd_len == 0) error_pulse <= 1'b1;
                end else begin
                    error_pulse <= 1'b1;
                end
            end else if (issue_affine) begin
                affine_pending <= 1'b0;
            end

            case (state)
                ST_IDLE: begin
                    if (issue_camera) begin
                        state <= ST_CAMERA;
                        last_owner_affine <= 1'b0;
                    end else if (issue_affine) begin
                        state <= ST_AFFINE;
                        last_owner_affine <= 1'b1;
                    end
                end
                ST_CAMERA, ST_AFFINE: begin
                    if (ctrl_cmd_done)
                        state <= ST_IDLE;
                end
                default: begin
                    state <= ST_IDLE;
                    error_pulse <= 1'b1;
                end
            endcase
        end
    end
endmodule
