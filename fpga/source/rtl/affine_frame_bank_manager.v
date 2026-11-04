`timescale 1ns/1ps
// Four-bank ownership policy for a full-frame affine pipeline.
// Banks 0/1 are camera input; banks 2/3 are completed/visible output frames.
// This block only manages ownership and frame-boundary switching; it does not
// generate DDR traffic or cross clock domains.
module affine_frame_bank_manager (
    input wire clk,
    input wire rst_n,

    input wire camera_frame_start,
    input wire camera_frame_done,
    output reg camera_capture_active = 1'b0,
    output reg camera_frame_drop = 1'b0,
    output reg [1:0] camera_write_bank = 2'd0,

    input wire process_start,
    output reg process_accept = 1'b0,
    output reg process_active = 1'b0,
    output reg [1:0] process_read_bank = 2'd0,
    output reg [1:0] process_write_bank = 2'd2,
    input wire process_done,
    input wire process_success,
    output wire process_can_start,
    output wire pending_frame_valid,

    input wire display_vsync,
    output reg display_valid = 1'b0,
    output reg [1:0] display_bank = 2'd2,
    output reg output_frame_valid = 1'b0,
    output reg [1:0] latest_output_bank = 2'd2,

    output reg drop_pulse = 1'b0,
    output reg error_pulse = 1'b0
);
    reg pending_input_valid = 1'b0;
    reg [1:0] pending_input_bank = 2'd0;

    wire input_bank0_busy = (pending_input_valid && pending_input_bank == 2'd0) ||
                            (process_active && process_read_bank == 2'd0);
    wire input_bank1_busy = (pending_input_valid && pending_input_bank == 2'd1) ||
                            (process_active && process_read_bank == 2'd1);
    wire input_bank0_free = !camera_capture_active && !input_bank0_busy;
    wire input_bank1_free = !camera_capture_active && !input_bank1_busy;

    wire output_bank2_busy = (output_frame_valid && latest_output_bank == 2'd2) ||
                             (display_valid && display_bank == 2'd2) ||
                             (process_active && process_write_bank == 2'd2);
    wire output_bank3_busy = (output_frame_valid && latest_output_bank == 2'd3) ||
                             (display_valid && display_bank == 2'd3) ||
                             (process_active && process_write_bank == 2'd3);
    wire output_bank2_free = !output_bank2_busy;
    wire output_bank3_free = !output_bank3_busy;
    assign process_can_start = !process_active && pending_input_valid &&
                               (output_bank2_free || output_bank3_free);
    assign pending_frame_valid = pending_input_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            camera_capture_active <= 1'b0;
            camera_frame_drop <= 1'b0;
            camera_write_bank <= 2'd0;
            pending_input_valid <= 1'b0;
            pending_input_bank <= 2'd0;
            process_accept <= 1'b0;
            process_active <= 1'b0;
            process_read_bank <= 2'd0;
            process_write_bank <= 2'd2;
            display_valid <= 1'b0;
            display_bank <= 2'd2;
            output_frame_valid <= 1'b0;
            latest_output_bank <= 2'd2;
            drop_pulse <= 1'b0;
            error_pulse <= 1'b0;
        end else begin
            process_accept <= 1'b0;
            drop_pulse <= 1'b0;
            error_pulse <= 1'b0;

            if (camera_frame_start) begin
                if (camera_capture_active) begin
                    // A new sync arrived before the prior frame was closed.
                    camera_capture_active <= 1'b0;
                    camera_frame_drop <= 1'b0;
                    error_pulse <= 1'b1;
                end else if (input_bank0_free) begin
                    camera_capture_active <= 1'b1;
                    camera_frame_drop <= 1'b0;
                    camera_write_bank <= 2'd0;
                end else if (input_bank1_free) begin
                    camera_capture_active <= 1'b1;
                    camera_frame_drop <= 1'b0;
                    camera_write_bank <= 2'd1;
                end else begin
                    camera_capture_active <= 1'b0;
                    camera_frame_drop <= 1'b1;
                    drop_pulse <= 1'b1;
                end
            end

            if (camera_frame_done) begin
                if (camera_capture_active) begin
                    camera_capture_active <= 1'b0;
                    camera_frame_drop <= 1'b0;
                    pending_input_valid <= 1'b1;
                    pending_input_bank <= camera_write_bank;
                end else if (!camera_frame_drop) begin
                    error_pulse <= 1'b1;
                end else begin
                    camera_frame_drop <= 1'b0;
                end
            end

            if (process_start) begin
                if (!process_active && pending_input_valid && (output_bank2_free || output_bank3_free)) begin
                    process_active <= 1'b1;
                    process_accept <= 1'b1;
                    process_read_bank <= pending_input_bank;
                    process_write_bank <= output_bank2_free ? 2'd2 : 2'd3;
                    pending_input_valid <= 1'b0;
                end else begin
                    error_pulse <= 1'b1;
                end
            end

            if (process_done) begin
                if (process_active) begin
                    process_active <= 1'b0;
                    if (process_success) begin
                        output_frame_valid <= 1'b1;
                        latest_output_bank <= process_write_bank;
                    end
                end else begin
                    error_pulse <= 1'b1;
                end
            end

            // A same-cycle process completion becomes visible at the next
            // vertical boundary, avoiding a partial-frame bank switch.
            if (display_vsync && output_frame_valid) begin
                display_valid <= 1'b1;
                display_bank <= latest_output_bank;
            end
        end
    end
endmodule
