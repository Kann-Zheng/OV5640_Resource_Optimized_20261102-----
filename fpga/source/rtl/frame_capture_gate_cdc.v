`timescale 1ns/1ps
// Synchronizes frame ownership permission into the camera pixel clock and
// blocks writes for frames that were not reserved by the DDR-domain manager.
module frame_capture_gate_cdc (
    input wire camera_clk,
    input wire camera_rst_n,
    input wire capture_allowed,
    input wire affine_mode,
    input wire pixel_enable_in,
    output wire pixel_enable_out
);
    reg allow_meta = 1'b0;
    reg allow_sync = 1'b0;

    always @(posedge camera_clk or negedge camera_rst_n) begin
        if (!camera_rst_n) begin
            allow_meta <= 1'b0;
            allow_sync <= 1'b0;
        end else begin
            allow_meta <= capture_allowed;
            allow_sync <= allow_meta;
        end
    end

    assign pixel_enable_out = pixel_enable_in && (!affine_mode || allow_sync);
endmodule
