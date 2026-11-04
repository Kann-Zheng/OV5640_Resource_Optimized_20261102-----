`timescale 1ns/1ps
// Transfers an infrequently updated, held configuration bus between clocks.
// The source data is captured with a toggle; the destination samples it only
// after the synchronized toggle changes, so all fields are accepted together.
module camera_config_cdc #(
    parameter WIDTH = 346
)(
    input wire src_clk,
    input wire src_rst_n,
    input wire config_valid_src,
    input wire [WIDTH-1:0] config_bus_src,
    input wire dst_clk,
    input wire dst_rst_n,
    output reg [WIDTH-1:0] config_bus_dst = {WIDTH{1'b0}},
    output reg config_applied_dst = 1'b0
);
    reg [WIDTH-1:0] config_hold_src = {WIDTH{1'b0}};
    reg config_toggle_src = 1'b0;
    reg [1:0] config_toggle_sync = 2'b00;
    reg config_toggle_seen = 1'b0;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            config_hold_src <= {WIDTH{1'b0}};
            config_toggle_src <= 1'b0;
        end else if (config_valid_src) begin
            config_hold_src <= config_bus_src;
            config_toggle_src <= ~config_toggle_src;
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            config_toggle_sync <= 2'b00;
            config_toggle_seen <= 1'b0;
            config_bus_dst <= {WIDTH{1'b0}};
            config_applied_dst <= 1'b0;
        end else begin
            config_toggle_sync <= {config_toggle_sync[0], config_toggle_src};
            config_applied_dst <= 1'b0;
            if (config_toggle_sync[1] != config_toggle_seen) begin
                config_toggle_seen <= config_toggle_sync[1];
                config_bus_dst <= config_hold_src;
                config_applied_dst <= 1'b1;
            end
        end
    end
endmodule
