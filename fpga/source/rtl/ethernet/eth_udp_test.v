`timescale 1ns/1ps
module eth_udp_test #(
    parameter LOCAL_MAC = 48'hA0_B1_C2_D3_E1_E1,
    parameter LOCAL_IP  = 32'hC0_A8_01_0B,
    parameter LOCL_PORT = 16'h1F90,
    parameter DEST_IP   = 32'hC0_A8_01_69,
    parameter DEST_PORT = 16'h1F90
)(
    input               rgmii_clk,
    input               rstn,
    input               gmii_rx_dv,
    input      [7:0]    gmii_rxd,
    output reg          gmii_tx_en,
    output reg [7:0]    gmii_txd,
    input      [15:0]   snapshot_pixel_data,
    input      [11:0]   snapshot_pixel_count,
    input               snapshot_pixel_empty,
    input               snapshot_capture_error,
    input      [1:0]    snapshot_mode,
    output wire         snapshot_pixel_rd_en,
    output wire         network_ready,
    output              udp_rec_data_valid,
    output     [7:0]    udp_rec_rdata,
    output     [15:0]   udp_rec_data_length
);
    localparam H_PIXELS = 1280;
    localparam V_PIXELS = 720;
    localparam PIXELS_PER_PACKET = 512;
    localparam FULL_PACKET_COUNT = (H_PIXELS * V_PIXELS) / PIXELS_PER_PACKET;
    localparam PREVIEW_PACKET_COUNT = ((H_PIXELS / 2) * (V_PIXELS / 2)) / PIXELS_PER_PACKET;
    localparam HEADER_BYTES = 12;
    localparam IMAGE_BYTES = PIXELS_PER_PACKET * 2;
    localparam [15:0] APP_BYTES = HEADER_BYTES + IMAGE_BYTES;
    localparam ONE_SECOND = 32'd125_000_000;

    localparam S_ARP_REQ      = 4'd0;
    localparam S_ARP_SEND    = 4'd1;
    localparam S_ARP_WAIT    = 4'd2;
    localparam S_WAIT_PIXELS = 4'd3;
    localparam S_CHECK_ARP   = 4'd4;
    localparam S_GEN_REQ     = 4'd5;
    localparam S_WRITE_DATA  = 4'd6;
    localparam S_WAIT_TX     = 4'd7;
    localparam S_DRAIN       = 4'd8;
    localparam S_ERROR       = 4'd9;

    reg [3:0] state;
    reg [31:0] arp_timer;
    reg [15:0] packet_index;
    reg [15:0] frame_id;
    reg [10:0] byte_index;
    reg [7:0] app_data;
    reg app_data_valid;
    reg arp_req;
    reg [7:0] rx_data_d;
    reg rx_valid_d;
    wire tx_valid_tmp;
    wire [7:0] tx_data_tmp;
    wire udp_app_ready;
    wire udp_tx_req;
    wire arp_found;
    wire mac_not_exist;
    wire mac_send_end;
    wire [15:0] packet_count = (snapshot_mode == 2'd0) ? FULL_PACKET_COUNT : PREVIEW_PACKET_COUNT;

    assign udp_tx_req = (state == S_GEN_REQ);
    assign network_ready = !mac_not_exist;
    assign snapshot_pixel_rd_en = ((state == S_WRITE_DATA) &&
                                   (byte_index >= HEADER_BYTES) &&
                                   (byte_index < APP_BYTES) && byte_index[0]) ||
                                  ((state == S_ERROR) && !snapshot_pixel_empty);
    assign udp_rec_data_valid = 1'b0;
    assign udp_rec_rdata = 8'd0;
    assign udp_rec_data_length = 16'd0;

    always @(posedge rgmii_clk or negedge rstn) begin
        if (!rstn) begin
            state <= S_ARP_REQ;
            arp_timer <= 0;
            packet_index <= 0;
            frame_id <= 0;
            byte_index <= 0;
            app_data <= 0;
            app_data_valid <= 0;
            arp_req <= 0;
            rx_data_d <= 0;
            rx_valid_d <= 0;
            gmii_tx_en <= 0;
            gmii_txd <= 0;
        end else begin
            rx_data_d <= gmii_rxd;
            rx_valid_d <= gmii_rx_dv;
            gmii_tx_en <= tx_valid_tmp;
            gmii_txd <= tx_data_tmp;
            app_data_valid <= 1'b0;

            case (state)
                S_ARP_REQ: begin
                    arp_req <= 1'b1;
                    state <= S_ARP_SEND;
                end
                S_ARP_SEND: begin
                    arp_req <= 1'b0;
                    if (mac_send_end) begin
                        arp_timer <= 0;
                        state <= S_ARP_WAIT;
                    end
                end
                S_ARP_WAIT: begin
                    if (arp_found || !mac_not_exist) begin
                        state <= S_WAIT_PIXELS;
                        arp_timer <= 0;
                    end else if (arp_timer >= ONE_SECOND) begin
                        state <= S_ARP_REQ;
                        arp_timer <= 0;
                    end else begin
                        arp_timer <= arp_timer + 1'b1;
                    end
                end
                S_WAIT_PIXELS: begin
                    if (snapshot_capture_error) begin
                        state <= S_ERROR;
                    end else if (snapshot_pixel_count >= PIXELS_PER_PACKET) begin
                        state <= S_CHECK_ARP;
                    end
                end
                S_CHECK_ARP: begin
                    if (mac_not_exist)
                        state <= S_ARP_REQ;
                    else
                        state <= S_GEN_REQ;
                end
                S_GEN_REQ: begin
                    if (snapshot_capture_error) begin
                        state <= S_ERROR;
                    end else if (udp_app_ready) begin
                        byte_index <= 0;
                        state <= S_WRITE_DATA;
                    end
                end
                S_WRITE_DATA: begin
                    if (byte_index < APP_BYTES) begin
                        app_data <= packet_byte(byte_index, packet_index, frame_id,
                                                snapshot_pixel_data);
                        app_data_valid <= 1'b1;
                        byte_index <= byte_index + 1'b1;
                    end else begin
                        app_data_valid <= 1'b0;
                        state <= S_WAIT_TX;
                    end
                end
                S_WAIT_TX: begin
                    if (mac_send_end) begin
                        if (packet_index == packet_count - 1'b1) begin
                            packet_index <= 0;
                            frame_id <= frame_id + 1'b1;
                            state <= S_DRAIN;
                        end else begin
                            packet_index <= packet_index + 1'b1;
                            if (snapshot_capture_error)
                                state <= S_ERROR;
                            else if (snapshot_pixel_count >= PIXELS_PER_PACKET)
                                state <= S_GEN_REQ;
                            else
                                state <= S_WAIT_PIXELS;
                        end
                    end
                end
                S_DRAIN: begin
                    if (snapshot_pixel_empty)
                        state <= S_WAIT_PIXELS;
                end
                S_ERROR: begin
                    if (snapshot_pixel_empty) begin
                        packet_index <= 0;
                        state <= S_WAIT_PIXELS;
                    end
                end
                default: state <= S_ARP_REQ;
            endcase
        end
    end

    function [7:0] packet_byte;
        input [10:0] index;
        input [15:0] seq;
        input [15:0] frame;
        input [15:0] pixel;
        begin
            case (index)
                0: packet_byte = 8'h46; // F
                1: packet_byte = 8'h50; // P
                2: packet_byte = 8'h47; // G
                3: packet_byte = 8'h31; // 1
                4: packet_byte = 8'h01; // protocol version
                5: packet_byte = (snapshot_mode == 2'd0) ? 8'h01 :
                                 ((snapshot_mode == 2'd1) ? 8'h02 : 8'h03);
                6: packet_byte = seq[15:8];
                7: packet_byte = seq[7:0];
                8: packet_byte = packet_count[15:8];
                9: packet_byte = packet_count[7:0];
                10: packet_byte = frame[15:8];
                11: packet_byte = frame[7:0];
                default: begin
                    if ((index - HEADER_BYTES) & 1'b1)
                        packet_byte = pixel[7:0];
                    else
                        packet_byte = pixel[15:8];
                end
            endcase
        end
    endfunction

    udp_ip_mac_top #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP), .LOCL_PORT(LOCL_PORT),
        .DEST_IP(DEST_IP), .DEST_PORT(DEST_PORT)
    ) network_stack (
        .rgmii_clk(rgmii_clk), .rstn(rstn),
        .app_data_in_valid(app_data_valid), .app_data_in(app_data),
        .app_data_length(APP_BYTES), .app_data_request(udp_tx_req),
        .udp_send_ack(udp_app_ready),
        .arp_req(arp_req), .arp_found(arp_found),
        .mac_not_exist(mac_not_exist), .mac_send_end(mac_send_end),
        .udp_rec_rdata(), .udp_rec_data_length(), .udp_rec_data_valid(),
        .mac_data_valid(tx_valid_tmp), .mac_tx_data(tx_data_tmp),
        .rx_en(rx_valid_d), .mac_rx_datain(rx_data_d)
    );
endmodule
