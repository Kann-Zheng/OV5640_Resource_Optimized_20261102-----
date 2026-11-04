`timescale 1ns/1ps
module ethernet_test #(
    parameter LOCAL_MAC = 48'hA0_B1_C2_D3_E1_E1,
    parameter LOCAL_IP  = 32'hC0_A8_01_0B,
    parameter LOCL_PORT = 16'h1F90,
    parameter DEST_IP   = 32'hC0_A8_01_69,
    parameter DEST_PORT = 16'h1F90
)(
    output       led,
    output       phy_rstn,
    input        rgmii_rxc,
    input        rgmii_rx_ctl,
    input  [3:0] rgmii_rxd,
    output       rgmii_txc,
    output       rgmii_tx_ctl,
    output [3:0] rgmii_txd,
    input        pix_clk,
    input        pix_rst_n,
    input        key4_n,
    input        capture_request,
    input  [1:0]  capture_request_mode,
    input        hdmi_vsync,
    input        hdmi_de,
    input  [7:0] hdmi_r,
    input  [7:0] hdmi_g,
    input  [7:0] hdmi_b,
    input        raw_vsync,
    input        raw_de,
    input  [15:0] raw_rgb565
);
    wire rstn;
    wire rgmii_clk;
    wire rgmii_clk_90p;
    wire mac_rx_data_valid;
    wire [7:0] mac_rx_data;
    wire mac_tx_data_valid;
    wire [7:0] mac_tx_data;
    wire [15:0] snapshot_pixel_data;
    wire [11:0] snapshot_pixel_count;
    wire snapshot_pixel_empty;
    wire snapshot_pixel_rd_en;
    wire network_ready;
    wire snapshot_capture_error;
    wire capture_busy;
    wire frame_done_unused;
    wire [31:0] frame_id_unused;
    wire [1:0] snapshot_mode;
    reg [23:0] led_divider;
    reg led_state;

    assign led = led_state;
    assign phy_rstn = rstn;

    ref_clock ref_clock_inst (
        .clkout0(rgmii_clk_90p),
        .lock(rstn),
        .clkin1(rgmii_rxc)
    );

    hdmi_snapshot_fifo #(
        .H_PIXELS(1280), .V_PIXELS(720), .FIFO_ADDR_BITS(11),
        .KEY_DEBOUNCE_CYCLES(750000)
    ) snapshot_fifo_inst (
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .key4_n(key4_n),
        .capture_request(capture_request),
        .capture_request_mode(capture_request_mode),
        .network_ready(network_ready),
        .raw_vsync(raw_vsync), .raw_de(raw_de), .raw_rgb565(raw_rgb565),
        .hdmi_vsync(hdmi_vsync), .hdmi_de(hdmi_de),
        .hdmi_rgb565({hdmi_r[7:3], hdmi_g[7:2], hdmi_b[7:3]}),
        .frame_done(frame_done_unused), .capture_error(), .capture_busy(capture_busy),
        .frame_id(frame_id_unused), .capture_mode(snapshot_mode),
        .net_clk(rgmii_clk), .net_rst_n(rstn),
        .pixel_rd_en(snapshot_pixel_rd_en), .pixel_rd_data(snapshot_pixel_data),
        .pixel_empty(snapshot_pixel_empty), .pixel_count(snapshot_pixel_count),
        .capture_error_net(snapshot_capture_error)
    );

    eth_udp_test #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP), .LOCL_PORT(LOCL_PORT),
        .DEST_IP(DEST_IP), .DEST_PORT(DEST_PORT)
    ) image_udp_app (
        .rgmii_clk(rgmii_clk), .rstn(rstn),
        .gmii_rx_dv(mac_rx_data_valid), .gmii_rxd(mac_rx_data),
        .gmii_tx_en(mac_tx_data_valid), .gmii_txd(mac_tx_data),
        .snapshot_pixel_data(snapshot_pixel_data),
        .snapshot_pixel_count(snapshot_pixel_count),
        .snapshot_pixel_empty(snapshot_pixel_empty),
        .snapshot_capture_error(snapshot_capture_error),
        .snapshot_mode(snapshot_mode),
        .snapshot_pixel_rd_en(snapshot_pixel_rd_en),
        .network_ready(network_ready),
        .udp_rec_data_valid(), .udp_rec_rdata(), .udp_rec_data_length()
    );

    rgmii_interface rgmii_interface_inst (
        .rst(~rstn), .rgmii_clk(rgmii_clk), .rgmii_clk_90p(),
        .mac_tx_data_valid(mac_tx_data_valid), .mac_tx_data(mac_tx_data),
        .mac_rx_error(), .mac_rx_data_valid(mac_rx_data_valid),
        .mac_rx_data(mac_rx_data), .rgmii_rxc(rgmii_clk_90p),
        .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl),
        .rgmii_txd(rgmii_txd)
    );

    always @(posedge rgmii_clk or negedge rstn) begin
        if (!rstn) begin
            led_divider <= 0;
            led_state <= 1'b0;
        end else if (led_divider == 24'd6_249_999) begin
            led_divider <= 0;
            led_state <= ~led_state;
        end else begin
            led_divider <= led_divider + 1'b1;
        end
    end
endmodule
