`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// 
// Create Date: 2023-03-17  
// Design Name:  
// Module Name: 
// Project Name: 
// Target Devices: Pango
// Tool Versions: 
// Description: 
//      
// Dependencies: 
// 
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
//cmos1、cmos2二选一，作为视频源输入
//`define CMOS_1      //cmos1作为视频输入；
`define CMOS_2      //cmos2作为视频输入；

module hdmi_ddr_ov5640_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8,
    parameter ENABLE_AFFINE_FRAME_PIPELINE = 1'b1,
    parameter AFFINE_DIAGNOSTIC_OVERLAY = 1'b0,
    // Exact affine-only bounds use four rectangle corners; Brown mode keeps the dense prepass.
    parameter AFFINE_FRAME_CORNER_PREPASS = 1'b1,
    // Brown-map ROI prepass samples every other output coordinate and expands
    // the sampled source bounds by two pixels to cover interpolation between
    // samples. The actual output map still processes every pixel.
    parameter AFFINE_FRAME_BROWN_PREPASS_STEP = 2,
    parameter AFFINE_FRAME_PREPASS_BOUND_MARGIN = 2
)(
	input                                sys_clk              ,//27MHz board clock (FDC)
    input                                clk_p ,
    input                                clk_n ,
//OV5640
    output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
`ifndef CMOS_2
    input                                cmos1_vsync          ,//cmos1 vsync
    input                                cmos1_href           ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data           ,//cmos1 data
`endif
    output                               cmos1_reset          ,//cmos1 reset
    //coms2
    inout                                cmos2_scl            ,//cmos2 i2c 
    inout                                cmos2_sda            ,//cmos2 i2c 
    input                                cmos2_vsync          ,//cmos2 vsync
    input                                cmos2_href           ,//cmos2 hsync refrence,data valid
    input                                cmos2_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos2_data           ,//cmos2 data
    output                               cmos2_reset          ,//cmos2 reset
//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成
//HDMI_OUT
    output                               pix_clk                   ,//pixclk                           
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out,
    input                                key4_n,
    input                                uart_rxd,
    output                               uart_txd,
    input                                rgmii_rxc,
    input                                rgmii_rx_ctl,
    input       [3:0]                    rgmii_rxd,
    output                               rgmii_txc,
    output                               rgmii_tx_ctl,
    output      [3:0]                    rgmii_txd
);
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;//cmos i2c clock
    wire                        cmos_sda            ;//cmos i2c data
    wire                        cmos_vsync          ;//cmos vsync
    wire                        cmos_href           ;//cmos hsync refrence,data valid
    wire                        cmos_pclk           ;//cmos pxiel clock
    wire   [7:0]                cmos_data           ;//cmos data
    wire                        cmos_reset          ;//cmos reset
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_pclk_16bit    ;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  o_rgb565            ;
    wire init_done;
    wire signed [15:0] affine_a_q14_pix, affine_b_q14_pix;
    wire signed [15:0] affine_c_q14_pix, affine_d_q14_pix;
    wire [345:0] affine_config_bus_pix;
    wire [345:0] affine_config_bus_core;
    wire affine_config_applied_core;
    wire affine_camera_frame_start, affine_camera_frame_done, affine_display_vsync;
    wire affine_camera_capture_active, affine_camera_frame_drop;
    wire [1:0] affine_camera_write_bank, affine_process_read_bank, affine_process_write_bank;
    wire affine_output_frame_valid;
    wire [1:0] affine_latest_output_bank;
    wire affine_process_busy, affine_process_done, affine_process_error;
    wire affine_drop_pulse, affine_error_pulse;
    wire affine_rd_cmd_en, affine_rd_cmd_ready, affine_rd_data_valid, affine_rd_error;
    wire [27:0] affine_rd_cmd_addr;
    wire [31:0] affine_rd_cmd_len;
    wire [255:0] affine_rd_data;
    wire affine_wr_cmd_en, affine_wr_cmd_ready, affine_wr_cmd_done, affine_wr_data_req, affine_wr_error;
    wire [27:0] affine_wr_cmd_addr;
    wire [31:0] affine_wr_cmd_len;
    wire [255:0] affine_wr_data;
    wire                        pclk_in_test        ;    
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565   /*synthesis PAP_MARK_DEBUG="1"*/         ;
    wire                        src_vs_read;
    wire                        src_hs_read;
    wire                        src_de_read;
    wire                        disp_vs_ref;
    wire                        disp_hs_ref;
    wire                        disp_de_ref;
    wire [11:0]                 disp_x_act;
    wire [11:0]                 disp_y_act;
//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_awvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire                        axi_arvalid                ;/*synthesis PAP_MARK_DEBUG="1"*/
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
/////////////////////////////////////////////////////////////////////////////////////
//PLL

    pll u_pll (
        .clkin1   (  sys_clk    ),//50MHz
        .clkout0  (  pix_clk    ),//37.125M 720P30
        .clkout1  (  cfg_clk    ),//10MHz
        .clkout2  (  clk_25M    ),//25M
        .lock (  locked     )
    );

//配置7210
wire init_over_tx;
    ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ), //input       clk,
        .rst_n           (  rstn_out       ), //input       rstn,
        .init_over    (  init_over_tx   ), //output      init_over,                                
        .init_over_rx    (  init_over_rx   ), //output      init_over,
//        .iic_tx_scl      (  iic_tx_scl     ), //output      iic_scl,
//        .iic_tx_sda      (  iic_tx_sda     ), //inout       iic_sda
        .iic_scl         (  iic_tx_scl        ), //output      iic_scl,
        .iic_sda         (  iic_tx_sda        )  //inout       iic_sda
    );
//   assign    hdmi_int_led    =    init_over_tx; 
    
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    
    assign rstn_out = (rstn_1ms == 16'h2710);

//配置CMOS///////////////////////////////////////////////////////////////////////////////////
//OV5640 register configure enable    
    reg [1:0] camera_power_reset_sync = 2'b00;
    always @(posedge sys_clk or negedge locked) begin
        if (!locked) camera_power_reset_sync <= 2'b00;
        else camera_power_reset_sync <= {camera_power_reset_sync[0], 1'b1};
    end
    power_on_delay power_on_delay_inst (
        .clk_50M(sys_clk), .reset_n(camera_power_reset_sync[1]),
        .camera1_rstn(cmos1_reset), .camera2_rstn(cmos2_reset),
        .camera_pwnd(), .initial_en(initial_en)
    );
//CMOS1 Camera 
    reg_config	coms1_reg_config(
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos1_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos1_scl          ),//output
    	.i2c_sdat                (cmos1_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[0]  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );

//CMOS2 Camera 
    reg_config	coms2_reg_config(
    	.clk_25M                 (clk_25M            ),//input
    	.camera_rstn             (cmos2_reset        ),//input
    	.initial_en              (initial_en         ),//input		
    	.i2c_sclk                (cmos2_scl          ),//output
    	.i2c_sdat                (cmos2_sda          ),//inout
    	.reg_conf_done           (cmos_init_done[1]  ),//output config_finished
    	.reg_index               (                   ),//output reg [8:0]
    	.clock_20k               (                   ) //output reg
    );
//CMOS 8bit转16bit///////////////////////////////////////////////////////////////////////////////////
`ifdef CMOS_1
// CMOS1 converter is only needed when CMOS1 is the selected video source.
    always@(posedge cmos1_pclk)
        begin
            cmos1_d_d0        <= cmos1_data    ;
            cmos1_href_d0     <= cmos1_href    ;
            cmos1_vsync_d0    <= cmos1_vsync   ;
        end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),//input
    	.rst_n          (cmos_init_done[0]),//input
    	.pdata_i        (cmos1_d_d0       ),//input[7:0]
    	.de_i           (cmos1_href_d0    ),//input
    	.vs_i           (cmos1_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos1_pclk_16bit ),//output
    	.pdata_o        (cmos1_d_16bit    ),//output[15:0]
    	.de_o           (cmos1_href_16bit ) //output
    );
`elsif CMOS_2
// CMOS2 converter is only needed when CMOS2 is the selected video source.
    always@(posedge cmos2_pclk)
        begin
            cmos2_d_d0        <= cmos2_data    ;
            cmos2_href_d0     <= cmos2_href    ;
            cmos2_vsync_d0    <= cmos2_vsync   ;
        end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),//input
    	.rst_n          (cmos_init_done[1]),//input
    	.pdata_i        (cmos2_d_d0       ),//input[7:0]
    	.de_i           (cmos2_href_d0    ),//input
    	.vs_i           (cmos2_vsync_d0    ),//input
    	
    	.pixel_clk      (cmos2_pclk_16bit ),//output
    	.pdata_o        (cmos2_d_16bit    ),//output[15:0]
    	.de_o           (cmos2_href_16bit ) //output
    );
`endif
//输入视频源选择//////////////////////////////////////////////////////////////////////////////////////////
`ifdef CMOS_1
assign     pclk_in_test    =    cmos1_pclk_16bit    ;
assign     vs_in_test      =    cmos1_vsync_d0      ;
assign     de_in_test      =    cmos1_href_16bit    ;
assign     i_rgb565        =    {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};//{r,g,b}
`elsif CMOS_2
assign     pclk_in_test    =    cmos2_pclk_16bit    ;
assign     vs_in_test      =    cmos2_vsync_d0      ;
assign     de_in_test      =    cmos2_href_16bit    ;
assign     i_rgb565        =    {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};//{r,g,b}
`endif
 assign    hdmi_int_led    = |i_rgb565;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//修改ddr读写模块v1
    fram_buf #(.ENABLE_AFFINE_CLIENTS(ENABLE_AFFINE_FRAME_PIPELINE)) fram_buf(
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  ddr_init_done        ),//input                         ddr_rstn,
        //data_in                                  
        .vin_clk        (  pclk_in_test         ),//input                         vin_clk,
        .wr_fsync       (  vs_in_test           ),//input                         wr_fsync,
        .wr_en          (  de_in_test           ),//input                         wr_en,
        .wr_data        (  i_rgb565             ),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  pix_clk              ),//input                         vout_clk,
        .rd_fsync       (  src_vs_read          ),//input                         rd_fsync,
        .rd_en          (  src_de_read          ),//input                         rd_en,
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),
        .affine_mode ( ENABLE_AFFINE_FRAME_PIPELINE ),
        .camera_capture_allowed ( affine_camera_capture_active && !affine_camera_frame_drop ),
        .affine_camera_bank ( affine_camera_write_bank ),
        .processed_frame_valid ( affine_output_frame_valid ),
        .processed_frame_bank ( affine_latest_output_bank ),
        .camera_frame_start ( affine_camera_frame_start ),
        .camera_frame_done ( affine_camera_frame_done ),
        .display_vsync ( affine_display_vsync ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .affine_rd_cmd_en    (  affine_rd_cmd_en    ),
        .affine_rd_cmd_addr  (  affine_rd_cmd_addr  ),
        .affine_rd_cmd_len   (  affine_rd_cmd_len   ),
        .affine_rd_cmd_ready (  affine_rd_cmd_ready ),
        .affine_rd_cmd_done  (          ),
        .affine_rd_data_valid(  affine_rd_data_valid),
        .affine_rd_data      (  affine_rd_data      ),
        .affine_rd_error     (  affine_rd_error     ),
        .affine_wr_cmd_en    (  affine_wr_cmd_en    ),
        .affine_wr_cmd_addr  (  affine_wr_cmd_addr  ),
        .affine_wr_cmd_len   (  affine_wr_cmd_len   ),
        .affine_wr_data      (  affine_wr_data      ),
        .affine_wr_cmd_ready (  affine_wr_cmd_ready ),
        .affine_wr_cmd_done  (  affine_wr_cmd_done  ),
        .affine_wr_data_req  (  affine_wr_data_req  ),
        .affine_wr_error     (  affine_wr_error     ),
        .init_done      (  init_done            ),//output reg                    init_done,
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (                       ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ) // input[3:0]         
    );

     // Set ENABLE_AFFINE_FRAME_PIPELINE=1 to use the full-frame DDR warp.
     // The default remains the previously verified streaming Brown + shear path.
     generate
         if (ENABLE_AFFINE_FRAME_PIPELINE) begin : gen_affine_frame_pipeline
             affine_camera_frame_controller #(
                 .IMAGE_WIDTH(1280), .IMAGE_HEIGHT(720),
                 // Emit one output row at a time. The raster writer batches
                 // each 1280-pixel row into the same 80-beat DDR request used
                 // by the proven camera frame path; this avoids tiny tile
                 // write bursts on the shared DDR controller.
                 .OUTPUT_TILE_WIDTH(40), .OUTPUT_TILE_HEIGHT(1),
                 .CACHE_WIDTH(64), .CACHE_HEIGHT(64),
                 // fram_buf passes LINE_ADDR_WIDTH=22 into wr_buf/rd_buf, so
                 // the affine channels must use the same 2^22-unit bank stride.
                 .ROW_STRIDE_UNITS(640), .BANK_STRIDE_BITS(22),
                 .AFFINE_CORNER_PREPASS(AFFINE_FRAME_CORNER_PREPASS),
                 .BROWN_PREPASS_STEP(AFFINE_FRAME_BROWN_PREPASS_STEP),
                 .PREPASS_BOUND_MARGIN(AFFINE_FRAME_PREPASS_BOUND_MARGIN)
             ) frame_pipeline (
                 .clk(core_clk), .rst_n(ddr_init_done),
                 .camera_frame_start(affine_camera_frame_start),
                 .camera_frame_done(affine_camera_frame_done),
                 .display_vsync(affine_display_vsync),
                 .brown_bypass(!affine_config_bus_core[1]),
                 .center_x(affine_config_bus_core[345:334]),
                 .center_y(affine_config_bus_core[333:322]),
                 .a_q14(affine_config_bus_core[0] ? $signed(affine_config_bus_core[321:306]) : 16'sd16384),
                 .b_q14(affine_config_bus_core[0] ? $signed(affine_config_bus_core[305:290]) : 16'sd0),
                 .c_q14(affine_config_bus_core[0] ? $signed(affine_config_bus_core[289:274]) : 16'sd0),
                 .d_q14(affine_config_bus_core[0] ? $signed(affine_config_bus_core[273:258]) : 16'sd16384),
                 .fx_q16(affine_config_bus_core[257:226]),
                 .fy_q16(affine_config_bus_core[225:194]),
                 .inv_fx_q30(affine_config_bus_core[193:162]),
                 .inv_fy_q30(affine_config_bus_core[161:130]),
                 .k1_q24(affine_config_bus_core[129:98]),
                 .k2_q24(affine_config_bus_core[97:66]),
                 .p1_q24(affine_config_bus_core[65:34]),
                 .p2_q24(affine_config_bus_core[33:2]),
                 .ddr_rd_cmd_en(affine_rd_cmd_en),
                 .ddr_rd_cmd_addr(affine_rd_cmd_addr), .ddr_rd_cmd_len(affine_rd_cmd_len),
                 .ddr_rd_cmd_ready(affine_rd_cmd_ready),
                 .ddr_rd_data_valid(affine_rd_data_valid), .ddr_rd_data(affine_rd_data),
                 .ddr_wr_cmd_en(affine_wr_cmd_en),
                 .ddr_wr_cmd_addr(affine_wr_cmd_addr), .ddr_wr_cmd_len(affine_wr_cmd_len),
                 .ddr_wr_cmd_ready(affine_wr_cmd_ready), .ddr_wr_data_req(affine_wr_data_req),
                 .ddr_wr_data(affine_wr_data), .ddr_wr_cmd_done(affine_wr_cmd_done),
                 .camera_capture_active(affine_camera_capture_active),
                 .camera_frame_drop(affine_camera_frame_drop),
                 .camera_write_bank(affine_camera_write_bank),
                 .process_active(affine_process_busy), .process_frame_done(affine_process_done),
                 .process_frame_error(affine_process_error), .process_read_bank(affine_process_read_bank),
                 .process_write_bank(affine_process_write_bank), .display_valid(), .display_bank(),
                 .output_frame_valid(affine_output_frame_valid),
                 .latest_output_bank(affine_latest_output_bank),
                 .drop_pulse(affine_drop_pulse), .error_pulse(affine_error_pulse)
             );
         end else begin : gen_legacy_video_path
             assign affine_camera_capture_active = 1'b0;
             assign affine_camera_frame_drop = 1'b0;
             assign affine_camera_write_bank = 2'd0;
             assign affine_process_read_bank = 2'd0;
             assign affine_process_write_bank = 2'd2;
             assign affine_output_frame_valid = 1'b0;
             assign affine_latest_output_bank = 2'd2;
             assign affine_process_busy = 1'b0;
             assign affine_process_done = 1'b0;
             assign affine_process_error = 1'b0;
             assign affine_drop_pulse = 1'b0;
             assign affine_error_pulse = 1'b0;
             assign affine_rd_cmd_en = 1'b0;
             assign affine_rd_cmd_addr = 28'd0;
             assign affine_rd_cmd_len = 32'd0;
             assign affine_wr_cmd_en = 1'b0;
             assign affine_wr_cmd_addr = 28'd0;
             assign affine_wr_cmd_len = 32'd0;
             assign affine_wr_data = 256'd0;
         end
     endgenerate
     // Reset release and video sync alignment belong to the pixel clock domain.
     reg [1:0] pix_reset_sync = 0;
     wire pix_reset_n = pix_reset_sync[1];
     reg [15:0] display_start_count = 0;
     reg display_start_ready = 0;
     always @(posedge pix_clk or negedge pix_reset_n) begin
         if (!pix_reset_n) begin
             display_start_count <= 0;
             display_start_ready <= 0;
         end else if (!display_start_ready) begin
             // Start HDMI output 20 raster lines after the source-read path.
             // The current calibration can require nearly 29 retained source
             // rows near the upper-left edge, so Brown uses a 30-row cache.
             if (display_start_count == 16'd33000)
                 display_start_ready <= 1'b1;
             else
                 display_start_count <= display_start_count + 1'b1;
         end
     end
     always @(posedge pix_clk or negedge ddr_init_done) begin
         if(!ddr_init_done) pix_reset_sync <= 0;
         else pix_reset_sync <= {pix_reset_sync[0], 1'b1};
     end
     // DDR/core-domain frame events cross as toggles. Periods and processing
     // duration are measured against the HDMI pixel clock, then returned over UART.
     reg camera_start_toggle_core = 1'b0;
     reg process_done_toggle_core = 1'b0;
     reg frame_drop_toggle_core = 1'b0;
     always @(posedge core_clk or negedge ddr_init_done) begin
         if (!ddr_init_done) begin
             camera_start_toggle_core <= 1'b0;
             process_done_toggle_core <= 1'b0;
             frame_drop_toggle_core <= 1'b0;
         end else begin
             if (affine_camera_frame_start) camera_start_toggle_core <= ~camera_start_toggle_core;
             if (affine_process_done) process_done_toggle_core <= ~process_done_toggle_core;
             if (affine_drop_pulse) frame_drop_toggle_core <= ~frame_drop_toggle_core;
         end
     end
     reg [1:0] camera_start_sync = 0, process_done_sync = 0, frame_drop_sync = 0;
     reg camera_start_seen = 0, process_done_seen = 0, frame_drop_seen = 0;
     reg [1:0] process_busy_sync = 0;
     reg process_busy_d = 0;
     reg [31:0] pix_tick_counter = 0;
     reg [31:0] last_input_tick = 0, last_output_tick = 0, process_start_tick = 0;
     reg have_input_tick = 0, have_output_tick = 0;
     reg [31:0] input_period_ticks = 0, output_period_ticks = 0, processing_ticks = 0;
     reg [15:0] dropped_frames = 0;
     always @(posedge pix_clk or negedge pix_reset_n) begin
         if (!pix_reset_n) begin
             camera_start_sync <= 0; process_done_sync <= 0; frame_drop_sync <= 0;
             camera_start_seen <= 0; process_done_seen <= 0; frame_drop_seen <= 0;
             process_busy_sync <= 0; process_busy_d <= 0;
             pix_tick_counter <= 0;
             last_input_tick <= 0; last_output_tick <= 0; process_start_tick <= 0;
             have_input_tick <= 0; have_output_tick <= 0;
             input_period_ticks <= 0; output_period_ticks <= 0; processing_ticks <= 0;
             dropped_frames <= 0;
         end else begin
             camera_start_sync <= {camera_start_sync[0], camera_start_toggle_core};
             process_done_sync <= {process_done_sync[0], process_done_toggle_core};
             frame_drop_sync <= {frame_drop_sync[0], frame_drop_toggle_core};
             process_busy_sync <= {process_busy_sync[0], affine_process_busy};
             process_busy_d <= process_busy_sync[1];
             pix_tick_counter <= pix_tick_counter + 1'b1;
             if (camera_start_sync[1] != camera_start_seen) begin
                 camera_start_seen <= camera_start_sync[1];
                 if (have_input_tick) input_period_ticks <= pix_tick_counter - last_input_tick;
                 last_input_tick <= pix_tick_counter;
                 have_input_tick <= 1'b1;
             end
             if (process_done_sync[1] != process_done_seen) begin
                 process_done_seen <= process_done_sync[1];
                 if (have_output_tick) output_period_ticks <= pix_tick_counter - last_output_tick;
                 last_output_tick <= pix_tick_counter;
                 have_output_tick <= 1'b1;
             end
             if (frame_drop_sync[1] != frame_drop_seen) begin
                 frame_drop_seen <= frame_drop_sync[1];
                 if (dropped_frames != 16'hffff) dropped_frames <= dropped_frames + 1'b1;
             end
             if (process_busy_sync[1] && !process_busy_d)
                 process_start_tick <= pix_tick_counter;
             else if (!process_busy_sync[1] && process_busy_d)
                 processing_ticks <= pix_tick_counter - process_start_tick;
         end
     end
     wire [11:0] calib_cx_pix;
     wire [11:0] calib_cy_pix;
     wire signed [15:0] imu_slope_q14_pix;
     wire signed [31:0] calib_fx_q16_pix, calib_fy_q16_pix;
     wire signed [31:0] calib_inv_fx_q30_pix, calib_inv_fy_q30_pix;
     wire signed [31:0] calib_k1_q24_pix, calib_k2_q24_pix;
     wire signed [31:0] calib_p1_q24_pix, calib_p2_q24_pix;
     wire                 warp_enable_pix;
     wire                 brown_enable_pix;
     wire                 gamma_enable_pix;
     wire                 grayscale_enable_pix;
     wire                 config_applied_pix;
     wire                 warp_hs;
     wire                 warp_vs;
     wire                 warp_de;
     wire [15:0]          warp_rgb565;
     wire                 gamma_hs;
     wire                 gamma_vs;
     wire                 gamma_de;
     wire [15:0]          gamma_rgb565;
     wire [1:0]           gamma_mode;
     wire                 brown_hs;
     wire                 brown_vs;
     wire                 brown_de;
     wire [15:0]          brown_rgb565;
     wire                 capture_request_pix;
     wire [1:0]           capture_mode_pix;
     wire                 telemetry_request_pix;

     // Parse UART commands in the same pixel-clock domain as the image warp.
     // This avoids a multi-bit configuration CDC from sys_clk into pix_clk.
     camera_uart_config #(.APPLY_AT_FRAME_START(1),
                          .FIRMWARE_MODE(ENABLE_AFFINE_FRAME_PIPELINE)) camera_uart_config (
         .clk(pix_clk), .rst_n(pix_reset_n), .uart_rxd(uart_rxd), .pix_clk(pix_clk),
         .frame_vs_pix(disp_vs_ref), .uart_txd(uart_txd),
         .cx_pix(calib_cx_pix), .cy_pix(calib_cy_pix),
         .slope_q14_pix(imu_slope_q14_pix),
         .affine_a_q14_pix(affine_a_q14_pix), .affine_b_q14_pix(affine_b_q14_pix),
         .affine_c_q14_pix(affine_c_q14_pix), .affine_d_q14_pix(affine_d_q14_pix),
         .warp_enable_pix(warp_enable_pix),
         .brown_enable_pix(brown_enable_pix),
         .gamma_enable_pix(gamma_enable_pix),
         .grayscale_enable_pix(grayscale_enable_pix),
         .fx_q16_pix(calib_fx_q16_pix), .fy_q16_pix(calib_fy_q16_pix),
         .inv_fx_q30_pix(calib_inv_fx_q30_pix), .inv_fy_q30_pix(calib_inv_fy_q30_pix),
         .k1_q24_pix(calib_k1_q24_pix), .k2_q24_pix(calib_k2_q24_pix),
         .p1_q24_pix(calib_p1_q24_pix), .p2_q24_pix(calib_p2_q24_pix),
         .config_applied_pix(config_applied_pix),
         .capture_request_pix(capture_request_pix),
         .capture_mode_pix(capture_mode_pix),
         .telemetry_request_pix(telemetry_request_pix),
         .input_period_ticks(input_period_ticks), .output_period_ticks(output_period_ticks),
         .processing_ticks(processing_ticks), .dropped_frames(dropped_frames)
     );

     assign affine_config_bus_pix = {
         calib_cx_pix, calib_cy_pix,
         affine_a_q14_pix, affine_b_q14_pix, affine_c_q14_pix, affine_d_q14_pix,
         calib_fx_q16_pix, calib_fy_q16_pix, calib_inv_fx_q30_pix, calib_inv_fy_q30_pix,
         calib_k1_q24_pix, calib_k2_q24_pix, calib_p1_q24_pix, calib_p2_q24_pix,
         brown_enable_pix, warp_enable_pix
     };
     generate
         if (ENABLE_AFFINE_FRAME_PIPELINE) begin : gen_affine_config_cdc
             camera_config_cdc #(.WIDTH(346)) config_to_ddr_clock (
                 .src_clk(pix_clk), .src_rst_n(pix_reset_n),
                 .config_valid_src(config_applied_pix), .config_bus_src(affine_config_bus_pix),
                 .dst_clk(core_clk), .dst_rst_n(ddr_init_done),
                 .config_bus_dst(affine_config_bus_core),
                 .config_applied_dst(affine_config_applied_core)
             );
         end else begin : gen_no_affine_config_cdc
             assign affine_config_bus_core = 346'd0;
             assign affine_config_applied_core = 1'b0;
         end
     endgenerate

     // Brown remapping reads from a 30-row circular cache. HDMI starts 20
     // lines behind the source raster; the current map can request source rows
     // nearly 9 lines behind the output coordinate at the upper-left edge.
     brown_conrady_video_warp #(
         .IMAGE_WIDTH(1280), .IMAGE_HEIGHT(720), .LINE_BANKS(30)
     ) brown_conrady_video_warp (
         .clk(pix_clk), .rst_n(pix_reset_n),
         .src_hs(src_hs_read), .src_vs(src_vs_read), .src_de(de_o), .src_rgb565(o_rgb565),
         .out_hs_in(disp_hs_ref), .out_vs_in(disp_vs_ref), .out_de_in(disp_de_ref),
         .out_x(disp_x_act), .out_y(disp_y_act), .brown_enable(brown_enable_pix && !ENABLE_AFFINE_FRAME_PIPELINE),
         .cx(calib_cx_pix), .cy(calib_cy_pix),
         .fx_q16(calib_fx_q16_pix), .fy_q16(calib_fy_q16_pix),
         .inv_fx_q30(calib_inv_fx_q30_pix), .inv_fy_q30(calib_inv_fy_q30_pix),
         .k1_q24(calib_k1_q24_pix), .k2_q24(calib_k2_q24_pix),
         .p1_q24(calib_p1_q24_pix), .p2_q24(calib_p2_q24_pix),
         .warp_hs(brown_hs), .warp_vs(brown_vs), .warp_de(brown_de), .warp_rgb565(brown_rgb565)
     );

     // When Brown correction is disabled, do not route the pose-only mode
     // through the identity remap's circular line cache. Use the verified
     // DDR raster directly so a cache/tag miss cannot turn active rows black.
     wire shear_hs_in = brown_enable_pix ? brown_hs : src_hs_read;
     wire shear_vs_in = brown_enable_pix ? brown_vs : src_vs_read;
     wire shear_de_in = brown_enable_pix ? brown_de : de_o;
     wire [15:0] shear_rgb_in = brown_enable_pix ? brown_rgb565 : o_rgb565;

     imu_row_shear #(.H_ACTIVE(1280)) imu_row_shear (
         .clk(pix_clk), .rst_n(pix_reset_n), .enable(warp_enable_pix && !ENABLE_AFFINE_FRAME_PIPELINE),
         .cx(calib_cx_pix), .cy(calib_cy_pix), .slope_q14(imu_slope_q14_pix),
         .hs_in(shear_hs_in), .vs_in(shear_vs_in), .de_in(shear_de_in), .rgb_in(shear_rgb_in),
         .hs_out(warp_hs), .vs_out(warp_vs), .de_out(warp_de), .rgb_out(warp_rgb565)
     );
     // Keep the uncorrected camera preview on the verified legacy DDR-to-HDMI
     // path. The line-cache remapper is selected only after correction is
     // explicitly enabled over UART (or when the frame-affine build is used).
     wire corrected_display_requested = ENABLE_AFFINE_FRAME_PIPELINE ||
                                        brown_enable_pix || warp_enable_pix ||
                                        gamma_enable_pix || grayscale_enable_pix;
     reg corrected_display_active = 1'b0;
     reg source_vs_d = 1'b0;
     reg [1:0] vs_pipe = 0, hs_pipe = 0;
     always @(posedge pix_clk or negedge pix_reset_n) begin
         if (!pix_reset_n) begin
             source_vs_d <= 1'b0;
             corrected_display_active <= ENABLE_AFFINE_FRAME_PIPELINE;
         end else begin
             source_vs_d <= src_vs_read;
             // Latch mode only at a frame boundary so the display does not
             // switch RGB/sync paths halfway through an active frame.
             if (src_vs_read && !source_vs_d)
                 corrected_display_active <= corrected_display_requested;
         end
     end
     always @(posedge pix_clk) begin
         if (!pix_reset_n) begin
             vs_pipe <= 0;
             hs_pipe <= 0;
         end else begin
             vs_pipe <= {vs_pipe[0],src_vs_read};
             hs_pipe <= {hs_pipe[0],src_hs_read};
         end
     end
     // Optional HDMI status strip used only by the diagnostic build. The six
     // blocks report: DDR frame ready, camera capture active, frame drop,
     // processed frame valid, processor busy, and processor error.
     wire [5:0] affine_diag_status_core = {
         affine_process_error, affine_process_busy, affine_output_frame_valid,
         affine_camera_frame_drop, affine_camera_capture_active, init_done
     };
     reg [5:0] affine_diag_status_meta = 0;
     reg [5:0] affine_diag_status_pix = 0;
     always @(posedge pix_clk or negedge pix_reset_n) begin
         if (!pix_reset_n) begin
             affine_diag_status_meta <= 0;
             affine_diag_status_pix <= 0;
         end else begin
             affine_diag_status_meta <= affine_diag_status_core;
             affine_diag_status_pix <= affine_diag_status_meta;
         end
     end
     reg diag_de_prev = 1'b0;
     reg [11:0] diag_x_count = 0;
     reg [10:0] diag_y_count = 0;
     always @(posedge pix_clk or negedge pix_reset_n) begin
         if (!pix_reset_n) begin
             diag_de_prev <= 1'b0;
             diag_x_count <= 0;
             diag_y_count <= 0;
         end else begin
             diag_de_prev <= de_o;
             if (src_vs_read && !source_vs_d) begin
                 diag_x_count <= 0;
                 diag_y_count <= 0;
             end else if (de_o) begin
                 if (!diag_de_prev)
                     diag_x_count <= 0;
                 else if (diag_x_count < 12'd1279)
                     diag_x_count <= diag_x_count + 1'b1;
             end else if (diag_de_prev) begin
                 diag_x_count <= 0;
                 if (diag_y_count < 11'd719)
                     diag_y_count <= diag_y_count + 1'b1;
                 else
                     diag_y_count <= 0;
             end
         end
     end
     reg [15:0] affine_diag_rgb565;
     always @* begin
         affine_diag_rgb565 = o_rgb565;
         if (AFFINE_DIAGNOSTIC_OVERLAY && ENABLE_AFFINE_FRAME_PIPELINE &&
             de_o && diag_y_count < 11'd56) begin
             if (diag_x_count < 11'd800) begin
                 if      (diag_x_count >= 12'd32  && diag_x_count < 12'd144) affine_diag_rgb565 = affine_diag_status_pix[0] ? 16'h07E0 : 16'hF800;
                 else if (diag_x_count >= 12'd160 && diag_x_count < 12'd272) affine_diag_rgb565 = affine_diag_status_pix[1] ? 16'h07FF : 16'h4208;
                 else if (diag_x_count >= 12'd288 && diag_x_count < 12'd400) affine_diag_rgb565 = affine_diag_status_pix[2] ? 16'hF800 : 16'h07E0;
                 else if (diag_x_count >= 12'd416 && diag_x_count < 12'd528) affine_diag_rgb565 = affine_diag_status_pix[3] ? 16'h07E0 : 16'hF800;
                 else if (diag_x_count >= 12'd544 && diag_x_count < 12'd656) affine_diag_rgb565 = affine_diag_status_pix[4] ? 16'hFFE0 : 16'h4208;
                 else if (diag_x_count >= 12'd672 && diag_x_count < 12'd784) affine_diag_rgb565 = affine_diag_status_pix[5] ? 16'hF800 : 16'h07E0;
                 else affine_diag_rgb565 = 16'h0000;
             end else begin
                 affine_diag_rgb565 = 16'h0000;
             end
         end
     end

     // Apply tone and grayscale after the selected geometry path. Parameter-1
     // firmware reads the completed affine frame from DDR, so it must feed
     // this stage directly instead of bypassing it as the old output mux did.
     wire tone_hs_in = ENABLE_AFFINE_FRAME_PIPELINE ? hs_pipe[1] : warp_hs;
     wire tone_vs_in = ENABLE_AFFINE_FRAME_PIPELINE ? vs_pipe[1] : warp_vs;
     wire tone_de_in = ENABLE_AFFINE_FRAME_PIPELINE ? de_o : warp_de;
     wire [15:0] tone_rgb_in = ENABLE_AFFINE_FRAME_PIPELINE ? affine_diag_rgb565 : warp_rgb565;
     adaptive_gamma_rgb565 adaptive_gamma_rgb565 (
         .clk(pix_clk), .rst_n(pix_reset_n), .enable(gamma_enable_pix),
         .grayscale_enable(grayscale_enable_pix),
         .hs_in(tone_hs_in), .vs_in(tone_vs_in),
         .de_in(tone_de_in), .rgb_in(tone_rgb_in),
         .hs_out(gamma_hs), .vs_out(gamma_vs), .de_out(gamma_de),
         .rgb_out(gamma_rgb565), .gamma_mode(gamma_mode)
     );

     always @(posedge pix_clk) begin
         if (ENABLE_AFFINE_FRAME_PIPELINE) begin
             // The affine path has already warped the complete frame in DDR.
             // Apply optional pixel processing to that completed frame; the
             // legacy streaming line-cache warp remains out of this path.
             r_out <= {gamma_rgb565[15:11],3'b0};
             g_out <= {gamma_rgb565[10:5],2'b0};
             b_out <= {gamma_rgb565[4:0],3'b0};
             vs_out <= gamma_vs;
             hs_out <= gamma_hs;
             de_out <= gamma_de;
         end else if (corrected_display_active) begin
            r_out <= {gamma_rgb565[15:11],3'b0};
            g_out <= {gamma_rgb565[10:5],2'b0};
            b_out <= {gamma_rgb565[4:0],3'b0};
            vs_out <= gamma_vs;
            hs_out <= gamma_hs;
            de_out <= gamma_de;
        end else begin
            // Match the known-good project: raw RGB565/DE with delayed source sync.
            r_out <= {o_rgb565[15:11],3'b0};
            g_out <= {o_rgb565[10:5],2'b0};
            b_out <= {o_rgb565[4:0],3'b0};
            vs_out <= vs_pipe[1];
            hs_out <= hs_pipe[1];
            de_out <= de_o;
        end
     end
/////////////////////////////////////////////////////////////////////////////////////
//产生visa时序 
     sync_vg source_read_timing (
        .clk            (  pix_clk              ),//input                   clk,                                 
        .rstn           (  pix_reset_n              ),//input                   rstn,                            
        .vs_out         (  src_vs_read          ),
        .hs_out         (  src_hs_read          ),
        .de_out         (                       ),
        .de_re          (  src_de_read          )
    );
     sync_vg display_timing (
        .clk            (  pix_clk              ),
        .rstn           (  pix_reset_n && display_start_ready ),
        .vs_out         (  disp_vs_ref          ),
        .hs_out         (  disp_hs_ref          ),
        .de_out         (  disp_de_ref          ),
        .de_re          (                       ),
        .x_act          (  disp_x_act           ),
        .y_act          (  disp_y_act           )
    );
////////////////////////////////////////////////////////////////////////////////////////////
//ddr    
/*        ddr3_100h ddr3_100h (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output
             .ddrphy_clkin              (core_clk           ),// output
             .pll_lock                  (pll_lock           ),// output

             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug
             .debug_data                (                   ),// output [135:0]
             .debug_slice_state         (                   ),// output [51:0]
             .debug_calib_ctrl          (                   ),// output [21:0]
             .ck_dly_set_bin            (                   ),// output [7:0]
             .force_ck_dly_en           (1'b0               ),// input
             .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
             .dll_step                  (                   ),// output [7:0]
             .dll_lock                  (                   ),// output
             .init_read_clk_ctrl        (2'b0               ),// input [1:0]
             .init_slip_step            (4'b0               ),// input [3:0]
             .force_read_clk_ctrl       (1'b0               ),// input
             .ddrphy_gate_update_en     (1'b0               ),// input
             .update_com_val_err_flag   (                   ),// output [3:0]
             .rd_fake_stop              (1'b0               ) // input
       );*/
wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);
 ddr3_test u_ddr3_test_h(
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),          // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
  .debug_cpd_state()                     // output [3:0]
       );
//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
// PHY reset is shared with the existing board reset output rstn_out (G25).
    wire unused_eth_led;
    wire unused_eth_phy_rstn;
    ethernet_test #(.LOCAL_IP(32'hC0_A8_01_0B), .DEST_IP(32'hC0_A8_01_69),
                    .LOCL_PORT(16'h1F90), .DEST_PORT(16'h1F90)) eth_snapshot (
        .led(unused_eth_led), .phy_rstn(unused_eth_phy_rstn),
        .rgmii_rxc(rgmii_rxc), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxd(rgmii_rxd),
        .rgmii_txc(rgmii_txc), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txd(rgmii_txd),
        .pix_clk(pix_clk), .pix_rst_n(pix_reset_n), .key4_n(key4_n),
        .capture_request(capture_request_pix),
        .capture_request_mode(capture_mode_pix),
        .raw_vsync(src_vs_read), .raw_de(de_o), .raw_rgb565(o_rgb565),
        .hdmi_vsync(vs_out), .hdmi_de(de_out), .hdmi_r(r_out), .hdmi_g(g_out), .hdmi_b(b_out)
    );
endmodule
