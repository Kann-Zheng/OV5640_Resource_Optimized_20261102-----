`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Meyesemi
// Engineer: Nill
// 
// Create Date: 15/03/23 14:17:29
// Design Name: 
// Module Name: fram_buf
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
module fram_buf #(
    parameter                     ENABLE_AFFINE_CLIENTS = 1'b0,
    parameter                     MEM_ROW_WIDTH        = 15    ,
    parameter                     MEM_COLUMN_WIDTH     = 10    ,
    parameter                     MEM_BANK_WIDTH       = 3     ,
    parameter                     CTRL_ADDR_WIDTH = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter                     MEM_DQ_WIDTH         = 32    ,
    parameter                     H_NUM                = 12'd1280,//12'd1920,
    parameter                     V_NUM                = 12'd720,//12'd1080,//12'd106,//
    parameter                     PIX_WIDTH            = 16//24 
)(
    input                         vin_clk,
    input                         wr_fsync,
    input                         wr_en,
    input  [PIX_WIDTH- 1'b1 : 0]  wr_data,
    output reg                    init_done=0,
    
    input                         ddr_clk,
    input                         ddr_rstn,
    
    input                         vout_clk,
    input                         rd_fsync,
    input                         rd_en,
    output                        vout_de,
    output [PIX_WIDTH- 1'b1 : 0]  vout_data,

    // Optional full-frame affine ownership/display controls.
    input                         affine_mode,
    input                         camera_capture_allowed,
    input  [1:0]                  affine_camera_bank,
    input                         processed_frame_valid,
    input  [1:0]                  processed_frame_bank,
    output                        camera_frame_start,
    output                        camera_frame_done,
    output                        display_vsync,

    // Optional affine tile read client shares the DDR read path with HDMI.
    input                         affine_rd_cmd_en,
    input  [CTRL_ADDR_WIDTH-1:0]  affine_rd_cmd_addr,
    input  [31:0]                 affine_rd_cmd_len,
    output                        affine_rd_cmd_ready,
    output                        affine_rd_cmd_done,
    output                        affine_rd_data_valid,
    output [MEM_DQ_WIDTH*8-1:0]   affine_rd_data,
    output                        affine_rd_error,
    input                         affine_wr_cmd_en,
    input  [CTRL_ADDR_WIDTH-1:0]  affine_wr_cmd_addr,
    input  [31:0]                 affine_wr_cmd_len,
    input  [MEM_DQ_WIDTH*8-1:0]  affine_wr_data,
    output                        affine_wr_cmd_ready,
    output                        affine_wr_cmd_done,
    output                        affine_wr_data_req,
    output                        affine_wr_error,
    
    output [CTRL_ADDR_WIDTH-1:0]  axi_awaddr     ,
    output [3:0]                  axi_awid       ,
    output [3:0]                  axi_awlen      ,
    output [2:0]                  axi_awsize     ,
    output [1:0]                  axi_awburst    ,
    input                         axi_awready    ,
    output                        axi_awvalid    ,
                                                  
    output [MEM_DQ_WIDTH*8-1:0]   axi_wdata      ,
    output [MEM_DQ_WIDTH -1 :0]   axi_wstrb      ,
    input                         axi_wlast      ,
    output                        axi_wvalid     ,
    input                         axi_wready     ,
    input  [3 : 0]                axi_bid        ,                                      
                                                  
    output [CTRL_ADDR_WIDTH-1:0]  axi_araddr     ,
    output [3:0]                  axi_arid       ,
    output [3:0]                  axi_arlen      ,
    output [2:0]                  axi_arsize     ,
    output [1:0]                  axi_arburst    ,
    output                        axi_arvalid    ,
    input                         axi_arready    ,
                                                  
    output                        axi_rready     ,
    input  [MEM_DQ_WIDTH*8-1:0]   axi_rdata      ,
    input                         axi_rvalid     ,
    input                         axi_rlast      ,
    input  [3:0]                  axi_rid            
);
    parameter LEN_WIDTH       = 32;
    parameter LINE_ADDR_WIDTH = 22;//19;//1440 * 1080 = 1 555 200 = 21'h17BB00
    parameter FRAME_CNT_WIDTH = CTRL_ADDR_WIDTH - LINE_ADDR_WIDTH;
    
    wire                        ddr_wreq;     
    wire [CTRL_ADDR_WIDTH- 1'b1 : 0] ddr_waddr;    
    wire [LEN_WIDTH- 1'b1 : 0]  ddr_wr_len;   
    wire                        ddr_wrdy;     
    wire                        ddr_wdone;    
    wire [8*MEM_DQ_WIDTH-1 : 0] ddr_wdata;    
    wire                        ddr_wdata_req;
    wire                        camera_wreq;
    wire [CTRL_ADDR_WIDTH-1:0]  camera_waddr;
    wire [LEN_WIDTH-1:0]        camera_wlen;
    wire                        camera_wrdy;
    wire                        camera_wdone;
    wire [MEM_DQ_WIDTH*8-1:0]   camera_wdata;
    wire                        camera_wdata_req;
    
    wire                        display_rd_cmd_en;
    wire [CTRL_ADDR_WIDTH-1:0]  display_rd_cmd_addr;
    wire [LEN_WIDTH-1:0]        display_rd_cmd_len;
    wire                        display_rd_cmd_ready;
    wire                        display_rd_cmd_done;
                                
    wire                        display_rd_data_valid;
    wire [MEM_DQ_WIDTH*8-1:0]   display_rd_data;
    wire                        ctrl_rd_cmd_en;
    wire [CTRL_ADDR_WIDTH-1:0]  ctrl_rd_cmd_addr;
    wire [31:0]                 ctrl_rd_cmd_len;
    wire                        ctrl_rd_cmd_ready;
    wire                        ctrl_rd_cmd_done;
    wire                        read_ready  = 1'b1;
    wire [MEM_DQ_WIDTH*8-1:0]   read_rdata  ;
    wire                        read_en     ;
    wire                        ddr_wr_bac;
    wire [FRAME_CNT_WIDTH-1:0] completed_bank;
    wire [1:0] display_bank;
    wire frame_wirq;
    wire frame_start_pulse;
    wire camera_wr_en;
    frame_capture_gate_cdc camera_capture_gate (
        .camera_clk(vin_clk), .camera_rst_n(ddr_rstn),
        .capture_allowed(camera_capture_allowed), .affine_mode(affine_mode),
        .pixel_enable_in(wr_en), .pixel_enable_out(camera_wr_en)
    );
    assign camera_frame_start = frame_start_pulse;
    assign camera_frame_done = frame_wirq;

    wr_buf #(
        .ADDR_WIDTH       (  CTRL_ADDR_WIDTH  ),//parameter                     ADDR_WIDTH      = 6'd27,
        .ADDR_OFFSET      (  32'd0            ),//parameter                     ADDR_OFFSET     = 32'h0000_0000,
        .H_NUM            (  H_NUM            ),//parameter                     H_NUM           = 12'd1920,
        .V_NUM            (  V_NUM            ),//parameter                     V_NUM           = 12'd1080,
        .DQ_WIDTH         (  MEM_DQ_WIDTH     ),//parameter                     DQ_WIDTH        = 7'd32,
        .LEN_WIDTH        (  LEN_WIDTH        ),//parameter                     LEN_WIDTH       = 6'd16,
        .PIX_WIDTH        (  PIX_WIDTH        ),//parameter                     PIX_WIDTH       = 6'd24,
        .LINE_ADDR_WIDTH  (  LINE_ADDR_WIDTH  ),//parameter                     LINE_ADDR_WIDTH = 4'd19,
        .FRAME_CNT_WIDTH  (  FRAME_CNT_WIDTH  ) //parameter                     FRAME_CNT_WIDTH = 4'd8
    ) wr_buf (                                       
        .ddr_clk          (  ddr_clk          ),//input                         ddr_clk,
        .ddr_rstn         (  ddr_rstn         ),//input                         ddr_rstn,
                                              
        .wr_clk           (  vin_clk          ),//input                         wr_clk,
        .wr_fsync         (  wr_fsync         ),//input                         wr_fsync,
        .wr_en            (  camera_wr_en     ),//input                         wr_en,
        .wr_data          (  wr_data          ),//input  [PIX_WIDTH- 1'b1 : 0]  wr_data,
        
        .display_bank     (  display_bank     ),
        .bank_override_en (  affine_mode      ),
        .bank_override    (  affine_camera_bank ),
        .rd_bac           (  ddr_wr_bac       ),//input                         rd_bac,                                      
        .ddr_wreq         (  camera_wreq      ),//output                        ddr_wreq,
        .ddr_waddr        (  camera_waddr     ),//output [ADDR_WIDTH- 1'b1 : 0] ddr_waddr,
        .ddr_wr_len       (  camera_wlen      ),//output [LEN_WIDTH- 1'b1 : 0]  ddr_wr_len,
        .ddr_wrdy         (  camera_wrdy      ),//input                         ddr_wrdy,
        .ddr_wdone        (  camera_wdone     ),//input                         ddr_wdone,
        .ddr_wdata        (  camera_wdata     ),//output [8*DQ_WIDTH- 1'b1 : 0] ddr_wdata,
        .ddr_wdata_req    (  camera_wdata_req ),//input                         ddr_wdata_req,
                                              
        .frame_wcnt       (  completed_bank   ),//output [FRAME_CNT_WIDTH-1 :0] frame_wcnt,
        .frame_wirq       (  frame_wirq       ),//output                        frame_wirq
        .frame_start_pulse(  frame_start_pulse )
    );
    
    always @(posedge ddr_clk)
    begin
        if(!ddr_rstn)
            init_done <= 1'b0;
        else if(frame_wirq)
            init_done <= 1'b1;
        else
            init_done <= init_done;
    end 
    
    rd_buf #(
        .ADDR_WIDTH       (  CTRL_ADDR_WIDTH  ),//parameter                     ADDR_WIDTH      = 6'd27,
        .ADDR_OFFSET      (  32'h0000_0000    ),//parameter                     ADDR_OFFSET     = 32'h0000_0000,
        .H_NUM            (  H_NUM            ),//parameter                     H_NUM           = 12'd1920,
        .V_NUM            (  V_NUM            ),//parameter                     V_NUM           = 12'd1080,
        .DQ_WIDTH         (  MEM_DQ_WIDTH     ),//parameter                     DQ_WIDTH        = 7'd32,
        .LEN_WIDTH        (  LEN_WIDTH        ),//parameter                     LEN_WIDTH       = 6'd16,
        .PIX_WIDTH        (  PIX_WIDTH        ),//parameter                     PIX_WIDTH       = 6'd24,
        .LINE_ADDR_WIDTH  (  LINE_ADDR_WIDTH  ),//parameter                     LINE_ADDR_WIDTH = 4'd19,
        .FRAME_CNT_WIDTH  (  FRAME_CNT_WIDTH  ) //parameter                     FRAME_CNT_WIDTH = 4'd8
    ) rd_buf (
        .ddr_clk         (  ddr_clk           ),//input                         ddr_clk,
        .ddr_rstn        (  ddr_rstn          ),//input                         ddr_rstn,

        .vout_clk        (  vout_clk          ),//input                         vout_clk,
        .rd_fsync        (  rd_fsync          ),//input                         rd_fsync,
        .rd_en           (  rd_en             ),//input                         rd_en,
        .vout_de         (  vout_de           ),//output                        vout_de,
        .vout_data       (  vout_data         ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        
        .completed_bank  (  completed_bank    ),
        .display_bank    (  display_bank      ),
        .affine_display_mode ( affine_mode ),
        .processed_frame_valid ( processed_frame_valid ),
        .processed_frame_bank ( processed_frame_bank ),
        .display_vsync ( display_vsync ),
        .init_done       (  init_done         ),//input                         init_done,
      
        .ddr_rreq        (  display_rd_cmd_en         ),//output                        ddr_rreq,
        .ddr_raddr       (  display_rd_cmd_addr       ),//output [ADDR_WIDTH- 1'b1 : 0] ddr_raddr,
        .ddr_rd_len      (  display_rd_cmd_len        ),//output [LEN_WIDTH- 1'b1 : 0]  ddr_rd_len,
        .ddr_rrdy        (  display_rd_cmd_ready      ),//input                         ddr_rrdy,
        .ddr_rdone       (  display_rd_cmd_done       ),//input                         ddr_rdone,
                                              
        .ddr_rdata       (  display_rd_data        ),//input [8*DQ_WIDTH- 1'b1 : 0]  ddr_rdata,
        .ddr_rdata_en    (  display_rd_data_valid           ) //input                         ddr_rdata_en,
    );
    
    generate
        if (ENABLE_AFFINE_CLIENTS) begin : gen_affine_ddr_arbitration
            wire write_arbiter_busy;
            affine_ddr_write_arbiter #(
                .ADDR_WIDTH(CTRL_ADDR_WIDTH), .LEN_WIDTH(LEN_WIDTH), .DATA_WIDTH(MEM_DQ_WIDTH*8)
            ) write_channel_arbiter (
                .clk(ddr_clk), .rst_n(ddr_rstn),
                .camera_cmd_en(camera_wreq), .camera_cmd_addr(camera_waddr), .camera_cmd_len(camera_wlen),
                .camera_data(camera_wdata), .camera_cmd_ready(camera_wrdy), .camera_cmd_done(camera_wdone),
                .camera_data_req(camera_wdata_req),
                .affine_cmd_en(affine_wr_cmd_en), .affine_cmd_addr(affine_wr_cmd_addr),
                .affine_cmd_len(affine_wr_cmd_len), .affine_data(affine_wr_data),
                .affine_cmd_ready(affine_wr_cmd_ready), .affine_cmd_done(affine_wr_cmd_done),
                .affine_data_req(affine_wr_data_req),
                .ctrl_cmd_en(ddr_wreq), .ctrl_cmd_addr(ddr_waddr), .ctrl_cmd_len(ddr_wr_len),
                .ctrl_cmd_ready(ddr_wrdy), .ctrl_cmd_done(ddr_wdone), .ctrl_data_req(ddr_wdata_req),
                .ctrl_data(ddr_wdata), .busy(write_arbiter_busy), .error_pulse(affine_wr_error)
            );
            wire read_arbiter_busy;
            affine_ddr_read_arbiter #(
                .ADDR_WIDTH(CTRL_ADDR_WIDTH), .LEN_WIDTH(32), .DATA_WIDTH(MEM_DQ_WIDTH*8)
            ) read_channel_arbiter (
                .clk(ddr_clk), .rst_n(ddr_rstn),
                .display_cmd_en(display_rd_cmd_en), .display_cmd_addr(display_rd_cmd_addr),
                .display_cmd_len(display_rd_cmd_len), .display_cmd_ready(display_rd_cmd_ready),
                .display_cmd_done(display_rd_cmd_done), .display_read_valid(display_rd_data_valid),
                .display_read_data(display_rd_data),
                .affine_cmd_en(affine_rd_cmd_en), .affine_cmd_addr(affine_rd_cmd_addr),
                .affine_cmd_len(affine_rd_cmd_len), .affine_cmd_ready(affine_rd_cmd_ready),
                .affine_cmd_done(affine_rd_cmd_done), .affine_read_valid(affine_rd_data_valid),
                .affine_read_data(affine_rd_data),
                .ctrl_cmd_en(ctrl_rd_cmd_en), .ctrl_cmd_addr(ctrl_rd_cmd_addr),
                .ctrl_cmd_len(ctrl_rd_cmd_len), .ctrl_cmd_ready(ctrl_rd_cmd_ready),
                .ctrl_cmd_done(ctrl_rd_cmd_done), .ctrl_read_valid(read_en), .ctrl_read_data(read_rdata),
                .busy(read_arbiter_busy), .error_pulse(affine_rd_error)
            );
        end else begin : gen_legacy_direct_ddr_channels
            // Preserve the proven ETH_SNAPSHOT camera/display DDR handshake
            // when the full-frame affine pipeline is compiled out.
            assign ddr_wreq = camera_wreq;
            assign ddr_waddr = camera_waddr;
            assign ddr_wr_len = camera_wlen;
            assign camera_wrdy = ddr_wrdy;
            assign camera_wdone = ddr_wdone;
            assign ddr_wdata = camera_wdata;
            assign camera_wdata_req = ddr_wdata_req;

            assign ctrl_rd_cmd_en = display_rd_cmd_en;
            assign ctrl_rd_cmd_addr = display_rd_cmd_addr;
            assign ctrl_rd_cmd_len = display_rd_cmd_len;
            assign display_rd_cmd_ready = ctrl_rd_cmd_ready;
            assign display_rd_cmd_done = ctrl_rd_cmd_done;
            assign display_rd_data_valid = read_en;
            assign display_rd_data = read_rdata;

            assign affine_wr_cmd_ready = 1'b0;
            assign affine_wr_cmd_done = 1'b0;
            assign affine_wr_data_req = 1'b0;
            assign affine_wr_error = 1'b0;
            assign affine_rd_cmd_ready = 1'b0;
            assign affine_rd_cmd_done = 1'b0;
            assign affine_rd_data_valid = 1'b0;
            assign affine_rd_data = {(MEM_DQ_WIDTH*8){1'b0}};
            assign affine_rd_error = 1'b0;
        end
    endgenerate
    wr_rd_ctrl_top#(
        .CTRL_ADDR_WIDTH  (  CTRL_ADDR_WIDTH  ),//parameter                    CTRL_ADDR_WIDTH      = 28,
        .MEM_DQ_WIDTH     (  MEM_DQ_WIDTH     ) //parameter                    MEM_DQ_WIDTH         = 16
    )wr_rd_ctrl_top (                         
        .clk              (  ddr_clk          ),//input                        clk            ,            
        .rstn             (  ddr_rstn         ),//input                        rstn           ,            
                                              
        .wr_cmd_en        (  ddr_wreq         ),//input                        wr_cmd_en   ,
        .wr_cmd_addr      (  ddr_waddr        ),//input  [CTRL_ADDR_WIDTH-1:0] wr_cmd_addr ,
        .wr_cmd_len       (  ddr_wr_len       ),//input  [31£º0]               wr_cmd_len  ,
        .wr_cmd_ready     (  ddr_wrdy         ),//output                       wr_cmd_ready,
        .wr_cmd_done      (  ddr_wdone        ),//output                       wr_cmd_done,
        .wr_bac           (  ddr_wr_bac       ),//output                       wr_bac,                                     
        .wr_ctrl_data     (  ddr_wdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  wr_ctrl_data,
        .wr_data_re       (  ddr_wdata_req    ),//output                       wr_data_re  ,
                                              
        .rd_cmd_en        (  ctrl_rd_cmd_en        ),//input                        ctrl_rd_cmd_en   ,
        .rd_cmd_addr      (  ctrl_rd_cmd_addr      ),//input  [CTRL_ADDR_WIDTH-1:0] ctrl_rd_cmd_addr ,
        .rd_cmd_len       (  ctrl_rd_cmd_len       ),//input  [31£º0]               ctrl_rd_cmd_len  ,
        .rd_cmd_ready     (  ctrl_rd_cmd_ready     ),//output                       ctrl_rd_cmd_ready, 
        .rd_cmd_done      (  ctrl_rd_cmd_done      ),//output                       ctrl_rd_cmd_done,
                                              
        .read_ready       (  read_ready       ),//input                        read_ready  ,    
        .read_rdata       (  read_rdata       ),//output [MEM_DQ_WIDTH*8-1:0]  read_rdata  ,    
        .read_en          (  read_en          ),//output                       read_en     ,                                          
        // write channel                        
        .axi_awaddr       (  axi_awaddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_awaddr     ,  
        .axi_awid         (  axi_awid         ),//output [3:0]                 axi_awid       ,
        .axi_awlen        (  axi_awlen        ),//output [3:0]                 axi_awlen      ,
        .axi_awsize       (  axi_awsize       ),//output [2:0]                 axi_awsize     ,
        .axi_awburst      (  axi_awburst      ),//output [1:0]                 axi_awburst    , //only support 2'b01: INCR
        .axi_awready      (  axi_awready      ),//input                        axi_awready    ,
        .axi_awvalid      (  axi_awvalid      ),//output                       axi_awvalid    ,
                                              
        .axi_wdata        (  axi_wdata        ),//output [MEM_DQ_WIDTH*8-1:0]  axi_wdata      ,
        .axi_wstrb        (  axi_wstrb        ),//output [MEM_DQ_WIDTH -1 :0]  axi_wstrb      ,
        .axi_wlast        (  axi_wlast        ),//output                       axi_wlast      ,
        .axi_wvalid       (  axi_wvalid       ),//output                       axi_wvalid     ,
        .axi_wready       (  axi_wready       ),//input                        axi_wready     ,
        .axi_bid          (  4'd0             ),//input  [3 : 0]               axi_bid        , // Master Interface Write Response.
        .axi_bresp        (  2'd0             ),//input  [1 : 0]               axi_bresp      , // Write response. This signal indicates the status of the write transaction.
        .axi_bvalid       (  1'b0             ),//input                        axi_bvalid     , // Write response valid. This signal indicates that the channel is signaling a valid write response.
        .axi_bready       (                   ),//output                       axi_bready     ,
                                              
        // read channel                          
        .axi_araddr       (  axi_araddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_araddr     ,    
        .axi_arid         (  axi_arid         ),//output [3:0]                 axi_arid       ,
        .axi_arlen        (  axi_arlen        ),//output [3:0]                 axi_arlen      ,
        .axi_arsize       (  axi_arsize       ),//output [2:0]                 axi_arsize     ,
        .axi_arburst      (  axi_arburst      ),//output [1:0]                 axi_arburst    ,
        .axi_arvalid      (  axi_arvalid      ),//output                       axi_arvalid    , 
        .axi_arready      (  axi_arready      ),//input                        axi_arready    , //only support 2'b01: INCR
                                              
        .axi_rready       (  axi_rready       ),//output                       axi_rready     ,
        .axi_rdata        (  axi_rdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  axi_rdata      ,
        .axi_rvalid       (  axi_rvalid       ),//input                        axi_rvalid     ,
        .axi_rlast        (  axi_rlast        ),//input                        axi_rlast      ,
        .axi_rid          (  axi_rid          ),//input  [3:0]                 axi_rid        ,
        .axi_rresp        (  2'd0             ) //input  [1:0]                 axi_rresp      
    );


endmodule
