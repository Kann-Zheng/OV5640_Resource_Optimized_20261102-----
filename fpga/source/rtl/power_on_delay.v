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
//camera power on timing requirement
module power_on_delay(                  
    input  clk_50M        ,
    input  reset_n        ,
    output camera1_rstn   ,
    output camera2_rstn   ,
    output camera_pwnd    ,
    output initial_en
);
reg [18:0]cnt1;
reg [15:0]cnt2;
reg [19:0]cnt3;
reg initial_en;
reg camera_rstn_reg;
reg camera_pwnd_reg;

assign camera1_rstn=camera_rstn_reg;
assign camera2_rstn=camera_rstn_reg;
assign camera_pwnd=camera_pwnd_reg;

// 0x40000 clocks: approximately 9.71 ms at the actual 27 MHz input.
always@(posedge clk_50M or negedge reset_n)begin
  if(reset_n==1'b0) begin
	    cnt1<=0;
		camera_pwnd_reg<=1'b1;// 1'b1 
  end
  else if(cnt1<19'h40000) begin
       cnt1<=cnt1+1'b1;
       camera_pwnd_reg<=1'b1;
  end
  else
     camera_pwnd_reg<=1'b0;         
end

// 0xffff clocks: approximately 2.43 ms at 27 MHz.
always@(posedge clk_50M or negedge reset_n)begin
  if(!reset_n) begin
     cnt2 <= 0;
     camera_rstn_reg <= 1'b0;
  end
  else if(camera_pwnd_reg==1)  begin
	 cnt2<=0;
     camera_rstn_reg<=1'b0;  
  end
  else if(cnt2<16'hffff) begin
       cnt2<=cnt2+1'b1;
       camera_rstn_reg<=1'b0;
  end
  else
     camera_rstn_reg<=1'b1;         
end

// 0xfffff clocks: approximately 38.84 ms before SCCB initialization.
always@(posedge clk_50M or negedge reset_n)begin
  if(!reset_n) begin
     cnt3 <= 0;
     initial_en <= 1'b0;
  end
  else if(camera_rstn_reg==0) begin
         cnt3<=0;
         initial_en<=1'b0;
  end
  else if(cnt3<20'hfffff) begin
        cnt3<=cnt3+1'b1;
        initial_en<=1'b0;
  end
  else
       initial_en<=1'b1;    
end

endmodule
