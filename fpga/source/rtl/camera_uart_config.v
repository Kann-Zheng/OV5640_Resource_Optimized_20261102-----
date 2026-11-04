`timescale 1ns/1ps
// UART command receiver for camera calibration/IMU pose parameters.
// V1: A5 5A 01 flags cx_hi cx_lo cy_hi cy_lo slope_hi slope_lo crc_hi crc_lo.
// V2: A5 5A 02 flags cx cy slope plus fx/fy/inv_fx/inv_fy/k1/k2/p1/p2 (8 x int32), CRC.
// V3: A5 5A 03 flags cx cy inverse affine matrix (4 x Q2.14) plus the same 8 x int32 K/D fields, CRC.
// V4: A5 5A 04 command CRC requests one snapshot (1=full, 2=raw preview, 3=corrected preview).
// V5: A5 5A 05 01 CRC requests FPGA frame and processing timing counters.
// CRC16-CCITT-FALSE covers version through the last payload byte.
module camera_uart_config #(
    // UART receiver runs in the pixel-clock domain in the board design.
    // 37.125 MHz gives 322 clocks/bit at 115200 baud (about +0.08%).
    parameter CLK_HZ = 37125000,
    parameter BAUD = 115200,
    parameter APPLY_AT_FRAME_START = 0,
    parameter FIRMWARE_MODE = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rxd,
    input  wire        pix_clk,
    input  wire        frame_vs_pix,
    output wire        uart_txd,
    output reg  [11:0] cx_pix,
    output reg  [11:0] cy_pix,
    output reg  signed [15:0] slope_q14_pix,
    output reg  signed [31:0] fx_q16_pix,
    output reg  signed [31:0] fy_q16_pix,
    output reg  signed [31:0] inv_fx_q30_pix,
    output reg  signed [31:0] inv_fy_q30_pix,
    output reg  signed [31:0] k1_q24_pix,
    output reg  signed [31:0] k2_q24_pix,
    output reg  signed [31:0] p1_q24_pix,
    output reg  signed [31:0] p2_q24_pix,
    output reg  signed [15:0] affine_a_q14_pix,
    output reg  signed [15:0] affine_b_q14_pix,
    output reg  signed [15:0] affine_c_q14_pix,
    output reg  signed [15:0] affine_d_q14_pix,
    output reg         [7:0] protocol_version_pix,
    output reg         warp_enable_pix,
    output reg         brown_enable_pix,
    output reg         gamma_enable_pix,
    output reg         grayscale_enable_pix,
    output reg         config_applied_pix,
    output reg         capture_request_pix,
    output reg  [1:0]  capture_mode_pix,
    output reg         telemetry_request_pix,
    input wire [31:0]  input_period_ticks,
    input wire [31:0]  output_period_ticks,
    input wire [31:0]  processing_ticks,
    input wire [15:0]  dropped_frames
);
    localparam CLKS_PER_BIT = CLK_HZ / BAUD;

    reg [1:0] rx_sync;
    reg [15:0] baud_count;
    reg [3:0] bit_index;
    reg [7:0] rx_shift;
    reg rx_busy;
    reg rx_valid;
    reg [7:0] rx_byte;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
            baud_count <= 0;
            bit_index <= 0;
            rx_shift <= 0;
            rx_busy <= 0;
            rx_valid <= 0;
            rx_byte <= 0;
        end else begin
            rx_sync <= {rx_sync[0], uart_rxd};
            rx_valid <= 1'b0;
            if (!rx_busy) begin
                if (rx_sync[1] == 1'b0) begin
                    rx_busy <= 1'b1;
                    baud_count <= CLKS_PER_BIT + CLKS_PER_BIT/2 - 1;
                    bit_index <= 0;
                end
            end else if (baud_count != 0) begin
                baud_count <= baud_count - 1'b1;
            end else begin
                baud_count <= CLKS_PER_BIT - 1;
                if (bit_index < 8) begin
                    rx_shift[bit_index] <= rx_sync[1];
                    bit_index <= bit_index + 1'b1;
                end else begin
                    rx_busy <= 1'b0;
                    if (rx_sync[1]) begin
                        rx_byte <= rx_shift;
                        rx_valid <= 1'b1;
                    end
                end
            end
        end
    end

    function [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer i;
        reg [15:0] c;
        begin
            c = crc_in ^ {data_in, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    localparam P_SOF0 = 2'd0, P_SOF1 = 2'd1, P_DATA = 2'd2;
    reg [1:0] parser_state;
    reg [5:0] byte_index;
    reg [5:0] expected_data_bytes;
    reg [7:0] crc_hi;
    reg [15:0] crc_work;
    reg [7:0] frame_data [0:45];
    reg cfg_toggle;
    reg [1:0] cfg_sync;
    reg cfg_seen;
    reg cfg_pending;
    reg [11:0] cx_bus;
    reg [11:0] cy_bus;
    reg signed [15:0] slope_bus;
    reg signed [31:0] fx_q16_bus, fy_q16_bus;
    reg signed [31:0] inv_fx_q30_bus, inv_fy_q30_bus;
    reg signed [31:0] k1_q24_bus, k2_q24_bus, p1_q24_bus, p2_q24_bus;
    reg signed [15:0] affine_a_q14_bus, affine_b_q14_bus;
    reg signed [15:0] affine_c_q14_bus, affine_d_q14_bus;
    reg [7:0] protocol_version_bus;
    reg [15:0] command_crc_bus;
    reg [15:0] command_crc_pix;
    reg [15:0] applied_sequence;
    reg frame_vs_d;
    reg enable_bus;
    reg brown_enable_bus;
    reg gamma_enable_bus;
    reg grayscale_enable_bus;
    reg capture_toggle;
    reg [1:0] capture_sync;
    reg capture_seen;
    reg [1:0] capture_mode_bus;
    reg telemetry_toggle;
    reg [1:0] telemetry_sync;
    reg telemetry_seen;

    // Brown-enabled packets use an unsigned Q2.30 inverse focal operand in
    // the pipelined mapper. Keep malformed/out-of-range packets from being
    // truncated by its 24-bit multiplier input.
    wire brown_inverse_fits = (frame_data[0] == 8'h01) || !frame_data[1][1] ||
        ((frame_data[0] == 8'h02) && (frame_data[16] == 8'h00) && (frame_data[20] == 8'h00)) ||
        ((frame_data[0] == 8'h03) && (frame_data[22] == 8'h00) && (frame_data[26] == 8'h00));
    wire center_fits_720p = ({frame_data[2][3:0], frame_data[3]} <= 12'd1279) &&
                            ({frame_data[4][3:0], frame_data[5]} <= 12'd719);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parser_state <= P_SOF0;
            crc_work <= 16'hFFFF;
            crc_hi <= 0;
            byte_index <= 0;
            expected_data_bytes <= 0;
            cfg_toggle <= 0;
            capture_toggle <= 0;
            telemetry_toggle <= 0;
            capture_mode_bus <= 0;
            cx_bus <= 12'd640;
            cy_bus <= 12'd360;
            slope_bus <= 0;
            fx_q16_bus <= 0; fy_q16_bus <= 0;
            inv_fx_q30_bus <= 0; inv_fy_q30_bus <= 0;
            k1_q24_bus <= 0; k2_q24_bus <= 0;
            p1_q24_bus <= 0; p2_q24_bus <= 0;
            affine_a_q14_bus <= 16'sd16384; affine_b_q14_bus <= 0;
            affine_c_q14_bus <= 0; affine_d_q14_bus <= 16'sd16384;
            protocol_version_bus <= 0;
            command_crc_bus <= 0;
            enable_bus <= 0;
            brown_enable_bus <= 0;
            gamma_enable_bus <= 0;
            grayscale_enable_bus <= 0;
        end else if (rx_valid) begin
            case (parser_state)
                P_SOF0: parser_state <= (rx_byte == 8'hA5) ? P_SOF1 : P_SOF0;
                P_SOF1: begin
                    if (rx_byte == 8'h5A) begin
                        parser_state <= P_DATA;
                        byte_index <= 0;
                        expected_data_bytes <= 0;
                        crc_work <= 16'hFFFF;
                    end else parser_state <= (rx_byte == 8'hA5) ? P_SOF1 : P_SOF0;
                end
                P_DATA: begin
                    if (byte_index == 0) begin
                        if (rx_byte == 8'h01 || rx_byte == 8'h02 || rx_byte == 8'h03 || rx_byte == 8'h04 || rx_byte == 8'h05) begin
                            frame_data[0] <= rx_byte;
                            case (rx_byte)
                                8'h01: expected_data_bytes <= 6'd8;
                                8'h02: expected_data_bytes <= 6'd40;
                                8'h03: expected_data_bytes <= 6'd46;
                                default: expected_data_bytes <= 6'd2;
                            endcase
                            crc_work <= crc16_byte(16'hFFFF, rx_byte);
                            byte_index <= 1;
                        end else begin
                            parser_state <= (rx_byte == 8'hA5) ? P_SOF1 : P_SOF0;
                        end
                    end else if (byte_index < expected_data_bytes) begin
                        frame_data[byte_index] <= rx_byte;
                        crc_work <= crc16_byte(crc_work, rx_byte);
                        byte_index <= byte_index + 1'b1;
                    end else if (byte_index == expected_data_bytes) begin
                        crc_hi <= rx_byte;
                        byte_index <= byte_index + 1'b1;
                    end else begin
                        if (({crc_hi, rx_byte} == crc_work) &&
                            ((frame_data[0] == 8'h04) ?
                             (frame_data[1] >= 8'h01 && frame_data[1] <= 8'h03) :
                             ((frame_data[0] == 8'h05) ? (frame_data[1] == 8'h01) :
                              (brown_inverse_fits && center_fits_720p)))) begin
                            if (frame_data[0] == 8'h04) begin
                                capture_mode_bus <= frame_data[1] - 1'b1;
                                capture_toggle <= ~capture_toggle;
                            end else if (frame_data[0] == 8'h05) begin
                                telemetry_toggle <= ~telemetry_toggle;
                            end else begin
                            cx_bus <= {frame_data[2][3:0], frame_data[3]};
                            cy_bus <= {frame_data[4][3:0], frame_data[5]};
                            slope_bus <= (frame_data[0] == 8'h03) ? 16'sd0 : {frame_data[6], frame_data[7]};
                            enable_bus <= frame_data[1][0];
                            gamma_enable_bus <= (frame_data[0] == 8'h03) && frame_data[1][2];
                            grayscale_enable_bus <= (frame_data[0] == 8'h03) && frame_data[1][3];
                            protocol_version_bus <= frame_data[0];
                            command_crc_bus <= {crc_hi, rx_byte};
                            if (frame_data[0] == 8'h02) begin
                                brown_enable_bus <= frame_data[1][1];
                                affine_a_q14_bus <= 16'sd16384; affine_b_q14_bus <= 0;
                                affine_c_q14_bus <= 0; affine_d_q14_bus <= 16'sd16384;
                                fx_q16_bus <= {frame_data[8], frame_data[9], frame_data[10], frame_data[11]};
                                fy_q16_bus <= {frame_data[12], frame_data[13], frame_data[14], frame_data[15]};
                                inv_fx_q30_bus <= {frame_data[16], frame_data[17], frame_data[18], frame_data[19]};
                                inv_fy_q30_bus <= {frame_data[20], frame_data[21], frame_data[22], frame_data[23]};
                                k1_q24_bus <= {frame_data[24], frame_data[25], frame_data[26], frame_data[27]};
                                k2_q24_bus <= {frame_data[28], frame_data[29], frame_data[30], frame_data[31]};
                                p1_q24_bus <= {frame_data[32], frame_data[33], frame_data[34], frame_data[35]};
                                p2_q24_bus <= {frame_data[36], frame_data[37], frame_data[38], frame_data[39]};
                            end else if (frame_data[0] == 8'h03) begin
                                brown_enable_bus <= frame_data[1][1];
                                affine_a_q14_bus <= {frame_data[6], frame_data[7]};
                                affine_b_q14_bus <= {frame_data[8], frame_data[9]};
                                affine_c_q14_bus <= {frame_data[10], frame_data[11]};
                                affine_d_q14_bus <= {frame_data[12], frame_data[13]};
                                fx_q16_bus <= {frame_data[14], frame_data[15], frame_data[16], frame_data[17]};
                                fy_q16_bus <= {frame_data[18], frame_data[19], frame_data[20], frame_data[21]};
                                inv_fx_q30_bus <= {frame_data[22], frame_data[23], frame_data[24], frame_data[25]};
                                inv_fy_q30_bus <= {frame_data[26], frame_data[27], frame_data[28], frame_data[29]};
                                k1_q24_bus <= {frame_data[30], frame_data[31], frame_data[32], frame_data[33]};
                                k2_q24_bus <= {frame_data[34], frame_data[35], frame_data[36], frame_data[37]};
                                p1_q24_bus <= {frame_data[38], frame_data[39], frame_data[40], frame_data[41]};
                                p2_q24_bus <= {frame_data[42], frame_data[43], frame_data[44], frame_data[45]};
                            end else brown_enable_bus <= 1'b0;
                            if (frame_data[0] == 8'h01) begin
                                brown_enable_bus <= 1'b0;
                                affine_a_q14_bus <= 16'sd16384; affine_b_q14_bus <= 0;
                                affine_c_q14_bus <= 0; affine_d_q14_bus <= 16'sd16384;
                            end
                            cfg_toggle <= ~cfg_toggle;
                            end
                        end
                        parser_state <= P_SOF0;
                        byte_index <= 0;
                    end
                end
                default: parser_state <= P_SOF0;
            endcase
        end
    end

    // Stable configuration bus plus toggle synchronizer crosses infrequent updates.
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_sync <= 0; cfg_seen <= 0; cfg_pending <= 0;
            capture_sync <= 0; capture_seen <= 0;
            telemetry_sync <= 0; telemetry_seen <= 0;
            capture_mode_pix <= 0;
            cx_pix <= 12'd640; cy_pix <= 12'd360;
            slope_q14_pix <= 0; warp_enable_pix <= 0;
            fx_q16_pix <= 0; fy_q16_pix <= 0;
            inv_fx_q30_pix <= 0; inv_fy_q30_pix <= 0;
            k1_q24_pix <= 0; k2_q24_pix <= 0;
            p1_q24_pix <= 0; p2_q24_pix <= 0;
            affine_a_q14_pix <= 16'sd16384; affine_b_q14_pix <= 0;
            affine_c_q14_pix <= 0; affine_d_q14_pix <= 16'sd16384;
            protocol_version_pix <= 0;
            command_crc_pix <= 0;
            applied_sequence <= 0;
            frame_vs_d <= 0;
            config_applied_pix <= 0;
            capture_request_pix <= 0;
            telemetry_request_pix <= 0;
            brown_enable_pix <= 1'b0;
            gamma_enable_pix <= 1'b0;
            grayscale_enable_pix <= 1'b0;
        end else begin
            frame_vs_d <= frame_vs_pix;
            cfg_sync <= {cfg_sync[0], cfg_toggle};
            capture_sync <= {capture_sync[0], capture_toggle};
            telemetry_sync <= {telemetry_sync[0], telemetry_toggle};
            config_applied_pix <= 1'b0;
            capture_request_pix <= 1'b0;
            telemetry_request_pix <= 1'b0;
            if (cfg_sync[1] != cfg_seen) begin
                cfg_seen <= cfg_sync[1];
                cfg_pending <= 1'b1;
            end
            if (capture_sync[1] != capture_seen) begin
                capture_seen <= capture_sync[1];
                capture_mode_pix <= capture_mode_bus;
                capture_request_pix <= 1'b1;
            end
            if (telemetry_sync[1] != telemetry_seen) begin
                telemetry_seen <= telemetry_sync[1];
                telemetry_request_pix <= 1'b1;
            end
            if (cfg_pending &&
                (!APPLY_AT_FRAME_START || (frame_vs_pix && !frame_vs_d))) begin
                cfg_pending <= 1'b0;
                cx_pix <= cx_bus;
                cy_pix <= cy_bus;
                slope_q14_pix <= slope_bus;
                warp_enable_pix <= enable_bus;
                fx_q16_pix <= fx_q16_bus;
                fy_q16_pix <= fy_q16_bus;
                inv_fx_q30_pix <= inv_fx_q30_bus;
                inv_fy_q30_pix <= inv_fy_q30_bus;
                k1_q24_pix <= k1_q24_bus;
                k2_q24_pix <= k2_q24_bus;
                p1_q24_pix <= p1_q24_bus;
                p2_q24_pix <= p2_q24_bus;
                affine_a_q14_pix <= affine_a_q14_bus;
                affine_b_q14_pix <= affine_b_q14_bus;
                affine_c_q14_pix <= affine_c_q14_bus;
                affine_d_q14_pix <= affine_d_q14_bus;
                protocol_version_pix <= protocol_version_bus;
                command_crc_pix <= command_crc_bus;
                applied_sequence <= applied_sequence + 1'b1;
                brown_enable_pix <= brown_enable_bus;
                gamma_enable_pix <= gamma_enable_bus;
                grayscale_enable_pix <= grayscale_enable_bus;
                config_applied_pix <= 1'b1;
            end
        end
    end
    camera_config_ack_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) ack_tx (
        .clk(pix_clk), .rst_n(rst_n), .request(config_applied_pix),
        .telemetry_request(telemetry_request_pix),
        .protocol_version(protocol_version_pix),
        .flags({(FIRMWARE_MODE != 0), (APPLY_AT_FRAME_START != 0), 2'b0,
                grayscale_enable_pix, gamma_enable_pix, brown_enable_pix, warp_enable_pix}),
        .sequence_id(applied_sequence), .command_crc(command_crc_pix),
        .input_period_ticks(input_period_ticks), .output_period_ticks(output_period_ticks),
        .processing_ticks(processing_ticks), .dropped_frames(dropped_frames), .tx(uart_txd)
    );
endmodule

// ACK: A5 5A 85 protocol flags applied_sequence[15:0] command_crc[15:0] CRC16.
// Telemetry: A5 5A 86 sequence[15:0], three 32-bit tick counters, drops[15:0], CRC16.
module camera_config_ack_tx #(parameter CLK_HZ=37125000, parameter BAUD=115200)(
    input wire clk, rst_n, request,
    input wire telemetry_request,
    input wire [7:0] protocol_version, flags,
    input wire [15:0] sequence_id, command_crc,
    input wire [31:0] input_period_ticks, output_period_ticks, processing_ticks,
    input wire [15:0] dropped_frames,
    output wire tx
);
    localparam CLKS=CLK_HZ/BAUD;
    function [15:0] crc7;
        input [55:0] data;
        reg [15:0] c; integer i,j;
        begin
            c=16'hffff;
            for(i=6;i>=0;i=i-1) begin
                c=c ^ ({8'd0,data[i*8 +: 8]} << 8);
                for(j=0;j<8;j=j+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            end
            crc7=c;
        end
    endfunction
    function [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0] data_in;
        reg [15:0] c; integer j;
        begin
            c=crc_in ^ {data_in,8'h00};
            for(j=0;j<8;j=j+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc16_byte=c;
        end
    endfunction
    reg ack_pending=0, telemetry_pending=0, telemetry_ready=0;
    reg telemetry_crc_busy=0, busy=0;
    reg [7:0] ack_protocol=0, ack_flags=0;
    reg [15:0] ack_sequence=0, ack_command_crc=0;
    reg [15:0] telemetry_sequence=0, telemetry_pending_sequence=0;
    reg [31:0] telemetry_input_pending=0, telemetry_output_pending=0;
    reg [31:0] telemetry_process_pending=0;
    reg [15:0] telemetry_drops_pending=0;
    reg [135:0] telemetry_payload_latched=0;
    reg [15:0] telemetry_crc_work=16'hffff, telemetry_crc=0;
    reg [4:0] telemetry_crc_index=0;
    reg [7:0] frame [0:20];
    reg [4:0] frame_length=0, frame_index=0;
    reg [9:0] shift=10'h3ff;
    reg [15:0] baud_count=0;
    reg [3:0] bit_count=0;
    wire [55:0] ack_payload = {8'h85,ack_protocol,ack_flags,ack_sequence,ack_command_crc};
    integer k;
    wire [15:0] ack_crc_value = crc7(ack_payload);
    wire [7:0] telemetry_crc_byte = telemetry_payload_latched[135-telemetry_crc_index*8 -: 8];
    wire [15:0] telemetry_crc_next = crc16_byte(telemetry_crc_work, telemetry_crc_byte);
    assign tx=busy ? shift[0] : 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0; shift<=10'h3ff;
            baud_count<=0; bit_count<=0; frame_length<=0; frame_index<=0;
            ack_pending<=0; telemetry_pending<=0; telemetry_ready<=0;
            telemetry_crc_busy<=0; telemetry_crc_work<=16'hffff;
            telemetry_crc_index<=0; telemetry_crc<=0; telemetry_sequence<=0;
            telemetry_pending_sequence<=0;
            ack_protocol<=0; ack_flags<=0; ack_sequence<=0; ack_command_crc<=0;
            telemetry_input_pending<=0; telemetry_output_pending<=0;
            telemetry_process_pending<=0; telemetry_drops_pending<=0;
            telemetry_payload_latched<=0;
            for(k=0;k<21;k=k+1) frame[k]<=0;
        end else begin
            if(request) begin
                ack_pending<=1'b1;
                ack_protocol<=protocol_version; ack_flags<=flags;
                ack_sequence<=sequence_id; ack_command_crc<=command_crc;
            end
            if(telemetry_request) begin
                telemetry_pending<=1'b1;
                telemetry_sequence<=telemetry_sequence+1'b1;
                telemetry_pending_sequence<=telemetry_sequence+1'b1;
                telemetry_input_pending<=input_period_ticks;
                telemetry_output_pending<=output_period_ticks;
                telemetry_process_pending<=processing_ticks;
                telemetry_drops_pending<=dropped_frames;
            end
            if(telemetry_crc_busy) begin
                telemetry_crc_work<=telemetry_crc_next;
                if(telemetry_crc_index==5'd16) begin
                    telemetry_crc<=telemetry_crc_next;
                    telemetry_crc_busy<=1'b0;
                    telemetry_ready<=1'b1;
                end else telemetry_crc_index<=telemetry_crc_index+1'b1;
            end
            if(!busy) begin
                if(ack_pending) begin
                    frame[0]<=8'ha5; frame[1]<=8'h5a;
                    frame[2]<=ack_payload[55:48]; frame[3]<=ack_payload[47:40];
                    frame[4]<=ack_payload[39:32]; frame[5]<=ack_payload[31:24];
                    frame[6]<=ack_payload[23:16]; frame[7]<=ack_payload[15:8];
                    frame[8]<=ack_payload[7:0];
                    frame[9]<=ack_crc_value[15:8]; frame[10]<=ack_crc_value[7:0];
                    frame_length<=5'd11; frame_index<=0;
                    shift<={1'b1,8'ha5,1'b0}; busy<=1'b1;
                    baud_count<=CLKS-1; bit_count<=0; ack_pending<=1'b0;
                end else if(telemetry_ready) begin
                    frame[0]<=8'ha5; frame[1]<=8'h5a;
                    for(k=0;k<17;k=k+1)
                        frame[k+2]<=telemetry_payload_latched[135-k*8 -: 8];
                    frame[19]<=telemetry_crc[15:8]; frame[20]<=telemetry_crc[7:0];
                    frame_length<=5'd21; frame_index<=0;
                    shift<={1'b1,8'ha5,1'b0}; busy<=1'b1;
                    baud_count<=CLKS-1; bit_count<=0; telemetry_ready<=1'b0;
                end else if(telemetry_pending && !telemetry_crc_busy) begin
                    telemetry_payload_latched<={8'h86,telemetry_pending_sequence,
                        telemetry_input_pending,telemetry_output_pending,
                        telemetry_process_pending,telemetry_drops_pending};
                    telemetry_crc_work<=16'hffff; telemetry_crc_index<=0;
                    telemetry_crc_busy<=1'b1; telemetry_pending<=1'b0;
                end
            end else if(baud_count!=0) baud_count<=baud_count-1'b1;
            else begin
                baud_count<=CLKS-1;
                if(bit_count==9) begin
                    bit_count<=0;
                    if(frame_index+1'b1>=frame_length) busy<=1'b0;
                    else begin
                        frame_index<=frame_index+1'b1;
                        shift<={1'b1,frame[frame_index+1'b1],1'b0};
                    end
                end else begin
                    shift<={1'b1,shift[9:1]}; bit_count<=bit_count+1'b1;
                end
            end
        end
    end
endmodule
