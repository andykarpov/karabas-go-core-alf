`timescale 1ns / 1ps
`default_nettype wire

/*-------------------------------------------------------------------------------------------------------------------
-- 
-- 
-- #       #######                                                 #                                               
-- #                                                               #                                               
-- #                                                               #                                               
-- ############### ############### ############### ############### ############### ############### ############### 
-- #             #               # #                             # #             #               # #               
-- #             # ############### #               ############### #             # ############### ############### 
-- #             # #             # #               #             # #             # #             #               # 
-- #             # ############### #               ############### ############### ############### ############### 
--                                                                                                                 
--         ####### ####### ####### #######                                         ############### ############### 
--                                                                                 #               #             # 
--                                                                                 #   ########### #             # 
--                                                                                 #             # #             # 
-- https://github.com/andykarpov/karabas-go                                        ############### ############### 
--
-- FPGA Alf core for Karabas-Go Mini revG
--
-- @author Andy Karpov <andy.karpov@gmail.com>
-- Ukraine, 2024, 2025
------------------------------------------------------------------------------------------------------------------*/

module karabas_minig_top (
   //---------------------------
   input wire CLK_50MHZ,

	//---------------------------
	inout wire UART_RX,
	inout wire UART_TX,
	inout wire UART_CTS,
	
   //---------------------------
   output wire [20:0] MA,
   inout wire [15:0] MD,
   output wire [1:0] MWR_N,
   output wire [1:0] MRD_N,

   //---------------------------
	output wire [1:0] SDR_BA,
	output wire [12:0] SDR_A,
	output wire SDR_CLK,
	output wire [1:0] SDR_DQM,
	output wire SDR_WE_N,
	output wire SDR_CAS_N,
	output wire SDR_RAS_N,
	inout wire [15:0] SDR_DQ,

   //---------------------------
   output wire SD_CS_N,
   output wire SD_CLK,
   output wire SD_DI,
   input wire SD_DO,
	input wire SD_DET_N,

   //---------------------------
   input wire [7:0] VGA_R,
   input wire [7:0] VGA_G,
   input wire [7:0] VGA_B,
   input wire VGA_HS,
   input wire VGA_VS,
	
   output wire [3:0] TMDS_P,
   output wire [3:0] TMDS_N,	
	
	//---------------------------
	output wire FT_SPI_CS_N,
	output wire FT_SPI_SCK,
	input wire FT_SPI_MISO,
	output wire FT_SPI_MOSI,
	input wire FT_INT_N,
	input wire FT_CLK,
	input wire FT_DE,
	output wire FT_CLK_OUT,

	//---------------------------
	output wire [2:0] WA,
	output wire [1:0] WCS_N,
	output wire WRD_N,
	output wire WWR_N,
	output wire WRESET_N,
	inout wire [15:0] WD,
	
   //---------------------------	
	output wire TAPE_OUT,
	input wire TAPE_IN,
	output wire DAC_BCK,
	output wire DAC_WS,
	output wire DAC_DAT,
	
	//---------------------------
	output wire ADC_CLK,
   inout wire ADC_BCK,
	inout wire ADC_LRCK,
   input wire ADC_DOUT,
	
	//---------------------------
	input wire MCU_CS_N,
	input wire MCU_SCK,
	inout wire MCU_MOSI,
	output wire MCU_MISO,	
	input wire [5:0] MCU_IO,
	
	//---------------------------
	output wire MIDI_TX,
	
	output wire ESP32_SPI_CS_N,
	input wire ESP32_PCM_BCK,
	input wire ESP32_PCM_RLCK,
	input wire ESP32_PCM_DAT,
	
	//---------------------------
	output wire FLASH_CS_N,
	input wire  FLASH_DO,
	output wire FLASH_DI,
	output wire FLASH_SCK,
	output wire FLASH_WP_N,
	output wire FLASH_HOLD_N	
);

// unused signals

assign SDR_BA = 2'b11;
assign SDR_A = 13'b1111111111111;
assign SDR_CLK = 1'b1;
assign SDR_DQM = 2'b11;
assign SDR_WE_N = 1'b1;
assign SDR_CAS_N = 1'b1;
assign SDR_RAS_N = 1'b1;

assign WA = 3'b000;
assign WCS_N = 2'b11;
assign WRD_N = 1'b1;
assign WWR_N = 1'b1;
assign WRESET_N = 1'b1;
	
assign TAPE_OUT = 1'b0;

assign ESP32_SPI_CS_N = 1'b1;
assign MIDI_TX = 1'b1;

assign FLASH_CS_N = 1'b1;
assign FLASH_DI = 1'b1;
assign FLASH_SCK = 1'b0;
assign FLASH_WP_N = 1'b1;
assign FLASH_HOLD_N = 1'b1;

assign FT_SPI_CS_N = 1'b1;
assign FT_SPI_SCK = 1'b1;
assign FT_SPI_MOSI = 1'b1;
assign FT_CLK_OUT = 1'b0;

assign SD_CS_N = 1'b1;
assign SD_CLK = 1'b1;
assign SD_DI = 1'b1;

// alf wires
wire clk, clk_vga, clk_midi, areset;
wire [15:0] audio_out_l, audio_out_r;
wire [7:0] video_r, video_g, video_b;
wire video_hsync, video_vsync;

// hdmi pll
wire clk_hdmi, clk_hdmi_n;
wire lockedx5;
pllx5 pllx5(
	.CLK_IN1(clk_vga),
	.CLK_OUT1(clk_hdmi),
	.CLK_OUT2(clk_hdmi_n),
	.LOCKED(lockedx5)
);	

// ---------- ALF -------------
alf alf(
	.clk_sys(CLK_50MHZ),
	
	.clk(clk),
	.clk_vga(clk_vga),
	.clk_midi(clk_midi),
	.areset(areset),
	
	.mcu_mosi(MCU_MOSI),
	.mcu_miso(MCU_MISO),
	.mcu_sck(MCU_SCK),
	.mcu_cs_n(MCU_CS_N),
	
	.audio_out_l(audio_out_l),
	.audio_out_r(audio_out_r),
	.audio_beeper(),
	
	.ma(MA),
	.md(MD),
	.mrd_n(MRD_N),
	.mwr_n(MWR_N),
	
	.video_red(video_r),
	.video_green(video_g),
	.video_blue(video_b),
	.video_hsync(video_hsync),
	.video_vsync(video_vsync),
	.video_blank(video_blank)
);

//---------- DAC ------------

PCM5102 #(.DAC_CLK_DIV_BITS(3)) PCM5102(
	.clk(clk_vga),
	.reset(areset),
	.left(audio_out_l),
	.right(audio_out_r),
	.din(DAC_DAT),
	.bck(DAC_BCK),
	.lrck(DAC_WS)
);

// ------- PCM1808 ADC ---------
wire signed [23:0] adc_l, adc_r;

i2s_transceiver adc(
	.reset_n(~areset),
	.mclk(clk_vga),
	.sclk(ADC_BCK),
	.ws(ADC_LRCK),
	.sd_tx(),
	.sd_rx(ADC_DOUT),
	.l_data_tx(24'b0),
	.r_data_tx(24'b0),
	.l_data_rx(adc_l),
	.r_data_rx(adc_r)
);

// ------- ADC_CLK output buf
ODDR2 oddr_adc2(
	.Q(ADC_CLK),
	.C0(clk_vga),
	.C1(~clk_vga),
	.CE(1'b1),
	.D0(1'b1),
	.D1(1'b0),
	.R(1'b0),
	.S(1'b0)
);

// ------------ HDMI ----------------

wire [9:0] tmds_red, tmds_green, tmds_blue;

hdmi hdmi(
	.I_CLK_PIXEL(clk_vga),
	.I_R(video_r),
	.I_G(video_g),
	.I_B(video_b),
	.I_BLANK(video_blank),
	.I_HSYNC(video_hsync),
	.I_VSYNC(video_vsync),
	.I_AUDIO_ENABLE(1'b1),
	.I_AUDIO_PCM_L(audio_out_l[15:0]),
	.I_AUDIO_PCM_R(audio_out_r[15:0]),
	.O_RED(tmds_red),
	.O_GREEN(tmds_green),
	.O_BLUE(tmds_blue)
);

hdmi_out_xilinx hdmiio(
	.clock_pixel_i(clk_vga),
	.clock_tdms_i(clk_hdmi),
	.clock_tdms_n_i(clk_hdmi_n),
	.red_i(tmds_red),
	.green_i(tmds_green),
	.blue_i(tmds_blue),
	.tmds_out_p(TMDS_P),
	.tmds_out_n(TMDS_N)
);

endmodule
