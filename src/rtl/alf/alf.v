`timescale 1ns / 1ps
`default_nettype wire

module alf (
   input wire clk_sys,
	
	output wire clk,
	output wire clk_vga,
	output wire clk_midi,
	output wire areset,
	
	input wire mcu_mosi,
	output wire mcu_miso,
	input wire mcu_sck,
	input wire mcu_cs_n,
	
	output wire [15:0] audio_out_l,
	output wire [15:0] audio_out_r,
	
	output wire audio_beeper,
	
	output wire [20:0] ma,
	inout wire [15:0] md,
	output wire [1:0] mrd_n,
	output wire [1:0] mwr_n,
	
	output wire [7:0] video_red, 
	output wire [7:0] video_green, 
	output wire [7:0] video_blue,
	output wire video_hsync,
	output wire video_vsync,
	output wire video_blank
);

// clock
wire locked;

wire clk_ay;

clock clock(
   .CLK50(clk_sys), 
   .CLK(clk), // 56
   .VGA_CLK(clk_vga), // 25
	.MIDI_CLK(clk_midi), // 12
	// TODO: clk_ay
   .LOCKED(locked)
);

assign areset = ~locked;

//---------- MCU ------------

wire [7:0] hid_kb_status, hid_kb_dat0, hid_kb_dat1, hid_kb_dat2, hid_kb_dat3, hid_kb_dat4, hid_kb_dat5;
wire [12:0] joy_l, joy_r;
wire romloader_act, fileloader_act, fileloader_reset;
reg fileloader_end_n, prev_fileloader_act;
reg [20:0] cart_size;
wire [31:0] romloader_addr, fileloader_addr;
wire [7:0] romloader_data, fileloader_data;
wire romloader_wr, fileloader_wr;
wire [15:0] softsw_command, osd_command;
wire ft_vga_on;
wire mcu_busy;
wire [15:0] debug_data;

always @(posedge clk)
begin
	fileloader_end_n <= 1'b1;
	if (fileloader_act == 1'b0 && prev_fileloader_act == 1'b1)
		fileloader_end_n <= 1'b0;
		cart_size <= fileloader_addr+1;
	prev_fileloader_act <= fileloader_act;	
end

mcu mcu(
	.CLK(clk),
	.N_RESET(~areset),
	
	.MCU_MOSI(mcu_mosi),
	.MCU_MISO(mcu_miso),
	.MCU_SCK(mcu_sck),
	.MCU_SS(mcu_cs_n),
	
	.MCU_SPI_FT_SS(1'b1),
	.MCU_SPI_SD2_SS(1'b1),
	
	.KB_STATUS(hid_kb_status),
	.KB_DAT0(hid_kb_dat0),
	.KB_DAT1(hid_kb_dat1),
	.KB_DAT2(hid_kb_dat2),
	.KB_DAT3(hid_kb_dat3),
	.KB_DAT4(hid_kb_dat4),
	.KB_DAT5(hid_kb_dat5),
	
	.JOY_L(joy_l),
	.JOY_R(joy_r),
	
	.ROMLOADER_ACTIVE(romloader_act),
	.ROMLOAD_ADDR(romloader_addr),
	.ROMLOAD_DATA(romloader_data),
	.ROMLOAD_WR(romloader_wr),
	
	.FILELOAD_ACTIVE(fileloader_act),
	.FILELOAD_RESET(fileloader_reset),
	.FILELOAD_ADDR(fileloader_addr),
	.FILELOAD_DATA(fileloader_data),
	.FILELOAD_WR(fileloader_wr),
	
	.SOFTSW_COMMAND(softsw_command),	
	.OSD_COMMAND(osd_command),
	
	.BUSY(mcu_busy),
	
	.FT_VGA_ON(ft_vga_on),
	
	.FT_CS_N(),
	.FT_MOSI(),
	.FT_MISO(),
	.FT_SCK(),
	
	.SD2_CS_N(),
	.SD2_MOSI(),
	.SD2_MISO(),
	.SD2_SCK(),
	
	.DEBUG_ADDR(16'h00),
	.DEBUG_DATA(debug_data)
);

//---------- HID Keyboard/Joy parser ------------

wire [7:0] alf_joy_l, alf_joy_r;
wire alf_restart;

hid_parser hid_parser(
	.CLK(clk),
	.RESET(areset),

	.KB_STATUS(hid_kb_status),
	.KB_DAT0(hid_kb_dat0),
	.KB_DAT1(hid_kb_dat1),
	.KB_DAT2(hid_kb_dat2),
	.KB_DAT3(hid_kb_dat3),
	.KB_DAT4(hid_kb_dat4),
	.KB_DAT5(hid_kb_dat5),
	
	.JOY_L(joy_l),
	.JOY_R(joy_r),
	
	.JOY_L_DO(alf_joy_l),
	.JOY_R_DO(alf_joy_r),
	.JOY_START(alf_restart)
);

//---------- Soft switches ------------

wire kb_reset;
wire btn_reset_n;

soft_switches soft_switches(
	.CLK(clk),
	.SOFTSW_COMMAND(softsw_command),
	.RESET(kb_reset)
);

assign btn_reset_n = ~kb_reset & ~mcu_busy & fileloader_end_n & ~alf_restart;

//--------- OSD --------------

wire [7:0] video_r, video_g, video_b, osd_r, osd_g, osd_b;

overlay overlay(
	.CLK(clk),
	.CLK_VGA(clk_vga),
	.RGB_I({video_r[7:0], video_g[7:0], video_b[7:0]}),
	.RGB_O({osd_r[7:0], osd_g[7:0], osd_b[7:0]}),
	.HSYNC_I(video_hsync),
	.VSYNC_I(video_vsync),
	.OSD_COMMAND(osd_command)
);

//--------- ALF --------------
reg [3:0] tick;
reg cpu_clk;
wire cpu_int, cpu_mreq, cpu_iorq, cpu_rd, cpu_wr, cpu_m1;
wire [15:0] cpu_a;
wire [7:0] cpu_di, cpu_do;
wire [20:0] rom_a, ram_a;
wire [7:0] rom_do, ram_do, ay_do;
wire [13:0] vram_a;
wire vram_wr;
wire [12:0] vram_va;
wire [7:0] vram_vd;
reg [2:0] bordercolor;
wire rom_cs, ram_cs, ay_cs;
reg [7:0] rom_reg;
reg [7:0] port_7ffd;
reg beeper;
reg [7:0] ay_a, ay_b, ay_c;

video video(
    .VGA_CLK(clk_vga),
    .RESET(areset),
    .BORDERCOLOR(bordercolor),
    .INT(cpu_int),
    .VA(vram_va),
    .VD(vram_vd),
    .VGA_R(video_r),
    .VGA_G(video_g),
    .VGA_B(video_b),
    .VGA_HSYNC(video_hsync),
    .VGA_VSYNC(video_vsync),
	 .VGA_BLANK(video_blank)
);

T80se cpu(
    .RESET_n(btn_reset_n),
    .CLK_n(clk),
    .CLKEN(cpu_clk),
    .WAIT_n(1'b1),
    .INT_n(cpu_int),
    .NMI_n(1'b1),
    .BUSRQ_n(1'b1),
    .M1_n(cpu_m1),
    .MREQ_n(cpu_mreq),
    .IORQ_n(cpu_iorq),
    .RD_n(cpu_rd),
    .WR_n(cpu_wr),
    .RFSH_n(),
    .HALT_n(),
    .BUSAK_n(),
    .A(cpu_a),
    .DI(cpu_di),
    .DO(cpu_do)
);

vram vram(
   .clka(clk),
   .wea(vram_wr),
   .addra(vram_a),
   .dina(cpu_do),
   .douta(),
   .clkb(clk_vga),
   .web(1'b0),
   .addrb({port_7ffd[3], vram_va}),
   .dinb(8'b0),
   .doutb(vram_vd)
);

ay8910 ay38910(
   .CLK(clk),
   .CLC(clk_ay),
   .RESET(btn_reset_n),
   .BDIR(~cpu_wr),
   .CS(ay_cs),
   .BC(cpu_a[14]),
   .DI(cpu_do),
   .DO(ay_do),
   .OUT_A(ay_a),
   .OUT_B(ay_b),
   .OUT_C(ay_c)
); 

always @(posedge clk) 
begin
	if (btn_reset_n == 1'b0)
	begin
		tick <= 4'b0;
		cpu_clk <= 1'b0;
	end
   else
	begin
		cpu_clk <= 1'b0;
		tick <= tick + 1;
		if (tick == 4'b1111) // 56 div 16 = 3.5
		begin
			cpu_clk <= 1'b1;
		end
	end
end

reg tick_ay;
always @(posedge clk)
begin
	if (cpu_clk)
		tick_ay <= ~tick_ay; 
end

assign clk_ay = cpu_clk && tick_ay; // 1.75 MHz enable signal

assign rom_cs = ~((cpu_a[15:14] == 2'b00) && ~cpu_mreq);
assign ram_cs = ~((cpu_a[15:14] != 2'b00) && ~cpu_mreq);

assign cpu_di = (~rom_cs) ? rom_do : // rom
				 (~ram_cs) ? ram_do : // ram
				 (~ay_cs) ? ay_do : // ay3-8912 #fffd
				 ((cpu_a == 16'h7ffd) && ~cpu_rd & ~cpu_iorq) ? port_7ffd : // #7ffd read
				 (~cpu_a[5] && ~cpu_rd && ~cpu_iorq) ? alf_joy_l :  // #1F (kempston) 101 + D[4:0]=FUDLR
				 (~cpu_a[0] && ~cpu_rd && ~cpu_iorq) ? alf_joy_r :  // #FE (keyb) 000 + D[4:0]=!(LURDF)
				 8'b11111111;

assign vram_wr = (((cpu_a[15:13] == 3'b010) || ((cpu_a[15:13] == 3'b110) && port_7ffd[2] && port_7ffd[0])) && ~cpu_mreq && ~cpu_wr);
assign vram_a = (cpu_a[15:13] == 3'b010) ? {1'b0, cpu_a[12:0]} : {port_7ffd[1], cpu_a[12:0]};
assign rom_a = (rom_reg[7]) ? {1'b1, rom_reg[5:0], cpu_a[13:0]} : {2'b01, rom_reg[4:0], cpu_a[13:0]}; // ext / int rom mux
assign ram_a = (cpu_a[15:14] == 2'b11) ? {4'b0000, port_7ffd[2:0], cpu_a[13:0]} : {4'b0000, cpu_a[14], cpu_a}; // ram

// using one sram chip for embedded and external rom
// internal rom: MA[20:19] = 01 (512KB)
// external rom: MA[20:19] = 1x (1.0MB)
// internal ram: MA[20:19] = 00 (512KB)
assign ma = (romloader_act) ? {2'b01, romloader_addr[18:0]} : // rom loader address to write rom from mcu
				((fileloader_act) ? {1'b1, fileloader_addr[19:0]} : // file loader address to wtite rom cart from mcu
				((~ram_cs) ? ram_a : // ram 128k access
				rom_a)); // rom (int/ext) access

assign mwr_n = (romloader_act) ? {1'b1, ~romloader_wr} : // rom loader write
					((fileloader_act) ? {1'b1, ~fileloader_wr} :  // file loader write
					((~cpu_mreq && ~cpu_wr && ~ram_cs) ? 2'b10 : 2'b11)); // ram write

assign mrd_n = (~cpu_mreq && ~cpu_rd) ? 2'b10 : 2'b11; // ram/rom read

assign md = (romloader_act) ? ((romloader_wr) ? {8'bz, romloader_data} : 16'bz) : // rom loader data to write
				((fileloader_act) ? ((fileloader_wr) ? {8'bz, fileloader_data} : 16'bz) :  // file loader data to write
				((~cpu_mreq && ~cpu_wr) ? {8'bz, cpu_do} : 16'bz)); // ram/rom to write

assign rom_do = md[7:0]; // todo: split ram_do/rom_do into latched reg
assign ram_do = md[7:0];

assign audio_out_l = {2'b0, beeper, 12'b0} + {3'b0, ay_a, 5'b0} + {4'b0, ay_b, 4'b0};
assign audio_out_r = {2'b0, beeper, 12'b0} + {3'b0, ay_c, 5'b0} + {4'b0, ay_b, 4'b0};
assign audio_beeper = beeper;

assign video_red = osd_r;
assign video_green = osd_g;
assign video_blue = osd_b;

assign ay_cs = ~(cpu_a[15] && cpu_a[13] && ~cpu_a[1] && cpu_m1 && ~cpu_iorq);

// ports
always @(posedge clk)
begin
	if (~btn_reset_n)
	begin
		rom_reg <= 8'b0;
		port_7ffd <= 8'b0;
	end
	else
	begin		
		if (~cpu_iorq && ~cpu_wr)
		begin
			if (~cpu_a[15] && ~cpu_a[1]) // #7ffd
			begin
				if (~port_7ffd[5]) // if not locked
					port_7ffd <= cpu_do; // 5 - locked, 4 - romsel, 3 - screen, 2:0 - page
			end
			else if (~cpu_a[5]) // port #5F - rom banks
				rom_reg <= cpu_do;

			else if (~cpu_a[0]) // port #FE - border + beeper
			begin
				beeper <= cpu_do[4];
				bordercolor <= cpu_do[2:0];
			end
		end	
	end
end

endmodule
