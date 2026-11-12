`timescale 1ns / 1ps
`default_nettype wire

module alf #(
	parameter BURST_MAX_MHZ = 0, // see mcu.vhd
	// the MCU link: 1 = v1 (mcu.vhd, soft_switches, overlay), 2 = v2 (kg_mcu2 and kg_l2_osd_overlay of
	// karabas-go-modules; mcu_burst_ready is then the interrupt to the mcu, on the same IO1 line)
	parameter LINK = 1
)(
   input wire clk_sys,
	
	output wire clk,
	output wire clk_vga,
	output wire clk_midi,
	output wire clk_hdmi, // 5 x clk_vga for the HDMI serializer (Mini / Mini G), from the same PLL
	output wire clk_hdmi_n, // the same, 180 degrees
	output wire areset,
	
	input wire mcu_mosi,
	output wire mcu_miso,
	input wire mcu_sck,
	input wire mcu_cs_n,
	input wire mcu_burst_cs_n,  // burst transfers (karabas-go-firmware docs/burst_protocol.md)
	output wire mcu_burst_ready,  // LINK 2: MCU_INT
	
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
	output wire video_blank,
	output wire dvi_only
);

// clock
wire locked;

// one PLL (karabas-go-modules kg_pll) for all clocks: VCO 1000 MHz; clk 55.56 MHz (CPU 3.97 / 3.47 MHz),
// VGA 25.0 MHz, MIDI 12.05 MHz, HDMI 125 MHz and 125 MHz 180 degrees. The HDMI clocks used to come from a
// DCM_SP (ipcore pllx5) without reset fed by the PLL output, which could start unlocked: no HDMI picture.
wire [5:0] pll_clk;
kg_pll #(
	.CLKIN_PERIOD(20.0), .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT(20),
	.CLKOUT0_DIVIDE(18), .CLKOUT1_DIVIDE(40), .CLKOUT2_DIVIDE(83), .CLKOUT3_DIVIDE(8), .CLKOUT4_DIVIDE(8),
	.CLKOUT4_PHASE(180.0), .IBUFG(1), .BUFG_MASK(6'b011111)
) clock (
	.clk_in(clk_sys), .rst(1'b0), .clk_out(pll_clk), .locked(locked)
);
assign clk = pll_clk[0];
assign clk_vga = pll_clk[1];
assign clk_midi = pll_clk[2];
assign clk_hdmi = pll_clk[3];
assign clk_hdmi_n = pll_clk[4];

assign areset = ~locked;

//---------- MCU ------------

wire [7:0] hid_kb_status, hid_kb_dat0, hid_kb_dat1, hid_kb_dat2, hid_kb_dat3, hid_kb_dat4, hid_kb_dat5;
wire [12:0] joy_l, joy_r;
wire romloader_act, fileloader_act, fileloader_reset;
reg fileloader_end_n, prev_fileloader_act;
reg [20:0] cart_size;
reg cart_ok = 1'b0; // a cartridge was loaded: before that its area reads FF
wire [31:0] romloader_addr, fileloader_addr;
wire [7:0] romloader_data, fileloader_data;
wire romloader_wr, fileloader_wr;
wire [15:0] softsw_command, osd_command;
wire kb_reset;
wire model_sw; // OSD switch 1: 0 = Alf, 1 = Alf-128
wire ay_sw; // OSD switch 2: the AY on (either model)
reg model = 1'b0;
reg model_prev = 1'b0;
reg [3:0] model_rst = 4'd0;
wire btn_reset_n;
// link v2: the OSD of kg_mcu2 to kg_l2_osd_overlay
wire osd_wr, osd_en, osd_zoom;
wire [11:0] osd_waddr;
wire [7:0] osd_wdata;
wire [23:0] osd_fg, osd_bg;
wire [1:0] osd_alpha, osd_place;
wire [4:0] osd_px, osd_ph;
wire [3:0] osd_py;
wire [5:0] osd_pw;
wire ft_vga_on;
wire mcu_busy;
wire [15:0] debug_data;

always @(posedge clk)
begin
	fileloader_end_n <= 1'b1;
	if (fileloader_act == 1'b0 && prev_fileloader_act == 1'b1)
	begin
		fileloader_end_n <= 1'b0;
		cart_ok <= 1'b1;
	end
	cart_size <= fileloader_addr+1;
	prev_fileloader_act <= fileloader_act;	
end

generate if (LINK == 1) begin : link1
mcu #(.BURST_MAX_MHZ(BURST_MAX_MHZ)) mcu(
	.CLK(clk),
	.N_RESET(~areset),
	
	.MCU_MOSI(mcu_mosi),
	.MCU_MISO(mcu_miso),
	.MCU_SCK(mcu_sck),
	.MCU_SS(mcu_cs_n),
	
	.MCU_SPI_FT_SS(1'b1),
	.MCU_SPI_SD2_SS(1'b1),
	.MCU_BURST_CS_N(mcu_burst_cs_n),
	.MCU_BURST_READY(mcu_burst_ready),
	
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
	
	.HWID(),
	.DVI_ONLY(dvi_only),
	
	.DEBUG_ADDR(16'h00),
	.DEBUG_DATA(debug_data)
);
end else begin : link2
	// MCU link v2 (karabas-go-modules kg_mcu2, karabas-go-firmware docs/link_v2.md): the roms of the core file
	// (LD kind 0) and the cartridge (the FL item of the OSD, kind 2; kind 1 taken too), the switches Reset
	// and Model, the OSD of link v2
	wire [55:0] kbd;
	wire [15:0] joy_l16, joy_r16;
	wire [1:0] ld_kind;
	wire ld_start, ld_active, ld_wr;
	wire [31:0] ld_addr;
	wire [7:0] ld_data;
	wire [23:0] sw;
	kg_mcu2 #(
		.CAPS(16'h0000), .SW_N(3), .SW_INIT(24'h010000), .CS_FILTER(3), .ENABLE_OSD(1), .ENABLE_IMG(0), .ENABLE_PORTS(4'b0000),
		.ENABLE_RTC(0), .ENABLE_AUX(0), .ENABLE_FT(0), .ENABLE_SD2(0)
	) mcu(
		.CLK(clk), .N_RESET(~areset),
		.MCU_SCK(mcu_sck), .MCU_MOSI(mcu_mosi), .MCU_MISO(mcu_miso), .MCU_SS(mcu_cs_n), .MCU_INT(mcu_burst_ready),
		.MCU_SPI_FT_SS(1'b1), .MCU_SPI_SD2_SS(1'b1),
		.SW(sw), .SW_WR(), .SW_INDEX(), .SW_VALUE(), .HW_ID(), .DVI_ONLY(dvi_only), .CORE_BUILD(), .FW_BUILD(),
		.BUSY(mcu_busy), .BTNS(), .RELOAD_REQ(1'b0), .RELOAD_CORE(8'd0),
		.KBD(kbd), .KBD_WR(), .PS2_CODE(), .PS2_KIND(), .PS2_WR(),
		.MS_X(), .MS_Y(), .MS_Z(), .MS_B(), .MS_WR(), .JOY_L(joy_l16), .JOY_R(joy_r16), .JOY_WR(),
		.PS2_CMD(16'h0000), .PS2_CMD_WR(1'b0),
		.LD_KIND(ld_kind), .LD_SLOT(), .LD_SIZE(), .LD_START(ld_start), .LD_ACTIVE(ld_active), .LD_END(),
		.LD_ADDR(ld_addr), .LD_DATA(ld_data), .LD_WR(ld_wr), .LD_READY(1'b1),
		.OSD_WR(osd_wr), .OSD_WADDR(osd_waddr), .OSD_WDATA(osd_wdata), .OSD_EN(osd_en), .OSD_FG(osd_fg),
		.OSD_BG(osd_bg), .OSD_ALPHA(osd_alpha), .OSD_PX(osd_px), .OSD_PY(osd_py), .OSD_PW(osd_pw), .OSD_PH(osd_ph),
		.OSD_PLACE(osd_place), .OSD_ZOOM(osd_zoom),
		.SDC_IMG_MOUNTED(), .SDC_IMG_SIZE(), .SDC_RD(8'd0), .SDC_WR(8'd0), .SDC_SECTOR(32'd0), .SDC_BUSY(),
		.SDC_DONE(), .SDC_ERROR(), .SDC_SLOT(), .SDC_BYTE_IN_STROBE(), .SDC_BYTE_ADDR(), .SDC_BYTE_IN_DATA(),
		.SDC_BYTE_OUT_DATA(8'd0), .IMG_WP(), .IMG_PHYS_WR(), .IMG_PHYS(), .IMG_FLUSH(), .FDD_CHNG_WR(), .FDD_CHNG(),
		.FDD_CHNG_REQ(1'b0), .FDD_CHNG_DS(2'd0), .IMG_ACTIVE(),
		.PORT_OUT_DATA(), .PORT_OUT_WR(), .PORT_OUT_READY(4'b1111), .PORT_IN_DATA(32'd0), .PORT_IN_WR(4'd0),
		.PORT_CFG(128'd0), .PORT_IN_FULL(), .PORT_STAT(), .PORT_STAT_WR(),
		.RTC_A(8'd0), .RTC_DI(8'd0), .RTC_CS(1'b0), .RTC_WR_N(1'b1), .MATRIX_CMD(24'd0), .MATRIX_CMD_WR(1'b0),
		.DEBUG_ADDR(16'd0), .DEBUG_DATA(16'd0), .RTC_DO(),
		.FT_SPI_ON(), .FT_VGA_ON(), .FT_RESET(), .FT_SCK(), .FT_MOSI(), .FT_MISO(1'b1), .FT_CS_N(),
		.SD2_SCK(), .SD2_MOSI(), .SD2_MISO(1'b1), .SD2_CS_N()
	);
	assign { hid_kb_dat5, hid_kb_dat4, hid_kb_dat3, hid_kb_dat2, hid_kb_dat1, hid_kb_dat0, hid_kb_status } = kbd;
	assign joy_l = joy_l16[12:0];
	assign joy_r = joy_r16[12:0];
	assign romloader_act = ld_active && ld_kind == 2'd0;
	assign romloader_wr = ld_wr && ld_kind == 2'd0;
	assign romloader_addr = ld_addr;
	assign romloader_data = ld_data;
	assign fileloader_act = ld_active && ld_kind != 2'd0;
	assign fileloader_reset = ld_start && ld_kind != 2'd0;
	assign fileloader_wr = ld_wr && ld_kind != 2'd0;
	assign fileloader_addr = ld_addr;
	assign fileloader_data = ld_data;
	assign kb_reset = sw[0];
	assign model_sw = sw[8];
	assign ay_sw = sw[16];
	assign ft_vga_on = 1'b0;
end endgenerate

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

generate if (LINK == 1) begin : sw1
//---------- Soft switches ------------


soft_switches soft_switches(
	.CLK(clk),
	.SOFTSW_COMMAND(softsw_command),
	.RESET(kb_reset),
	.MODEL(model_sw),
	.AY(ay_sw)
);
end endgenerate

// the model of the OSD; a change holds the machine in reset for 16 clocks, the model taken in it
always @(posedge clk)
begin
	model_prev <= model_sw;
	if (model_prev != model_sw)
		model_rst <= 4'd15;
	else if (model_rst != 4'd0)
		model_rst <= model_rst - 1'd1;
	if (model_rst == 4'd8)
		model <= model_sw;
end

assign btn_reset_n = ~kb_reset & ~mcu_busy & fileloader_end_n & ~alf_restart & (model_rst == 4'd0);

//--------- OSD --------------

wire [7:0] video_r, video_g, video_b, osd_r, osd_g, osd_b;
wire vid_hsync, vid_vsync, vid_blank;

generate if (LINK == 1) begin : osd1
overlay overlay(
	.CLK(clk),
	.CLK_VGA(clk_vga),
	.RGB_I({video_r[7:0], video_g[7:0], video_b[7:0]}),
	.RGB_O({osd_r[7:0], osd_g[7:0], osd_b[7:0]}),
	.HSYNC_I(vid_hsync),
	.VSYNC_I(vid_vsync),
	.OSD_COMMAND(osd_command)
);
	assign video_hsync = vid_hsync;
	assign video_vsync = vid_vsync;
	assign video_blank = vid_blank;
end else begin : osd2
	// the OSD of link v2 on the VGA clock; the syncs go out with its picture (2 clocks later)
	wire hs_o, vs_o, de_o;
	kg_l2_osd_overlay overlay(
		.wclk(clk), .osd_wr(osd_wr), .osd_waddr(osd_waddr), .osd_wdata(osd_wdata), .osd_en(osd_en),
		.osd_fg(osd_fg), .osd_bg(osd_bg), .osd_alpha(osd_alpha), .osd_px(osd_px), .osd_py(osd_py), .osd_pw(osd_pw),
		.osd_ph(osd_ph), .osd_place(osd_place), .osd_zoom(osd_zoom), .icon_on(8'd0), .icon_act(8'd0),
		.clk(clk_vga), .rgb_i({video_r, video_g, video_b}), .hs_i(vid_hsync), .vs_i(vid_vsync), .de_i(~vid_blank),
		.rgb_o({osd_r, osd_g, osd_b}), .hs_o(hs_o), .vs_o(vs_o), .de_o(de_o)
	);
	assign video_hsync = hs_o;
	assign video_vsync = vs_o;
	assign video_blank = ~de_o;
end endgenerate

//--------- ALF --------------
// One core for both machines, the model of the OSD (switch 1): 0 = Alf (48K, the CPU at 4.0 MHz, no #7FFD),
// 1 = Alf-128 (128K RAM paged by #7FFD, the CPU at 3.5 MHz). The AY-3-8912 at #FFFD / #BFFD is switch 2,
// for either model (on by default). The RAM of both is in the SRAM (MA[20:19] = 00), the screen (#4000..#5FFF,
// and in the 128K pages 5 / 7 at #C000) is copied into the vram for the video. A change of the model resets.
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
wire [7:0] ay_a, ay_b, ay_c;
wire clk_ay;
wire m128 = model;
wire ay_on = ay_sw;

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
    .VGA_HSYNC(vid_hsync),
    .VGA_VSYNC(vid_vsync),
	 .VGA_BLANK(vid_blank)
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

// the CPU clock enable: 55.56 MHz / 14 = 3.97 MHz (Alf), / 16 = 3.47 MHz (Alf-128)
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
		if (tick == (m128 ? 4'd15 : 4'd13))
		begin
			tick <= 4'b0;
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
assign ay_cs = ~(ay_on && cpu_a[15] && cpu_a[13] && ~cpu_a[1] && cpu_m1 && ~cpu_iorq);

// rom area: embedded rom (rom_reg[7] = 0) or the cartridge (rom_reg[7] = 1), always readable.
// Bank bits above the rom size select a mirror, as on real roms where those address lines are not
// connected.
assign cpu_di = (~rom_cs && rom_reg[7] && ~cart_ok) ? 8'hFF : // no cartridge yet
				 (~rom_cs) ? rom_do : // embedded / cartridge rom
				 (~ram_cs) ? ram_do : // ram
				 (~ay_cs) ? ay_do : // ay3-8912 #fffd (128)
				 (m128 && (cpu_a == 16'h7ffd) && ~cpu_rd && ~cpu_iorq) ? port_7ffd : // #7ffd read (128)
				 (~cpu_a[5] && ~cpu_rd && ~cpu_iorq) ? alf_joy_l :  // #1F (kempston) 101 + D[4:0]=FUDLR
				 (~cpu_a[0] && ~cpu_rd && ~cpu_iorq) ? alf_joy_r :  // #FE (keyb) 000 + D[4:0]=!(LURDF)
				 8'b11111111;

// cartridge bank, masked to the cartridge size (smaller cartridges are mirrored)
wire [5:0] cart_bank = (cart_size == 1048576) ? rom_reg[5:0] :
					(cart_size == 524288) ? {1'b0, rom_reg[4:0]} :
					(cart_size == 262144) ? {2'b00, rom_reg[3:0]} :
					(cart_size == 131072) ? {3'b000, rom_reg[2:0]} :
					(cart_size == 65536) ? {4'b0000, rom_reg[1:0]} :
					(cart_size == 32768) ? {5'b00000, rom_reg[0]} :
					(cart_size == 16384) ? 6'b000000 :
					rom_reg[5:0]; // other sizes: no mirroring
// the embedded rom (32 KB, two banks) is mirrored and does not depend on the cartridge
assign rom_a = (rom_reg[7]) ? {1'b1, cart_bank, cpu_a[13:0]} : {2'b01, 4'b0000, rom_reg[0], cpu_a[13:0]};
// ram: #4000 = page 5, #8000 = page 2, #C000 = the page of #7FFD (always 0 on the 48K: 5, 2, 0)
assign ram_a = (cpu_a[15:14] == 2'b11) ? {4'b0000, port_7ffd[2:0], cpu_a[13:0]} : {4'b0000, cpu_a[14], cpu_a};
// the screen copy: #4000..#5FFF (page 5, screen 0) and pages 5 / 7 at #C000 (screen 0 / 1)
assign vram_wr = (((cpu_a[15:13] == 3'b010) || ((cpu_a[15:13] == 3'b110) && port_7ffd[2] && port_7ffd[0])) && ~cpu_mreq && ~cpu_wr);
assign vram_a = (cpu_a[15:13] == 3'b010) ? {1'b0, cpu_a[12:0]} : {port_7ffd[1], cpu_a[12:0]};

// one sram chip for the ram, the embedded and the external rom
// internal ram: MA[20:19] = 00 (512KB)
// internal rom: MA[20:19] = 01 (512KB)
// external rom: MA[20] = 1 (1MB)
assign ma = (romloader_act) ? {2'b01, romloader_addr[18:0]} :
				((fileloader_act) ? {1'b1, fileloader_addr[19:0]} :
				((~ram_cs) ? ram_a : rom_a));
assign mwr_n = (romloader_act) ? {1'b1, ~romloader_wr} :
					((fileloader_act) ? {1'b1, ~fileloader_wr} :
					((~cpu_mreq && ~cpu_wr && ~ram_cs) ? 2'b10 : 2'b11));
assign mrd_n = 2'b10;
assign md = (romloader_act) ? ((romloader_wr) ? {8'bz, romloader_data} : 16'bz) :
				((fileloader_act) ? ((fileloader_wr) ? {8'bz, fileloader_data} : 16'bz) :
				((~cpu_mreq && ~cpu_wr) ? {8'bz, cpu_do} : 16'bz));
assign rom_do = md[7:0];
assign ram_do = md[7:0];

// without the AY the louder beeper of the old core, with it the AY mixed in (A + B left, C + B right)
assign audio_out_l = ay_on ? ({2'b0, beeper, 12'b0} + {3'b0, ay_a, 5'b0} + {4'b0, ay_b, 4'b0}) : {1'b0, beeper, 13'b0};
assign audio_out_r = ay_on ? ({2'b0, beeper, 12'b0} + {3'b0, ay_c, 5'b0} + {4'b0, ay_b, 4'b0}) : {1'b0, beeper, 13'b0};
assign audio_beeper = beeper;

assign video_red = osd_r;
assign video_green = osd_g;
assign video_blue = osd_b;

// ports
always @(posedge clk)
begin
	if (btn_reset_n == 1'b0)
	begin
		rom_reg <= 8'b0;
		port_7ffd <= 8'b0;
	end
	else if (~cpu_iorq && ~cpu_wr)
	begin
		if (m128)
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
		else
		begin
			if (~cpu_a[5]) // port #5F - rom banks
				rom_reg <= cpu_do;
			if (~cpu_a[0]) // port #FE - border + beeper
			begin
				beeper <= cpu_do[4];
				bordercolor <= cpu_do[2:0];
			end
		end
	end
end

endmodule
