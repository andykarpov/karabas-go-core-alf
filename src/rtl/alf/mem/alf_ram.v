`timescale 1ns / 1ps
`default_nettype none

// The memories of the Alf as inferred block RAM (they were coregen cores whose netlists were not in git):
// ram, 16K x 8, one port; vram, 16K x 8, port a (the CPU) read / write, port b (the video, another clock)
// read. Both read the address of the clock before, write first, as the Block Memory Generator made them.

module ram(
	input wire clka,
	input wire [0:0] wea,
	input wire [13:0] addra,
	input wire [7:0] dina,
	output reg [7:0] douta
);
reg [7:0] mem [0:16383];
always @(posedge clka) begin
	if (wea[0]) begin
		mem[addra] <= dina;
		douta <= dina;
	end
	else
		douta <= mem[addra];
end
endmodule

module vram(
	input wire clka,
	input wire [0:0] wea,
	input wire [13:0] addra,
	input wire [7:0] dina,
	output reg [7:0] douta,
	input wire clkb,
	input wire [0:0] web,
	input wire [13:0] addrb,
	input wire [7:0] dinb,
	output reg [7:0] doutb
);
reg [7:0] mem [0:16383];
always @(posedge clka) begin
	if (wea[0]) begin
		mem[addra] <= dina;
		douta <= dina;
	end
	else
		douta <= mem[addra];
end
// port b only reads here (web = 0 in alf.v)
always @(posedge clkb)
	doutb <= mem[addrb];
endmodule

`default_nettype wire
