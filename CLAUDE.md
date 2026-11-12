# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

The Alf ("Эльф") core for the Karabas Go, Mini and Mini G boards: Xilinx Spartan-6 XC6SLX25 (ftg256), ISE 14.7 (XST). Alf is a Z80 cartridge game console derived from the ZX Spectrum (https://zxbyte.ru/alf.htm). The port is based on the Aeon-Lite project sources; license GPL v3 (`LICENSE`). This is branch `master` of `andykarpov/karabas-go-core-alf`; the branch `alf-128` (cloned locally as `karabas-go-core-alf128`) is the old separate Alf-128 core, kept as an archive since the merge into this core (link v2).

The Mini G build uses MCU link v2 (`kg_mcu2`, `kg_l2_osd_overlay` of the `modules` submodule, karabas-go-modules, branch `link-v2`; karabas-go-firmware `docs/link_v2.md`): `alf #(.LINK(2))` in its top. Go and Mini still use link v1, the own MCU link and OSD modules in `src/rtl` (`LINK` 1). The core is one machine for both models: Alf and Alf-128 (the former `alf-128` branch) are chosen in the OSD.

## Layout

- `src/rtl/karabas_go_top.v` (VGA; the FT812 pins are held inactive), `karabas_mini_top.v`, `karabas_minig_top.v` (HDMI, `dvi_only` from the MCU): the board tops. All instantiate `alf` (`src/rtl/alf/alf.v`); only the Mini G top sets `BURST_MAX_MHZ(32)`.
- `src/rtl/alf/alf.v`: the machine. T80 CPU (`alf/cpu`), video (`alf/video/video.vhd`), the screen copy `vram` (`alf/mem/alf_ram.v`, inferred; Go / Mini xise still use the coregen `alf/vram`), AY (`alf/ay8910/ay8910.vhd`), clocks (`kg_pll` of modules, VCO 1000 MHz: clk 55.56 MHz, VGA 25 MHz, MIDI 12.05 MHz, HDMI 125 MHz and 125 MHz 180 degrees; `alf/clock` and `ipcore_dir/pllx5` are left for the Go / Mini xise only), the MCU link (`generate` on `LINK`), `hid_parser`, `soft_switches` (link 1), the overlay.
- `src/rtl/mcu`: `mcu.vhd` (own copy of the SPI link), `burst_rx.vhd`, `hid_parser.vhd` (keyboard / joysticks to the Alf joypad ports), `soft_switches.vhd`.
- `src/rtl/osd`, `hdmi`, `dac` (`pcm5102.v`), `spi`, `vga`, `adc`: board helpers. `src/ipcore_dir`: ISE coregen cores (many are unused leftovers from other cores; the `.prj` files list what is built).
- `src/rom/alf.rom`: the 32 KB system ROM (start menu and BASIC-48) with the originality check removed; `alf_orig.rom` / `alf_orig_big.rom` are the originals (`src/rom/readme.txt`, Russian).
- `roms/`: Alf cartridge images for the SD card (`/alf/`).

## Build

- Mini G: `cd src && make minig` (`src/Makefile`: `karabas_minig.prj` / `.xst` / `.ut`, map `-t 2`) writes `src/karabas_minig.bin` and packs `alf.kg`. Go / Mini: open `src/karabas-go.xise` or `karabas-mini.xise` in ISE 14.7 and run Generate Programming File (`src/karabas_<board>_top.bin`); they build `alf.v` with `LINK` 1, but their file lists predate the merge (no `ay8910.vhd`, the old `ram` cores): to be fixed in the Go / Mini pass of link v2, their `.bin` in git are the old separate Alf. ISE intermediates stay in `src/` (gitignored; the Mini G `.prj` / `.xst` / `.ut` are committed).
- Pack with `make_core.py` (a symlink to `../../karabas-go-tools/make_core/make_core.py`, gitignored) from `src/core.json` (one file for all boards: the common keys, `boards` for what differs; kgo2, karabas-go-tools `docs/kgo2.md`) into `alf.kg` (`make_core.py core.json alf.kg`, or `--boards <board>` for one board's bitstream). The `.bin` and `.kg` files are committed as the release. Bump `build` in the json for a new release.
- Mini G HDMI: `zhdmi_tx` of modules (the old `rtl/hdmi/hdmi.vhd` had its own 720x480 counters), audio 48 kHz, DVI with dvi_only; DAC `pcm5102.v` of modules; FT_CLK_OUT = clk / 7 = 7.9 MHz (karabas-go-notes ft-clk-out-required).
- Each board has its own `.ucf` (`karabas_go.ucf`, `karabas_mini.ucf`, `karabas_minig.ucf`); Mini G: PERIOD on CLK_50MHZ (the PLL clocks derived), TIG between clk and clk_vga (dual clock vram, OSD RAM, quasi static registers) and the MCU SCK; Go / Mini keep the old `TS_CLK` 64 MHz.

## MCU link and features

- Core type `osd`, `eeprom_bank` 255 (none). OSD items (switch index = position): 0 Reset (F12), 1 Model (Alf / Alf-128, Menu+M; a change resets the machine), 2 AY (Off / On, default On, Menu+A), 3 FL Cart (slot 0, `/alf`, `.rom,.bin`, autoload: the last cartridge loads at the core start, Menu+F).
- SRAM map: RAM `MA[20:19]` = 00 (page 5 at #4000, 2 at #8000, the #7FFD page at #C000; on the Alf the page bits stay 0), the system ROM 01 (the rom loader, LD kind 0), the cartridge `MA[20]` = 1 (the FL item, LD kind 2). Until a cartridge is loaded its area reads FF. The end of a cartridge load resets the CPU (the start menu then runs it), so does the gamepad Start (`alf_restart`).
- Alf: CPU 3.97 MHz (55.56 / 14), ports `#5F` / `#FE` decoded as on the old core. Alf-128: CPU 3.47 MHz (55.56 / 16), `#7FFD` (pages, screen 0 / 1, lock; readable). The AY (`#FFFD` / `#BFFD`, 1.75 MHz) on either model when switched on; without it the beeper is louder.
- `cart_size` is taken from the last file loader address; cartridge banks above the size are mirrored (power-of-two sizes 16 KB..1 MB), and the embedded ROM is mirrored too.
- Video (`alf/video/video.vhd`, 25 MHz): VESA 640x480@60 since 2026-11-12 (line 800, borders 64 / 64 and 48 / 48 around the 512x384 paper; it was a 784-clock line with 80 / 48 borders that HDMI captures did not take). The line counter steps at the start of the hsync (the visible line runs from the left border at the end of the count over 0 into the right border; stepping at 0 cut the vertical blank and the vsync into a line: DE runs of 64 / 576 pixels, no picture on HDMI, the bug of the released v1 too). The INT runs on its own line counter, every 625 lines = 20.0 ms (50 Hz), not tied to the frame.
- Ports: `#5F` ROM bank (`rom_reg`, bit 7 selects the cartridge), `#FE` border and beeper, reads `#1F` / `#FE` give the two joypads.
- Burst transfers: `mcu.vhd` answers `CMD_CAPS` with caps `03` (burst write, `MCU_BURST_READY` driven, always ready). Max burst clock 32 MHz on the Mini G, 16 MHz on Go and Mini. `core.json` sets `spi_freq` 64 for the Mini G (`boards.minig`).
- `src/sim/run_burst_sim.sh` (ISim, optional path to `settings64.sh`): `tb_mcu_burst.vhd` runs the CAPS answer and the loaders with classic frames and bursts at the 63.9 MHz core clock; it prints `SIMULATION PASSED`.

## Release

`alf.kg` (all boards) goes into the release repo `karabas-go` (`cores/alf.kg`); `karabas-mini` has no cores any more.

## Memory

The Claude Code memory of this repo is shared with all karabas-go repos: `~/.claude/projects/<this project>/memory` is a symlink to `../karabas-go-notes` (private repo `andykarpov/karabas-go-notes`, `link.sh` there makes the links). Its `MEMORY.md` is split by repo; commit and push it after every memory change.
