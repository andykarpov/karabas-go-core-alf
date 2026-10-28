# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

The Alf-128 core for the Karabas Go, Mini and Mini G boards: Xilinx Spartan-6 XC6SLX25 (ftg256), ISE 14.7 (XST). Alf ("Эльф") is a Z80 cartridge game console derived from the ZX Spectrum (https://zxbyte.ru/alf.htm); Alf-128 extends it with 128 KB of paged RAM (port `#7FFD`) and an AY-3-8912. The port is based on the Aeon-Lite project sources; license GPL v3 (`LICENSE`).

This checkout is the branch `alf-128` of `andykarpov/karabas-go-core-alf` (the plain Alf core is `master`, cloned locally as `karabas-go-core-alf`). The two branches share most files; the differences are `src/rtl/alf/alf.v`, the clock, `src/rtl/alf/ay8910`, `osd/overlay.vhd` and the core json. The README is the one of the plain Alf core.

The core is not converted to `karabas-go-modules`: it carries its own MCU link, OSD, HDMI and DAC modules in `src/rtl`.

## Layout

- `src/rtl/karabas_go_top.v` (VGA; the FT812 pins are held inactive), `karabas_mini_top.v`, `karabas_minig_top.v` (HDMI, `dvi_only` from the MCU): the board tops. All instantiate `alf` (`src/rtl/alf/alf.v`); only the Mini G top sets `BURST_MAX_MHZ(32)`.
- `src/rtl/alf/alf.v`: the machine. T80 CPU (`alf/cpu`), video (`alf/video/video.vhd`), VRAM (`alf/vram`, coregen, two screens), `alf/ay8910/ay8910.vhd`, clocks (`alf/clock/clock.v`, a PLL: clk 55.9 MHz, VGA 25 MHz, MIDI 12 MHz), the MCU link, `hid_parser`, `soft_switches`, `overlay`.
- `src/rtl/mcu`: `mcu.vhd` (own copy of the SPI link), `burst_rx.vhd`, `hid_parser.vhd`, `soft_switches.vhd`. `src/rtl/osd`, `hdmi`, `dac`, `spi`, `vga`, `adc`: board helpers. `src/ipcore_dir`: coregen cores (the `.prj` files list what is built).
- `src/rom/alf.rom`: the system ROM with the originality check removed (`src/rom/readme.txt`, Russian). `roms/`: Alf cartridge images for the SD card (`/alf/`).

## Build

- No Makefile or build script: open `src/karabas-go.xise`, `karabas-mini.xise` or `karabas-minig.xise` in ISE 14.7 and run Generate Programming File. The projects write `src/karabas_<board>_top.bin` and leave the ISE intermediates in `src/` (gitignored, as are the `.prj` / `.xst` files).
- Pack with `make_core.py` (a symlink to `../../karabas-go-tools/make_core/make_core.py`, gitignored) from `src/core.json` (Go, `alf128.kg1`), `core_mini.json` (`alf128.kg2`), `core_minig.json` (`alf128.kg3`). The `.bin` and `.kg*` files are committed as the release. Bump `build` in the json for a new release.
- The `.ucf` files still say `TS_CLK` 64 MHz, the clock of the plain Alf core; the real clk here is 55.9 MHz.

## Memory map and features

- SRAM: `MA[20:19]` 00 RAM (128 KB used), 01 internal ROM (the rom loader writes here), 1x cartridge (the file loader). `#0000-#3FFF` ROM, `#4000-#FFFF` RAM with page `#7FFD[2:0]` at `#C000`; `#7FFD` bit 3 the screen, bit 5 the lock, readable back. VRAM writes are mirrored from `#4000-#5FFF` and from `#C000` when page 5 or 7 is mapped.
- AY on `#FFFD` / `#BFFD` (clock enable 1.75 MHz), mixed with the beeper: A left, C right, B both. `#5F` ROM bank, `#FE` border and beeper, `#1F` / `#FE` reads the joypads.
- Unlike `master`, the cartridge banks are not masked to the cartridge size (no `cart_size` mirroring).
- Core type `fileloader`, id `alf128`, dir `alf`, extensions `.rom,.bin`, `eeprom_bank` 255. The OSD has one item: Reset (Ctrl+Alt+Del). The ROM block loads `rom/alf.rom` at 0. The end of a file load and the gamepad Start reset the CPU.
- Burst transfers: `mcu.vhd` answers `CMD_CAPS` with caps `03` (burst write, ready always high). Max burst clock 32 MHz on the Mini G, 16 MHz on Go and Mini. `core_mini.json` and `core_minig.json` set `spi_freq` 64.
- `src/sim/run_burst_sim.sh` (ISim): `tb_mcu_burst.vhd`; the extra cases run at the 55.9 MHz clock of this branch (the default `CLK_PS` and its comment are the 63.9 MHz of `master`).

## Release

The `.kg1` is copied into the release repo `karabas-go` (`cores/alf128.kg1`); `.kg2` / `.kg3` go to `karabas-mini`.

## Memory

The Claude Code memory of this repo is shared with all karabas-go repos: `~/.claude/projects/<this project>/memory` is a symlink to `../karabas-go-notes` (private repo `andykarpov/karabas-go-notes`, `link.sh` there makes the links). Its `MEMORY.md` is split by repo; commit and push it after every memory change.
