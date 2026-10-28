# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

The Alf ("Эльф") core for the Karabas Go, Mini and Mini G boards: Xilinx Spartan-6 XC6SLX25 (ftg256), ISE 14.7 (XST). Alf is a Z80 cartridge game console derived from the ZX Spectrum (https://zxbyte.ru/alf.htm). The port is based on the Aeon-Lite project sources; license GPL v3 (`LICENSE`). This is branch `master` of `andykarpov/karabas-go-core-alf`; the branch `alf-128` (cloned locally as `karabas-go-core-alf128`) is the Alf-128 variant with 128 KB RAM and an AY.

The core is not converted to `karabas-go-modules`: it carries its own MCU link, OSD, HDMI and DAC modules in `src/rtl`.

## Layout

- `src/rtl/karabas_go_top.v` (VGA; the FT812 pins are held inactive), `karabas_mini_top.v`, `karabas_minig_top.v` (HDMI, `dvi_only` from the MCU): the board tops. All instantiate `alf` (`src/rtl/alf/alf.v`); only the Mini G top sets `BURST_MAX_MHZ(32)`.
- `src/rtl/alf/alf.v`: the machine. T80 CPU (`alf/cpu`), video (`alf/video/video.vhd`), VRAM (`alf/vram`, coregen), clocks (`alf/clock/clock.v`, a PLL: clk 63.9 MHz, VGA 25 MHz, MIDI 12 MHz), the MCU link, `hid_parser`, `soft_switches`, `overlay`.
- `src/rtl/mcu`: `mcu.vhd` (own copy of the SPI link), `burst_rx.vhd`, `hid_parser.vhd` (keyboard / joysticks to the Alf joypad ports), `soft_switches.vhd`.
- `src/rtl/osd`, `hdmi`, `dac` (`pcm5102.v`), `spi`, `vga`, `adc`: board helpers. `src/ipcore_dir`: ISE coregen cores (many are unused leftovers from other cores; the `.prj` files list what is built).
- `src/rom/alf.rom`: the 32 KB system ROM (start menu and BASIC-48) with the originality check removed; `alf_orig.rom` / `alf_orig_big.rom` are the originals (`src/rom/readme.txt`, Russian).
- `roms/`: Alf cartridge images for the SD card (`/alf/`).

## Build

- No Makefile or build script: open `src/karabas-go.xise`, `karabas-mini.xise` or `karabas-minig.xise` in ISE 14.7 and run Generate Programming File. The projects write `src/karabas_<board>_top.bin` and leave all ISE intermediates in `src/` (gitignored, as are the `.prj` / `.xst` files).
- Pack with `make_core.py` (a symlink to `../../karabas-go-tools/make_core/make_core.py`, gitignored) from `src/core.json` (Go, `alf.kg1`), `core_mini.json` (`alf.kg2`), `core_minig.json` (`alf.kg3`). The `.bin` and `.kg*` files are committed as the release. Bump `build` in the json for a new release.
- Each board has its own `.ucf` (`karabas_go.ucf`, `karabas_mini.ucf`, `karabas_minig.ucf`); `TS_CLK` is 64 MHz.

## MCU link and features

- Core type `fileloader`, dir `alf`, extensions `.rom,.bin`, `eeprom_bank` 255 (none). The OSD has one item: Reset (Ctrl+Alt+Del).
- The ROM block of the json loads `rom/alf.rom` at address 0 through the rom loader (`MA[20]` = 0); the file loader writes the cartridge at `MA[20]` = 1. The end of a file load resets the CPU, so does the gamepad Start (`alf_restart`).
- `cart_size` is taken from the last file loader address; cartridge banks above the size are mirrored (power-of-two sizes 16 KB..1 MB), and the embedded ROM is mirrored too.
- Ports: `#5F` ROM bank (`rom_reg`, bit 7 selects the cartridge), `#FE` border and beeper, reads `#1F` / `#FE` give the two joypads. Audio is the beeper only.
- Burst transfers: `mcu.vhd` answers `CMD_CAPS` with caps `03` (burst write, `MCU_BURST_READY` driven, always ready). Max burst clock 32 MHz on the Mini G, 16 MHz on Go and Mini. `core_minig.json` sets `spi_freq` 64.
- `src/sim/run_burst_sim.sh` (ISim, optional path to `settings64.sh`): `tb_mcu_burst.vhd` runs the CAPS answer and the loaders with classic frames and bursts at the 63.9 MHz core clock; it prints `SIMULATION PASSED`.

## Release

The `.kg1` is copied into the release repo `karabas-go` (`cores/alf.kg1`); `.kg2` / `.kg3` go to `karabas-mini`.

## Memory

The Claude Code memory of this repo is shared with all karabas-go repos: `~/.claude/projects/<this project>/memory` is a symlink to `../karabas-go-notes` (private repo `andykarpov/karabas-go-notes`, `link.sh` there makes the links). Its `MEMORY.md` is split by repo; commit and push it after every memory change.
