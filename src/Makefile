# Command line build of the Alf core: the steps and options of the ISE 14.7 project navigator (the .xise
# project of the board, its karabas_<board>_top.cmd_log), as karabas-go-core-tsconf does it.
#
#   make                 all the boards (alf.kg)
#   make minig           one board: synthesis, implementation, bitstream, .kg file
#   make minig-xst       one step: <board>-xst (.ngc), <board>-impl (.bin), <board>-core (.kg)
#   make minig-report    LUTs, slices, block RAMs and the timing result of the last build
#   make clean           the intermediate files (not the .bin / .kg files, they are in git)
#
# Inputs per board: karabas_<board>.prj (the source list), .xst, .ucf, .ut. The Mini G is on MCU link v2;
# Go and Mini still build in ISE from their .xise projects (they move with the second pass of link v2).

ISE_SETTINGS ?= /opt/Xilinx/14.7/ISE_DS/settings64.sh
PART = xc6slx25-ftg256-2
BOARDS = minig

SHELL = /bin/bash
.SHELLFLAGS = -e -o pipefail -c
.NOTPARALLEL:
# settings64.sh takes the arguments of the shell that sources it: none here
XIL = source $(ISE_SETTINGS) > /dev/null;

# all boards in one file: a board build puts its bitstream in (make_core --boards)
KG = alf.kg
JSON = core.json

# map / par options of the .xise project (cmd_log), cost table 2 (the default of the karabas cores)
MAP_OPT = -logic_opt off -ol high -xe n -xt 0 -register_duplication off -r 4 -global_opt off -mt 2 -detail -ir off -pr b -lc off -power off
PAR_OPT = -ol high -xe n -mt 4
MAP_T = 2

# included by the sources, not in the prj
INCLUDES = $(wildcard ../modules/rtl/mcu/*.vh)

.PHONY: all clean $(BOARDS) $(BOARDS:=-xst) $(BOARDS:=-impl) $(BOARDS:=-core) $(BOARDS:=-report)

all: $(BOARDS)

define board
SRC_$(1) := $$(shell sed -n 's/^[a-z]* work "\(.*\)"/\1/p' karabas_$(1).prj)

karabas_$(1).ngc: karabas_$(1).prj karabas_$(1).xst $$(SRC_$(1)) $(INCLUDES)
	mkdir -p xst/projnav.tmp
	$(XIL) xst -intstyle ise -ifn karabas_$(1).xst -ofn karabas_$(1).syr

karabas_$(1).bin: karabas_$(1).ngc karabas_$(1).ucf karabas_$(1).ut
	$(XIL) ngdbuild -intstyle ise -dd _ngo -sd ipcore_dir -nt timestamp -uc karabas_$(1).ucf -p $(PART) karabas_$(1).ngc karabas_$(1).ngd
	$(XIL) map -intstyle ise -p $(PART) -w $(MAP_OPT) -t $(MAP_T) -o karabas_$(1)_map.ncd karabas_$(1).ngd karabas_$(1).pcf
	$(XIL) par -w -intstyle ise $(PAR_OPT) karabas_$(1)_map.ncd karabas_$(1).ncd karabas_$(1).pcf
	$(XIL) trce -intstyle ise -v 3 -s 2 -n 3 -fastpaths -xml karabas_$(1).twx karabas_$(1).ncd -o karabas_$(1).twr karabas_$(1).pcf -ucf karabas_$(1).ucf
	$(XIL) bitgen -intstyle ise -f karabas_$(1).ut karabas_$(1).ncd

kg_$(1).stamp: karabas_$(1).bin $(JSON) $$(wildcard rom/*)
	python3 make_core.py $(JSON) $(KG) --boards $(1)
	touch $$@

$(1): kg_$(1).stamp
$(1)-xst: karabas_$(1).ngc
$(1)-impl: karabas_$(1).bin
$(1)-core: kg_$(1).stamp
$(1)-report:
	@grep -E "Number of (Slice LUTs|occupied Slices|RAMB16BWERs|RAMB8BWERs):" karabas_$(1)_map.mrp
	@grep -E "constraints? (was|were) met|constraints? not met|Timing Score" karabas_$(1).par | sort -u
endef

$(foreach b,$(BOARDS),$(eval $(call board,$(b))))

clean:
	rm -f kg_*.stamp
	rm -rf xst _ngo _xmsgs xlnx_auto_0_xdb
	rm -f $(foreach b,$(BOARDS),$(addprefix karabas_$(b),.ngc .ngd .ngr .syr .lso .bld .pcf .ncd _map.ncd _map.ngm \
		_map.map _map.mrp _map.xrpt .par .pad _pad.csv _pad.txt .twr .twx .unroutes .xpi .ptwx .bgn .bit .drc \
		_bitgen.xwbt _summary.xml _usage.xml _xst.xrpt _ngdbuild.xrpt _par.xrpt _guide.ncd .cmd_log .stx \
		_envsettings.html _summary.html))
