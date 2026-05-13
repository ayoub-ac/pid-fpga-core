# pid-fpga-core - build / lint / sim / synth
#
# Targets:
#   make lint          verilator --lint-only on RTL + TB
#   make sim           build the PID-core simulator
#   make test          build + run sim, check for "+PASS"
#   make synth         Yosys: ice40 + ecp5 + xilinx generic for pid_core
#   make synth_report  run all available toolchains and emit SYNTH_REPORT.md
#   make vhdl-test     GHDL+Verilator co-sim of the VHDL wrapper (if installed)
#   make clean         remove build artifacts

VERILATOR ?= verilator
YOSYS     ?= yosys
GHDL      ?= ghdl
VIVADO    ?= vivado
QUARTUS   ?= quartus_sh

RTL := \
    rtl/pid_saturate.sv \
    rtl/pid_integrator.sv \
    rtl/pid_core.sv

TB_TOP := tb/pid_tb.sv
TB_AUX := tb/pid_assertions.sv tb/pid_cov.sv
TB_CPP := tb/sim_main.cpp

VFLAGS := -Wall -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL

.PHONY: lint sim test synth synth_report vhdl-test clean

lint:
	$(VERILATOR) --lint-only $(VFLAGS) --top-module pid_tb \
	    $(RTL) $(TB_TOP) $(TB_AUX)

sim: obj_dir/Vpid_tb

obj_dir/Vpid_tb: $(RTL) $(TB_TOP) $(TB_AUX) $(TB_CPP)
	$(VERILATOR) --cc --exe --build $(VFLAGS) --assert --public-flat-rw \
	    --top-module pid_tb \
	    $(RTL) $(TB_TOP) $(TB_AUX) $(TB_CPP) \
	    -o Vpid_tb

test: sim
	./obj_dir/Vpid_tb | tee test.log
	@grep -q "+PASS" test.log && echo "TESTS PASSED" || (echo "TESTS FAILED" && exit 1)

# ---------- Open synthesis (Yosys) ------------------------------------------
synth:
	$(YOSYS) -p "read_verilog -sv $(RTL); hierarchy -top pid_core; synth_ice40 -top pid_core; stat" \
	    | tee synth_ice40.log
	$(YOSYS) -p "read_verilog -sv $(RTL); hierarchy -top pid_core; synth_ecp5 -top pid_core -abc9; stat" \
	    | tee synth_ecp5.log
	$(YOSYS) -p "read_verilog -sv $(RTL); hierarchy -top pid_core; synth_xilinx -top pid_core; stat" \
	    | tee synth_xilinx.log

# ---------- Cross-toolchain synthesis report --------------------------------
synth_report:
	@bash scripts/synth_report.sh

# ---------- VHDL co-sim (optional) ------------------------------------------
vhdl-test:
	@which $(GHDL) >/dev/null 2>&1 || { echo "ghdl not installed - skipping vhdl-test"; exit 0; }
	@which $(VERILATOR) >/dev/null 2>&1 || { echo "verilator not installed"; exit 1; }
	@bash scripts/vhdl_cosim.sh

clean:
	rm -rf obj_dir \
	    test.log \
	    synth_ice40.log synth_ecp5.log synth_xilinx.log \
	    synth_report/*.log SYNTH_REPORT.md
