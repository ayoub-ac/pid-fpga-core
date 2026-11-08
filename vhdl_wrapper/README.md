# VHDL wrapper

`pid_core_vhdl.vhd` is a thin VHDL-2008 entity that re-exposes the
SystemVerilog `pid_core` module under a VHDL-friendly interface. Use
it from a VHDL design that does not want to touch SystemVerilog directly.

## Status

**Wrapper only.** The PID datapath, integrator, and saturating clamp
all live in `rtl/*.sv` and are instantiated as a SystemVerilog
black-box. Mixed-language elaboration requires a simulator or
synthesiser that supports it; the wrapper has not been verified with
any specific tool.

No pure VHDL re-implementation of the datapath exists. Treat this
directory as a binding layer, not as a second implementation.

## Usage

### Vivado / Quartus / Diamond

Add both the SystemVerilog files and `pid_core_vhdl.vhd` to the same
project library (`work` is fine). The toolchain resolves the SV
component automatically:

```tcl
# Vivado
read_verilog -sv {rtl/pid_saturate.sv rtl/pid_integrator.sv rtl/pid_core.sv}
read_vhdl -vhdl2008 vhdl_wrapper/pid_core_vhdl.vhd
```

### ModelSim / Questa / Aldec

```
vlog -sv rtl/pid_saturate.sv rtl/pid_integrator.sv rtl/pid_core.sv
vcom -2008 vhdl_wrapper/pid_core_vhdl.vhd
```

### GHDL + Verilator (open-source co-sim)

The provided `make vhdl-test` target is a smoke check only: GHDL analyses
the VHDL wrapper and Verilator lints the SV core. It does not link the
two or run a mixed-language simulation. Run it from the repository root:

```
make vhdl-test
```

The target is a no-op (with a notice) if `ghdl` is not on `$PATH`.

## Instantiation example

```vhdl
library ieee;
  use ieee.std_logic_1164.all;

entity my_design is end entity;
architecture rtl of my_design is
  signal clk, rstn                : std_logic;
  signal tick, freeze, clr, awu   : std_logic;
  signal sp, pv, u_out            : std_logic_vector(15 downto 0);
  signal kp, ki, kd, kb           : std_logic_vector(31 downto 0);
  signal o_min, o_max             : std_logic_vector(15 downto 0);
  signal valid, sat_hi, sat_lo    : std_logic;
begin
  u_pid : entity work.pid_core_vhdl
    generic map (
      DATA_W => 16,
      GAIN_W => 32
    )
    port map (
      clk_i        => clk,
      rst_ni       => rstn,
      tick_i       => tick,
      freeze_i     => freeze,
      clear_i_i    => clr,
      awu_mode_i   => awu,
      sp_i         => sp,
      pv_i         => pv,
      kp_i         => kp,
      ki_ts_i      => ki,
      kd_over_ts_i => kd,
      kb_i         => kb,
      out_min_i    => o_min,
      out_max_i    => o_max,
      u_o          => u_out,
      valid_o      => valid,
      sat_hi_o     => sat_hi,
      sat_lo_o     => sat_lo
    );
end architecture;
```

## What this wrapper does NOT change

* Latency: intended to be identical to `pid_core.sv` (4 cycles between
  `tick_i` and `valid_o`); not verified.
* Endianness: scalar fields. The byte ordering question doesn't apply
  to integer fixed-point inputs.
* Reset polarity: still active-low synchronous (`rst_ni`).
* Q-format: the wrapper does not change the Q15.16 convention. The host
  computes `Ki*Ts` and `Kd/Ts` once at config time (the controller does
  not embed a divider).
