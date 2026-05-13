-- SPDX-License-Identifier: GPL-3.0-or-later OR Commercial
--
-- VHDL-2008 wrapper for the SystemVerilog pid_core module.
--
-- This entity exposes the PID core's port list in VHDL syntax. The
-- architecture instantiates the SystemVerilog module by component name;
-- mixed-language elaboration is supported natively by Vivado, Quartus,
-- ModelSim/Questa, and Aldec, and via the GHDL VHPI bridge for open
-- simulation.
--
-- Status: WRAPPER ONLY. The PID datapath itself remains in SystemVerilog
-- (rtl/*.sv). A native VHDL port of the datapath is listed as future work.
--
-- I/O contract: identical to pid_core.sv (see PORT_DESCRIPTION.md).
--   * Single clock, active-low synchronous reset.
--   * Strobe-driven step (`tick_i`).
--   * Q15.16 signed gains. Outputs are signed `DATA_W` bits (16 default).

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity pid_core_vhdl is
  generic (
    DATA_W : integer := 16;
    GAIN_W : integer := 32
  );
  port (
    clk_i        : in  std_logic;
    rst_ni       : in  std_logic;

    tick_i       : in  std_logic;
    freeze_i     : in  std_logic;
    clear_i_i    : in  std_logic;
    awu_mode_i   : in  std_logic;

    sp_i         : in  std_logic_vector(DATA_W-1 downto 0);
    pv_i         : in  std_logic_vector(DATA_W-1 downto 0);

    kp_i         : in  std_logic_vector(GAIN_W-1 downto 0);
    ki_ts_i      : in  std_logic_vector(GAIN_W-1 downto 0);
    kd_over_ts_i : in  std_logic_vector(GAIN_W-1 downto 0);
    kb_i         : in  std_logic_vector(GAIN_W-1 downto 0);

    out_min_i    : in  std_logic_vector(DATA_W-1 downto 0);
    out_max_i    : in  std_logic_vector(DATA_W-1 downto 0);

    u_o          : out std_logic_vector(DATA_W-1 downto 0);
    valid_o      : out std_logic;
    sat_hi_o     : out std_logic;
    sat_lo_o     : out std_logic
  );
end entity pid_core_vhdl;

architecture rtl of pid_core_vhdl is

  component pid_core
    generic (
      DATA_W : integer := 16;
      GAIN_W : integer := 32;
      Q_FRAC : integer := 16;
      WIDE_W : integer := 40;
      ACC_W  : integer := 48
    );
    port (
      clk_i        : in  std_logic;
      rst_ni       : in  std_logic;
      tick_i       : in  std_logic;
      freeze_i     : in  std_logic;
      clear_i_i    : in  std_logic;
      awu_mode_i   : in  std_logic;
      sp_i         : in  std_logic_vector(DATA_W-1 downto 0);
      pv_i         : in  std_logic_vector(DATA_W-1 downto 0);
      kp_i         : in  std_logic_vector(GAIN_W-1 downto 0);
      ki_ts_i      : in  std_logic_vector(GAIN_W-1 downto 0);
      kd_over_ts_i : in  std_logic_vector(GAIN_W-1 downto 0);
      kb_i         : in  std_logic_vector(GAIN_W-1 downto 0);
      out_min_i    : in  std_logic_vector(DATA_W-1 downto 0);
      out_max_i    : in  std_logic_vector(DATA_W-1 downto 0);
      u_o          : out std_logic_vector(DATA_W-1 downto 0);
      valid_o      : out std_logic;
      sat_hi_o     : out std_logic;
      sat_lo_o     : out std_logic
    );
  end component;

begin

  u_pid : pid_core
    generic map (
      DATA_W => DATA_W,
      GAIN_W => GAIN_W
    )
    port map (
      clk_i        => clk_i,
      rst_ni       => rst_ni,
      tick_i       => tick_i,
      freeze_i     => freeze_i,
      clear_i_i    => clear_i_i,
      awu_mode_i   => awu_mode_i,
      sp_i         => sp_i,
      pv_i         => pv_i,
      kp_i         => kp_i,
      ki_ts_i      => ki_ts_i,
      kd_over_ts_i => kd_over_ts_i,
      kb_i         => kb_i,
      out_min_i    => out_min_i,
      out_max_i    => out_max_i,
      u_o          => u_o,
      valid_o      => valid_o,
      sat_hi_o     => sat_hi_o,
      sat_lo_o     => sat_lo_o
    );

end architecture rtl;
