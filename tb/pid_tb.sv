// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Top-level testbench wrapper for pid_core.
// Stimulus from tb/sim_main.cpp.

module pid_tb #(
  parameter int DATA_W = 16,
  parameter int GAIN_W = 32,
  parameter int Q_FRAC = 16,
  parameter int WIDE_W = 40,
  parameter int ACC_W  = 48
) (
  input  logic                      clk_i,
  input  logic                      rst_ni,

  input  logic                      tick_i,
  input  logic                      freeze_i,
  input  logic                      clear_i_i,
  input  logic                      awu_mode_i,

  input  logic signed [DATA_W-1:0]  sp_i,
  input  logic signed [DATA_W-1:0]  pv_i,

  input  logic signed [GAIN_W-1:0]  kp_i,
  input  logic signed [GAIN_W-1:0]  ki_ts_i,
  input  logic signed [GAIN_W-1:0]  kd_over_ts_i,
  input  logic signed [GAIN_W-1:0]  kb_i,

  input  logic signed [DATA_W-1:0]  out_min_i,
  input  logic signed [DATA_W-1:0]  out_max_i,

  output logic signed [DATA_W-1:0]  u_o,
  output logic                      valid_o,
  output logic                      sat_hi_o,
  output logic                      sat_lo_o
);

  pid_core #(
    .DATA_W (DATA_W),
    .GAIN_W (GAIN_W),
    .Q_FRAC (Q_FRAC),
    .WIDE_W (WIDE_W),
    .ACC_W  (ACC_W)
  ) u_dut (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .tick_i       (tick_i),
    .freeze_i     (freeze_i),
    .clear_i_i    (clear_i_i),
    .awu_mode_i   (awu_mode_i),
    .sp_i         (sp_i),
    .pv_i         (pv_i),
    .kp_i         (kp_i),
    .ki_ts_i      (ki_ts_i),
    .kd_over_ts_i (kd_over_ts_i),
    .kb_i         (kb_i),
    .out_min_i    (out_min_i),
    .out_max_i    (out_max_i),
    .u_o          (u_o),
    .valid_o      (valid_o),
    .sat_hi_o     (sat_hi_o),
    .sat_lo_o     (sat_lo_o)
  );

  pid_assertions #(
    .DATA_W (DATA_W)
  ) u_assert (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tick_i    (tick_i),
    .valid_o   (valid_o),
    .u_o       (u_o),
    .sat_hi_o  (sat_hi_o),
    .sat_lo_o  (sat_lo_o),
    .out_min_i (out_min_i),
    .out_max_i (out_max_i),
    .sp_i      (sp_i),
    .pv_i      (pv_i)
  );

  pid_cov #(
    .DATA_W (DATA_W),
    .GAIN_W (GAIN_W)
  ) u_cov (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .tick_i       (tick_i),
    .freeze_i     (freeze_i),
    .sat_hi_o     (sat_hi_o),
    .sat_lo_o     (sat_lo_o),
    .valid_o      (valid_o),
    .kp_i         (kp_i),
    .ki_ts_i      (ki_ts_i),
    .kd_over_ts_i (kd_over_ts_i)
  );

endmodule
