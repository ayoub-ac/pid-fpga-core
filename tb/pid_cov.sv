// SPDX-License-Identifier: GPL-3.0-or-later OR Commercial
//
// Functional coverage collector for pid_core. 8 sticky bins recorded
// across the regression. The C++ harness reads them at end-of-test.
//
// Bins:
//   c_p_only         : a session run with Ki = Kd = 0 (P-only)
//   c_pi             : a session run with Kd = 0, Ki != 0
//   c_pid            : a session with all three gains non-zero
//   c_saturated_high : sat_hi_o asserted at least once
//   c_saturated_low  : sat_lo_o asserted at least once
//   c_normal_op      : a tick fired with !sat_hi_o && !sat_lo_o
//   c_reset_mid_run  : rst_ni went low while ticks had been firing
//   c_freeze_active  : freeze_i asserted with tick_i (integrator hold)

module pid_cov #(
  parameter int DATA_W = 16,
  parameter int GAIN_W = 32
) (
  input logic                       clk_i,
  input logic                       rst_ni,
  input logic                       tick_i,
  input logic                       freeze_i,
  input logic                       sat_hi_o,
  input logic                       sat_lo_o,
  input logic                       valid_o,
  input logic signed [GAIN_W-1:0]   kp_i,
  input logic signed [GAIN_W-1:0]   ki_ts_i,
  input logic signed [GAIN_W-1:0]   kd_over_ts_i
);

  logic c_p_only         /*verilator public*/;
  logic c_pi             /*verilator public*/;
  logic c_pid            /*verilator public*/;
  logic c_saturated_high /*verilator public*/;
  logic c_saturated_low  /*verilator public*/;
  logic c_normal_op      /*verilator public*/;
  logic c_reset_mid_run  /*verilator public*/;
  logic c_freeze_active  /*verilator public*/;

  logic any_tick_seen_q;
  logic prev_rst_q;

  always_ff @(posedge clk_i) begin
    prev_rst_q <= rst_ni;

    if (!rst_ni) begin
      // Reset asserted: if ticks had been running, this is a mid-run
      // reset. Latch it BEFORE clearing the running flag, then clear it.
      if (any_tick_seen_q && prev_rst_q) c_reset_mid_run <= 1'b1;
      any_tick_seen_q <= 1'b0;
    end else begin
      if (tick_i) any_tick_seen_q <= 1'b1;

      // Gain-shape bins triggered on each tick (gains are stable).
      if (tick_i) begin
        if ((ki_ts_i == '0) && (kd_over_ts_i == '0))
          c_p_only <= 1'b1;
        if ((ki_ts_i != '0) && (kd_over_ts_i == '0))
          c_pi <= 1'b1;
        if ((kp_i != '0) && (ki_ts_i != '0) && (kd_over_ts_i != '0))
          c_pid <= 1'b1;
        if (freeze_i) c_freeze_active <= 1'b1;
      end

      // Saturation bins observed when valid_o pulses.
      if (valid_o && sat_hi_o)  c_saturated_high <= 1'b1;
      if (valid_o && sat_lo_o)  c_saturated_low  <= 1'b1;
      if (valid_o && !sat_hi_o && !sat_lo_o)
                                c_normal_op      <= 1'b1;
    end
  end

  initial begin
    c_p_only         = 1'b0;
    c_pi             = 1'b0;
    c_pid            = 1'b0;
    c_saturated_high = 1'b0;
    c_saturated_low  = 1'b0;
    c_normal_op      = 1'b0;
    c_reset_mid_run  = 1'b0;
    c_freeze_active  = 1'b0;
    any_tick_seen_q  = 1'b0;
    prev_rst_q       = 1'b1;
  end

endmodule
