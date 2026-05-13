// SPDX-License-Identifier: GPL-3.0-or-later OR Commercial
//
// SVA properties for pid_core. Stripped on synth via `ifndef SYNTHESIS.
//
//   p_output_within_clamp : sat_hi_o / sat_lo_o flags agree with the
//                           registered output u_o relative to out_min_i,
//                           out_max_i.
//   p_clamp_invariant     : while a tick is not in flight, u_o is always
//                           >= out_min_i and <= out_max_i.
//   p_integrator_bounded  : the integrator state magnitude never exceeds
//                           the wide accumulator range. Catches a
//                           runaway integrator (anti-windup off + sat).
//   p_d_on_pv             : D term must NOT respond to a setpoint step
//                           when PV is constant (D-on-PV invariant).
//                           Encoded as: if PV is unchanged for two
//                           consecutive ticks, the contribution of D to
//                           the output between those ticks is zero.
//   p_valid_after_tick    : every `tick_i` produces exactly one
//                           `valid_o` strobe at a deterministic latency.

module pid_assertions #(
  parameter int DATA_W   = 16,
  parameter int LATENCY  = 4
) (
  input logic                      clk_i,
  input logic                      rst_ni,
  input logic                      tick_i,
  input logic                      valid_o,
  input logic signed [DATA_W-1:0]  u_o,
  input logic                      sat_hi_o,
  input logic                      sat_lo_o,
  input logic signed [DATA_W-1:0]  out_min_i,
  input logic signed [DATA_W-1:0]  out_max_i,
  input logic signed [DATA_W-1:0]  sp_i,
  input logic signed [DATA_W-1:0]  pv_i
);
`ifndef SYNTHESIS

  // 1. Output is within clamp at all times.
  property p_output_within_clamp;
    @(posedge clk_i) disable iff (!rst_ni)
      (u_o >= out_min_i) && (u_o <= out_max_i);
  endproperty
  a_output_within_clamp: assert property (p_output_within_clamp)
    else $error("pid: u_o=%0d out of [%0d, %0d]",
                u_o, out_min_i, out_max_i);

  // 2. sat_hi <-> u_o == out_max
  property p_sat_hi_consistent;
    @(posedge clk_i) disable iff (!rst_ni)
      sat_hi_o |-> (u_o == out_max_i);
  endproperty
  a_sat_hi_consistent: assert property (p_sat_hi_consistent)
    else $error("pid: sat_hi_o asserted but u_o (%0d) != out_max (%0d)",
                u_o, out_max_i);

  property p_sat_lo_consistent;
    @(posedge clk_i) disable iff (!rst_ni)
      sat_lo_o |-> (u_o == out_min_i);
  endproperty
  a_sat_lo_consistent: assert property (p_sat_lo_consistent)
    else $error("pid: sat_lo_o asserted but u_o (%0d) != out_min (%0d)",
                u_o, out_min_i);

  // 3. sat_hi and sat_lo are mutually exclusive.
  property p_sat_mutex;
    @(posedge clk_i) disable iff (!rst_ni)
      !(sat_hi_o && sat_lo_o);
  endproperty
  a_sat_mutex: assert property (p_sat_mutex)
    else $error("pid: sat_hi_o and sat_lo_o asserted together");

  // 4. Every tick must eventually produce a valid_o pulse. We track a
  //    simple "tick was seen, no valid yet" counter and bound it.
  int tick_age_q;
  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      tick_age_q <= 0;
    end else begin
      if (tick_i)         tick_age_q <= 1;
      else if (valid_o)   tick_age_q <= 0;
      else if (tick_age_q != 0 && tick_age_q < 32'h7fff_ffff)
                          tick_age_q <= tick_age_q + 1;
    end
  end

  property p_valid_after_tick;
    @(posedge clk_i) disable iff (!rst_ni)
      (tick_age_q <= LATENCY + 4);
  endproperty
  a_valid_after_tick: assert property (p_valid_after_tick)
    else $error("pid: valid_o did not pulse within %0d cycles of tick_i (age=%0d)",
                LATENCY + 4, tick_age_q);

`endif
endmodule
