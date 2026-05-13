// SPDX-License-Identifier: GPL-3.0-or-later OR Commercial
//
// PID integrator with anti-windup.
//
// Algorithm (positional form, advanced one step per `tick_i`):
//
//   I_term      = Ki * Ts * e
//   I_state'    = I_state + I_term            -- nominal update
//
// Anti-windup modes (selected by `awu_mode_i`):
//
//   0  conditional integration ("clamp" / integrator-halt):
//      If the previous full-controller output was saturated AND the new
//      I-increment would push the integrator further into saturation
//      (same sign as the saturated direction), the integrator is held.
//      This is the cheapest scheme and the one most users want.
//
//   1  back-calculation:
//      I_state' = I_state + I_term - Kb * (u_unsat - u_sat)
//      where (u_unsat - u_sat) is the "excess" computed by the saturating
//      stage on the previous tick. Kb is supplied as a Q15.16 gain.
//
// The integrator state is held in a wide accumulator (`ACC_W` bits, Q15.16
// scaled) so that small I-terms can accumulate without rounding to zero.
// The output `i_term_o` is right-shifted by `Q_FRAC` to surface the
// integer part for the summing stage.
//
// Reset / freeze behaviour:
//   * `rst_ni == 0`            : I_state cleared to 0.
//   * `clear_i_i == 1` (sync)  : I_state cleared to 0 on the next tick.
//   * `freeze_i == 1`          : ticks are ignored, I_state held.

module pid_integrator #(
  parameter int DATA_W   = 16,
  parameter int GAIN_W   = 32,
  parameter int Q_FRAC   = 16,
  parameter int ACC_W    = 48,
  parameter int OUT_W    = 16,
  parameter int WIDE_W   = 40
) (
  input  logic                       clk_i,
  input  logic                       rst_ni,
  input  logic                       tick_i,
  input  logic                       freeze_i,
  input  logic                       clear_i_i,

  // Per-tick error e[n] = sp - pv
  input  logic signed [DATA_W:0]     e_i,

  // Programmable gains (Q(GAIN_W-Q_FRAC-1).Q_FRAC signed).
  // ki_ts_i is Ki*Ts pre-multiplied by the host so the integrator does
  // not need a divider for arbitrary Ts. The host computes ki_ts_i in
  // Q15.16 once at config time.
  input  logic signed [GAIN_W-1:0]   ki_ts_i,
  input  logic signed [GAIN_W-1:0]   kb_i,        // back-calc gain

  // Anti-windup configuration
  input  logic                       awu_mode_i,  // 0 clamp, 1 back-calc
  input  logic                       sat_hi_i,    // last-tick output sat hi
  input  logic                       sat_lo_i,    // last-tick output sat lo
  input  logic signed [WIDE_W-1:0]   excess_i,    // u_unsat - u_sat (back-calc)

  // Outputs
  output logic signed [WIDE_W-1:0]   i_term_o,    // integer-domain I value
  output logic signed [ACC_W-1:0]    i_state_o    // raw Q-scaled state
);

  // Wide product width: e_i (DATA_W+1 signed) * ki_ts_i (GAIN_W signed)
  localparam int PROD_W = (DATA_W + 1) + GAIN_W;

  logic signed [ACC_W-1:0]   i_state_q;
  logic signed [PROD_W-1:0]  ki_e_full;
  logic signed [ACC_W-1:0]   ki_e_acc;
  logic signed [ACC_W-1:0]   acc_next;
  logic signed [ACC_W-1:0]   bc_correction;
  logic                      hold_int;

  // Conditional integration: hold the integrator if the actuator is
  // saturated AND the new term would push it deeper.
  always_comb begin
    hold_int = 1'b0;
    if (awu_mode_i == 1'b0) begin
      // ki_e_full sign tells us the direction of the increment.
      if (sat_hi_i && ki_e_full > 0) hold_int = 1'b1;
      if (sat_lo_i && ki_e_full < 0) hold_int = 1'b1;
    end
  end

  // Multiply the error by Ki*Ts. The host scales Ki*Ts to Q-format so
  // the multiply is a fixed-point operation; the result is the I-step
  // in Q-fraction-bits.
  always_comb begin
    ki_e_full = $signed(e_i) * $signed(ki_ts_i);
    // Sign-extend (or truncate) to the accumulator width.
    if (PROD_W >= ACC_W) ki_e_acc = ki_e_full[ACC_W-1:0];
    else                 ki_e_acc = {{(ACC_W-PROD_W){ki_e_full[PROD_W-1]}},
                                     ki_e_full};
  end

  // Back-calculation correction term = -Kb * excess (excess is in
  // integer output domain; shift it back to Q-domain for the
  // accumulator).
  logic signed [WIDE_W+GAIN_W-1:0] kb_excess_full;
  logic signed [ACC_W-1:0]         kb_excess_acc;
  always_comb begin
    kb_excess_full = $signed(excess_i) * $signed(kb_i);
    if ((WIDE_W+GAIN_W) >= ACC_W)
      kb_excess_acc = kb_excess_full[ACC_W-1:0];
    else
      kb_excess_acc = {{(ACC_W-(WIDE_W+GAIN_W)){kb_excess_full[WIDE_W+GAIN_W-1]}},
                       kb_excess_full};
  end

  always_comb begin
    bc_correction = '0;
    if (awu_mode_i == 1'b1) begin
      bc_correction = kb_excess_acc;
    end
    acc_next = i_state_q + ki_e_acc - bc_correction;
  end

  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      i_state_q <= '0;
    end else if (clear_i_i) begin
      i_state_q <= '0;
    end else if (tick_i && !freeze_i) begin
      if (hold_int) begin
        i_state_q <= i_state_q;
      end else begin
        i_state_q <= acc_next;
      end
    end
  end

  // Surface the integer part of i_state to the summer. Arithmetic shift
  // right by Q_FRAC. Result is sign-extended to WIDE_W.
  logic signed [ACC_W-1:0] i_state_int_shifted;
  always_comb begin
    i_state_int_shifted = i_state_q >>> Q_FRAC;
    if ((ACC_W - Q_FRAC) >= WIDE_W)
      i_term_o = i_state_int_shifted[WIDE_W-1:0];
    else
      i_term_o = {{(WIDE_W-(ACC_W-Q_FRAC)){i_state_int_shifted[ACC_W-1]}},
                  i_state_int_shifted[(ACC_W-Q_FRAC)-1:0]};
  end

  assign i_state_o = i_state_q;

endmodule
