// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Discrete-time PID controller (positional form) with anti-windup,
// derivative-on-PV, programmable output clamp, and a strobe-driven
// sample period.
//
// Algorithm executed once per `tick_i` rising edge:
//
//   e[n]  = sp - pv
//   P     = Kp * e[n]
//   I    += Ki * Ts * e[n]                          (anti-windup gated)
//   D     = (Kd / Ts) * (pv[n-1] - pv[n])           (D-on-PV: avoids the
//                                                    derivative kick on
//                                                    setpoint changes)
//   u     = clamp(P + I + D, out_min, out_max)
//
// Fixed-point convention:
//   * Inputs / outputs are signed integers, `DATA_W` bits wide
//     (default 16, range -32768..32767).
//   * Gains are signed Q(GAIN_W-Q_FRAC-1).Q_FRAC fixed-point.
//     Default GAIN_W=32, Q_FRAC=16  ->  Q15.16.
//   * `ki_ts_i` is Ki*Ts pre-multiplied by the integrator host
//     (host computes once at config time; controller does not need a
//     divider / multiplier for an arbitrary Ts).
//   * `kd_over_ts_i` is Kd/Ts pre-divided likewise.
//
// Pipeline (latency: 4 cycles between `tick_i` and `valid_o`):
//   stage 0  : capture sp/pv, compute e and dpv (registered)
//   stage 1  : compute P, I-update, D (registered as wide products)
//   stage 2  : sum (registered)
//   stage 3  : saturate, register output, pulse `valid_o`
//
// Control signals:
//   tick_i        Strobe; one new PID step per rising tick.
//   freeze_i      Hold integrator (no I update). P and D still respond.
//   clear_i_i     Synchronous integrator clear (one-shot).
//   awu_mode_i    0 = conditional integration (clamp), 1 = back-calc.
//   pv_i, sp_i    Process variable, setpoint. Signed DATA_W.
//   kp_i, ki_ts_i, kd_over_ts_i, kb_i  : Q(GAIN_W-Q_FRAC-1).Q_FRAC gains.
//   out_min_i, out_max_i              : Output clamp limits, signed DATA_W.
//
// Outputs:
//   u_o           Final clamped output.
//   valid_o       Pulses for one cycle when `u_o` is fresh.
//   sat_hi_o, sat_lo_o : Saturation flags on the latest sample.

module pid_core #(
  parameter int DATA_W   = 16,
  parameter int GAIN_W   = 32,
  parameter int Q_FRAC   = 16,
  parameter int WIDE_W   = 40,
  parameter int ACC_W    = 48
) (
  input  logic                       clk_i,
  input  logic                       rst_ni,

  // Strobe + control
  input  logic                       tick_i,
  input  logic                       freeze_i,
  input  logic                       clear_i_i,
  input  logic                       awu_mode_i,

  // Plant interface
  input  logic signed [DATA_W-1:0]   sp_i,
  input  logic signed [DATA_W-1:0]   pv_i,

  // Gains
  input  logic signed [GAIN_W-1:0]   kp_i,
  input  logic signed [GAIN_W-1:0]   ki_ts_i,
  input  logic signed [GAIN_W-1:0]   kd_over_ts_i,
  input  logic signed [GAIN_W-1:0]   kb_i,

  // Output clamp
  input  logic signed [DATA_W-1:0]   out_min_i,
  input  logic signed [DATA_W-1:0]   out_max_i,

  // Outputs
  output logic signed [DATA_W-1:0]   u_o,
  output logic                       valid_o,
  output logic                       sat_hi_o,
  output logic                       sat_lo_o
);

  // ---------------------------------------------------------------
  // Stage 0 : capture and compute e and dpv (PV[n-1] - PV[n])
  // ---------------------------------------------------------------
  logic signed [DATA_W:0]   e_s0_q;     // sp - pv, one extra bit
  logic signed [DATA_W:0]   dpv_s0_q;   // pv_prev - pv (D on PV)
  logic signed [DATA_W-1:0] pv_prev_q;
  logic                     v_s0_q;

  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      e_s0_q     <= '0;
      dpv_s0_q   <= '0;
      pv_prev_q  <= '0;
      v_s0_q     <= 1'b0;
    end else begin
      v_s0_q <= tick_i;
      if (tick_i) begin
        e_s0_q    <= $signed({sp_i[DATA_W-1], sp_i})
                   - $signed({pv_i[DATA_W-1], pv_i});
        dpv_s0_q  <= $signed({pv_prev_q[DATA_W-1], pv_prev_q})
                   - $signed({pv_i[DATA_W-1], pv_i});
        pv_prev_q <= pv_i;
      end
    end
  end

  // ---------------------------------------------------------------
  // Stage 1 : compute P, request I-update, compute D (registered as
  // wide products surfaced as integer-domain WIDE_W values).
  // ---------------------------------------------------------------
  // Multiply widths: e (DATA_W+1) * Kp (GAIN_W) -> DATA_W+1+GAIN_W bits.
  localparam int MUL_W = (DATA_W + 1) + GAIN_W;

  logic signed [MUL_W-1:0]  p_full;
  logic signed [MUL_W-1:0]  d_full;
  logic signed [WIDE_W-1:0] p_term_s1_q;
  logic signed [WIDE_W-1:0] d_term_s1_q;
  logic signed [WIDE_W-1:0] i_term_s1_q;
  logic                     v_s1_q;
  logic                     sat_hi_q;
  logic                     sat_lo_q;
  logic signed [WIDE_W-1:0] excess_q;
  logic signed [WIDE_W-1:0] u_pre_clamp_q;

  // Combinational products, then sign-extended right-shift by Q_FRAC.
  // Host gains are Q(GAIN_W-Q_FRAC-1).Q_FRAC and inputs are integer, so
  // the integer-domain term is `(input * gain) >>> Q_FRAC`.
  function automatic logic signed [WIDE_W-1:0] mul_qshift_clip(
      input logic signed [MUL_W-1:0] product
  );
    logic signed [MUL_W-1:0] shifted;
    logic signed [WIDE_W-1:0] saturated_v;
    logic signed [MUL_W-1:0] max_pos;
    logic signed [MUL_W-1:0] min_neg;
    begin
      shifted = product >>> Q_FRAC;
      max_pos = (MUL_W'({1'b0, {(WIDE_W-1){1'b1}}}));    // 2^(WIDE_W-1)-1
      min_neg = -max_pos - 1;                            // -2^(WIDE_W-1)
      if (shifted > max_pos) saturated_v = {1'b0, {(WIDE_W-1){1'b1}}};
      else if (shifted < min_neg)
                             saturated_v = {1'b1, {(WIDE_W-1){1'b0}}};
      else                   saturated_v = shifted[WIDE_W-1:0];
      mul_qshift_clip = saturated_v;
    end
  endfunction

  // Wire integrator output forward (combinational from its register).
  logic signed [WIDE_W-1:0] i_term_w;

  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      p_full      <= '0;
      d_full      <= '0;
      p_term_s1_q <= '0;
      d_term_s1_q <= '0;
      i_term_s1_q <= '0;
      v_s1_q      <= 1'b0;
    end else begin
      v_s1_q <= v_s0_q;
      if (v_s0_q) begin
        p_full      <= $signed(e_s0_q)   * $signed(kp_i);
        d_full      <= $signed(dpv_s0_q) * $signed(kd_over_ts_i);
        p_term_s1_q <= mul_qshift_clip(
                         $signed(e_s0_q) * $signed(kp_i));
        d_term_s1_q <= mul_qshift_clip(
                         $signed(dpv_s0_q) * $signed(kd_over_ts_i));
        i_term_s1_q <= i_term_w;
      end
    end
  end

  // ---------------------------------------------------------------
  // Stage 2 : sum P + I + D
  // ---------------------------------------------------------------
  logic signed [WIDE_W-1:0] sum_s2_q;
  logic                     v_s2_q;

  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      sum_s2_q <= '0;
      v_s2_q   <= 1'b0;
    end else begin
      v_s2_q <= v_s1_q;
      if (v_s1_q) begin
        sum_s2_q <= p_term_s1_q + i_term_s1_q + d_term_s1_q;
      end
    end
  end

  // ---------------------------------------------------------------
  // Stage 3 : saturate and register the output
  // ---------------------------------------------------------------
  logic signed [DATA_W-1:0] u_clamped;
  logic                     sat_hi_w;
  logic                     sat_lo_w;

  pid_saturate #(
    .WIDE_W (WIDE_W),
    .OUT_W  (DATA_W)
  ) u_sat (
    .in_i           (sum_s2_q),
    .out_min_i      (out_min_i),
    .out_max_i      (out_max_i),
    .out_o          (u_clamped),
    .saturated_hi_o (sat_hi_w),
    .saturated_lo_o (sat_lo_w)
  );

  // u_unsat - u_sat (excess), used by back-calculation anti-windup.
  logic signed [WIDE_W-1:0] excess_w;
  always_comb begin
    excess_w = sum_s2_q - $signed({{(WIDE_W-DATA_W){u_clamped[DATA_W-1]}},
                                    u_clamped});
  end

  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      u_o      <= '0;
      valid_o  <= 1'b0;
      sat_hi_q <= 1'b0;
      sat_lo_q <= 1'b0;
      excess_q <= '0;
      u_pre_clamp_q <= '0;
    end else begin
      valid_o <= v_s2_q;
      if (v_s2_q) begin
        u_o           <= u_clamped;
        sat_hi_q      <= sat_hi_w;
        sat_lo_q      <= sat_lo_w;
        excess_q      <= excess_w;
        u_pre_clamp_q <= sum_s2_q;
      end
    end
  end

  assign sat_hi_o = sat_hi_q;
  assign sat_lo_o = sat_lo_q;

  // ---------------------------------------------------------------
  // Integrator instance.
  // The integrator runs on its own copy of `tick_i` so it advances in
  // step with stage-0 capture.
  // ---------------------------------------------------------------
  // The integrator looks at the e[n] from stage 0; we rebuild it
  // combinationally so the integrator can apply its update on the
  // SAME `tick_i` cycle without an extra cycle of skew.
  logic signed [DATA_W:0] e_for_int;
  always_comb begin
    e_for_int = $signed({sp_i[DATA_W-1], sp_i})
              - $signed({pv_i[DATA_W-1], pv_i});
  end

  pid_integrator #(
    .DATA_W (DATA_W),
    .GAIN_W (GAIN_W),
    .Q_FRAC (Q_FRAC),
    .ACC_W  (ACC_W),
    .OUT_W  (DATA_W),
    .WIDE_W (WIDE_W)
  ) u_int (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .tick_i     (tick_i),
    .freeze_i   (freeze_i),
    .clear_i_i  (clear_i_i),
    .e_i        (e_for_int),
    .ki_ts_i    (ki_ts_i),
    .kb_i       (kb_i),
    .awu_mode_i (awu_mode_i),
    .sat_hi_i   (sat_hi_q),
    .sat_lo_i   (sat_lo_q),
    .excess_i   (excess_q),
    .i_term_o   (i_term_w),
    .i_state_o  (i_state_unused)
  );

  // Surface the raw integrator state to a top-level wire for debug /
  // assertion access (kept internal; not exposed on the port list).
  /* verilator lint_off UNUSEDSIGNAL */
  logic signed [ACC_W-1:0] i_state_unused;
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
