# Security model

`pid-fpga-core` is a control IP, not a cryptographic primitive. The
"security" considerations are different from an AES / SHA / signature
core: there are no keys, no nonces, and no confidentiality goal. What
the consumer of this core needs to evaluate is **functional safety** —
the conditions under which the controller can produce an unsafe
actuator command.

If this core sits in a path where a wrong actuator command can damage
hardware or harm a human (drone motor, industrial valve, medical
infusion pump, automotive BLDC, mains power converter), read this whole
document and decide whether the failure modes match your deployment.

## What the RTL is intended to provide (not verified)

None of the properties below has been independently verified in
simulation, formal proof or hardware. Treat them as design intent.

### 1. Output stays within `[out_min_i, out_max_i]`

Every published sample is intended to be clamped by `pid_saturate.sv`
before being registered to `u_o`, in hardware rather than in software
configuration; not verified. The testbench includes an assertion
(`p_output_within_clamp`) written to fail the regression if `u_o` falls
outside the configured limits; results not independently confirmed.

### 2. Deterministic sample-to-sample latency

The pipeline is intended to be fixed at 4 cycles from `tick_i` to
`valid_o` regardless of the input values, gain values, or saturation
state, with no data-dependent timing; not verified. This matters for
hard-real-time control loops: the intended actuator update jitter is
one clock cycle of the system clock.

### 3. Bounded integrator with anti-windup

The integrator state is held in a wide `ACC_W = 48`-bit accumulator
(default). Both anti-windup modes (conditional integration and
back-calculation) are implemented in RTL and selectable at runtime via
`awu_mode_i`. The testbench is written to check that after a long
saturation event the integrator unwinds within a bounded number of
ticks; results not independently confirmed. Failure to unwind would
imply runaway integration.

### 4. Reset clears all state

`rst_ni == 0` is intended to synchronously zero the integrator state,
the pipeline registers, and `u_o`, so that after de-assertion the first
`tick_i` produces a sample as if the controller had never run (no
leftover integral contribution from the previous session); not
verified.

### 5. Synchronous freeze

`freeze_i` is intended to halt integrator updates without disturbing P
and D; not verified. P and D continue to respond to setpoint /
process-variable changes; only the integral state is frozen. The C++
harness includes a test for this path; results not independently
confirmed.

## What is NOT covered (be honest)

- **Faulty gain values.** If the host writes `Kp = 0x7fff_ffff`
  (the maximum positive Q15.16 gain), the controller will produce
  whatever the math says. The output is still clamped to
  `[out_min_i, out_max_i]`, but the loop dynamics may be unstable.
  A safe-mode supervisor in the host is the right place to validate
  gain ranges.
- **Faulty `tick_i`.** If the host stops issuing `tick_i`, the
  controller stops. There is no internal watchdog. The integrator
  retains its last value until a new tick arrives. If that is a hazard
  in your system, add a watchdog in the host (a sample-count of
  zero ticks for N system-clock cycles raises a fault).
- **Sample-period drift.** `Ts` is implicit in `Ki * Ts` and `Kd / Ts`,
  pre-computed by the host. If the host drifts the inter-tick interval
  (e.g., in a software RTOS without hard scheduling guarantees), the
  effective gains drift accordingly. The hardware does not detect or
  correct this; it executes whatever step the gains and the tick
  cadence specify.
- **Setpoint command corruption.** The host owns the setpoint bus.
  If the host can be tricked into commanding a setpoint that drives
  the actuator into a damaging region, the controller will follow
  it (subject to `out_min_i / out_max_i`). Set those clamp limits
  conservatively and validate setpoint inputs at the host level.
- **Single-event upsets / radiation.** No TMR, no parity, no
  duplicated counters. For aerospace deployment, wrap the core in
  TMR or use the FPGA's safety primitives.
- **Side channels.** This is a control core. Power / EM analysis to
  recover gain values is possible in principle but uninteresting in
  practice (gains are typically not secret). No specific countermeasure.

## Evaluating the implementation

The testbench is intended to cover six failure modes. Tests are
included; results not independently confirmed:

1. **Reference parity** (256 random vectors). Intended to check that
   the DUT matches the C++ reference implementation bit-exactly across
   the gain space.
2. **Step-response settling** for P-only, PI, PID configurations on a
   discretised first-order plant. Intended to check settling time
   within the configured bound and overshoot below the configured
   limit, on every gain set.
3. **Output clamping** at both rails, intended to check that the
   saturation flags assert exactly when the unclamped output would have
   exceeded the limit.
4. **Anti-windup** in both modes: after 200 ticks of saturating
   command, sign-flipping the error must un-stick the output within
   <= 50 ticks.
5. **Reset mid-run**: assert `rst_ni` while the controller is
   integrating, and verify the first post-reset sample is zero
   (integrator has been cleared).
6. **Setpoint tracking** with three different gain profiles, each
   reaches a 1% tracking band within a configured number of ticks.

The testbench includes SVA properties (clamp invariant, saturation
flag consistency, mutex of `sat_hi` / `sat_lo`, valid follows tick
within latency, and `tick_age` boundedness) and eight functional
coverage bins (P_only, PI, PID, sat_hi, sat_lo, normal_op,
reset_mid_run, freeze_active). Whether the properties hold and the
bins are reached has not been independently confirmed.

## Safe-deployment checklist

1. Pick `out_min_i / out_max_i` to match the actuator's physical safe
   range (e.g., motor PWM duty 5..95%, valve open 0..100%).
2. Choose anti-windup mode 1 (back-calc) for fast actuators where the
   I-state must drain quickly after a saturation event; choose mode 0
   (conditional integration) for slow actuators where minimal
   computation is preferred.
3. Pre-compute `Ki * Ts` and `Kd / Ts` in the host with `Ts` matching
   the actual tick cadence, not the nominal cadence.
4. Issue a synchronous `clear_i_i` pulse on every "controller ON"
   transition to discard any leftover integrator state from a previous
   run.
5. Hold `rst_ni` low for at least 4 cycles after power-on and before
   the first command. This is intended to leave the pipeline clean; not
   verified.
6. Wrap the core in a host-side supervisor that validates gain values
   and watchdog-checks the tick cadence. The hardware does not do this.

## Reporting issues

Found a functional bug, a discrepancy with classical PID behaviour, or
a corner case the assertions miss? Open a GitHub issue (do not include
confidential details). No support or maintenance is provided. We do not
currently offer a bug bounty.
