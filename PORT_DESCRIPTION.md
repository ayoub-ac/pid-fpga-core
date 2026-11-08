# Port description

Detailed signal protocol and timing for `pid_core`.

## Reset

`rst_ni` is **synchronous active-low**. Hold it low for at least 4
clock cycles before the first `tick_i`. After de-assertion, the
controller is in the IDLE-equivalent state with all pipeline registers
and the integrator accumulator zeroed.

```
clk_i      __|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_
rst_ni     ‾‾‾‾‾‾‾|_______________________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾
u_o        X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
valid_o    X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
```

## Tick / sample cadence

The controller advances by one PID step per rising `tick_i`. The host
asserts `tick_i` for a single cycle together with `sp_i` and `pv_i`.
The pipeline has 4 stages (capture, multiply, sum, saturate), so
`valid_o` pulses 4 cycles after `tick_i`:

```
clk_i      |‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_
tick_i     |‾‾|________________________________________
sp_i / pv_i|val|________________________________________
                    (capture)(mul)(sum)(sat)
valid_o    ____________________________|‾‾|_____________
u_o        XXXXXXXXXXXXXXXXXXXXXXXXXXXX|fresh sample|
```

The host may issue successive `tick_i` strobes at any cadence >= 1
clock cycle apart; the controller handles back-to-back ticks via the
pipeline. With ticks every cycle, the steady-state throughput is one
new sample per cycle.

The pre-multiplied gain convention (`ki_ts_i = Ki * Ts`,
`kd_over_ts_i = Kd / Ts`) means the host can change the tick cadence
at runtime; the controller does not embed a divider.

## Control inputs

| Signal       | Function                                                                                |
|--------------|------------------------------------------------------------------------------------------|
| `tick_i`     | One-cycle strobe; advances the controller by one PID step.                               |
| `freeze_i`   | When 1 with `tick_i`, P and D are computed but the integrator is held at its current value. |
| `clear_i_i`  | One-cycle synchronous integrator clear. Useful on "controller ON" transitions.           |
| `awu_mode_i` | 0 = conditional integration (clamp). 1 = back-calculation. Stable for the session.       |

## Plant interface

| Signal | Function                                              |
|--------|-------------------------------------------------------|
| `sp_i` | Setpoint, signed `DATA_W`-bit (default 16, -32768..32767). |
| `pv_i` | Process variable, same encoding.                       |

The error `e[n] = sp_i - pv_i` is computed in one extra bit of
precision to absorb the difference of two `DATA_W` values.

## Gains

All gains are **signed Q(GAIN_W-Q_FRAC-1).Q_FRAC** fixed-point. With
default `GAIN_W = 32` and `Q_FRAC = 16`, this is **Q15.16**: the
integer part is 16 bits (range -32768..32767) and the fractional part
is 16 bits (resolution ~1.5e-5).

| Signal           | Pre-computation         | Notes |
|------------------|-------------------------|-------|
| `kp_i`           | Kp directly             | Signed Q15.16. |
| `ki_ts_i`        | `Ki * Ts` (pre-multiplied) | Host computes once at config time so the controller does not need a runtime divider. |
| `kd_over_ts_i`   | `Kd / Ts` (pre-divided)  | Same. |
| `kb_i`           | Kb directly             | Back-calculation gain (only used when `awu_mode_i = 1`). Common starting point: `Kb = sqrt(Ki / Kd)` or `Kb = 1 / Tt` with `Tt` the back-calc time constant. |

## Output clamp

| Signal       | Function                                                            |
|--------------|---------------------------------------------------------------------|
| `out_min_i`  | Lower clamp limit, signed `DATA_W`-bit.                              |
| `out_max_i`  | Upper clamp limit, signed `DATA_W`-bit. Must be >= `out_min_i`.      |

The clamp is enforced on every published sample. The internal
unclamped sum is `WIDE_W`-bit (default 40) so a transient overflow
of the multiplied-and-summed contributions is absorbed before the
saturating mux selects `out_min_i / out_max_i / sum`.

## Outputs

| Signal     | Function                                                            |
|------------|---------------------------------------------------------------------|
| `u_o`      | Final clamped controller output, signed `DATA_W`-bit.                |
| `valid_o`  | Pulses for one cycle when `u_o` is fresh.                            |
| `sat_hi_o` | High while the latest published `u_o` was clamped at `out_max_i`.    |
| `sat_lo_o` | High while the latest published `u_o` was clamped at `out_min_i`.    |

`sat_hi_o` and `sat_lo_o` are intended to be mutually exclusive (the
testbench includes an SVA property for this; results not independently
confirmed).

## Latency summary

| Phase                            | Cycles |
|----------------------------------|--------|
| `tick_i` -> `valid_o`            | 4      |
| Reset assertion -> all-zero state | 1     |
| `clear_i_i` -> integrator zeroed  | 1     |

## Throughput

With `tick_i` asserted every cycle, the steady-state throughput is one
new `u_o` sample per cycle. The pipeline is fully feed-through after
the first 4-cycle warmup.

## Q-format conversion cheat sheet

Float -> Q15.16:  `q = round(f * 65536)`, signed.

| f       | q (Q15.16)    |
|---------|---------------|
| 1.0     | 0x0001_0000   |
| 0.5     | 0x0000_8000   |
| 0.1     | 0x0000_1999   |
| 0.05    | 0x0000_0CCC   |
| 0.001   | 0x0000_0042   |
| -1.0    | 0xFFFF_0000   |

Q15.16 -> float:  `f = q / 65536.0`, signed.

## Parameter overview

| Parameter | Default | Function                                              |
|-----------|---------|-------------------------------------------------------|
| `DATA_W`  | 16      | Width of `sp_i`, `pv_i`, `u_o`, clamp limits.         |
| `GAIN_W`  | 32      | Width of `kp_i`, `ki_ts_i`, `kd_over_ts_i`, `kb_i`.   |
| `Q_FRAC`  | 16      | Fractional bits in the gain Q-format. Default Q15.16. |
| `WIDE_W`  | 40      | Internal post-multiply / pre-clamp width.              |
| `ACC_W`   | 48      | Integrator accumulator width.                         |

The defaults give 16-bit signed I/O and Q15.16 gains. For 24-bit DAC outputs, override
`DATA_W = 24` (and update `WIDE_W` and `ACC_W` to maintain headroom).
