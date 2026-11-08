# pid-fpga-core

[![License: GPL-3.0 | commercial](https://img.shields.io/badge/license-GPL--3.0%20%7C%20commercial-blue.svg)](LICENSE.md)

> **Status: experimental and unverified.** This core was written with AI assistance and has not been independently verified in simulation regressions, formal proofs or hardware. Claims in this repository about protocol compliance, test results, timing and resource use are unverified. Do not use it in production or safety-critical designs without your own verification. No support or maintenance is provided.

A small, synthesisable discrete-time PID controller IP core in
SystemVerilog. Positional form with derivative-on-PV, programmable
output clamp, and selectable anti-windup (conditional integration or
back-calculation). Targets iCE40, ECP5, Xilinx 7-series, Cyclone V,
and Tang Nano 9K.

The included tests are intended to compare the controller bit-exactly
against a software fixed-point reference across 256 random gain /
setpoint vectors, plus closed-loop step-response tests against a
discretised first-order plant; results not independently confirmed. No
FPGA hardware is required for the test suite.

## What this core is for

| Domain                      | Loop                                              |
|-----------------------------|---------------------------------------------------|
| Drone / UAV flight control  | Roll/pitch/yaw rate, position, altitude           |
| BLDC / PMSM motor drives    | Current (~10 kHz) and speed/position inner loops  |
| Industrial process control  | Temperature, pressure, flow, level                |
| Switched-mode power supply  | Voltage regulation, current sharing               |
| Robotics                    | Joint position / torque, gimbal stabilisation     |

Intended to deliver one new sample per cycle once primed, with a fixed
4-cycle latency from `tick_i` to `valid_o` (actuator-update jitter of
one system clock); not verified.

## Algorithm

```
e[n]  = sp - pv
P     = Kp * e[n]
I    += Ki * Ts * e[n]                   (anti-windup gated)
D     = (Kd / Ts) * (pv[n-1] - pv[n])    (D-on-PV: no derivative kick)
u     = clamp(P + I + D, out_min, out_max)
```

Anti-windup modes (`awu_mode_i`): **0** = conditional integration
(integrator held when the prior output saturated and the new step would
push deeper); **1** = back-calculation (`I -= Kb * (u_unsat - u_sat)`,
faster recovery, needs the extra `kb_i` gain).

## Quickstart

```bash
git clone https://github.com/ayoub-ac/pid-fpga-core.git
cd pid-fpga-core
make lint test            # Verilator lint + reference parity + closed loop (results not independently confirmed)
make synth_report         # SYNTH_REPORT.md across iCE40/ECP5/Xilinx/Vivado/Quartus
```

Requires Verilator 5.0+ for simulation. Yosys 0.30+ for the open synth
flow.

## Architecture

```
        sp_i ---->|                           |---> u_o
        pv_i ---->|         pid_core          |---> valid_o
        tick_i--->|       (4-stage pipe)      |---> sat_hi_o, sat_lo_o
       Kp/Ki/Kd-->|  pid_integrator + AWU     |
        clamps--->|  pid_saturate (clamp)     |
```

Pipeline: `capture (e, dpv)` -> `multiply (P, I, D)` -> `sum` -> `saturate`.

## Port summary

| Signal                                   | Dir | Width | Description                                        |
|------------------------------------------|-----|-------|----------------------------------------------------|
| `clk_i / rst_ni`                         | in  | 1     | System clock; synchronous active-low reset.         |
| `tick_i`                                 | in  | 1     | Strobe; one PID step per rising edge.               |
| `freeze_i / clear_i_i`                   | in  | 1     | Integrator hold / synchronous clear.                |
| `awu_mode_i`                             | in  | 1     | 0 = conditional integration; 1 = back-calc.         |
| `sp_i / pv_i`                            | in  | 16    | Setpoint / process variable, signed.                |
| `kp_i / ki_ts_i / kd_over_ts_i / kb_i`   | in  | 32    | Q15.16 gains. Host pre-computes Ki*Ts and Kd/Ts.   |
| `out_min_i / out_max_i`                  | in  | 16    | Output clamp limits, signed.                         |
| `u_o`                                    | out | 16    | Final clamped output.                                |
| `valid_o`                                | out | 1     | Pulses one cycle when `u_o` is fresh.                |
| `sat_hi_o / sat_lo_o`                    | out | 1     | Saturation flags (mutually exclusive).               |

Detailed handshake timing in [`PORT_DESCRIPTION.md`](PORT_DESCRIPTION.md).

## Build & test

```bash
make lint            # static check the RTL
make test            # build and run the full regression
```

The testbench is written to end with `+PASS all tests passed` and exit 0
on success. Tests are included; results not independently confirmed.

## Verification

Intended tests (results not independently confirmed):

| # | Test                              | Intended check                                        |
|---|-----------------------------------|-------------------------------------------------------|
| 1 | Reference parity (256 vectors)    | DUT bit-exact against software fixed-point reference  |
| 2 | Step response (P, PI, PID)        | Closed loop on tau=10 Ts plant, settling + overshoot  |
| 3 | Output clamping at both rails     | Sat flags, clamp values exact                         |
| 4 | Anti-windup (both modes)          | Output un-sticks within 50 ticks after sign-flip      |
| 5 | Reset mid-control                 | Post-reset output = 0; integrator cleared             |
| 6 | Setpoint tracking (3 gain sets)   | 1% tracking band reached within budget                |

The testbench includes SVA properties (output within clamp, sat-flag
consistency, sat mutex, valid follows tick, tick-age bounded) and eight
functional coverage bins (`P_only`, `PI`, `PID`, `saturated_high`,
`saturated_low`, `normal_op`, `reset_mid_run`, `freeze_active`).
Whether the properties hold and the bins are reached has not been
independently confirmed.

## Synthesis

`make synth_report` is intended to run every available toolchain on the
same RTL and emit [`SYNTH_REPORT.md`](SYNTH_REPORT.md). The figures
currently in that file and in
[`RESOURCE_ESTIMATES.md`](RESOURCE_ESTIMATES.md) are unverified
estimates; run `make synth_report` for real numbers. Yosys (ice40 / ecp5 /
xilinx) is mandatory; Vivado and Quartus are detected automatically.

## Variants

| Variant                            | Use case                                            |
|------------------------------------|-----------------------------------------------------|
| `rtl/pid_core.sv`                  | Default 16-bit / Q15.16 controller, both AWU modes  |
| `rtl/pid_integrator.sv`            | Anti-windup integrator only (drop-in for cascades)  |
| `rtl/pid_saturate.sv`              | Saturating clamp only                                |
| `vhdl_wrapper/pid_core_vhdl.vhd`   | VHDL-2008 entity wrapping the SV core                |

Not implemented: a time-multiplexed single-multiplier variant, a
pipelined-multiplier variant and a fault-tolerant TMR variant. No
release of these is planned or promised.

## License

Dual-licensed: **GPL-3.0-or-later** for open-source projects, or
**commercial** for closed-source products. See [`LICENSE.md`](LICENSE.md).

## Citation

```bibtex
@misc{pid-fpga-core,
  title  = {{pid-fpga-core}: a small dual-licensed discrete-time PID controller IP core in SystemVerilog},
  author = {Achour, Ayoub},
  year   = {2026},
  howpublished = {\url{https://github.com/ayoub-ac/pid-fpga-core}}
}
```

## Author

Ayoub Achour - [github.com/ayoub-ac](https://github.com/ayoub-ac)
