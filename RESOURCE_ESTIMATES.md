# Resource estimates

Numbers below are from Yosys 0.x generic synthesis (`make synth_report`).
They are pre-place-and-route. See `SYNTH_REPORT.md` for the most recent
run; this document explains what dominates the area and how to shrink
it.

## Summary (post-Yosys, pre-PnR)

| Target                         | Comment |
|--------------------------------|---------|
| Lattice iCE40 UP5K             | LUT-heavy: the four signed multipliers (P, I, D, back-calc) cost ~7600 LUT4 because iCE40 has no DSP blocks. Fits on the UP5K (5280 logic tiles is tight; a smaller `GAIN_W` helps). |
| Lattice ECP5 LFE5UM-25         | ECP5 has MULT18X18 blocks; Yosys with `-abc9` packs the multiplies into them and the soft LUT count drops dramatically (~300 LUT4 plus the DSPs). |
| Xilinx Artix-7 XC7A35T         | Yosys-generic synthesis here counts LUT6 + CARRY4. Vivado synth_design will fold each multiplier into a single DSP48E1 and the LUT count drops sharply. |
| Cyclone V 5CSEMA5F31C6         | Quartus folds the multipliers into M9K-backed DSP blocks (one DSP per multiply). ALM count is much lower than the Yosys soft estimate. |
| Gowin GW1NR-9 (Tang Nano 9K)   | Not characterised here. Vendor flow uses MULT9X9 / MULT18X18 hard blocks; size comparable to ECP5. |

For exact LUT/FF/BRAM numbers run `make synth_report` and read
`SYNTH_REPORT.md`. The synthesis-comparison report is regenerated on
every CI build.

## What dominates the area

- **Four signed multipliers**: `e * Kp`, `e * Ki*Ts`, `dpv * Kd/Ts`,
  and `excess * Kb` (back-calculation). With default `GAIN_W = 32` and
  `Q_FRAC = 16`, each is a 17-bit signed * 32-bit signed product
  (49-bit result). Yosys generic flow maps these to LUT4 carry chains;
  vendor flows pack each into a single DSP block.
- **Integrator accumulator**: 48 bits of FF state plus a 48-bit adder.
  The adder is on the integrator-update critical path; the synthesiser
  packs the carry chain.
- **Pipeline registers**: 4 stages of mostly `WIDE_W = 40`-bit and
  `DATA_W = 16`-bit registers. Total ~300 FF on iCE40.
- **Saturating clamp**: a pair of `WIDE_W`-bit comparators plus a 3:1
  mux. ~50 LUT4 on iCE40, dwarfed by the multipliers.

## What you can do to shrink it

- **Reduce `GAIN_W`** if your target precision allows. Q7.8 (`GAIN_W = 16`,
  `Q_FRAC = 8`) cuts each multiplier by 4x area and may be perfectly fine
  for slow process control.
- **Drop the back-calculation path** if you only ever use anti-windup
  mode 0 (conditional integration). Stripping the `kb_i` multiply
  saves about 25% of the multiplier area.
- **Time-multiplex one multiplier** for P, I, D, and back-calc on
  resource-constrained targets. Costs ~4x latency per tick; the
  pipeline becomes 16 cycles instead of 4. Available in the Premium
  tier as a `PID_SHARED_MUL` define.
- **Vendor flow with DSP inference**: on Vivado / Quartus / Diamond,
  setting the multiplier registers as input + output pipeline stages
  lets the DSP block absorb both stages. The Yosys-generic estimate is
  the upper bound.

## What you can do to push throughput / accuracy

- **Pipelined multipliers** (Premium): break each 49-bit multiply
  across 2 stages so the multiplier critical path drops by ~2x. Lifts
  `f_max` to ~250 MHz on Artix-7 vendor flow.
- **Increased `ACC_W`** (parameter): widen the integrator accumulator
  to 64 bits if very small `Ki * Ts` values must accumulate over very
  long sessions (hours of regulator running without clear).
- **Cascade two cores**: a fast inner loop (current) at 100 kHz and a
  slow outer loop (position) at 1 kHz on the same FPGA, sharing one
  multiplier per loop. Common pattern in BLDC servo drives.

## Frequency

- iCE40 UP5K, default toolchain (Yosys + nextpnr), no constraints:
  limited by the 49-bit multiplier carry chain (~30-40 MHz on generic
  synth). Plenty for typical PID rates (1-100 kHz).
- ECP5 with vendor MULT18X18 packing: ~100-150 MHz.
- Artix-7 -1 speed grade: ~100 MHz typical with Yosys, ~200+ MHz with
  Vivado synth_design and DSP retiming.
- Cyclone V: similar to Artix-7 with M9K-backed DSP blocks.

## Power

Ballpark: 10-20 mW dynamic on iCE40 UP5K at 30 MHz, depending on tick
duty cycle. The four 49-bit multipliers toggle once per `tick_i`; the
integrator accumulator toggles once per non-frozen tick. Static power
is dominated by the FPGA itself.

## How to reproduce

```bash
make synth_report
```

This runs Yosys with `synth_ice40`, `synth_ecp5` (with `-abc9`), and
`synth_xilinx`, then `stat`, and writes the result to
`SYNTH_REPORT.md`. Vendor flows (Vivado, Quartus) are detected
automatically when on PATH.

For end-to-end place-and-route numbers (post-PnR LUT counts and
timing), run the vendor flow:
- iCE40 / ECP5: `nextpnr-ice40` / `nextpnr-ecp5` with Yosys output.
- Xilinx: Vivado with a project pointing at the `rtl/` files.
- Lattice Diamond / Radiant: project with `rtl/` added.
- Quartus: project with `rtl/` added.
