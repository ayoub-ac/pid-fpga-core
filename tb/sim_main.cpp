// SPDX-License-Identifier: GPL-3.0-or-later OR Commercial
//
// Verilator C++ harness for pid_core. The host model implements the
// same fixed-point PID algorithm in software and the loop checks that
// the DUT result matches sample-for-sample. A first-order plant model
// closes the loop for a step-response test; settling time and overshoot
// are checked against thresholds.
//
// Tests:
//   1. Reference parity: 256 random tick sequences, gains, sp / pv
//      profiles. The C++ reference computes the same fixed-point output
//      and the DUT must match every sample.
//   2. Step response (P-only, PI, PID): drive a first-order plant
//      G(s) = 1/(tau s + 1) discretised at Ts; the DUT closes the loop
//      and the steady-state error / overshoot are checked.
//   3. Output clamping: a P-only configuration with a Kp large enough
//      to saturate the output; sat_hi_o / sat_lo_o must assert and the
//      output must be exactly out_min_i / out_max_i.
//   4. Anti-windup: a saturated controller must not let the integrator
//      grow unboundedly. Both modes (clamp, back-calc) are exercised.
//   5. Reset mid-control: assert rst_ni in the middle of a session and
//      verify the integrator + pipeline come back from zero.
//   6. Setpoint tracking: tracking error reaches < 1% of full-scale in
//      < N ticks for a given Kp/Ki.
//
// Output: prints "+PASS" or "+FAIL" with a coverage report.

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

#include "verilated.h"
#include "Vpid_tb.h"
#include "Vpid_tb___024root.h"
#include "Vpid_tb_pid_tb.h"
#include "Vpid_tb_pid_cov.h"

// ---------------------------------------------------------------------------
// Sim primitives
// ---------------------------------------------------------------------------
static int g_failures = 0;
static vluint64_t g_time = 0;

static void tick_clk(Vpid_tb* dut) {
    dut->clk_i = 0; dut->eval(); g_time++;
    dut->clk_i = 1; dut->eval(); g_time++;
}

static void reset(Vpid_tb* dut) {
    dut->rst_ni       = 0;
    dut->tick_i       = 0;
    dut->freeze_i     = 0;
    dut->clear_i_i    = 0;
    dut->awu_mode_i   = 0;
    dut->sp_i         = 0;
    dut->pv_i         = 0;
    dut->kp_i         = 0;
    dut->ki_ts_i      = 0;
    dut->kd_over_ts_i = 0;
    dut->kb_i         = 0;
    dut->out_min_i    = -32768;
    dut->out_max_i    =  32767;
    for (int i = 0; i < 4; i++) tick_clk(dut);
    dut->rst_ni = 1;
    tick_clk(dut);
}

// ---------------------------------------------------------------------------
// Drive one PID step. Inputs are sampled by the strobe; the function
// blocks until valid_o asserts. The DUT pipeline depth is 4 cycles, so
// this completes within 6 cycles in practice.
// ---------------------------------------------------------------------------
static int16_t pid_step(Vpid_tb* dut,
                        int16_t sp, int16_t pv,
                        bool freeze = false, bool clear_i = false) {
    dut->sp_i      = sp;
    dut->pv_i      = pv;
    dut->freeze_i  = freeze ? 1 : 0;
    dut->clear_i_i = clear_i ? 1 : 0;
    dut->tick_i    = 1;
    tick_clk(dut);
    dut->tick_i    = 0;
    dut->clear_i_i = 0;
    dut->freeze_i  = 0;

    int budget = 16;
    while (budget-- > 0) {
        if (dut->valid_o) return (int16_t)dut->u_o;
        tick_clk(dut);
    }
    g_failures++;
    std::fprintf(stderr, "valid_o never asserted within budget\n");
    return 0;
}

// ---------------------------------------------------------------------------
// Software reference (matches the DUT's fixed-point arithmetic exactly).
// All gains are Q15.16. The host pre-multiplies Ki*Ts and Kd/Ts for
// runtime efficiency.
// ---------------------------------------------------------------------------
struct PidRef {
    int32_t kp, ki_ts, kd_over_ts, kb;
    int16_t out_min, out_max;
    int     awu_mode;        // 0 clamp, 1 back-calc
    int64_t i_state;         // Q-format accumulator (48 bits used)
    int16_t pv_prev;
    bool    sat_hi, sat_lo;
    int64_t excess;          // u_unsat - u_sat (integer domain)

    void reset() {
        i_state = 0; pv_prev = 0; sat_hi = false; sat_lo = false;
        excess = 0;
    }

    static int64_t qclip(int64_t v, int wide_w) {
        int64_t hi = (1LL << (wide_w - 1)) - 1;
        int64_t lo = -(1LL << (wide_w - 1));
        if (v > hi) return hi;
        if (v < lo) return lo;
        return v;
    }

    int16_t step(int16_t sp, int16_t pv,
                 bool freeze = false, bool clear_i = false) {
        if (clear_i) i_state = 0;

        int32_t e = (int32_t)sp - (int32_t)pv;
        int32_t dpv = (int32_t)pv_prev - (int32_t)pv;

        // P (integer-domain): (e * kp) >> 16, then clip to WIDE_W
        int64_t p_full = (int64_t)e * (int64_t)kp;
        int64_t p_term = qclip(p_full >> 16, 40);

        // D
        int64_t d_full = (int64_t)dpv * (int64_t)kd_over_ts;
        int64_t d_term = qclip(d_full >> 16, 40);

        // I update (Q-domain accumulator). Anti-windup gating:
        int64_t ki_e = (int64_t)e * (int64_t)ki_ts;
        bool hold = false;
        if (awu_mode == 0) {
            if (sat_hi && ki_e > 0) hold = true;
            if (sat_lo && ki_e < 0) hold = true;
        }
        int64_t bc = 0;
        if (awu_mode == 1) bc = excess * (int64_t)kb;

        if (!freeze) {
            if (!hold) i_state += ki_e - bc;
        }

        // Integer-domain I
        int64_t i_int = i_state >> 16;
        int64_t i_term = qclip(i_int, 40);

        int64_t sum = p_term + i_term + d_term;
        int16_t u;
        if (sum > out_max) {
            u = out_max; sat_hi = true; sat_lo = false;
        } else if (sum < out_min) {
            u = out_min; sat_hi = false; sat_lo = true;
        } else {
            u = (int16_t)sum; sat_hi = false; sat_lo = false;
        }
        excess = sum - (int64_t)u;
        pv_prev = pv;
        return u;
    }
};

// ---------------------------------------------------------------------------
// First-order plant: y[n+1] = a*y[n] + b*u[n]. Discrete approximation of
// G(s) = K/(tau s + 1) at sample period Ts: a = exp(-Ts/tau),
// b = K*(1-a).
// We carry y as a real number for plant-side accuracy and round to
// int16 for the DUT input.
// ---------------------------------------------------------------------------
struct Plant {
    double a, b;
    double y;
    void reset() { y = 0.0; }
    int16_t advance(int16_t u) {
        y = a * y + b * (double)u;
        if (y > 32767.0)  y = 32767.0;
        if (y < -32768.0) y = -32768.0;
        return (int16_t)std::lround(y);
    }
};

// Fixed-point helpers
static int32_t to_q1616(double f) {
    double scaled = f * 65536.0;
    if (scaled > 2147483647.0)  return 0x7fffffff;
    if (scaled < -2147483648.0) return (int32_t)0x80000000;
    return (int32_t)std::lround(scaled);
}

// ---------------------------------------------------------------------------
// 1. Reference parity test
// ---------------------------------------------------------------------------
static void test_reference_parity(Vpid_tb* dut) {
    std::printf("---- Reference parity (256 random vectors) ----\n");
    int passes = 0, fails = 0;
    std::srand(0xC0FFEE);
    for (int v = 0; v < 256; v++) {
        // Random gains within reasonable Q15.16 range.
        double kp_f = ((std::rand() % 2000) - 1000) / 100.0;       // [-10, 10]
        double ki_f = ((std::rand() % 200)  - 100)  / 1000.0;      // [-0.1, 0.1]
        double kd_f = ((std::rand() % 200)  - 100)  / 1000.0;      // [-0.1, 0.1]
        bool   awu = (std::rand() & 1);

        PidRef ref;
        ref.kp        = to_q1616(kp_f);
        ref.ki_ts     = to_q1616(ki_f);
        ref.kd_over_ts= to_q1616(kd_f);
        ref.kb        = to_q1616(0.5);
        ref.out_min   = -10000;
        ref.out_max   =  10000;
        ref.awu_mode  = awu ? 1 : 0;
        ref.reset();

        // Push gains into the DUT, then reset it cleanly so the
        // integrator starts from zero matching the ref.
        dut->kp_i         = ref.kp;
        dut->ki_ts_i      = ref.ki_ts;
        dut->kd_over_ts_i = ref.kd_over_ts;
        dut->kb_i         = ref.kb;
        dut->out_min_i    = ref.out_min;
        dut->out_max_i    = ref.out_max;
        dut->awu_mode_i   = ref.awu_mode;

        // Force reset of the DUT to match the ref's clean state.
        reset(dut);
        // Re-apply gains after reset (gains live in input ports, not
        // registers, but reset() zeroes the input-side handles).
        dut->kp_i         = ref.kp;
        dut->ki_ts_i      = ref.ki_ts;
        dut->kd_over_ts_i = ref.kd_over_ts;
        dut->kb_i         = ref.kb;
        dut->out_min_i    = ref.out_min;
        dut->out_max_i    = ref.out_max;
        dut->awu_mode_i   = ref.awu_mode;

        bool ok = true;
        for (int n = 0; n < 32; n++) {
            int16_t sp = (int16_t)((std::rand() % 8000) - 4000);
            int16_t pv = (int16_t)((std::rand() % 8000) - 4000);
            int16_t r = ref.step(sp, pv);
            int16_t d = pid_step(dut, sp, pv);
            if (r != d) {
                if (fails < 3) {
                    std::printf("  vec %d step %d: ref=%d dut=%d sp=%d pv=%d\n",
                                v, n, r, d, sp, pv);
                }
                ok = false;
                break;
            }
        }
        if (ok) passes++; else fails++;
    }
    std::printf("  reference parity: %d/256 PASS\n", passes);
    if (fails > 0) g_failures += fails;
}

// ---------------------------------------------------------------------------
// 2. Step-response closed-loop test
// ---------------------------------------------------------------------------
static bool run_step_response(Vpid_tb* dut,
                              double kp_f, double ki_f, double kd_f,
                              int16_t setpoint,
                              int     n_ticks,
                              int     settle_within,
                              double  settle_band_pct,
                              double  max_overshoot_pct,
                              const char* label) {
    Plant plant;
    plant.a = std::exp(-1.0 / 10.0);   // tau = 10 Ts
    plant.b = 1.0 * (1.0 - plant.a);   // K = 1
    plant.reset();

    PidRef ref;
    ref.kp         = to_q1616(kp_f);
    ref.ki_ts      = to_q1616(ki_f);
    ref.kd_over_ts = to_q1616(kd_f);
    ref.kb         = to_q1616(0.5);
    ref.out_min    = -32768;
    ref.out_max    =  32767;
    ref.awu_mode   = 0;
    ref.reset();

    reset(dut);
    dut->kp_i         = ref.kp;
    dut->ki_ts_i      = ref.ki_ts;
    dut->kd_over_ts_i = ref.kd_over_ts;
    dut->kb_i         = ref.kb;
    dut->out_min_i    = ref.out_min;
    dut->out_max_i    = ref.out_max;
    dut->awu_mode_i   = 0;

    int16_t pv = 0;
    int16_t peak = 0;
    int     settle_at = -1;
    for (int n = 0; n < n_ticks; n++) {
        int16_t u = pid_step(dut, setpoint, pv);
        // Cross-check the DUT against the SW reference at every step.
        int16_t r = ref.step(setpoint, pv);
        if (u != r) {
            std::printf("  [FAIL %s] dut/ref divergence at n=%d (dut=%d ref=%d)\n",
                        label, n, u, r);
            return false;
        }
        pv = plant.advance(u);
        if (std::abs((int)pv) > std::abs((int)peak)) peak = pv;
        if (settle_at < 0) {
            double err = std::abs((double)setpoint - (double)pv);
            double band = settle_band_pct / 100.0 * std::abs((double)setpoint);
            if (err <= band) settle_at = n;
        }
    }
    bool settle_ok = (settle_at >= 0) && (settle_at <= settle_within);
    double overshoot = (peak > setpoint && setpoint > 0)
        ? (100.0 * ((double)peak - (double)setpoint) / (double)setpoint)
        : 0.0;
    bool over_ok = (overshoot <= max_overshoot_pct);

    if (settle_ok && over_ok) {
        std::printf("  [PASS %s] settle@%d/%d ticks, overshoot %.1f%% (<=%.1f%%)\n",
                    label, settle_at, settle_within, overshoot,
                    max_overshoot_pct);
        return true;
    } else {
        std::printf("  [FAIL %s] settle@%d/%d, overshoot %.1f%% (limit %.1f%%)\n",
                    label, settle_at, settle_within, overshoot,
                    max_overshoot_pct);
        return false;
    }
}

static void test_step_response(Vpid_tb* dut) {
    std::printf("---- Step response (closed-loop, plant tau=10 Ts) ----\n");
    int pass = 0, fail = 0;
    // P-only has a steady-state error; settle band is wide.
    if (run_step_response(dut, 1.0, 0.0,  0.0,  1000, 200, 200, 60.0, 30.0, "P-only"))
        pass++; else fail++;
    // PI removes steady-state error; tight 5% band reachable.
    if (run_step_response(dut, 0.5, 0.05, 0.0,  1000, 400, 400, 5.0, 50.0, "PI"))
        pass++; else fail++;
    // PID with derivative damping.
    if (run_step_response(dut, 0.5, 0.05, 0.02, 1000, 400, 400, 5.0, 30.0, "PID"))
        pass++; else fail++;
    std::printf("  step response: %d/3 PASS\n", pass);
    if (fail > 0) g_failures += fail;
}

// ---------------------------------------------------------------------------
// 3. Output clamping
// ---------------------------------------------------------------------------
static void test_output_clamp(Vpid_tb* dut) {
    std::printf("---- Output clamping ----\n");
    reset(dut);
    dut->kp_i         = to_q1616(100.0);   // huge gain to force saturation
    dut->ki_ts_i      = 0;
    dut->kd_over_ts_i = 0;
    dut->kb_i         = 0;
    dut->out_min_i    = -1000;
    dut->out_max_i    =  1000;
    dut->awu_mode_i   = 0;

    int16_t u_hi = pid_step(dut, 5000, 0);
    bool hi_ok = (u_hi == 1000) && (dut->sat_hi_o == 1);
    int16_t u_lo = pid_step(dut, -5000, 0);
    bool lo_ok = (u_lo == -1000) && (dut->sat_lo_o == 1);
    if (hi_ok && lo_ok)
        std::printf("  [PASS] clamps at +1000 / -1000 with sat flags\n");
    else {
        std::printf("  [FAIL] clamp: u_hi=%d (sat_hi=%u) u_lo=%d (sat_lo=%u)\n",
                    u_hi, dut->sat_hi_o, u_lo, dut->sat_lo_o);
        g_failures++;
    }
}

// ---------------------------------------------------------------------------
// 4. Anti-windup
// ---------------------------------------------------------------------------
static void test_anti_windup(Vpid_tb* dut) {
    std::printf("---- Anti-windup (clamp + back-calc) ----\n");
    int local_fail = 0;
    for (int mode = 0; mode <= 1; mode++) {
        reset(dut);
        dut->kp_i         = to_q1616(0.05);
        dut->ki_ts_i      = to_q1616(0.5);
        dut->kd_over_ts_i = 0;
        dut->kb_i         = to_q1616(0.5);
        dut->out_min_i    = -500;
        dut->out_max_i    =  500;
        dut->awu_mode_i   = mode;

        // Drive a large constant error for a long time -> output saturates.
        // pv stays at 0 (open-loop test). With anti-windup, the integrator
        // must be small enough that flipping the sign of the error
        // un-saturates the output in just a few ticks. Without anti-windup,
        // the integrator would have grown unboundedly and would take
        // hundreds of ticks to unwind.
        for (int n = 0; n < 200; n++) {
            (void)pid_step(dut, 3000, 0);    // saturating positive error
        }
        // Sign-flip the error: sp negative pushes the controller the
        // other way. With proper anti-windup the output unsticks fast.
        int unstick_at = -1;
        for (int n = 0; n < 100; n++) {
            int16_t u = pid_step(dut, -3000, 0);
            if (!dut->sat_hi_o) { unstick_at = n; break; }
            (void)u;
        }
        bool ok = (unstick_at >= 0) && (unstick_at <= 50);
        if (ok)
            std::printf("  [PASS] mode=%d unstick at tick %d (<=50)\n",
                        mode, unstick_at);
        else {
            std::printf("  [FAIL] mode=%d still saturated after 100 ticks\n",
                        mode);
            local_fail++;
        }
    }
    if (local_fail) g_failures += local_fail;
}

// ---------------------------------------------------------------------------
// 5. Reset mid-control
// ---------------------------------------------------------------------------
static void test_reset_midrun(Vpid_tb* dut) {
    std::printf("---- Reset mid-control ----\n");
    reset(dut);
    dut->kp_i         = to_q1616(0.5);
    dut->ki_ts_i      = to_q1616(0.05);
    dut->kd_over_ts_i = 0;
    dut->kb_i         = 0;
    dut->out_min_i    = -32768;
    dut->out_max_i    =  32767;
    dut->awu_mode_i   = 0;

    for (int n = 0; n < 100; n++) (void)pid_step(dut, 1000, 0);
    int16_t pre_reset_u = (int16_t)dut->u_o;
    // Pulse reset.
    dut->rst_ni = 0;
    for (int i = 0; i < 4; i++) tick_clk(dut);
    dut->rst_ni = 1;
    tick_clk(dut);
    int16_t u_after = pid_step(dut, 0, 0);

    if (u_after == 0)
        std::printf("  [PASS] post-reset output = 0 (was %d before reset)\n",
                    pre_reset_u);
    else {
        std::printf("  [FAIL] post-reset output = %d (expected 0)\n", u_after);
        g_failures++;
    }
}

// ---------------------------------------------------------------------------
// 6. Setpoint tracking with various combinations
// ---------------------------------------------------------------------------
static void test_setpoint_tracking(Vpid_tb* dut) {
    std::printf("---- Setpoint tracking (multiple gain sets) ----\n");
    int local_fail = 0;
    struct GainSet {
        const char* label;
        double kp, ki, kd;
        int    settle_within;
    } sets[] = {
        {"low-Kp",     0.3,  0.02, 0.0,  400},
        {"high-Kp",    1.5,  0.10, 0.0,  300},
        {"with-D",     0.8,  0.05, 0.05, 350},
    };
    for (auto& s : sets) {
        Plant plant;
        plant.a = std::exp(-1.0 / 10.0);
        plant.b = 1.0 * (1.0 - plant.a);
        plant.reset();
        reset(dut);
        dut->kp_i         = to_q1616(s.kp);
        dut->ki_ts_i      = to_q1616(s.ki);
        dut->kd_over_ts_i = to_q1616(s.kd);
        dut->kb_i         = 0;
        dut->out_min_i    = -32768;
        dut->out_max_i    =  32767;
        dut->awu_mode_i   = 0;

        int16_t pv = 0;
        int settle = -1;
        const int16_t SP = 2000;
        for (int n = 0; n < 800; n++) {
            int16_t u = pid_step(dut, SP, pv);
            pv = plant.advance(u);
            if (settle < 0 && std::abs((int)SP - (int)pv) <= 20) settle = n;
        }
        bool ok = (settle >= 0) && (settle <= s.settle_within);
        if (ok)
            std::printf("  [PASS %s] settle@%d/%d (1%% band)\n",
                        s.label, settle, s.settle_within);
        else {
            std::printf("  [FAIL %s] settle@%d/%d\n",
                        s.label, settle, s.settle_within);
            local_fail++;
        }
    }
    if (local_fail) g_failures += local_fail;
}

// ---------------------------------------------------------------------------
// Coverage report
// ---------------------------------------------------------------------------
static void report_coverage(const Vpid_tb* dut) {
    auto* cov = dut->rootp->pid_tb->u_cov;
    struct Bin { const char* name; uint8_t hit; };
    Bin bins[] = {
        {"P_only",         cov->c_p_only},
        {"PI",             cov->c_pi},
        {"PID",            cov->c_pid},
        {"saturated_high", cov->c_saturated_high},
        {"saturated_low",  cov->c_saturated_low},
        {"normal_op",      cov->c_normal_op},
        {"reset_mid_run",  cov->c_reset_mid_run},
        {"freeze_active",  cov->c_freeze_active},
    };
    int total = sizeof(bins) / sizeof(bins[0]);
    int hit = 0;
    std::printf("\n---- Functional coverage ----\n");
    for (int i = 0; i < total; i++) {
        std::printf("  [%s] %-22s\n", bins[i].hit ? "HIT " : "MISS",
                    bins[i].name);
        if (bins[i].hit) hit++;
    }
    std::printf("Coverage: %d/%d bins (%.1f%%)\n",
                hit, total, 100.0 * hit / total);
    if (hit < total) g_failures++;
}

// Drive a freeze-active tick once to seed the freeze coverage bin.
static void seed_freeze_cov(Vpid_tb* dut) {
    reset(dut);
    dut->kp_i      = to_q1616(1.0);
    dut->ki_ts_i   = to_q1616(0.05);
    dut->kd_over_ts_i = 0;
    dut->kb_i      = 0;
    dut->out_min_i = -32768;
    dut->out_max_i =  32767;
    dut->awu_mode_i = 0;
    (void)pid_step(dut, 100, 0, /*freeze=*/true);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    setvbuf(stdout, nullptr, _IOLBF, 0);
    Vpid_tb* dut = new Vpid_tb();
    reset(dut);

    test_reference_parity(dut);
    test_step_response(dut);
    test_output_clamp(dut);
    test_anti_windup(dut);
    test_reset_midrun(dut);
    test_setpoint_tracking(dut);
    seed_freeze_cov(dut);

    report_coverage(dut);

    if (g_failures == 0) {
        std::printf("+PASS all tests passed\n");
        delete dut;
        return 0;
    } else {
        std::printf("+FAIL %d failures\n", g_failures);
        delete dut;
        return 1;
    }
}
