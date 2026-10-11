#!/usr/bin/env python3
"""
calibrate_host.py -- host-side test runner + curve fitting for calibration_suite.ino

Flash calibration_suite.ino, then (wheels LIFTED for T0-T7):

    pip install pyserial numpy matplotlib
    python3 calibrate_host.py --port /dev/ttyUSB0 --tests all
    python3 calibrate_host.py --port /dev/ttyUSB0 --tests t2,t3 --motors A
    python3 calibrate_host.py --sim --yes --tests all --reps 1    # dry run, no hardware

Tests (each is open-loop; nothing here tunes a controller on the ESP32):
    t0  encoder CPR check by hand rotation
    t1  polarity / direction convention
    t2  deadband (static start + kinetic stall), per motor, per direction
    t3  steady-state PWM -> speed sweep (up+down), FF slope/offset, max speed
    t4  open-loop step response -> tau, delay, accel limit, PID starting gains
    t5  raw capture -> velocity-estimator noise vs window/EMA table
    t6  brake deceleration + reversal-interlock timing
    t7  left/right matching (both motors at the same PWM)

Results accumulate in <out>/results.json; raw captures are saved as CSV.
Re-run with --out <existing dir> to continue/append. T4 uses T3's FF slope
as the plant gain if T3 has already been run in the same --out directory.
Closed-loop validation (T8) and the loaded repeat (T9) need the real control
firmware, not this one.
"""

import argparse
import json
import os
import sys
import threading
import time
from bisect import bisect_left, bisect_right
from datetime import datetime

import numpy as np

try:
    import serial
except ImportError:  # only needed for real hardware
    serial = None

CPR = 5070.0
DEG_PER_COUNT = 360.0 / CPR
BAUD = 460800
MOTOR_NAMES = "AB"


# =====================================================================
# Rig: serial link to the ESP32 (+ a simulator with the same interface)
# =====================================================================
class Rig:
    is_sim = False

    def __init__(self, port, rate_hz=200):
        if serial is None:
            sys.exit("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        # pyserial on Windows shares ONE overlapped-write object per port, so two
        # threads writing at once can corrupt each other's result ("Write timeout").
        # Every write therefore goes through send() under this lock.
        self._wlock = threading.Lock()
        self.rows = []      # (t_us, cA, cB, pA, pB)
        self.tl = []        # t_us list, parallel to rows (for bisect)
        self.msgs = []
        self.cmd = [0, 0]
        self._wrap = 0
        self._last_raw = None
        self._alive = True
        self.boots = 0      # count of "CAL,v1,ready" banners seen (= ESP32 boots)
        self._boots0 = 0
        time.sleep(2.0)     # ESP32 resets when the port opens
        self.ser.reset_input_buffer()
        self.rate_hz = rate_hz
        threading.Thread(target=self._read_loop, daemon=True).start()
        self._handshake()
        self._boots0 = self.boots
        self.send(f"R {rate_hz}")
        self.send("T 1")
        threading.Thread(target=self._keepalive_loop, daemon=True).start()

    # ---- low level ----
    def _read_loop(self):
        while self._alive:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            if line.startswith("D,"):
                p = line.split(",")
                if len(p) != 6:
                    continue
                try:
                    t, a, b, pa, pb = (int(x) for x in p[1:])
                except ValueError:
                    continue
                if self._last_raw is not None and t < self._last_raw - 2**31:
                    self._wrap += 2**32          # micros() wrapped (~71 min)
                self._last_raw = t
                self.tl.append(t + self._wrap)
                self.rows.append((t + self._wrap, a, b, pa, pb))
            else:
                self.msgs.append(line)
                if line == "CAL,v1,ready":
                    self.boots += 1
                if line.startswith(("WATCHDOG", "INTERLOCK")):
                    print(f"  [esp32] {line}")

    def _keepalive_loop(self):
        while self._alive:
            time.sleep(0.3)
            try:
                self.send("K")
            except Exception:
                pass

    def _handshake(self):
        for _ in range(10):
            self.send("V")
            time.sleep(0.3)
            if any(m.startswith("CAL,") for m in self.msgs):
                return
        sys.exit("No handshake from ESP32. Is calibration_suite.ino flashed, and the port right?")

    def close(self):
        try:
            self.send("S")
        finally:
            self._alive = False
            time.sleep(0.1)
            self.ser.close()

    # ---- api used by tests ----
    def send(self, line):
        with self._wlock:
            self.ser.write((line + "\n").encode())

    def set_pwm(self, m, pwm):
        pwm = int(pwm)
        self.send(f"M {m} {pwm}")
        self.cmd[m] = pwm

    def stop(self):
        self.send("S")
        self.cmd = [0, 0]

    def now_us(self):
        return self.tl[-1] if self.tl else 0

    def counts(self):
        r = self.rows[-1]
        return r[1], r[2]

    def wait(self, sec):
        """Wait `sec` of DEVICE time (immune to host jitter)."""
        start = self.now_us()
        last_progress = time.time()
        last_t = start
        while self.now_us() - start < sec * 1e6:
            time.sleep(0.004)
            if self.boots > self._boots0:
                raise RuntimeError("ESP32 rebooted mid-test (boot banner seen) -- "
                                   "likely a power brownout when the motors drew current")
            t = self.now_us()
            if t != last_t:
                last_t, last_progress = t, time.time()
            elif time.time() - last_progress > 3.0:
                raise RuntimeError("encoder stream stalled (no data for 3 s)")

    def window(self, t0_us, t1_us):
        i, j = bisect_left(self.tl, t0_us), bisect_right(self.tl, t1_us)
        return np.array(self.rows[i:j], dtype=np.float64)

    def ask(self, text, sim_action=None):
        return input(text)


class SimRig(Rig):
    """Software stand-in for ESP32 + two motors, for dry-running the script.
    Parameters differ per motor so the fits have something to recover."""
    is_sim = True

    TRUE = [
        dict(dead_s=35.0, dead_k=24.0, off=25.1, slope=0.4062, tau=0.18, tau_b=0.30),
        dict(dead_s=33.0, dead_k=22.0, off=30.0, slope=0.4500, tau=0.22, tau_b=0.35),
    ]

    def __init__(self, rate_hz=200):
        self.rows, self.tl, self.msgs = [], [], []
        self.cmd = [0, 0]
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.t = 0.0
        self.v = [0.0, 0.0]
        self.c = [0.0, 0.0]
        self.applied = [0, 0]
        self.moving = [False, False]
        self.interlock = True
        self.rows.append((0, 0, 0, 0, 0))
        self.tl.append(0)

    def send(self, line):
        p = line.split()
        if not p:
            return
        if p[0] == "M":
            self.cmd[int(p[1])] = int(p[2])
        elif p[0] == "B":
            self.cmd = [int(p[1]), int(p[2])]
        elif p[0] == "S":
            self.cmd = [0, 0]
        elif p[0] == "Z":
            self.c = [0.0, 0.0]
        elif p[0] == "I":
            self.interlock = bool(int(p[1]))

    def hand(self, m, turns):
        self.c[m] += turns * CPR

    def close(self):
        pass

    def wait(self, sec):
        n = max(1, int(round(sec / self.dt)))
        for _ in range(n):
            self.t += self.dt
            for m in (0, 1):
                P = self.TRUE[m]
                u = self.cmd[m]
                if self.interlock and u != 0 and abs(self.v[m]) > 30 and (u > 0) != (self.v[m] > 0):
                    u = 0
                self.applied[m] = u
                a = abs(u)
                thresh = P["dead_k"] if self.moving[m] else P["dead_s"]
                if u == 0:
                    self.v[m] += -self.v[m] / P["tau_b"] * self.dt
                elif a < thresh:
                    self.v[m] += -self.v[m] / (0.5 * P["tau_b"]) * self.dt
                else:
                    vss = np.sign(u) * max(0.0, (a - P["off"]) / P["slope"])
                    self.v[m] += (vss - self.v[m]) / P["tau"] * self.dt
                self.moving[m] = abs(self.v[m]) > 5.0
                self.c[m] += self.v[m] * self.dt / DEG_PER_COUNT
            t_us = int(self.t * 1e6)
            self.rows.append((t_us, int(np.floor(self.c[0])), int(np.floor(self.c[1])),
                              self.applied[0], self.applied[1]))
            self.tl.append(t_us)

    def ask(self, text, sim_action=None):
        if sim_action:
            sim_action()
        print(f"  (sim) {text.strip()}")
        return "cw"


# =====================================================================
# Helpers
# =====================================================================
def speed(w, m):
    """Mean velocity (deg/s) of motor m over a window, by linear regression."""
    if len(w) < 3:
        return 0.0
    t = (w[:, 0] - w[0, 0]) * 1e-6
    return float(np.polyfit(t, w[:, 1 + m], 1)[0] * DEG_PER_COUNT)


def vel_series(w, m, win_s):
    """Sliding-window velocity estimate (deg/s) with window win_s."""
    t = w[:, 0] * 1e-6
    c = w[:, 1 + m]
    idx = np.searchsorted(t, t - win_s, side="left")
    ok = ((t - win_s) >= t[0]) & ((t - t[idx]) > 0)
    v = (c[ok] - c[idx[ok]]) / (t[ok] - t[idx[ok]]) * DEG_PER_COUNT
    return t[ok], v


def ramp(rig, m, target, rate=300.0, step=2):
    """Ramp PWM to target at `rate` PWM/s. Same-direction only: never crosses
    zero, so it can never plug the driver."""
    cur = rig.cmd[m]
    if cur * target < 0:
        raise ValueError("ramp() must not cross zero; use ensure_rest() first")
    sgn = 1 if target >= cur else -1
    while cur != target:
        cur = cur + sgn * min(step, abs(target - cur))
        rig.set_pwm(m, cur)
        rig.wait(step / rate)


def ensure_rest(rig, m, timeout=8.0):
    """Command stop and wait until the motor has actually stopped."""
    rig.cmd[m] = 0
    rig.send(f"M {m} 0")
    waited = 0.0
    while waited < timeout:
        rig.wait(0.3)
        waited += 0.3
        t1 = rig.now_us()
        w = rig.window(t1 - 300000, t1)
        if abs(speed(w, m)) < 2.0:
            rig.wait(0.2)
            return True
    print(f"  WARNING: motor {MOTOR_NAMES[m]} did not come to rest in {timeout}s")
    return False


def measure(rig, m, dur):
    t0 = rig.now_us()
    rig.wait(dur)
    return rig.window(t0, rig.now_us())


def fit_step(t_rel, v, tail_s=0.5):
    """First-order-plus-delay fit from a step response (Broida 28%/63% method).
    v must already be sign-corrected so the steady state is positive."""
    sel = t_rel >= 0
    t, v = t_rel[sel], v[sel]
    vss = float(np.mean(v[t > t.max() - tail_s]))
    if vss < 10.0:
        return None
    sm = np.convolve(v, np.ones(5) / 5, mode="same")

    def cross(frac):
        hit = np.nonzero(sm >= frac * vss)[0]
        if len(hit) == 0:
            return float("nan")
        i = hit[0]
        if i == 0:
            return float(t[0])
        y0, y1 = sm[i - 1], sm[i]
        f = (frac * vss - y0) / (y1 - y0) if y1 != y0 else 0.0
        return float(t[i - 1] + f * (t[i] - t[i - 1]))

    t28, t63 = cross(0.283), cross(0.632)
    if np.isnan(t28) or np.isnan(t63):
        return None
    tau = 1.5 * (t63 - t28)
    theta = t63 - tau
    if theta < 0:
        theta, tau = 0.0, t63
    return dict(v_ss=vss, tau=float(tau), delay=float(theta), t28=t28, t63=t63)


def save_csv(path, w, header="t_us,countA,countB,pwmA,pwmB"):
    np.savetxt(path, w, delimiter=",", header=header, comments="", fmt="%d")


def checkpoint(out, R, name, res):
    """Merge partial results into R and write results.json right away, so a
    crash or link drop loses at most the block that was running."""
    if not isinstance(R.get(name), dict):
        R[name] = {}
    R[name].update(res)
    with open(os.path.join(out, "results.json"), "w") as f:
        json.dump(R, f, indent=1, default=float)


def try_plot(out, name, draw):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    draw(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(out, name), dpi=120)
    plt.close(fig)


# =====================================================================
# Tests
# =====================================================================
def t0_cpr(rig, args, out, R):
    res = {}
    for m in args.motors:
        n = MOTOR_NAMES[m]
        rig.stop()
        rig.send("Z")
        rig.wait(0.3)
        rig.ask(f"\n[T0] Motor {n}: rotate the OUTPUT shaft exactly 10 full turns CLOCKWISE "
                f"by hand, then press Enter... ", lambda: rig.hand(m, 10))
        rig.wait(0.2)
        c_cw = rig.counts()[m]
        rig.ask(f"[T0] Motor {n}: now rotate exactly 10 turns COUNTER-clockwise "
                f"(back to the start mark), press Enter... ", lambda: rig.hand(m, -10))
        rig.wait(0.2)
        c_back = rig.counts()[m]
        cpr_est = abs(c_cw) / 10.0
        res[n] = dict(counts_10_turns_cw=int(c_cw), implied_cpr=cpr_est,
                      cpr_error_pct=100.0 * (cpr_est - CPR) / CPR,
                      return_residual_counts=int(c_back))
        print(f"  {n}: 10 turns = {c_cw} counts -> {cpr_est:.1f} CPR "
              f"({res[n]['cpr_error_pct']:+.2f}% vs {CPR:.0f}); "
              f"return-to-start residual = {c_back} counts")
    return res


def t1_polarity(rig, args, out, R):
    res = {}
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for d in (+1, -1):
            ensure_rest(rig, m)
            rig.send("Z")
            ramp(rig, m, d * 80)
            w = measure(rig, m, 2.0)
            sp = speed(w, m)
            ramp(rig, m, 0)
            sign = "+" if sp > 0 else "-"
            ans = ""
            if not args.no_prompts:
                ans = rig.ask(f"[T1] Motor {n}, PWM {d*80:+d}. Viewed from the shaft end, "
                              f"which way did it turn? [cw/ccw/skip] ", None).strip().lower()
            res[f"{n}{'+' if d > 0 else '-'}pwm"] = dict(
                count_sign=sign, speed_deg_s=sp, observed_direction=ans or "n/a")
            print(f"  {n}: PWM {d*80:+d} -> counts {sign}, {sp:+.1f} deg/s, observed {ans or 'n/a'}")
        ensure_rest(rig, m)
    return res


def t2_deadband(rig, args, out, R):
    res = {}
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for d in (+1, -1):
            stat, kin = [], []
            for trial in range(args.trials):
                ensure_rest(rig, m)
                found = None
                p = 0
                while p < args.dead_max:
                    p += 1
                    rig.set_pwm(m, d * p)
                    w = measure(rig, m, args.dead_step_s)
                    if abs(w[-1, 1 + m] - w[0, 1 + m]) >= args.dead_counts:
                        found = p
                        break
                if found is None:
                    print(f"  {n}{d:+d} trial {trial}: no motion up to PWM {args.dead_max}")
                    rig.stop()
                    continue
                # get well into motion, then step down until it stalls
                ramp(rig, m, d * (found + 12), rate=100)
                rig.wait(0.8)
                q = found + 12
                kmin = None
                while q > 0:
                    q -= 1
                    rig.set_pwm(m, d * q)
                    w = measure(rig, m, args.dead_step_s * 1.5)
                    if abs(w[-1, 1 + m] - w[0, 1 + m]) < args.dead_counts:
                        kmin = q + 1
                        break
                stat.append(found)
                if kmin is not None:
                    kin.append(kmin)
                rig.stop()
            key = f"{n}{'+' if d > 0 else '-'}"
            res[key] = dict(static_pwm=stat, kinetic_pwm=kin,
                            static_median=float(np.median(stat)) if stat else None,
                            static_max=max(stat) if stat else None,
                            kinetic_median=float(np.median(kin)) if kin else None,
                            kinetic_max=max(kin) if kin else None)
            print(f"  {key}: static start PWM median {res[key]['static_median']} "
                  f"(max {res[key]['static_max']}), kinetic stall PWM median "
                  f"{res[key]['kinetic_median']} (max {res[key]['kinetic_max']})")
            checkpoint(out, R, "t2", {key: res[key]})
        ensure_rest(rig, m)
    return res


def t3_sweep(rig, args, out, R):
    pts = list(range(args.sweep_lo, args.sweep_hi + 1, args.sweep_step))
    if pts[-1] != args.sweep_hi:
        pts.append(args.sweep_hi)
    seq = [(p, "up") for p in pts] + [(p, "down") for p in pts[-2::-1]]
    res = {}
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for d in (+1, -1):
            key = f"{n}{'+' if d > 0 else '-'}"
            prev = R.get("t3", {}).get(key, {})
            if args.resume and "ff_slope_pwm_per_deg_s" in prev:
                print(f"  {key}: already in results.json, skipping (--resume)")
                res[key] = prev
                continue
            data = []
            for rep in range(args.reps):
                ensure_rest(rig, m)
                for p, leg in seq:
                    ramp(rig, m, d * p, rate=args.ramp_rate)
                    rig.wait(args.settle)
                    w = measure(rig, m, args.meas)
                    data.append((p, d * speed(w, m), leg, rep))
                ramp(rig, m, 0, rate=args.ramp_rate)
            ensure_rest(rig, m)
            key = f"{n}{'+' if d > 0 else '-'}"
            arr = np.array([(p, s) for p, s, _, _ in data], dtype=float)
            legs = np.array([l for _, _, l, _ in data])
            good = arr[:, 1] > 5.0
            if good.sum() < 4:
                print(f"  {key}: too few moving points to fit")
                res[key] = dict(error="too few moving points", raw=arr.tolist())
                continue
            x, y = arr[good, 1], arr[good, 0]          # speed -> PWM (what FF needs)
            A = np.vstack([x, np.ones_like(x)]).T
            (slope, off), *_ = np.linalg.lstsq(A, y, rcond=None)
            rms = float(np.sqrt(np.mean((A @ [slope, off] - y) ** 2)))
            q = np.polyfit(x, y, 2)
            rms_q = float(np.sqrt(np.mean((np.polyval(q, x) - y) ** 2)))
            # hysteresis: speed at same PWM, up leg vs down leg
            hyst = []
            for p in pts:
                u = arr[(arr[:, 0] == p) & (legs == "up") & good, 1]
                dn = arr[(arr[:, 0] == p) & (legs == "down") & good, 1]
                if len(u) and len(dn):
                    hyst.append(np.mean(dn) - np.mean(u))
            vmax = float(np.mean(arr[arr[:, 0] == pts[-1], 1]))
            res[key] = dict(ff_slope_pwm_per_deg_s=float(slope), ff_offset_pwm=float(off),
                            rms_pwm=rms, rms_quadratic_pwm=rms_q,
                            hysteresis_deg_s_mean=float(np.mean(hyst)) if hyst else None,
                            v_max_deg_s=vmax, pwm_at_vmax=pts[-1], raw=arr.tolist())
            print(f"  {key}: PWM = {slope:.4f}*speed + {off:.2f}  (RMS {rms:.2f} PWM; "
                  f"quadratic {rms_q:.2f})  vmax@{pts[-1]} = {vmax:.0f} deg/s  "
                  f"hysteresis(down-up) = {res[key]['hysteresis_deg_s_mean']}")
            checkpoint(out, R, "t3", {key: res[key]})

            def draw(ax, arr=arr, good=good, slope=slope, off=off, key=key):
                ax.scatter(arr[:, 1], arr[:, 0], s=12, label="measured")
                xs = np.linspace(0, arr[:, 1].max(), 50)
                ax.plot(xs, slope * xs + off, "r-", label="linear fit")
                ax.set_xlabel("speed (deg/s)")
                ax.set_ylabel("PWM")
                ax.set_title(f"T3 {key}")
                ax.legend()
                ax.grid(alpha=0.3)
            try_plot(out, f"t3_{key}.png", draw)
    # asymmetry / combined hints
    for m in args.motors:
        n = MOTOR_NAMES[m]
        a, b = res.get(f"{n}+", {}), res.get(f"{n}-", {})
        if "ff_slope_pwm_per_deg_s" in a and "ff_slope_pwm_per_deg_s" in b:
            s1, s2 = a["ff_slope_pwm_per_deg_s"], b["ff_slope_pwm_per_deg_s"]
            asym = 100 * abs(s1 - s2) / max(s1, s2)
            print(f"  {n}: direction asymmetry in slope = {asym:.1f}%"
                  f"{'  <-- significant, consider per-direction FF' if asym > 5 else ''}")
    return res


def t4_step(rig, args, out, R):
    res = {}
    t3 = R.get("t3", {})
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for d in ((+1, -1) if args.both_dirs else (+1,)):
            for p in args.step_pwms:
                ensure_rest(rig, m)
                rig.wait(0.3)
                t_start = rig.now_us()
                rig.set_pwm(m, d * p)                    # true step
                rig.wait(args.step_time)
                w = rig.window(t_start, rig.now_us())
                rig.stop()
                save_csv(os.path.join(out, f"t4_raw_{n}{d:+d}_pwm{p}.csv"), w)
                applied = w[:, 3 + m]
                i0 = int(np.argmax(applied != 0))
                t_step = w[i0, 0] * 1e-6
                tt, vv = vel_series(w, m, 0.02)
                f = fit_step(tt - t_step, d * vv)
                key = f"{n}{'+' if d > 0 else '-'}_pwm{p}"
                if f is None:
                    print(f"  {key}: no usable step response (PWM below deadband?)")
                    res[key] = dict(error="no steady state reached")
                    continue
                f["k_eff_deg_s_per_pwm"] = f["v_ss"] / p
                f["a_initial_est_deg_s2"] = f["v_ss"] / max(f["tau"], 1e-3)
                slope = t3.get(f"{n}{'+' if d > 0 else '-'}", {}).get("ff_slope_pwm_per_deg_s")
                if slope:
                    K, Ksrc = 1.0 / slope, "T3 slope"
                else:
                    K, Ksrc = f["v_ss"] / p, "v_ss/PWM (run T3 for a better K)"
                lam = max(f["tau"], 0.05)
                kp = f["tau"] / (K * (lam + f["delay"]))
                f.update(K_deg_s_per_pwm=K, K_source=Ksrc, pid_kp_start=kp,
                         pid_ki_start=kp / max(f["tau"], 1e-3))
                res[key] = f
                print(f"  {key}: v_ss={f['v_ss']:.0f} deg/s  tau={f['tau']*1000:.0f} ms  "
                      f"delay={f['delay']*1000:.0f} ms  initial accel~{f['a_initial_est_deg_s2']:.0f} deg/s^2"
                      f"  -> IMC start: Kp={kp:.3f} Ki={f['pid_ki_start']:.3f} (K from {Ksrc})")
                checkpoint(out, R, "t4", {key: res[key]})

                def draw(ax, tt=tt, vv=vv, t_step=t_step, d=d, f=f, key=key):
                    ax.plot(tt - t_step, d * vv, lw=0.8)
                    ax.axhline(f["v_ss"], color="g", ls="--", lw=0.8)
                    ax.axvline(f["t63"], color="r", ls=":", lw=0.8)
                    ax.set_xlim(-0.1, args.step_time)
                    ax.set_xlabel("time since step (s)")
                    ax.set_ylabel("speed (deg/s)")
                    ax.set_title(f"T4 {key}  tau={f['tau']*1000:.0f} ms")
                    ax.grid(alpha=0.3)
                try_plot(out, f"t4_{key}.png", draw)
        ensure_rest(rig, m)
    return res


def t5_noise(rig, args, out, R):
    res = {}
    wins = [0.02, 0.05, 0.1, 0.2]
    alphas = [0.1, 0.2, 0.3, 0.5]
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for p in args.noise_pwms:
            ensure_rest(rig, m)
            ramp(rig, m, p)
            rig.wait(2.5)
            w = measure(rig, m, args.noise_dur)
            ramp(rig, m, 0)
            save_csv(os.path.join(out, f"t5_raw_{n}_pwm{p}.csv"), w)
            key = f"{n}_pwm{p}"
            entry = {}
            print(f"  {key}:")
            for ws in wins:
                t, v = vel_series(w, m, ws)
                mu, sd = float(np.mean(v)), float(np.std(v))
                entry[f"win_{int(ws*1000)}ms"] = dict(mean=mu, sd=sd, sd_pct=100 * sd / abs(mu) if mu else None,
                                                      resolution_deg_s=DEG_PER_COUNT / ws)
                print(f"    window {int(ws*1000):>3} ms: mean {mu:7.1f}  sd {sd:6.2f} deg/s "
                      f"({100*sd/abs(mu) if mu else float('nan'):4.1f}%)  "
                      f"1-count resolution {DEG_PER_COUNT/ws:5.2f} deg/s")
            # EMA on non-overlapping 50 ms samples (what the control loop would see)
            t, v = vel_series(w, m, 0.05)
            step = max(1, int(0.05 * args.rate))
            vs = v[::step]
            for a in alphas:
                y = np.empty_like(vs)
                y[0] = vs[0]
                for i in range(1, len(vs)):
                    y[i] = a * vs[i] + (1 - a) * y[i - 1]
                lag_ms = 50.0 * (1 - a) / a
                entry[f"ema_{a}_on_50ms"] = dict(sd=float(np.std(y[5:])), lag_ms=lag_ms)
                print(f"    50 ms window + EMA alpha={a}: sd {np.std(y[5:]):6.2f} deg/s, "
                      f"added lag ~{lag_ms:.0f} ms")
            res[key] = entry
            checkpoint(out, R, "t5", {key: entry})
        ensure_rest(rig, m)
    return res


def t6_brake(rig, args, out, R):
    res = {}
    for m in args.motors:
        n = MOTOR_NAMES[m]
        for p in args.brake_pwms:
            ensure_rest(rig, m)
            ramp(rig, m, p)
            rig.wait(2.5)
            v0 = speed(measure(rig, m, 1.0), m)
            t0 = rig.now_us()
            rig.set_pwm(m, 0)                      # PWM 0 = short-brake
            rig.wait(2.5)
            w = rig.window(t0 - 100000, rig.now_us())
            save_csv(os.path.join(out, f"t6_brake_{n}_pwm{p}.csv"), w)
            tt, vv = vel_series(w, m, 0.02)
            tt = tt - t0 * 1e-6
            sel = tt >= 0
            tt, vv = tt[sel], vv[sel]

            def first_below(level):
                h = np.nonzero(vv <= level)[0]
                return float(tt[h[0]]) if len(h) else float("nan")
            t90, t20 = first_below(0.9 * v0), first_below(0.2 * v0)
            t_safe = first_below(args.interlock_degs)
            a_mean = 0.7 * v0 / (t20 - t90) if t20 > t90 else float("nan")
            key = f"{n}_pwm{p}"
            res[key] = dict(v0=v0, brake_decel_mean_deg_s2=a_mean,
                            time_to_interlock_speed_s=t_safe, interlock_speed=args.interlock_degs)
            print(f"  {key}: v0={v0:.0f} deg/s  mean brake decel (90%->20%) = {a_mean:.0f} deg/s^2  "
                  f"time to <{args.interlock_degs:.0f} deg/s = {t_safe*1000:.0f} ms")
            checkpoint(out, R, "t6", {key: res[key]})
        ensure_rest(rig, m)
    # reversal interlock timing (interlock ON in firmware; safe)
    rig.send("I 1")
    for m in args.motors:
        n = MOTOR_NAMES[m]
        p = args.brake_pwms[0]
        ensure_rest(rig, m)
        ramp(rig, m, p)
        rig.wait(2.5)
        t0 = rig.now_us()
        rig.set_pwm(m, -p)                          # opposite-direction command, interlock must hold it
        rig.wait(2.0)
        w = rig.window(t0 - 50000, rig.now_us())
        rig.stop()
        applied = w[:, 3 + m]
        tt = (w[:, 0] - t0) * 1e-6
        nz = np.nonzero((applied < 0) & (tt >= 0))[0]
        t_release = float(tt[nz[0]]) if len(nz) else float("nan")
        res[f"{n}_reversal"] = dict(pwm=p, interlock_hold_s=t_release)
        print(f"  {n}: reverse command at speed -> driver held at 0 for {t_release*1000:.0f} ms "
              f"before opposite drive was released")
        ensure_rest(rig, m)
    return res


def t7_matching(rig, args, out, R):
    res = {}
    rows = []
    for p in args.match_pwms:
        rig.stop()
        rig.wait(1.0)
        ensure_rest(rig, 0)
        ensure_rest(rig, 1)
        rig.send(f"B {p} {p}")
        rig.cmd = [p, p]
        rig.wait(2.5)
        w = measure(rig, 0, 2.0)
        sa, sb = speed(w, 0), speed(w, 1)
        rows.append((p, sa, sb, sa / sb if sb else float("nan")))
        print(f"  PWM {p}: A={sa:.1f} deg/s  B={sb:.1f} deg/s  A/B = {sa/sb:.3f}")
        rig.stop()
    res["rows_pwm_A_B_ratio"] = rows
    return res


TESTS = [("t0", t0_cpr), ("t1", t1_polarity), ("t2", t2_deadband), ("t3", t3_sweep),
         ("t4", t4_step), ("t5", t5_noise), ("t6", t6_brake), ("t7", t7_matching)]


# =====================================================================
# Summary
# =====================================================================
def summary(R):
    print("\n" + "=" * 70)
    print("SUMMARY (candidate firmware constants -- check the plots/raw data first)")
    print("=" * 70)
    t3 = R.get("t3", {})
    for n in MOTOR_NAMES:
        p, m = t3.get(f"{n}+", {}), t3.get(f"{n}-", {})
        if "ff_slope_pwm_per_deg_s" in p and "ff_slope_pwm_per_deg_s" in m:
            s = 0.5 * (p["ff_slope_pwm_per_deg_s"] + m["ff_slope_pwm_per_deg_s"])
            o = 0.5 * (p["ff_offset_pwm"] + m["ff_offset_pwm"])
            print(f"double FF_SLOPE_{n}  = {s:.4f};   // +dir {p['ff_slope_pwm_per_deg_s']:.4f}, "
                  f"-dir {m['ff_slope_pwm_per_deg_s']:.4f}")
            print(f"double FF_OFFSET_{n} = {o:.2f};   // +dir {p['ff_offset_pwm']:.2f}, "
                  f"-dir {m['ff_offset_pwm']:.2f}")
        elif "ff_slope_pwm_per_deg_s" in p:
            print(f"double FF_SLOPE_{n}  = {p['ff_slope_pwm_per_deg_s']:.4f};  "
                  f"FF_OFFSET_{n} = {p['ff_offset_pwm']:.2f};   // + direction only")
    t2 = R.get("t2", {})
    allmax = [v["static_max"] for v in t2.values() if v.get("static_max")]
    if allmax:
        print(f"// worst-case static deadband over all motors/directions: PWM {max(allmax)}")
    t4 = R.get("t4", {})
    for k, v in t4.items():
        if "tau" in v:
            print(f"// {k}: tau={v['tau']*1000:.0f} ms, delay={v['delay']*1000:.0f} ms, "
                  f"accel~{v['a_initial_est_deg_s2']:.0f} deg/s^2, IMC start Kp={v['pid_kp_start']:.3f} "
                  f"Ki={v['pid_ki_start']:.3f}")
    t6 = R.get("t6", {})
    for k, v in t6.items():
        if "brake_decel_mean_deg_s2" in v:
            print(f"// {k}: brake decel ~{v['brake_decel_mean_deg_s2']:.0f} deg/s^2")


# =====================================================================
# Main
# =====================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--sim", action="store_true", help="dry-run against a software motor model")
    ap.add_argument("--tests", default="all", help="comma list of t0..t7 or 'all'")
    ap.add_argument("--motors", default="AB", help="A, B or AB")
    ap.add_argument("--out", default=None, help="output dir (reuse one to continue a session)")
    ap.add_argument("--yes", action="store_true", help="skip the wheels-lifted confirmation")
    ap.add_argument("--no-prompts", action="store_true", help="skip T1 visual-direction prompts")
    ap.add_argument("--resume", action="store_true",
                    help="T3: skip motor/direction blocks that already have a fit in results.json")
    ap.add_argument("--rate", type=int, default=200, help="stream rate Hz (max 500)")
    # t2
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--dead-max", type=int, default=120)
    ap.add_argument("--dead-step-s", type=float, default=0.2)
    ap.add_argument("--dead-counts", type=int, default=3)
    # t3
    ap.add_argument("--sweep-lo", type=int, default=40)
    ap.add_argument("--sweep-hi", type=int, default=230)
    ap.add_argument("--sweep-step", type=int, default=15)
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--settle", type=float, default=2.0, help="settle time per point (s); use ~5*tau from T4")
    ap.add_argument("--meas", type=float, default=2.0)
    ap.add_argument("--ramp-rate", type=float, default=300.0, help="PWM/s used between points")
    # t4
    ap.add_argument("--step-pwms", type=int, nargs="+", default=[80, 120, 160])
    ap.add_argument("--step-time", type=float, default=2.5)
    ap.add_argument("--both-dirs", action="store_true")
    # t5
    ap.add_argument("--noise-pwms", type=int, nargs="+", default=[45, 100, 200])
    ap.add_argument("--noise-dur", type=float, default=10.0)
    # t6
    ap.add_argument("--brake-pwms", type=int, nargs="+", default=[100, 160, 220])
    ap.add_argument("--interlock-degs", type=float, default=30.0)
    # t7
    ap.add_argument("--match-pwms", type=int, nargs="+", default=[60, 100, 150, 200])
    args = ap.parse_args()

    args.motors = [MOTOR_NAMES.index(c) for c in args.motors.upper() if c in MOTOR_NAMES]
    wanted = [t for t, _ in TESTS] if args.tests == "all" else [t.strip() for t in args.tests.split(",")]

    out = args.out or os.path.join("calib_out", datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    rpath = os.path.join(out, "results.json")
    R = json.load(open(rpath)) if os.path.exists(rpath) else {}

    if not args.sim and not args.yes:
        ans = input("Wheels LIFTED off the ground and the motor supply ON? Type 'yes' to continue: ")
        if ans.strip().lower() != "yes":
            sys.exit("Aborted.")

    rig = SimRig(args.rate) if args.sim else Rig(args.port, args.rate)
    if args.sim:
        args.no_prompts = True
    failed = False
    try:
        for name, fn in TESTS:
            if name not in wanted:
                continue
            print(f"\n=== {name.upper()}: {fn.__doc__ or fn.__name__} ===")
            t_begin = time.time()
            res = fn(rig, args, out, R)
            if isinstance(res, dict) and isinstance(R.get(name), dict):
                R[name].update(res)      # keep earlier entries (e.g. the other motor)
            else:
                R[name] = res
            with open(rpath, "w") as f:
                json.dump(R, f, indent=1, default=float)
            print(f"  ({name} done in {time.time()-t_begin:.0f}s wall, results saved)")
    except KeyboardInterrupt:
        print("\nInterrupted -- stopping motors.")
    except Exception as e:
        failed = True
        print(f"\nTEST ABORTED: {type(e).__name__}: {e}")
        print("A serial operation failed (USB drop, ESP32 reset, or port contention). "
              "The firmware watchdog stops the motors within 1 s.")
        print(f"Blocks finished before this are saved in {rpath}. "
              f"Fix the cause, then re-run the same command with --resume (T3).")
    finally:
        try:
            rig.stop()
        except Exception:
            pass
        try:
            rig.close()
        except Exception:
            pass
    summary(R)
    print(f"\nAll data in: {out}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
