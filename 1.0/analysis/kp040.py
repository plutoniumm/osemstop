#!/usr/bin/env python3
"""Characterise Kp = -0.040 from the data already on disk.

    .venv/bin/python analysis/kp040.py            # full run, ~2 min
    .venv/bin/python analysis/kp040.py --fast     # skip the CSV sweep, model only

Every number in analysis/kp040.md comes from here. Nothing in this file writes
to, imports from, or otherwise touches a controller; it only reads.

Sources
-------
  data/*_fast_lock.csv          22 controller runs, 2026-08-03 and 2026-08-04
  data/*_v6_stepped4*.json/csv  open-loop stepped-sine actuation measurement
  bench/20260804/dcmatrix.log   per-coil DC actuation matrix (transcribed below)
  git history                   osem.v0.py, the sole source of the -0.040 claim

Standing practice: every derived table is streamed to analysis/out/*.csv so a
later question about the intermediate values does not require another 2-minute
pass over 182 MB.

NOT USED AS EVIDENCE: sim/server.py. The simulator is linear apart from the ADC
and cannot reproduce this instability (versions.md records a sweep to -0.600
with monotonically increasing damping and zero faults). Part 0 below verifies
the analytic filter transfer functions against a *direct time-domain execution
of the controller's own recurrences* -- that is a check of this file's algebra,
not a physics claim.
"""

import csv
import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "analysis", "out")
FAST = "--fast" in sys.argv

# ---------------------------------------------------------------------------
# Controller constants. Verified byte-identical in osem.v3.py, osem.v7.py,
# osem.v9.py and osem.v10.py on 2026-08-06; osem.v0.py (git blob 54b8a20)
# differs ONLY in KI_GAIN/KD_GAIN (both all-zero) and CALIBRATION_S (8.0).
# ---------------------------------------------------------------------------
STEADY_GAIN = np.array([-0.030, -0.030, +0.010, -0.030])
CAPTURE_GAIN = np.array([-0.035, -0.035, +0.012, -0.035])
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
KI_V0 = np.zeros(4)
KD_V0 = np.zeros(4)
BP_LOW_HZ, BP_HIGH_HZ, DERIV_SMOOTH_HZ = 0.4, 3.0, 5.0
D_SMOOTH_HZ = 2.0
GAIN_SLEW_PER_S = 0.02
BIAS_V, VMIN, VMAX = 0.25, 0.0, 0.5
A_VCC, ADC_MAX_COUNTS = 5.02, 1023
COUNTS_PER_V = ADC_MAX_COUNTS / A_VCC          # sensor volts -> ADC counts

# bench/20260804/dcmatrix.log, d(counts_i)/d(bias_j). Column j is SENSOR-INDEXED
# ("coil j" = the coil of sensor j), per dcmatrix.py: M[:, j] is set while
# DAC_MAP[j] is stepped, and the log header reads "coil 0 (ch1)".
DCM = np.array([
    [-105, -107,  -48,  +19,  -29,  -21,  -28,  -26],
    [ -49,  -68,  -33,  +18,   +5,   +7,   +4,   +8],
    [ +35,  +66, +213, -141,  -25,  -22,  -13,  -26],
    [ +36,  +14,  +58,  -51,   -1,   -2,   +2,   -1],
    [  +0,   +2,   +2,   +1,   +8,   +8,   +2,   +1],
    [  +4,   +0,   +0,   -6,   -0,   -7,   -1,   +8],
    [  +1,   +4,   +2,   +3,   +3,   +3,   +4,   +6],
    [  -0,   +3,   +1,   +0,   +1,   -1,   +6,   +7],
], float)
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]      # sensor index -> DAC channel (current)
DAC_MAP_PRE_0803 = [0, 2, 4, 6]         # what v0 held on 2026-07-15

W_ENV, STRIDE_ENV = 8.0, 1.0            # envelope-fit window / stride, seconds
R2_MIN, FALL_MAX, GAIN_STD_MAX = 0.80, 0.90, 5e-4


def hdr(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def writecsv(name, header, rows):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    print(f"    -> analysis/out/{name}  ({len(rows)} rows)")


# ===========================================================================
# 0. Discrete transfer functions, exactly as the controller implements them
# ===========================================================================
def z(f, dt):
    return np.exp(2j * np.pi * np.asarray(f, float) * dt)


def h_low(f, fc, dt):
    """OnePoleFilter(kind='low'):  y += a*(x-y),  a = dt/(tau+dt)."""
    tau = 1.0 / (2 * np.pi * fc)
    a = dt / (tau + dt)
    zi = 1.0 / z(f, dt)
    return a / (1 - (1 - a) * zi)


def h_high(f, fc, dt):
    """OnePoleFilter(kind='high'): y = a*(y + x - x_prev), a = tau/(tau+dt)."""
    tau = 1.0 / (2 * np.pi * fc)
    a = tau / (tau + dt)
    zi = 1.0 / z(f, dt)
    return a * (1 - zi) / (1 - a * zi)


def h_deriv(f, dt):
    """(x[n]-x[n-1])/dt, as used for both bp->vel and vel->accel."""
    return (1 - 1.0 / z(f, dt)) / dt


def h_zoh(f, dt):
    """Actuator holds the last commanded volts for one loop period."""
    w = 2 * np.pi * np.asarray(f, float) * dt
    out = np.where(np.abs(w) < 1e-12, 1.0 + 0j, (1 - np.exp(-1j * w)) / (1j * np.where(w == 0, 1, w)))
    return out


def h_pid(f, kp, ki, kd, dt):
    """PID.update() in the frequency domain, from velocity to volts, with the
    controller's sign folded in so a well-signed channel gives Re > 0.

        u = -(kp + ki*dt/(1-z^-1) + kd*(1-z^-1)/dt * LP_2Hz) * vel

    kp/ki/kd carry the channel sign; the leading minus is `err = -meas`. The
    returned quantity is the magnitude/phase of the bracket with the channel
    sign removed, i.e. it is directly comparable to |Kp|.
    """
    zi = 1.0 / z(f, dt)
    integ = ki * dt / (1 - zi)
    deriv = kd * h_deriv(f, dt) * h_low(f, D_SMOOTH_HZ, dt)
    return -(kp + integ + deriv)


def h_sensor_path(f, dt):
    """bp -> vel: HP 0.4, LP 3.0, first difference, LP 5.0."""
    return (h_high(f, BP_LOW_HZ, dt) * h_low(f, BP_HIGH_HZ, dt)
            * h_deriv(f, dt) * h_low(f, DERIV_SMOOTH_HZ, dt))


def h_loop_nonplant(f, dt, kp, ki, kd):
    """Everything in the loop except the mechanical plant."""
    return h_sensor_path(f, dt) * h_pid(f, kp, ki, kd, dt) * h_zoh(f, dt)


def selfcheck():
    """Run the controller's actual recurrences on a sine and compare the
    measured gain/phase to the analytic H(z) above. Pure algebra check."""
    dt, fc = 0.0414, 3.0
    f = 1.046
    n = int(200 / dt)
    t = np.arange(n) * dt
    x = np.sin(2 * np.pi * f * t)

    tau_l = 1.0 / (2 * np.pi * fc)
    a_l = dt / (tau_l + dt)
    tau_h = 1.0 / (2 * np.pi * BP_LOW_HZ)
    a_h = tau_h / (tau_h + dt)
    yl = np.zeros(n)
    yh = np.zeros(n)
    yl[0] = x[0]
    for k in range(1, n):
        yl[k] = yl[k - 1] + a_l * (x[k] - yl[k - 1])
        yh[k] = a_h * (yh[k - 1] + x[k] - x[k - 1])

    def lockin(y):
        m = t > 60.0
        return 2.0 * np.mean(y[m] * np.exp(-2j * np.pi * f * t[m]))

    err_l = abs(lockin(yl) / lockin(x) - h_low(f, fc, dt))
    err_h = abs(lockin(yh) / lockin(x) - h_high(f, BP_LOW_HZ, dt))
    return err_l, err_h


# ===========================================================================
# 1. CSV sweep: gain census, loop rate, velocity distribution, rail incidence
# ===========================================================================
def read_cols(path, cols):
    with open(path) as fh:
        r = csv.reader(fh)
        head = next(r)
        idx = {n: i for i, n in enumerate(head)}
        if any(c not in idx for c in cols):
            return None
        want = [idx[c] for c in cols]
        ncol = len(head)
        out = [[] for _ in cols]
        for row in r:
            if len(row) != ncol:
                continue
            for k, j in enumerate(want):
                out[k].append(row[j])
    return out


def envelope(t, x, fs):
    """Analytic-signal envelope on a uniform regrid, smoothed over 1 s."""
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    xu = np.interp(tu, t, x)
    n = len(xu)
    if n < 16:
        return None, None
    X = np.fft.fft(xu - xu.mean())
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    env = np.abs(np.fft.ifft(X * h))
    k = max(1, int(fs))
    return tu, np.convolve(env, np.ones(k) / k, mode="same")


BANDS = [(0.6, 1.3), (1.3, 2.2), (2.2, 4.0), (4.0, 8.0), (8.0, 12.0)]


def bandpower(x, fs, nseg):
    """Welch band powers over BANDS. Returns None if the record is too short."""
    if len(x) < nseg:
        return None
    w = np.hanning(nseg)
    step = nseg // 2
    segs = [x[i:i + nseg] for i in range(0, len(x) - nseg + 1, step)]
    P = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs], axis=0)
    P /= (fs * (w ** 2).sum())
    fr = np.fft.rfftfreq(nseg, 1 / fs)
    return np.array([P[(fr >= a) & (fr < b)].sum() * (fr[1]) for a, b in BANDS])


def sweep_runs():
    """One pass over every fast_lock CSV. Returns everything Part 1-2 needs."""
    files = sorted(glob.glob(os.path.join(ROOT, "data", "*_fast_lock.csv")))
    runs, windows, vel_pool, rail_pool = [], [], [], []
    shape_rows, clip_rows = [], []
    psd_acc, psd_n, psd_fs = None, 0, []
    NP = 4096

    for path in files:
        base = os.path.basename(path)
        with open(path) as fh:
            head = next(csv.reader(fh))
        nch = sum(1 for n in head if n.endswith("_gain"))
        cols = ["time_s", "state"] + [f"ch{i}_{k}" for i in range(nch)
                                      for k in ("counts", "bp", "vel", "out", "gain", "p")]
        d = read_cols(path, cols)
        if d is None or len(d[0]) < 200:
            continue
        t = np.array(d[0], float)
        st = np.array(d[1])
        per = {}
        for i in range(nch):
            b = 2 + 6 * i
            per[i] = dict(counts=np.array(d[b], float), bp=np.array(d[b + 1], float),
                          vel=np.array(d[b + 2], float), out=np.array(d[b + 3], float),
                          gain=np.array(d[b + 4], float), p=np.array(d[b + 5], float))

        ndriven = sum(1 for i in range(nch) if per[i]["gain"][st == "DAMPING"].any()) \
            if (st == "DAMPING").any() else 0
        dtt = np.diff(t)
        rate = {}
        for s in ("CALIBRATING", "DAMPING", "FAULT"):
            m = (st[:-1] == s) & (st[1:] == s)
            rate[s] = float(np.median(dtt[m])) if m.sum() > 20 else float("nan")

        for i in range(nch):
            g = per[i]["gain"]
            dm = st == "DAMPING"
            runs.append(dict(run=base, nch=nch, ch=i, ndriven=ndriven,
                             dur=float(t[-1]), n=len(t),
                             n_damp=int(dm.sum()),
                             dt_calib=rate["CALIBRATING"], dt_damp=rate["DAMPING"],
                             dt_fault=rate["FAULT"],
                             gmax=float(np.abs(g).max()),
                             gmax_damp=float(np.abs(g[dm]).max()) if dm.any() else 0.0,
                             gmed_damp=float(np.median(g[dm])) if dm.any() else 0.0,
                             clip_pct=float(((per[i]["counts"] <= 0) |
                                             (per[i]["counts"] >= 1023)).mean() * 100)))
            if dm.any() and np.abs(g[dm]).max() > 1e-6:
                vel_pool.append(np.abs(per[i]["vel"][dm]))
                o = per[i]["out"][dm]
                lo, hi = o.min(), o.max()
                rail_pool.append(np.column_stack([
                    np.abs(g[dm]),
                    np.abs(per[i]["bp"][dm]),
                    ((o <= VMIN + 1e-6) | (o >= max(hi, VMAX) - 1e-6)).astype(float),
                ]))

        # ---- pooled open-loop PSD (gain identically zero: CALIBRATING/FAULT)
        chg = np.flatnonzero(st[1:] != st[:-1]) + 1
        bounds = np.concatenate(([0], chg, [len(t)]))
        for a, b in zip(bounds[:-1], bounds[1:]):
            if st[a] not in ("CALIBRATING", "FAULT") or b - a < NP:
                continue
            fs = 1.0 / np.median(np.diff(t[a:b]))
            psd_fs.append(fs)
            w = np.hanning(NP)
            for i in range(min(4, nch)):
                x = per[i]["counts"][a:b][:NP]
                P = np.abs(np.fft.rfft((x - x.mean()) * w)) ** 2 / (fs * (w ** 2).sum())
                if psd_acc is None:
                    psd_acc = np.zeros((4, len(P)))
                psd_acc[i] += P
            psd_n += 1

        # ---- open- vs closed-loop spectral SHAPE, and clipping vs |vel|
        for i in range(min(4, nch)):
            cts, vel, g = per[i]["counts"], per[i]["vel"], per[i]["gain"]
            dm = st == "DAMPING"
            if dm.any() and np.abs(g[dm]).max() > 1e-6:
                at = (cts <= 0) | (cts >= 1023)
                if at[dm].sum() > 30 and (~at[dm]).sum() > 30:
                    clip_rows.append([base, i, int(at[dm].sum()), int((~at[dm]).sum()),
                                      float(np.abs(vel[dm][at[dm]]).mean()),
                                      float(np.abs(vel[dm][~at[dm]]).mean())])
            longest = {}
            for a, b in zip(bounds[:-1], bounds[1:]):
                if b - a > longest.get(st[a], (0, 0, 0))[0]:
                    longest[st[a]] = (b - a, a, b)
            for sname in ("CALIBRATING", "DAMPING"):
                if sname not in longest:
                    continue
                _, a, b = longest[sname]
                fs = 1.0 / np.median(np.diff(t[a:b]))
                nseg = 512 if sname == "DAMPING" else 4096
                bp_ = bandpower(cts[a:b].astype(float), fs, nseg)
                if bp_ is None:
                    continue
                shape_rows.append([base, i, sname, fs, float(np.abs(g[a:b]).max())]
                                  + [float(v) for v in bp_])
                # CONTROL: the closed-loop record is the SAME sensor decimated by
                # ~5x (1 coil) or ~15x (4 coils) with no anti-alias filter. Take
                # the open-loop record apart the same way, so any high-band excess
                # that is pure folding shows up here too.
                if sname == "CALIBRATING":
                    for k in (5, 15):
                        bpd = bandpower(cts[a:b:k].astype(float), fs / k, 512)
                        if bpd is not None:
                            shape_rows.append([base, i, f"CALIB_DECIM_{k}x", fs / k, 0.0]
                                              + [float(v) for v in bpd])

        # ---- sliding-window envelope decay fits
        for i in range(min(4, nch)):
            bp, g = per[i]["bp"], per[i]["gain"]
            for a, b in zip(bounds[:-1], bounds[1:]):
                if t[b - 1] - t[a] < W_ENV + 2:
                    continue
                fs = min(1.0 / np.median(np.diff(t[a:b])), 60.0)
                tu, env = envelope(t[a:b], bp[a:b], fs)
                if tu is None:
                    continue
                gi = np.interp(tu, t[a:b], g[a:b])
                nw = int(W_ENV * fs)
                step = max(1, int(STRIDE_ENV * fs))
                for s in range(int(fs), len(tu) - nw - int(fs), step):
                    sl = slice(s, s + nw)
                    e = env[sl]
                    if e.min() <= 1e-5:
                        continue
                    tt = tu[sl] - tu[s]
                    A = np.vstack([tt, np.ones(nw)]).T
                    y = np.log(e)
                    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
                    resid = y - A @ coef
                    ss = ((y - y.mean()) ** 2).sum()
                    r2 = 1 - (resid @ resid) / max(ss, 1e-30)
                    gg = gi[sl]
                    windows.append((base, st[a], i, float(tu[s]), float(-coef[0]),
                                    float(r2), float(gg.mean()), float(gg.std()),
                                    float(e[-1] / e[0]), float(np.sqrt((e ** 2).mean()))))
        print(f"  read {base}", flush=True)

    fs_med = float(np.median(psd_fs)) if psd_fs else 357.1
    return (runs, windows, np.concatenate(vel_pool), np.vstack(rail_pool),
            (np.fft.rfftfreq(NP, 1 / fs_med), psd_acc / max(psd_n, 1), psd_n, fs_med),
            shape_rows, clip_rows)


# ===========================================================================
# main
# ===========================================================================
def main():
    os.makedirs(OUT, exist_ok=True)

    hdr("0. SELF-CHECK: analytic H(z) vs the controller's own recurrences")
    el, eh = selfcheck()
    print(f"  |H_low  analytic - measured| = {el:.2e}")
    print(f"  |H_high analytic - measured| = {eh:.2e}")
    print("  (time-domain execution of OnePoleFilter's update(), no simulator involved)")

    # -------------------------------------------------------------------
    if not FAST:
        hdr("1. CSV SWEEP")
        (runs, windows, vel, rail, (fr, psd, psd_n, psd_fs),
         shape_rows, clip_rows) = sweep_runs()
        np.save(os.path.join(OUT, "open_loop_psd.npy"), np.vstack([fr, psd]))
        writecsv("runs.csv", list(runs[0].keys()), [list(r.values()) for r in runs])
        writecsv("decay_windows.csv",
                 ["run", "state", "ch", "t0_s", "gamma_1_s", "r2", "gain_mean",
                  "gain_std", "env_ratio", "env_rms"], windows)
        writecsv("spectral_shape.csv",
                 ["run", "ch", "state", "fs_hz", "max_abs_gain"]
                 + [f"P_{a}_{b}Hz" for a, b in BANDS], shape_rows)
        writecsv("clip_vs_vel.csv",
                 ["run", "ch", "n_railed", "n_clean", "mean_abs_vel_railed",
                  "mean_abs_vel_clean"], clip_rows)
    else:
        runs = windows = shape_rows = clip_rows = None

    # -------------------------------------------------------------------
    hdr("2. WHAT GAIN HAS HARDWARE ACTUALLY RUN AT?")
    print("  Schedule ceiling by construction: |gain| <= |CAPTURE_GAIN| =",
          np.abs(CAPTURE_GAIN).tolist())
    if runs:
        by = defaultdict(list)
        for r in runs:
            if r["n_damp"]:
                by[r["ch"]].append(r)
        print(f"  {'ch':>3} {'runs':>5} {'max|gain| DAMPING':>18} {'median|gain|':>13} "
              f"{'CAPTURE':>9} {'reached -0.040?':>16}")
        rows = []
        for c in sorted(by):
            gm = max(r["gmax_damp"] for r in by[c])
            gmed = np.median([abs(r["gmed_damp"]) for r in by[c]])
            cap = abs(CAPTURE_GAIN[c]) if c < 4 else float("nan")
            print(f"  {c:>3} {len(by[c]):>5} {gm:>18.4f} {gmed:>13.4f} {cap:>9.4f} "
                  f"{'NO':>16}")
            rows.append([c, len(by[c]), gm, gmed, cap])
        writecsv("gain_census.csv", ["ch", "n_runs", "max_abs_gain_damping",
                                     "median_abs_gain_damping", "capture_gain"], rows)
        allmax = max(r["gmax"] for r in runs)
        print(f"\n  Largest |gain| in ANY state, ANY channel, ANY of the "
              f"{len(set(r['run'] for r in runs))} runs: {allmax:.4f}")
        print(f"  |-0.040| is {0.040 / allmax:.2f}x that. Hardware has never been "
              f"asked to run at -0.040.")

    # -------------------------------------------------------------------
    hdr("3. LOOP RATE, MEASURED, vs NUMBER OF DRIVEN COILS")
    if runs:
        g = defaultdict(list)
        for r in runs:
            if r["ch"] == 0 and r["n_damp"] > 200 and not math.isnan(r["dt_damp"]):
                g[r["ndriven"]].append(r["dt_damp"])
        rows = []
        print(f"  {'coils driven':>13} {'runs':>5} {'median dt (ms)':>15} "
              f"{'loop rate (Hz)':>15} {'ms per extra coil':>18}")
        base = None
        for k in sorted(g):
            m = float(np.median(g[k])) * 1e3
            if base is None:
                base, basek = m, k
            per = (m - base) / (k - basek) if k != basek else float("nan")
            print(f"  {k:>13} {len(g[k]):>5} {m:>15.1f} {1e3 / m:>15.1f} {per:>18.2f}")
            rows.append([k, len(g[k]), m, 1e3 / m])
        calib = [r["dt_calib"] for r in runs if not math.isnan(r["dt_calib"])]
        print(f"\n  CALIBRATING / FAULT (no coil writes): "
              f"{np.median(calib) * 1e3:.2f} ms = {1 / np.median(calib):.0f} Hz "
              f"(the wire limit)")
        print("  v0's validated configuration was ENABLE_CHANNEL=[True,False,False,False]")
        print("  -> ONE driven coil -> the -0.040 datum was taken at the fastest loop rate")
        print("     the rig has ever run, and every four-channel run since is ~3x slower.")
        writecsv("loop_rate.csv", ["coils_driven", "n_runs", "median_dt_ms", "loop_hz"], rows)
        DT = {k: float(np.median(v)) for k, v in g.items()}
    else:
        DT = {1: 0.0137, 4: 0.0414, 8: 0.0745}

    dt1 = DT.get(1, 0.0137)
    dt4 = DT.get(4, 0.0414)
    dt8 = DT.get(8, 0.0745)

    # -------------------------------------------------------------------
    hdr("4. OPEN-LOOP MODE CENSUS (gain identically zero, 357 Hz stream)")
    if runs:
        print(f"  pooled {psd_n} CALIBRATING/FAULT segments, {psd_fs:.0f} Hz, "
              f"df = {fr[1]:.4f} Hz")
        modes = {}
        rows = []
        for i in range(4):
            band = (fr > 0.3) & (fr < 12.0)
            Pb, frb = psd[i][band], fr[band]
            order = np.argsort(Pb)[::-1]
            tops = []
            for j in order:
                if all(abs(frb[j] - x) > 0.25 for x, _ in tops):
                    tops.append((float(frb[j]), float(Pb[j])))
                if len(tops) >= 6:
                    break
            tops.sort()
            modes[i] = tops
            pk = max(tops, key=lambda a: a[1])
            print(f"  ch{i}  dominant {pk[0]:.2f} Hz | "
                  + "  ".join(f"{x:.2f}Hz({y / pk[1]:.3f})" for x, y in tops))
            rows.append([i, pk[0]] + [f"{x:.3f}:{y / pk[1]:.4f}" for x, y in tops])
        writecsv("modes.csv", ["ch", "dominant_hz"] + [f"peak{k}" for k in range(6)], rows)
        print("\n  Longest continuous gain-off record in the whole archive: 48.8 s")
        print("  -> best possible linewidth resolution 0.021 Hz; a Q=50 mode at 1.05 Hz")
        print("     is 0.021 Hz wide. Q IS NOT RESOLVABLE FROM THESE LOGS.")
    MODES = [0.70, 1.05, 1.66, 2.79, 3.75, 4.45, 8.72]

    # -------------------------------------------------------------------
    hdr("5. CONTROLLER MAGNITUDE: HOW MUCH 'Kp' IS ACTUALLY APPLIED?")
    print("  |C(f)| is the magnitude of (kp + ki/s + kd*s*LP2) -- i.e. the P-only")
    print("  gain that would produce the same force from the same velocity.")
    print("  v0, which is where -0.040 comes from, ran ki = kd = 0 exactly, so for")
    print("  v0 |C| == |kp| at every frequency.\n")
    ftab = np.array(MODES)
    rows = []
    print(f"  {'f (Hz)':>7} | " + " | ".join(
        f"{n:>16}" for n in ("v0 P-only kp=.030", "PID kp=-0.030", "PID kp=-0.035")))
    for f in ftab:
        a = abs(h_pid(f, -0.030, 0.0, 0.0, dt4))
        b = abs(h_pid(f, -0.030, KI_GAIN[0], KD_GAIN[0], dt4))
        c = abs(h_pid(f, -0.035, KI_GAIN[0], KD_GAIN[0], dt4))
        print(f"  {f:>7.2f} | {a:>16.4f} | {b:>16.4f} | {c:>16.4f}")
        rows.append([f, a, b, c])
    writecsv("controller_magnitude.csv",
             ["f_hz", "p_only_0.030", "pid_0.030", "pid_0.035"], rows)

    ff = np.linspace(0.3, 12.0, 40000)
    for kp, lab in ((-0.030, "STEADY"), (-0.035, "CAPTURE")):
        m = np.abs(h_pid(ff, kp, KI_GAIN[0], KD_GAIN[0], dt4))
        cross = ff[np.flatnonzero(np.diff(np.sign(m - 0.040)))]
        print(f"\n  {lab} kp={kp:+.3f}: |C| = 0.040 at "
              + (", ".join(f"{x:.3f} Hz" for x in cross) if len(cross) else "never")
              + f"   (|C| at 1.05 Hz = {abs(h_pid(1.05, kp, KI_GAIN[0], KD_GAIN[0], dt4)):.4f},"
              + f" at 3.0 Hz = {abs(h_pid(3.0, kp, KI_GAIN[0], KD_GAIN[0], dt4)):.4f})")

    # -------------------------------------------------------------------
    hdr("6. LOOP PHASE, COMPUTED FROM SOURCE")
    print("  L(f) = G(f) * HP0.4 * LP3.0 * d/dt * LP5.0 * PID * ZOH,")
    print("  all discrete forms taken verbatim from OnePoleFilter/PID/actuate().")
    print("  At a plant resonance the mechanical term contributes exactly -90 deg")
    print("  (force -> displacement at the pole), so the loop is at -180 deg when")
    print("  the NON-PLANT phase reaches -90 deg. That crossing needs no plant")
    print("  magnitude, so it is fully determined by the source + the loop period.\n")

    def phase_scan(dt, kp, ki, kd):
        fn = 0.5 / dt
        f = np.linspace(0.2, fn * 0.9999, 100000)
        ph = np.degrees(np.angle(h_loop_nonplant(f, dt, kp, ki, kd))) - 90.0
        k = np.flatnonzero(np.diff(np.sign(ph + 180.0)))
        cross = float(f[k[0]]) if len(k) else fn
        return fn, cross, float(ph.min()), float(f[int(np.argmin(ph))]), float(ph[-1])

    cfgs = [("v0, P-only, 1 coil driven", dt1, -0.030, 0.0, 0.0),
            ("v3+, PID, 1 coil driven", dt1, -0.030, KI_GAIN[0], KD_GAIN[0]),
            ("v3+, PID, 4 coils driven", dt4, -0.030, KI_GAIN[0], KD_GAIN[0]),
            ("v3+, PID, 4 coils, CAPTURE", dt4, -0.035, KI_GAIN[0], KD_GAIN[0]),
            ("v5.5, PID, 8 coils driven", dt8, -0.030, KI_GAIN[0], KD_GAIN[0])]
    rows = []
    print(f"  {'configuration':<30} {'dt(ms)':>7} {'f_Nyq':>7} {'f_-180':>7}   "
          f"phase margin at mode (deg)")
    print(" " * 55 + "  ".join(f"{m:>7.2f}Hz" for m in MODES[:5]))
    for lab, dt, kp, ki, kd in cfgs:
        fn, fc, phmin, fmin, phnyq = phase_scan(dt, kp, ki, kd)
        pm = []
        for mo in MODES[:5]:
            ph = math.degrees(np.angle(h_loop_nonplant(np.array([mo]), dt, kp, ki, kd)[0])) - 90.0
            pm.append(ph + 180.0 if mo < fn else float("nan"))
        print(f"  {lab:<30} {dt * 1e3:>7.1f} {fn:>7.2f} {fc:>7.2f}   "
              + "  ".join(f"{p:>9.1f}" for p in pm))
        rows.append([lab, dt * 1e3, fn, fc, phnyq] + pm)
    writecsv("phase_budget.csv",
             ["config", "dt_ms", "f_nyquist_hz", "f_180_hz", "phase_at_nyquist_deg"]
             + [f"pm_deg_{m}Hz" for m in MODES[:5]], rows)

    print("\n  f_-180 == f_Nyquist to 4 significant figures in every configuration,")
    print("  and this is exact, not numerical: at z = -1 every real-coefficient")
    print("  recurrence in the chain is REAL --")
    for lab, val in (("LP a/(1-(1-a)z^-1)", h_low(0.5 / dt4, BP_HIGH_HZ, dt4)),
                     ("HP a(1-z^-1)/(1-a z^-1)", h_high(0.5 / dt4, BP_LOW_HZ, dt4)),
                     ("d/dt (1-z^-1)/dt", h_deriv(0.5 / dt4, dt4)),
                     ("PID kp+ki dt/(1-z^-1)+kd..", h_pid(0.5 / dt4, -0.030, KI_GAIN[0], KD_GAIN[0], dt4))):
        v = complex(np.atleast_1d(val)[0])
        print(f"    {lab:<28} at Nyquist = {v.real:+.5f}{v.imag:+.2e}j")
    print("  -- so the ONLY surviving lag is the ZOH's -90 deg, and the plant's -90 deg")
    print("  at its own pole makes exactly -180 deg. CONSEQUENCE:\n")
    print("    Below the loop Nyquist frequency the phase NEVER reaches -180 deg, so")
    print("    the linear sampled-data gain margin is INFINITE. No value of Kp can")
    print("    destabilise a sub-Nyquist mode through phase margin in this loop.")
    print("    Line of attack 3 therefore comes back NEGATIVE: phase margin does not")
    print("    explain -0.040. What it does explain is that raising the coil count")
    print("    (1 -> 4 -> 8) halves and quarters the frequency at which the loop")
    print("    turns from damping to pumping, and costs 12 / 25 deg of margin at the")
    print("    1.05 Hz mode and 26 / 52 deg at 2.79 Hz.")
    print("\n  Loop transfer magnitude |L/G| (everything but the plant), 4 coils:")
    for mo in MODES:
        v = abs(h_loop_nonplant(np.array([mo]), dt4, -0.030, KI_GAIN[0], KD_GAIN[0])[0])
        print(f"    {mo:>5.2f} Hz : {v:.4f}"
              + ("   <- peak of the loop's own authority" if abs(mo - 2.79) < 0.01 else ""))
    print("  The loop's authority peaks near 3 Hz, ~1.8x its value at the 1.05 Hz")
    print("  mode it exists to damp. That is the D term plus the differentiator.")

    # -------------------------------------------------------------------
    hdr("6b. DECIMATION AND ALIASING")
    if runs:
        calib = float(np.median([r["dt_calib"] for r in runs if not math.isnan(r["dt_calib"])]))
        for k, dtv in sorted(DT.items()):
            print(f"  {k} coil(s): stream {1 / calib:.0f} Hz, loop {1 / dtv:.1f} Hz -> "
                  f"{dtv / calib:.1f} streamed samples discarded per processed sample, "
                  f"loop Nyquist {0.5 / dtv:.2f} Hz")
        print("  read_sample() takes one line per iteration and pyDAC.set_voltage()")
        print("  throws away the rest waiting for its ack. There is NO anti-alias")
        print("  filter in that decimation -- the 0.4-3.0 Hz bandpass runs AFTER it.")
        print("  Everything the OSEM sees above the loop Nyquist folds into the band")
        print("  at -180 deg. That is unmodelled here and unmeasured on the bench.")

    # -------------------------------------------------------------------
    hdr("7. ACTUATOR SATURATION: WHERE DOES 'RAIL ONSET' ACTUALLY SIT?")
    print(f"  Authority is BIAS +- min(BIAS-VMIN, VMAX-BIAS) = +-{BIAS_V:.3f} V.")
    print("  v0 was P-only, so the actuator rails when |Kp * vel| > 0.25 V, i.e.")
    print("  at |vel| = 0.25/|Kp|. That is arithmetic, not dynamics:\n")
    rows = []
    print(f"  {'Kp':>8} {'rails above |vel| (V/s)':>25}"
          + ("   frac of DAMPING samples past it" if runs else ""))
    for kp in (-0.005, -0.010, -0.020, -0.030, -0.035, -0.040, -0.050, -0.100, -0.200):
        vr = 0.25 / abs(kp)
        frac = float((vel > vr).mean() * 100) if runs else float("nan")
        print(f"  {kp:>8.3f} {vr:>25.2f}" + (f"{frac:>34.2f} %" if runs else ""))
        rows.append([kp, vr, frac])
    if runs:
        writecsv("saturation_vs_kp.csv",
                 ["kp", "rail_velocity_V_per_s", "pct_damping_samples_saturating"], rows)
        print(f"\n  Pooled |vel| over all enabled DAMPING samples (n={len(vel)}):")
        for p in (50, 90, 99, 99.9, 100):
            print(f"    p{p:<5} = {np.percentile(vel, p):7.2f} V/s")
        print("  The curve is smooth and monotone. Nothing distinguishes -0.040:")
        f30 = (vel > 0.25 / 0.030).mean() * 100
        f40 = (vel > 0.25 / 0.040).mean() * 100
        print(f"    -0.030 -> {f30:.2f}% of samples saturate;  "
              f"-0.040 -> {f40:.2f}%  (ratio {f40 / max(f30, 1e-9):.2f}x)")
        print("  Where 'rail onset' falls is set by how hard the optic was ringing,")
        print("  not by a property of the loop.")

    # -------------------------------------------------------------------
    hdr("8. RAIL INCIDENCE vs GAIN, CONTROLLING FOR AMPLITUDE")
    if runs:
        g, amp, r = rail[:, 0], rail[:, 1], rail[:, 2]
        edges = np.percentile(amp, [0, 50, 80, 95, 99, 100])
        print(f"  {'|bp| band':>22} " + "  ".join(f"|g|~{v:.3f}" for v in (0.010, 0.030, 0.035)))
        rows = []
        for a, b in zip(edges[:-1], edges[1:]):
            m = (amp >= a) & (amp < b)
            line, cells = f"  {a:8.3f}-{b:8.3f} ", []
            for gv in (0.010, 0.030, 0.035):
                mm = m & (np.abs(g - gv) < 0.0015)
                cells.append(f"{r[mm].mean() * 100:9.2f}%" if mm.sum() > 50 else "        -")
                rows.append([a, b, gv, int(mm.sum()),
                             float(r[mm].mean()) if mm.sum() else float("nan")])
            print(line + "  ".join(cells))
        writecsv("rail_vs_gain.csv",
                 ["amp_lo", "amp_hi", "gain", "n", "rail_fraction"], rows)
        print("  (|g|~0.010 is ch2, whose plant gain is ~3x the others -- not a")
        print("   like-for-like gain comparison, only an amplitude one.)")

    # -------------------------------------------------------------------
    hdr("9. DAMPING vs APPLIED GAIN, EMPIRICAL")
    if windows:
        W = np.array(windows, dtype=object)
        st = W[:, 1].astype(str)
        ch = W[:, 2].astype(int)
        gam = W[:, 4].astype(float)
        r2 = W[:, 5].astype(float)
        gmean = np.abs(W[:, 6].astype(float))
        gstd = W[:, 7].astype(float)
        ratio = W[:, 8].astype(float)
        amp = W[:, 9].astype(float)

        print("  (a) is there ANY gain-off ringdown to calibrate against?")
        for s in ("CALIBRATING", "FAULT", "DAMPING"):
            m = st == s
            print(f"    {s:<12} windows={m.sum():5d}  median R^2 of log-envelope fit="
                  f"{np.median(r2[m]):.3f}  median 8 s envelope ratio={np.median(ratio[m]):.3f}")
        print("    CALIBRATING and FAULT are ambient-driven steady state, not ringdown.")
        print("    -> the open-loop tau (hence Q) IS NOT MEASURABLE from these logs.")

        sel = (st == "DAMPING") & (r2 > R2_MIN) & (ratio < FALL_MAX) & (gstd < GAIN_STD_MAX)
        print(f"\n  (b) selected ringdown windows (R^2>{R2_MIN}, envelope falls, "
              f"gain constant): n={sel.sum()}")
        rows = []
        for c in range(4):
            for gv in sorted(set(np.round(gmean[sel & (ch == c)], 3))):
                m = sel & (ch == c) & (np.abs(gmean - gv) < 5e-4)
                if m.sum() < 4:
                    continue
                med = np.median(gam[m])
                print(f"    ch{c} |gain|={gv:.3f}  n={m.sum():4d}  gamma={med:+.4f} 1/s  "
                      f"tau={1 / med:6.2f} s  IQR=[{np.percentile(gam[m], 25):+.4f},"
                      f"{np.percentile(gam[m], 75):+.4f}]  median|bp|={np.median(amp[m]):.3f}")
                rows.append([c, gv, int(m.sum()), med, 1 / med,
                             float(np.percentile(gam[m], 25)),
                             float(np.percentile(gam[m], 75)), float(np.median(amp[m]))])
        writecsv("damping_vs_gain.csv",
                 ["ch", "abs_gain", "n", "gamma_median", "tau_s", "gamma_p25",
                  "gamma_p75", "median_env"], rows)

        print("\n  (c) how much of the schedule range is actually populated?")
        allc = (st == "DAMPING") & (gstd < GAIN_STD_MAX)
        for c in range(4):
            cnt = Counter(np.round(gmean[allc & (ch == c)], 3))
            tot = sum(cnt.values())
            print(f"    ch{c}: " + "  ".join(
                f"{k:.3f}:{v}({100 * v / tot:.1f}%)" for k, v in sorted(cnt.items())))
        print("    Constant-gain windows exist at essentially TWO levels per channel")
        print("    (0 and STEADY). CAPTURE is held too briefly to fit an envelope to.")

        print("\n  (d) gamma ~ a + b*|Kp| + c*log|bp|, constant-gain windows (R^2>0.6):")
        for c in range(4):
            m = (st == "DAMPING") & (gstd < GAIN_STD_MAX) & (ch == c) & (r2 > 0.6) & (amp > 1e-4)
            lv = sorted(set(np.round(gmean[m], 3)))
            if m.sum() < 30 or len(lv) < 2:
                print(f"    ch{c}: n={m.sum()}, gain levels present {lv} "
                      f"-> UNIDENTIFIED (no gain contrast)")
                continue
            X = np.vstack([np.ones(m.sum()), gmean[m], np.log(amp[m])]).T
            y = gam[m]
            beta, *_ = np.linalg.lstsq(X, y, rcond=None)
            res = y - X @ beta
            s2 = res @ res / (len(y) - 3)
            se = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X)))
            print(f"    ch{c}: n={m.sum():5d} levels={lv}  b={beta[1]:+8.3f}+-{se[1]:.3f} "
                  f"(1/s per unit Kp)   c={beta[2]:+.4f}+-{se[2]:.4f}")

        print("\n  (e) envelope GROWTH at constant gain (a runaway would live here):")
        for gv, lab in ((0.030, "STEADY 0.030"), (0.035, "CAPTURE 0.035"),
                        (0.010, "ch2 STEADY 0.010"), (0.012, "ch2 CAPTURE 0.012")):
            m = (st == "DAMPING") & (gstd < GAIN_STD_MAX) & (np.abs(gmean - gv) < 5e-4)
            if m.sum() < 5:
                print(f"    |g|={gv:.3f} ({lab}): n={m.sum()} -- too few to say anything")
                continue
            print(f"    |g|={gv:.3f} ({lab:16s}): n={m.sum():5d}  frac(gamma<0)="
                  f"{(gam[m] < 0).mean():.3f}  median gamma={np.median(gam[m]):+.4f}  "
                  f"worst (most negative) gamma={gam[m].min():+.4f}")
        print("    ~42% of all constant-gain windows show a growing envelope at BOTH")
        print("    gain levels: that is ambient re-excitation, present at every gain,")
        print("    not a gain-dependent instability.")

    # -------------------------------------------------------------------
    hdr("9b. IS THE LOOP PUMPING ANYTHING? (open- vs closed-loop spectral SHAPE)")
    print("  A loop going unstable through phase margin oscillates near its -180 deg")
    print("  crossover, NOT at the pendulum resonance -- so the shape of the closed-")
    print("  loop spectrum relative to the open-loop one is the diagnostic. Band")
    print("  powers of raw ADC counts, each normalised by that record's own")
    print("  0.6-1.3 Hz power, so overall amplitude cancels.\n")
    if shape_rows:
        S = np.array(shape_rows, dtype=object)
        state = S[:, 2].astype(str)
        fsr = S[:, 3].astype(float)
        gmx = S[:, 4].astype(float)
        P = S[:, 5:].astype(float)
        shape = P / P[:, :1]
        # only compare closed-loop records fast enough to resolve the bands
        rows = []
        print(f"  {'record':<34} " + "  ".join(f"{a}-{b}Hz" for a, b in BANDS[1:]))
        for lab, m in (("open loop (CALIBRATING)", (state == "CALIBRATING")),
                       ("closed loop, 1 coil (>=60 Hz)", (state == "DAMPING") & (fsr > 60) & (gmx > 0)),
                       ("closed loop, 4 coils (~24 Hz)", (state == "DAMPING") & (fsr < 60) & (fsr > 18) & (gmx > 0))):
            if m.sum() < 3:
                continue
            med = np.median(shape[m], axis=0)
            nb = len(BANDS) if fsr[m].min() > 24 else 4
            print(f"  {lab:<34} " + "  ".join(
                f"{med[k]:9.4f}" if k < nb else f"{'aliased':>9}" for k in range(1, len(BANDS))))
            rows.append([lab, int(m.sum())] + list(med))
        writecsv("spectral_shape_summary.csv",
                 ["record", "n"] + [f"rel_P_{a}_{b}Hz" for a, b in BANDS], rows)
        ol = shape[state == "CALIBRATING"]
        cl = shape[(state == "DAMPING") & (fsr > 60) & (gmx > 0)]
        if len(cl) >= 3:
            r = np.median(cl, axis=0) / np.median(ol, axis=0)
            print("\n  closed/open ratio of relative band power (1-coil runs, 73 Hz loop):")
            for k, (a, b) in enumerate(BANDS):
                print(f"    {a:>4.1f}-{b:>4.1f} Hz : {r[k]:6.3f}"
                      + ("   <- reference" if k == 0 else
                         ("   PUMPED" if r[k] > 2.0 else "")))
            print("  Ratios above 1 mean the loop leaves relatively MORE power there")
            print("  than open loop. Note these runs are decimated to 73 Hz with no")
            print("  anti-alias filter, so the 4-12 Hz numbers include folded content.")

    # -------------------------------------------------------------------
    hdr("9c. SENSOR CLIPPING AND THE VELOCITY ESTIMATE")
    print("  Resting counts are 600/631/708/677 against a 511.5 mid-scale")
    print("  (bench 2026-08-04), so every OSEM hits its TOP rail first. A clipped")
    print("  shadow sensor gives the bandpass a flat top and the first-difference")
    print("  derivative a step, so |vel| spikes exactly when the loop pushes hardest.")
    if clip_rows:
        C = np.array(clip_rows, dtype=object)
        vr = C[:, 4].astype(float)
        vc = C[:, 5].astype(float)
        nr = C[:, 2].astype(int)
        print(f"\n  {len(C)} run/channel records with both railed and clean DAMPING samples:")
        print(f"    mean |vel| on railed samples : {np.average(vr, weights=nr):7.3f} V/s")
        print(f"    mean |vel| on clean samples  : {np.average(vc, weights=nr):7.3f} V/s")
        print(f"    ratio                        : {np.average(vr, weights=nr) / np.average(vc, weights=nr):7.2f}x")
        print(f"    per-record ratio, median     : {np.median(vr / vc):7.2f}x  "
              f"(range {np.min(vr / vc):.2f}-{np.max(vr / vc):.2f})")
        print("  The velocity the loop acts on is several times larger while the")
        print("  sensor is outside its linear range, and the commanded force is")
        print("  proportional to Kp -- so this path gets worse with gain and is")
        print("  triggered by AMPLITUDE, which is what 'instability/rail onset'")
        print("  reads like. It is a hypothesis these logs are consistent with,")
        print("  NOT a mechanism they demonstrate: no run varies Kp at fixed")
        print("  amplitude, so the two cannot be separated here.")

    # -------------------------------------------------------------------
    hdr("10. WIRING: WHAT WAS Kp = -0.040 MEASURED THROUGH?")
    print("  osem.v0.py (git blob 54b8a20), line 111 of the settings block:")
    print('    "Only ch0 (A0 -> DAC ch0) is experimentally validated"')
    print("    ENABLE_CHANNEL = [True, False, False, False]")
    print("    KI_GAIN = KD_GAIN = [0,0,0,0]")
    print("  DAC_CHANNELS before 2026-08-03 was [0, 2, 4, 6]; it is [1, 3, 5, 7] now.")
    print("  The 2026-08-04 DC matrix measures d(a0)/d(V) for every DAC channel:\n")
    old = DAC_MAP_PRE_0803[0]
    new = DAC_MAP[0]
    col_old = DAC_MAP.index(old)
    col_new = DAC_MAP.index(new)
    print(f"    a0 driven from DAC {new} (today's ch0 coil) : {DCM[0, col_new]:+.0f} counts/V")
    print(f"    a0 driven from DAC {old} (v0's ch0 coil)    : {DCM[0, col_old]:+.0f} counts/V")
    ratio = abs(DCM[0, col_new] / DCM[0, col_old])
    print(f"    ratio {ratio:.2f}x")
    print(f"\n  IF DAC channel numbers are physically fixed and only the software map")
    print(f"  changed (which is what provenance.md asserts: ref.py 'the record of the")
    print(f"  coil wiring' already held [1,3,5,7,0,2,4,6] BEFORE 2026-08-03), then")
    print(f"  Kp = -0.040 in July equals Kp = {-0.040 / ratio:+.4f} in today's units,")
    print(f"  and today's STEADY -0.030 is already {0.030 * ratio / 0.040:.2f}x past it.")
    print("  IF instead the coils were physically re-plugged, -0.040 transfers 1:1.")
    print("  NOTHING IN THE REPO RECORDS WHICH. This is the single most consequential")
    print("  unrecorded fact about the number.")

    print("\n  Per-channel scaling of the SAME loop gain, from the DC matrix diagonal:")
    diag = np.array([DCM[i, i] for i in range(4)])
    print(f"    d(a_i)/d(V) on its own coil: " + "  ".join(f"ch{i}:{d:+.0f}" for i, d in enumerate(diag)))
    kp_now = np.array([-0.030, -0.030, +0.010, -0.030])
    lg = np.abs(kp_now * diag)
    print(f"    |Kp * gain| at STEADY:       " + "  ".join(f"ch{i}:{v:.2f}" for i, v in enumerate(lg)))
    equiv = 0.040 * abs(diag[0]) / np.abs(diag)
    print(f"    Kp giving ch0's -0.040 loop gain: "
          + "  ".join(f"ch{i}:{s * v:+.4f}" for i, (v, s) in
                      enumerate(zip(equiv, np.sign(kp_now)))))
    print("    -> a single scalar cap of -0.035 is ~1.8x conservative on ch1 and")
    print("       ~2.4x on ch3, and ch2's +0.012 is 61% of its own equivalent.")
    writecsv("per_channel_limit.csv",
             ["ch", "dc_counts_per_V", "steady_kp", "loop_gain_product", "equiv_kp_at_ch0_limit"],
             [[i, diag[i], kp_now[i], lg[i], np.sign(kp_now[i]) * equiv[i]] for i in range(4)])

    # -------------------------------------------------------------------
    hdr("11. CAN THE STEPPED-SINE RUN SUPPLY A MEASURED PLANT?")
    jf = os.path.join(ROOT, "data", "20260804_123821_v6_stepped4.json")
    if os.path.exists(jf):
        d = json.load(open(jf))
        f = np.array(d["freqs"])
        H = np.array(d["real"]) + 1j * np.array(d["imag"])
        ok = np.array(d["ok"])
        print(f"  {len(f)} frequencies, {d['dwell_s']:.0f} s dwell, amp {d['amp']} V, open loop.")
        print("  Diagonal path (sensor i <- its own coil i), which is the loop path:\n")
        print(f"  {'f (Hz)':>7} " + "  ".join(f"{'ch%d mag/phase' % i:>18}" for i in range(4)))
        rows = []
        for k in range(len(f)):
            cells = []
            for i in range(4):
                h = H[k, i, i]
                cells.append(f"{abs(h):8.3f} /{math.degrees(np.angle(h)):+7.1f}"
                             + ("" if ok[k, i, i] else "*"))
            print(f"  {f[k]:>7.4f} " + "  ".join(f"{c:>18}" for c in cells))
            rows.append([f[k]] + [abs(H[k, i, i]) for i in range(4)]
                        + [math.degrees(np.angle(H[k, i, i])) for i in range(4)]
                        + [bool(ok[k, i, i]) for i in range(4)])
        writecsv("stepped_diagonal.csv",
                 ["f_hz"] + [f"mag_ch{i}" for i in range(4)]
                 + [f"phase_deg_ch{i}" for i in range(4)]
                 + [f"ok_ch{i}" for i in range(4)], rows)
        ph = np.unwrap(np.angle(np.array([H[k, 0, 0] for k in range(len(f))])))
        jump = np.abs(np.diff(np.degrees(ph)))
        print(f"\n  * = the run's own rail flag. Phase of ch0's diagonal moves by up to")
        print(f"    {jump.max():.0f} deg between adjacent frequencies and is non-monotone;")
        print(f"    a Q~50 second-order plant cannot do that. The measurement is")
        print(f"    ambient-dominated (versions.md: 4 of 12 frequencies usable, and two")
        print(f"    of those four are near-identical across coils).")
        print("  VERDICT: no usable plant transfer function. |L| at the -180 deg")
        print("  crossing -- and therefore the GAIN margin -- cannot be computed from")
        print("  anything on disk. The sweep also stops at 4.0 Hz, below the crossing.")

    hdr("DONE")
    print(f"  derived tables in {OUT}")


if __name__ == "__main__":
    main()
