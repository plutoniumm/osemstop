#!/usr/bin/env python3
"""Retune CALIB_AGREE_TOL from the bench logs already on disk.

    .venv/bin/python analysis/calib_tol.py            # ~90 s cold, ~5 s cached
    .venv/bin/python analysis/calib_tol.py --rebuild  # ignore the cache

Every number in analysis/calib_tol.md comes from here. Nothing in this file
writes to, imports from, or otherwise touches a controller, the harness or the
simulator; it only reads data/*.csv.

What it does
------------
`fast-calib` (osem.v9.py, v9 onward) makes CALIBRATION_S a ceiling: calibration
ends early once the trailing CALIB_AGREE_N sub-window RMSs agree with each other
AND their median agrees with the median of every sub-window so far, both as a
max/min ratio against CALIB_AGREE_TOL. On the bench it has fired exactly once
(v10, 2026-08-06). This file replays the SAME logic over the CALIBRATING windows
of every recorded run and asks what tolerance the bench floor actually needs.

Sources
-------
  data/*_fast_lock.csv        27 controller runs, 2026-08-03 / 08-04 / 08-06;
                              42 CALIBRATING episodes, 35 of them a full 20 s.
  bench/2026080?/v*.log       three printed baselines, used to VERIFY the replay.

Standing practice: the extracted CALIBRATING windows and every derived table are
streamed to analysis/out/calib_tol_*, so a later question about an intermediate
value does not need another pass over 200 MB of CSV.

NOT USED AS EVIDENCE: sim/server.py. The simulator's calibration noise is what
1.20 was tuned against in the first place; using it again would re-derive the
number that failed. The simulator appears in the .md only where a *simulator*
assertion constrains the answer, and those runs are made with harness.py, not
here.
"""

import csv
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "analysis", "out")
CACHE = os.path.join(OUT, "calib_tol_windows")
REBUILD = "--rebuild" in sys.argv

# ---------------------------------------------------------------------------
# Controller constants, copied from osem.v9.py (verified identical in v10/v11
# on 2026-08-06). The replay below reproduces _close_subwindow, _calib_stationary
# and _set_baseline exactly; do not "simplify" it.
# ---------------------------------------------------------------------------
CALIBRATION_S, CALIB_SUBWINDOWS = 20.0, 5
CALIB_SUBWINDOW_S, CALIB_MIN_SUBWINDOWS = 2.0, 3
CALIB_AGREE_N, CALIB_AGREE_TOL = 3, 1.20
BP_LOW_HZ, BP_HIGH_HZ = 0.4, 3.0
A_VCC, ADC_MAX_COUNTS = 5.02, 1023

# The channels whose baseline is the OPTIC. harness.py calls these `solid`:
# a4/a6/a7 carry 0.1-9% of their power in 0.4-3 Hz and a5 runs at ~1/12 of a0's
# counts-per-metre (research.md 6b), so their 2 s RMS is their own noise, not the
# body. Every skew number below is worst-over-SOLID, which is what the 35% line
# in harness.py is asserted over.
SOLID = [0, 1, 2, 3]


def say(*a):
    print(*a, flush=True)


# ===========================================================================
# 1. extract the CALIBRATING windows
# ===========================================================================
def build_cache():
    """One pass over every data/*_fast_lock.csv, keeping only CALIBRATING rows.

    `chN_bp` is logged straight from Controller.bp, which is the exact array
    _close_subwindow folds into an RMS -- so replaying from this column replays
    the controller's own input, not a reconstruction of it. t0 is the timestamp
    of the row BEFORE the episode (0.0 for the first): _start_calibration sets
    calib_start = sub_start = t at the sample that ENTERS the state, one sample
    before the first CALIBRATING row, and that ~3 ms offset decides whether a
    20.00 s window closes 9 sub-windows or 10.
    """
    os.makedirs(CACHE, exist_ok=True)
    for path in sorted(glob.glob(os.path.join(ROOT, "data", "*_fast_lock.csv"))):
        tag = os.path.basename(path)[:-4].replace("_fast_lock", "")
        dst = os.path.join(CACHE, tag + ".npz")
        if os.path.exists(dst) and not REBUILD:
            continue
        with open(path, newline="") as fh:
            rd = csv.reader(fh)
            hdr = next(rd)
            ix = {k: i for i, k in enumerate(hdr)}
            n = sum(1 for k in hdr if k.endswith("_counts"))
            bi = [ix[f"ch{i}_bp"] for i in range(n)]
            it, ist = ix["time_s"], ix["state"]
            T, B, EP, T0 = [], [], [], []
            ep, prev, prev_t = -1, None, 0.0
            for row in rd:
                st = row[ist]
                if st != prev:
                    if st == "CALIBRATING":
                        ep += 1
                        T0.append(0.0 if ep == 0 else prev_t)
                    prev = st
                if st == "CALIBRATING":
                    T.append(row[it])
                    EP.append(ep)
                    B.append([row[j] for j in bi])
                prev_t = float(row[it])
        np.savez_compressed(dst, t=np.array(T, float), bp=np.array(B, float),
                            ep=np.array(EP, int), t0=np.array(T0, float), n=n)
        say(f"    cached {tag}  {n} ch, {len(T)} rows, {ep + 1} episode(s)")


# ===========================================================================
# 2. the controller's own logic, reproduced
# ===========================================================================
def close_subwindows(t, bp, t0):
    """osem.v9.py _close_subwindow, unrolled over a recorded episode.

    The controller appends the CURRENT sample to self.calib before testing the
    boundary, so the boundary sample belongs to the window that is CLOSING and
    sub_start is reset to the boundary time rather than to t0 + 2k -- both
    reproduced here. Returns [(t_close, rms_vector)].
    """
    out, sub_n0, sub_start = [], 0, t0
    for k in range(len(t)):
        if t[k] - sub_start >= CALIB_SUBWINDOW_S:
            a = bp[sub_n0:k + 1]
            if len(a) >= 2:
                out.append((t[k], np.sqrt((a ** 2).mean(0))))
            sub_n0, sub_start = k + 1, t[k]
    return out


def ceiling_baseline(bp):
    """osem.v9.py _set_baseline(early=False) -- v5's estimator, unchanged: the
    median of CALIB_SUBWINDOWS equal splits of the WHOLE window. This is the
    number the bench actually stores today, and the reference every skew below
    is measured against."""
    parts = [p for p in np.array_split(bp, CALIB_SUBWINDOWS) if len(p)]
    if not parts:
        return None
    return np.maximum(np.median([np.sqrt((p ** 2).mean(0)) for p in parts], 0), 1e-6)


def stationary(w, ch, tol1, tol2, agree_n):
    """osem.v9.py _calib_stationary, with its ONE tolerance split into the two it
    actually applies, so the tests can be moved independently. tol1 == tol2 ==
    CALIB_AGREE_TOL reproduces the shipped function exactly.

    NOTE the structural fact this file turns on: test (2) compares the median of
    the trailing agree_n windows against the median of ALL of them, so when the
    number of windows EQUALS agree_n the two sets are identical and r2 == 1.000
    by construction. At CALIB_MIN_SUBWINDOWS = 3 = CALIB_AGREE_N the earliest
    exit is therefore guarded by test (1) alone.
    """
    a = w[:, ch]
    tail = a[-agree_n:]
    r1 = float((tail.max(0) / np.maximum(tail.min(0), 1e-9)).max())
    mt, ma = np.median(tail, 0), np.median(a, 0)
    r2 = float((np.maximum(mt, ma) / np.maximum(np.minimum(mt, ma), 1e-9)).max())
    return r1, r2, (r1 <= tol1 and r2 <= tol2)


def replay(d, tol1, tol2, min_sub=CALIB_MIN_SUBWINDOWS, agree_n=CALIB_AGREE_N,
           ch=None):
    """Walk one recorded episode the way step() does. Returns (t_elapsed,
    baseline) for an early exit, or None if it would have run the ceiling."""
    ch = SOLID if ch is None else ch
    sw, cl = d["sw"], d["cl"]
    for k in range(max(min_sub, agree_n) - 1, len(cl)):
        if cl[k][0] - d["t0"] >= CALIBRATION_S:
            return None
        if stationary(sw[:k + 1], ch, tol1, tol2, agree_n)[2]:
            # _set_baseline(early=True): median of the trailing windows that
            # were just shown to agree, NOT of everything.
            base = np.maximum(np.median(sw[k - agree_n + 1:k + 1], 0), 1e-6)
            return cl[k][0] - d["t0"], base
    return None


def load_episodes():
    eps = []
    for p in sorted(glob.glob(os.path.join(CACHE, "*.npz"))):
        z = np.load(p)
        t, bp, ep, t0s, n = z["t"], z["bp"], z["ep"], z["t0"], int(z["n"])
        tag = os.path.basename(p)[:-4]
        for e in np.unique(ep):
            m = ep == e
            te, be, t0 = t[m], bp[m], float(t0s[e])
            cl = close_subwindows(te, be, t0)
            if len(cl) < CALIB_AGREE_N:
                continue
            eps.append(dict(
                tag=tag, ep=int(e), n=n, t0=t0, dur=te[-1] - t0,
                fs=len(te) / (te[-1] - t0), cl=cl,
                sw=np.array([c[1] for c in cl]), base=ceiling_baseline(be),
                nsw=len(cl)))
    for d in eps:
        # how far the floor moved across the window, worst solid channel: >1 is
        # a decaying window (ringdown), <1 a rising one (drift/build-up).
        d["decay"] = float((d["sw"][0, SOLID] /
                            np.maximum(d["sw"][-1, SOLID], 1e-9)).max())
    return eps


def skew(d, base_early):
    """|early / full-window - 1|, per solid channel. Signed max is kept because
    the two directions are not equally dangerous: a floor read HIGH makes the
    1.8x runaway breaker slow and the 0.35x lock detector permissive."""
    s = base_early[SOLID] / d["base"][SOLID] - 1.0
    return float(np.abs(s).max()), float(s[np.abs(s).argmax()])


def summarise(eps, tol1, tol2, min_sub=CALIB_MIN_SUBWINDOWS,
              agree_n=CALIB_AGREE_N, ch=None):
    got = [(d, replay(d, tol1, tol2, min_sub, agree_n, ch)) for d in eps]
    got = [(d, r) for d, r in got if r]
    if not got:
        return dict(n=0, tot=len(eps), mean=CALIBRATION_S, medt=float("nan"),
                    med=float("nan"), p90=float("nan"), mx=float("nan"),
                    hi=float("nan"), rows=[])
    rows = [(d, r[0]) + skew(d, r[1]) for d, r in got]
    ab = np.array([r[2] for r in rows])
    sg = np.array([r[3] for r in rows])
    ts = np.array([r[1] for r in rows])
    return dict(n=len(got), tot=len(eps),
                mean=(ts.sum() + CALIBRATION_S * (len(eps) - len(got))) / len(eps),
                medt=float(np.median(ts)), med=float(np.median(ab)),
                p90=float(np.percentile(ab, 90)), mx=float(ab.max()),
                hi=float(max(sg.max(), 0.0)), rows=rows)


def dump(name, header, rows):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "calib_tol_" + name + ".csv"), "w",
              newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


# ===========================================================================
def main():
    say("\n=== 0. extracting CALIBRATING windows " + "=" * 40)
    build_cache()
    eps = load_episodes()
    full = [d for d in eps if d["dur"] >= CALIBRATION_S - 0.1]
    cold = [d for d in full if d["ep"] == 0]
    warm = [d for d in full if d["ep"] > 0]
    say(f"  {len(eps)} CALIBRATING episodes with >= {CALIB_AGREE_N} sub-windows; "
        f"{len(full)} ran a full {CALIBRATION_S:.0f} s window")
    say(f"  of those: {len(cold)} cold (first calibration of a run), "
        f"{len(warm)} post-fault re-calibration (i.e. after a kick)")
    dump("episodes", ["run", "episode", "channels", "duration_s", "rate_hz",
                      "n_subwindows", "decay_first_over_last"] +
         [f"ceiling_base_ch{i}" for i in SOLID],
         [[d["tag"], d["ep"], d["n"], f"{d['dur']:.2f}", f"{d['fs']:.1f}",
           d["nsw"], f"{d['decay']:.3f}"] +
          [f"{d['base'][i]:.5f}" for i in SOLID] for d in eps])

    # ---- 1. verify the replay against printed bench baselines ---------------
    say("\n=== 1. replay verified against the bench logs " + "=" * 32)
    say("  bench log line                                   printed        replayed")
    checks = [
        ("bench/20260804/v9_4ch_3kicks.log:30  ceiling", "20260804_135853", 0,
         "ceiling", [0.2100, 0.0993, 0.3151, 0.0794]),
        ("bench/20260804/v8_4ch_3kicks.log:30  ceiling", "20260804_134707", 0,
         "ceiling", [0.2548, 0.1146, 0.4234, 0.1040]),
        ("bench/20260806/v10_4ch_3kicks.log:30 ceiling", "20260806_155848", 0,
         "ceiling", [0.4968, 0.2144, 0.6736, 0.1789]),
        ("bench/20260806/v10_4ch_3kicks.log:15549 EARLY", "20260806_155848", 1,
         "early", [0.7427, 0.4404, 0.8563, 0.4656]),
    ]
    worst_err = 0.0
    for name, tag, e, kind, printed in checks:
        d = [x for x in eps if x["tag"] == tag and x["ep"] == e][0]
        got = (d["base"] if kind == "ceiling"
               else np.median(d["sw"][-CALIB_AGREE_N:], 0))
        err = float(np.abs(got[SOLID] / np.array(printed) - 1).max())
        worst_err = max(worst_err, err)
        say(f"  {name:47s} " +
            "/".join(f"{v:.4f}" for v in printed) + "  " +
            "/".join(f"{v:.4f}" for v in got[SOLID]) + f"   {err * 100:.2f}%")
    say(f"  worst disagreement {worst_err * 100:.2f}% -- that is the 5-decimal "
        f"rounding of chN_bp in the CSV, not a difference in the algorithm.")

    # ---- 2. what the agreement ratio IS on this bench -----------------------
    say("\n=== 2. the agreement ratio, measured " + "=" * 41)
    say("  test (1) = trailing-3 max/min;  test (2) = median(tail) vs median(all)")
    say(f"  {'k':>2s} {'t':>5s} {'n':>3s} | {'r1 min':>7s} {'med':>6s} "
        f"{'p90':>6s} {'max':>6s} | {'r2 med':>7s} {'max':>6s} | pass at 1.20")
    tab = []
    for k in range(CALIB_AGREE_N, 10):
        r1s, r2s = [], []
        for d in full:
            if k > d["nsw"] or d["cl"][k - 1][0] - d["t0"] >= CALIBRATION_S:
                continue
            r1, r2, _ = stationary(d["sw"][:k], SOLID, 9e9, 9e9, CALIB_AGREE_N)
            r1s.append(r1)
            r2s.append(r2)
        if not r1s:
            continue
        r1s, r2s = np.array(r1s), np.array(r2s)
        npass = int(((r1s <= 1.20) & (r2s <= 1.20)).sum())
        say(f"  {k:2d} {2.0 * k:5.1f} {len(r1s):3d} | {r1s.min():7.2f} "
            f"{np.median(r1s):6.2f} {np.percentile(r1s, 90):6.2f} {r1s.max():6.2f} "
            f"| {np.median(r2s):7.2f} {r2s.max():6.2f} | {npass:2d}/{len(r1s)}")
        tab.append([k, 2.0 * k, len(r1s), f"{r1s.min():.3f}",
                    f"{np.median(r1s):.3f}", f"{np.percentile(r1s, 90):.3f}",
                    f"{r1s.max():.3f}", f"{np.median(r2s):.3f}",
                    f"{r2s.max():.3f}", npass])
    dump("ratio_vs_time", ["k", "t_s", "n_episodes", "r1_min", "r1_median",
                           "r1_p90", "r1_max", "r2_median", "r2_max",
                           "n_pass_at_1.20"], tab)
    req = sorted(min(stationary(d["sw"][:k], SOLID, 9e9, 9e9, CALIB_AGREE_N)[0]
                     for k in range(CALIB_AGREE_N, d["nsw"] + 1)
                     if d["cl"][k - 1][0] - d["t0"] < CALIBRATION_S)
                 for d in full)
    say(f"  smallest test-(1) tolerance each window ever needs: min {req[0]:.2f} "
        f"median {np.median(req):.2f} p90 {np.percentile(req, 90):.2f} "
        f"max {req[-1]:.2f}")
    say(f"  fraction of windows that never get below 1.20: "
        f"{sum(1 for r in req if r > 1.20)}/{len(req)}")

    # ---- 3. does the sample rate matter? ------------------------------------
    say("\n=== 3. sample rate " + "=" * 59)
    for day in ("20260803", "20260804", "20260806"):
        f = [d["fs"] for d in eps if d["tag"].startswith(day)]
        if f:
            say(f"  {day}: {len(f)} episodes, stream rate inside CALIBRATING "
                f"{min(f):.0f}-{max(f):.0f} Hz")
    say("  decimating the 1112 Hz run (bp is band-limited to 3 Hz, so this only "
        "removes samples):")
    dec_rows = []
    for p in sorted(glob.glob(os.path.join(CACHE, "20260806_171500*.npz"))):
        z = np.load(p)
        t, bp, ep, t0s = z["t"], z["bp"], z["ep"], z["t0"]
        for e in np.unique(ep):
            m = ep == e
            line = []
            for dec in (1, 3, 8, 32):
                te, be, t0 = t[m][::dec], bp[m][::dec], float(t0s[e])
                cl = close_subwindows(te, be, t0)
                if len(cl) < CALIB_AGREE_N:
                    continue
                sw = np.array([c[1] for c in cl])
                r = [stationary(sw[:k], SOLID, 9e9, 9e9, CALIB_AGREE_N)[0]
                     for k in range(CALIB_AGREE_N, len(cl) + 1)
                     if cl[k - 1][0] - t0 < CALIBRATION_S]
                line.append((len(te) / (te[-1] - t0), min(r)))
                dec_rows.append([f"ep{e}", dec, f"{line[-1][0]:.0f}",
                                 f"{min(r):.3f}"])
            say(f"    ep{e}: " + "   ".join(f"{f:6.0f} Hz -> {v:.3f}"
                                            for f, v in line))
    dump("decimation", ["episode", "decimation", "rate_hz",
                        "min_test1_tolerance"], dec_rows)

    # ---- 4. the null: exit early with NO agreement test ---------------------
    say("\n=== 4. null -- median-of-3 at an arbitrary position " + "=" * 26)
    null = []
    for d in full:
        for k in range(CALIB_AGREE_N, d["nsw"] + 1):
            if d["cl"][k - 1][0] - d["t0"] >= CALIBRATION_S:
                break
            a, _ = skew(d, np.median(d["sw"][k - CALIB_AGREE_N:k], 0))
            null.append((k, a))
    nk = np.array([x[0] for x in null])
    ns = np.array([x[1] for x in null])
    for k in sorted(set(nk)):
        s = ns[nk == k]
        say(f"    k={k} (t={2 * k:2d} s) n={len(s):2d}  median {np.median(s):5.1%}  "
            f"p90 {np.percentile(s, 90):5.1%}  max {s.max():5.1%}")
    say(f"  all positions: n={len(ns)} median {np.median(ns):.1%} "
        f"p90 {np.percentile(ns, 90):.1%} max {ns.max():.1%}, "
        f"{(ns > 0.35).mean():.1%} of them over the 35% line")
    say(f"  'always exit at the 6 s floor, no test' worst case: "
        f"{ns[nk == CALIB_AGREE_N].max():.1%}  "
        f"(research.md's simulator sweep gives 74.3% for CALIBRATION_S = 5 s)")
    dump("null", ["k", "worst_solid_skew"], [[k, f"{s:.5f}"] for k, s in null])

    # ---- 5. the sweep the question asks for ---------------------------------
    say("\n=== 5. CALIB_AGREE_TOL swept, shipped structure " + "=" * 30)
    say(f"  {'tol':>5s} {'fired':>7s} {'mean cal':>9s} {'median t':>9s} "
        f"{'med skew':>9s} {'p90':>6s} {'max':>6s} {'worst HIGH':>11s}")
    rows = []
    for tol in (1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.60, 1.80):
        s = summarise(full, tol, tol)
        say(f"  {tol:5.2f} {s['n']:3d}/{s['tot']:<3d} {s['mean']:8.2f}s "
            f"{s['medt']:8.2f}s {s['med']:8.1%} {s['p90']:5.1%} {s['mx']:5.1%} "
            f"{s['hi']:10.1%}")
        rows.append([f"{tol:.2f}", s["n"], s["tot"], f"{s['mean']:.2f}",
                     f"{s['medt']:.2f}", f"{s['med']:.4f}", f"{s['p90']:.4f}",
                     f"{s['mx']:.4f}", f"{s['hi']:.4f}"])
    dump("sweep", ["tol", "fired", "episodes", "mean_calibration_s",
                   "median_exit_s", "median_skew", "p90_skew", "max_skew",
                   "worst_high_skew"], rows)

    # ---- 6. structural variants ---------------------------------------------
    say("\n=== 6. variants -- test (2) pinned at 1.20 " + "=" * 35)
    say("  test (2) is left at 1.20 on purpose: the simulator's 6 V/s shock "
        "reads r2 = 1.400\n  there (osem.v9.py _calib_stationary docstring), so "
        "a shared constant above ~1.39\n  disarms the guard that keeps "
        "`runaway-baseline` at 17.6%.")
    say(f"  {'MIN':>4s} {'tol1':>5s} {'fired':>7s} {'mean cal':>9s} "
        f"{'median t':>9s} {'med':>6s} {'p90':>6s} {'max':>6s} {'HIGH':>6s}")
    rows = []
    for mn in (3, 4, 5, 6):
        for tol in (1.30, 1.35, 1.40, 1.45, 1.50, 1.60):
            s = summarise(full, tol, 1.20, mn)
            say(f"  {mn:4d} {tol:5.2f} {s['n']:3d}/{s['tot']:<3d} "
                f"{s['mean']:8.2f}s {s['medt']:8.2f}s {s['med']:5.1%} "
                f"{s['p90']:5.1%} {s['mx']:5.1%} {s['hi']:5.1%}")
            rows.append([mn, f"{tol:.2f}", s["n"], s["tot"], f"{s['mean']:.2f}",
                         f"{s['medt']:.2f}", f"{s['med']:.4f}",
                         f"{s['p90']:.4f}", f"{s['mx']:.4f}", f"{s['hi']:.4f}"])
        say("")
    dump("variants", ["min_subwindows", "tol1", "fired", "episodes",
                      "mean_calibration_s", "median_exit_s", "median_skew",
                      "p90_skew", "max_skew", "worst_high_skew"], rows)
    say("  CALIB_AGREE_N raised instead (test (2) still 1.20):")
    for an in (4, 5):
        for tol in (1.40, 1.50, 1.60):
            s = summarise(full, tol, 1.20, an, an)
            say(f"    AGREE_N={an} tol1={tol:.2f}  fired {s['n']:2d}/{s['tot']} "
                f"mean {s['mean']:5.2f}s  max skew {s['mx']:5.1%}")

    # ---- 7. is the recommendation robust? -----------------------------------
    say("\n=== 7. robustness of the recommendation " + "=" * 38)
    REC = dict(tol1=1.35, tol2=1.35, min_sub=3)      # the one-constant answer
    ALT = dict(tol1=1.50, tol2=1.20, min_sub=5)      # section 7 of the .md
    for nm, kw in (("RECOMMENDED shipped structure, tol 1.35", REC),
                   ("MIN=5 tol1=1.50 tol2=1.20", ALT),
                   ("today, tol 1.20", dict(tol1=1.20, tol2=1.20, min_sub=3))):
        subsets = [("all", full),
                   ("2026-08-03", [d for d in full if d["tag"][:8] == "20260803"]),
                   ("2026-08-04", [d for d in full if d["tag"][:8] == "20260804"]),
                   ("2026-08-06", [d for d in full if d["tag"][:8] == "20260806"]),
                   ("cold", cold), ("post-fault", warm),
                   ("4-channel runs", [d for d in full if d["n"] == 4]),
                   ("8-channel runs", [d for d in full if d["n"] == 8]),
                   ("~350 Hz", [d for d in full if d["fs"] < 600]),
                   ("~1112 Hz", [d for d in full if d["fs"] > 600])]
        say(f"  {nm}:")
        for sn, ss in subsets:
            if not ss:
                continue
            s = summarise(ss, kw["tol1"], kw["tol2"], kw["min_sub"])
            say(f"    {sn:16s} n={len(ss):2d}  fired {s['n']:2d}  "
                f"mean {s['mean']:5.2f}s  max skew {s['mx']:5.1%}")
    loo = [summarise([d for j, d in enumerate(full) if j != i],
                     REC["tol1"], REC["tol2"], REC["min_sub"])["mx"]
           for i in range(len(full))]
    say(f"  leave-one-out max skew at the recommendation: "
        f"{min(loo):.1%} .. {max(loo):.1%} (median {np.median(loo):.1%}) -- "
        f"the worst case is not one episode")

    # ---- 8. the eight-channel .all() ----------------------------------------
    say("\n=== 8. which channels the test runs over " + "=" * 37)
    e8 = [d for d in full if d["n"] == 8]
    for ch, nm in ((SOLID, "solid only (ch0-3)"), (list(range(8)), "all 8")):
        s = summarise(e8, 1.50, 1.20, 4, ch=ch)
        say(f"  {nm:20s} fired {s['n']:2d}/{len(e8)}  mean {s['mean']:5.2f}s  "
            f"max skew {s['mx']:5.1%}")
    say("  per-episode smallest test-(1) tolerance:")
    for d in e8:
        out = []
        for ch, nm in ((SOLID, "ch0-3"), (list(range(8)), "ch0-7")):
            r = min(stationary(d["sw"][:k], ch, 9e9, 9e9, CALIB_AGREE_N)[0]
                    for k in range(CALIB_AGREE_N, d["nsw"] + 1)
                    if d["cl"][k - 1][0] - d["t0"] < CALIBRATION_S)
            out.append(f"{nm} {r:5.2f}")
        say(f"    {d['tag']} ep{d['ep']}   " + "   ".join(out))

    # ---- 9. the recommendation, episode by episode --------------------------
    say("\n=== 9. the recommendation, per episode " + "=" * 39)
    s = summarise(full, REC["tol1"], REC["tol2"], REC["min_sub"])
    rows = sorted(s["rows"], key=lambda r: -r[2])
    say(f"  CALIB_AGREE_TOL = {REC['tol1']}, second test kept at {REC['tol2']}, "
        f"CALIB_MIN_SUBWINDOWS = {REC['min_sub']}")
    say(f"  fired {s['n']}/{s['tot']}, mean calibration {s['mean']:.2f} s "
        f"(was {CALIBRATION_S:.0f}), median exit {s['medt']:.2f} s")
    say(f"  worst-channel skew vs the same window's 20 s answer: "
        f"median {s['med']:.1%}, p90 {s['p90']:.1%}, max {s['mx']:.1%} "
        f"(worst on the HIGH side {s['hi']:.1%}) against a 35% line")
    for d, t, a, sg in rows[:8]:
        say(f"    {a:6.1%} ({sg:+.1%})  {d['tag']} ep{d['ep']} "
            f"{d['n']}ch  exit {t:5.2f}s  window decay {d['decay']:.2f}")
    dump("recommended", ["run", "episode", "channels", "exit_s",
                         "worst_solid_skew", "signed_skew", "window_decay"],
         [[d["tag"], d["ep"], d["n"], f"{t:.2f}", f"{a:.4f}", f"{sg:.4f}",
           f"{d['decay']:.3f}"] for d, t, a, sg in rows])
    say(f"\n  tables written to {OUT}/calib_tol_*.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
