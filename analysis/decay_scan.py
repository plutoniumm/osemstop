#!/usr/bin/env python3
"""Opportunistic decay rates from every controller CSV already on disk.

WHAT THIS IS FOR. README's per-rung column was a DAMPING DUTY -- the fraction of
a run spent in the DAMPING state. That is uptime. It says nothing about how fast
the optic stops ringing, which is the thing the whole stack exists to do. A decay
rate is a dissipation measurement; a duty and a `ratio` are not.

WHAT IT MEASURES. Runs before 2026-08-17 predate `jerk.py`, so there are no
commanded kicks in them -- but a rig on a bench gets bumped, faults clear, and
gains ramp, and each of those leaves an amplitude excursion followed by a decay.
This scans for those, fits ln(envelope) vs t over the decaying part, and reports
the distribution. **These are ACCIDENTAL transients of unknown provenance, not
commanded kicks.** They are weaker evidence than a `jerk.py` set and are marked as
such wherever they are quoted.

THE ONE RULE THAT MATTERS. A fit window must be DAMPING with a non-zero gain at
EVERY sample. On 2026-08-18 six previously-reported kicks turned out to have been
fitted across FAULT stretches where every gain was +0.0000; those decays measured
the FREE PLANT (0.0072 /s, tau > 138 s, `analysis/ringdown.md`) and not any
control law, and they were the whole basis of the "modal beats diagonal 5x"
claim. Windows that leave DAMPING part-way are discarded, counted, and the count
is printed -- a refusal that is invisible in the score is its own trap
(`CLAUDE.md`, the sign sweep).

METHOD, deliberately simple.

  envelope   median over live channels of `ch<i>_ratio`, exactly `jerk.py`'s
             `Tail.rows`. `ratio` is already a 1.0 s sliding RMS normalised to
             the run's calibration baseline, so no Hilbert transform and no
             extra smoothing. Live = `ch<i>_healthy == 1` where that column
             exists (2026-08-04 onwards); all channels where it does not
             (the four 2026-08-03 files).
  quiet      median envelope over all VALID samples of that run. `jerk.py` uses
             a trailing 30 s median instead; a whole-run median is cruder and is
             what makes this offline and one-pass. Every threshold below is a
             multiple of it, because measured quiet spans 20x across laws --
             0.08 (modal, signtest) to 1.58 (diagonal, signtest).
  excursion  a contiguous stretch with envelope > 1.40 x quiet that reaches
             1.80 x quiet somewhere inside it.
  fit        peak of the excursion to its first return to the 1.40 x band,
             least squares on log(envelope). Slope negated: positive is decay.

THRESHOLDS ARE `jerk.py`'s, NOT NEW ONES. 1.40 / 1.80 / 1.68 / 0.19 are
STABLE_MULT / KICK_MULT / KICK_RISE / R2_FLOOR, each with its measurement in that
file's constants block. Reusing them is the point: this tool has to be gradeable
against the only fits known to be valid.

CALIBRATION, and it reproduces. `jerk.py --replay` on the two files with valid
commanded kicks gives:

    data/20260817_213642_fast_lock.csv   0.1355 /s          r2 0.944
    data/20260818_024302_fast_lock.csv   0.1093/0.0624/0.1098   r2 0.92/0.85/0.42

This scanner recovers those same events from the same files (see the run at the
bottom of README's table discussion). If it ever stops doing so, the method is
wrong and the numbers here mean nothing.

WHAT IT CANNOT SAY. Nothing here separates DAMPING from HOLDING: a loop that
pins the optic and one that dissipates its energy look the same in `ratio`
(`CLAUDE.md` § *Not established*). Nor does it know what caused any transient,
so peaks are not matched across runs the way `jerk.py` matches hand kicks --
and decay rate is only weakly peak-dependent, but "weakly" has not been measured
on this rig either.

Usage:  python analysis/decay_scan.py 'data/*_fast_lock.csv' [-v]
"""

import csv
import glob
import math
import os
import re
import sys
from collections import Counter
from statistics import median

import numpy as np

# --- thresholds, all of them jerk.py's -------------------------------------
BAND_MULT = 1.40        # jerk.py STABLE_MULT: the quiet band, from how far a
                        # quiet plate wanders in TIME (diagonal p90/median
                        # 1.403 binds, modal 1.123).
TRIG_MULT = 1.80        # jerk.py KICK_MULT: an excursion must reach this.
                        # sqrt(1.40 * 2.32), 2.32 being the weakest measured
                        # peak-over-quiet of a real hand kick.
RISE_MULT = 1.68        # jerk.py KICK_RISE: the peak must be this much above
                        # the level it ROSE FROM, or it is a baseline drifting
                        # across the trigger. This is what kills the two false
                        # kicks of 2026-08-18 (peak 2.73 out of a stalled 2.253
                        # FAULT baseline).
R2_FLOOR = 0.19         # jerk.py R2_FLOOR, and jerk.py's own comment applies
                        # doubly here: A FREE RINGDOWN IS EXPONENTIAL TOO, so a
                        # good r2 says the envelope decayed, not that the loop
                        # decayed it. Only `state` and `gain` say that.
PREKICK_S = 5.0         # the "level it rose from" is a median over this much
PREKICK_LAG_S = 1.0     # history, ending this far before the excursion starts,
                        # because `ratio` is a 1.0 s sliding RMS.
PREKICK_MIN_N = 20      # ...and this many VALID samples must be in it. Valid
                        # only: a prekick level read off FAULT rows is a frozen
                        # `ratio` (CLAUDE.md § 1 -- 3.59 held for 1085 s), not a
                        # level. Consequence, stated because it is a real loss:
                        # a fault-clear transient has no valid baseline behind
                        # it and is therefore never fitted here.

GAIN_FLOOR_FRAC = 0.50  # a sample counts as driven only if max|gain| over the
                        # channels is at least this fraction of the run's own
                        # max. NOT a jerk.py number and NOT measured: the gain
                        # scheduler ramps 0.00024 -> 0.03500 over ~30 samples,
                        # and a decay fitted inside that ramp is nearly the free
                        # plant with extra steps. 0.5 is a round number chosen
                        # to exclude the ramp; on every file here it moves
                        # nothing, because the ramp is <0.1% of DAMPING rows.

MIN_FIT_S = 2.0         # a window shorter than 1.4 cycles of the slowest mode
                        # (0.72194 Hz) is not an envelope decay. Not measured.
MIN_FIT_N = 8           # jerk.py fit_decay's own minimum.
MAX_FIT_S = 180.0       # jerk.py DECAY_TIMEOUT_S. Nothing on this rig has ever
                        # taken that long to re-quiet under control.

FREE_PLANT = 0.0072     # /s, analysis/ringdown.md, tau > 138 s. The number every
                        # rate here has to beat to mean anything.


# ---------------------------------------------------------------------------
# which rung wrote which file
# ---------------------------------------------------------------------------
def rung_map():
    """CSV path -> rung name, read out of the session logs that name them.

    Nothing in a controller CSV records its own version, so the attribution is
    external: `bench/*/*.log` (the 2026-08-04 and 2026-08-06 campaigns, named
    v4..v12) and `data/*_jerk_*.log` / `data/signtest_*.log` (2026-08-17/18,
    named zeta/eta). Files no log mentions are reported as `?` rather than
    guessed -- CLAUDE.md's standing practice: if a number cannot be sourced, say
    that it cannot.
    """
    alias = {"v4": "alpha", "v9": "beta", "v12": "delta", "v0": "zero"}
    out = {}
    logs = sorted(glob.glob("bench/*/*.log")) + sorted(glob.glob("data/*.log"))
    for lg in logs:
        base = os.path.basename(lg)
        m = re.match(r"(v\d+(?:\d)?)_", base)
        if m:
            ver = alias.get(m.group(1), m.group(1))
        elif "_jerk_" in base:
            ver = base.split("_jerk_")[1].split(".")[0]
        elif base.startswith("signtest_"):
            ver = "zeta"                      # signtest only ever ran zeta
        else:
            continue
        try:
            txt = open(lg, errors="replace").read()
        except OSError:
            continue
        for p in set(re.findall(r"data/[0-9_]+_fast_lock\.csv", txt)):
            out.setdefault(os.path.abspath(p), ver)
    return out


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------
def load(path):
    """Stream one controller CSV into arrays. Returns None if unusable.

    Streams rather than loads: the 2026-08-06 files are 130-330 MB each and the
    whole tree is 3.3 GB of `_fast_lock.csv`.
    """
    fh = open(path, errors="replace")
    try:
        cols = next(csv.reader(fh))
    except StopIteration:
        return None
    idx = {k: i for i, k in enumerate(cols)}
    nch = sum(1 for c in cols if c.endswith("_counts"))
    if "time_s" not in idx or "state" not in idx or nch == 0:
        return None
    if "ch0_ratio" not in idx or "ch0_gain" not in idx:
        return None
    ti, si = idx["time_s"], idx["state"]
    rat = [idx["ch%d_ratio" % i] for i in range(nch)]
    gn = [idx["ch%d_gain" % i] for i in range(nch)]
    hd = [idx.get("ch%d_healthy" % i) for i in range(nch)]
    mi = idx.get("modal")
    ri = idx.get("mrank")

    T, E, G, D, M, R, NL = [], [], [], [], [], [], []
    states = Counter()
    for line in fh:
        f = line.split(",")
        if len(f) < len(cols):
            continue                     # a half-written final line
        try:
            live = [r for r, h in zip(rat, hd) if h is None or f[h] == "1"]
            if not live:
                continue
            v = [float(f[r]) for r in live]
            T.append(float(f[ti]))
            E.append(median(v))
            NL.append(len(v))
            G.append(max(abs(float(f[g])) for g in gn))
            st = f[si]
            states[st] += 1
            D.append(st == "DAMPING")
            M.append(f[mi] if mi is not None else "-")
            R.append(f[ri] if ri is not None else "-")
        except (ValueError, IndexError):
            continue
    fh.close()
    if len(T) < 100:
        return None
    return dict(path=path, nch=nch, t=np.asarray(T), env=np.asarray(E),
                gain=np.asarray(G), damp=np.asarray(D, bool),
                modal=M, mrank=R, nlive=np.asarray(NL), states=states,
                has_modal=mi is not None)


# ---------------------------------------------------------------------------
# fitting
# ---------------------------------------------------------------------------
def fit_decay(t, y):
    """Positive decay rate in 1/s and r2, or (nan, nan). jerk.py's fit_decay."""
    m = y > 1e-9
    if m.sum() < MIN_FIT_N:
        return float("nan"), float("nan")
    tt, ly = t[m] - t[m][0], np.log(y[m])
    A = np.column_stack([tt, np.ones_like(tt)])
    beta, *_ = np.linalg.lstsq(A, ly, rcond=None)
    pred = A @ beta
    ss = float(np.sum((ly - pred) ** 2))
    tot = float(np.sum((ly - ly.mean()) ** 2))
    return -float(beta[0]), (1.0 - ss / tot) if tot > 0 else float("nan")


def scan(d):
    """Every fittable decay in one run, plus why the others were refused."""
    t, env, damp, gain = d["t"], d["env"], d["damp"], d["gain"]
    gmax = float(gain.max()) if gain.size else 0.0
    valid = damp & (gain >= GAIN_FLOOR_FRAC * gmax) & (gain > 0)
    d["valid"] = valid
    why = Counter()
    if valid.sum() < MIN_FIT_N:
        return [], why, float("nan")

    quiet = float(np.median(env[valid]))
    d["quiet"] = quiet
    if not (quiet > 1e-9):
        return [], why, quiet
    band, trig = BAND_MULT * quiet, TRIG_MULT * quiet

    above = env > band
    n = len(env)
    fits = []
    i = 0
    while i < n:
        if not above[i]:
            i += 1
            continue
        s = i
        while i < n and above[i]:
            i += 1
        e = i                              # first sample back inside the band
        if e >= n:
            why["ran off the end of the record, no return to quiet"] += 1
            break
        seg = env[s:e]
        if seg.max() < trig:
            continue                       # a wander, never a transient
        p = s + int(np.argmax(seg))
        if t[e] - t[p] < MIN_FIT_S or (e - p) < MIN_FIT_N:
            why["decay shorter than %.0f s" % MIN_FIT_S] += 1
            continue
        if t[e] - t[p] > MAX_FIT_S:
            why["decay longer than %.0f s" % MAX_FIT_S] += 1
            continue

        # THE RULE. Every sample of the window, and of the rise into it, must be
        # DAMPING with a live gain. Anything else fits the free plant.
        if not valid[s:e + 1].all():
            why["left DAMPING or gain went to zero inside the window"] += 1
            continue

        # ...and it must have risen from an established damping baseline.
        lo, hi = t[s] - PREKICK_LAG_S - PREKICK_S, t[s] - PREKICK_LAG_S
        pre = (t >= lo) & (t <= hi) & valid
        if pre.sum() < PREKICK_MIN_N:
            why["no valid baseline in the %.0f s before it" % PREKICK_S] += 1
            continue
        base = float(np.median(env[pre]))
        if env[p] < RISE_MULT * base:
            why["peak under %.2fx the level it rose from" % RISE_MULT] += 1
            continue

        rate, r2 = fit_decay(t[p:e + 1], env[p:e + 1])
        if not math.isfinite(rate) or rate <= 0:
            why["fitted rate is not a decay"] += 1
            continue
        if not math.isfinite(r2) or r2 < R2_FLOOR:
            why["r2 under %.2f" % R2_FLOOR] += 1
            continue

        mo = set(d["modal"][p:e + 1])
        law = ("modal" if mo == {"1"} else
               "diag" if mo in ({"0"}, {"-"}) else "mixed")
        rk = Counter(d["mrank"][p:e + 1]).most_common(1)[0][0]
        fits.append(dict(t=float(t[p]), peak=float(env[p]),
                         over=float(env[p] / quiet), rise=float(env[p] / base),
                         rate=rate, r2=r2, win=float(t[e] - t[p]),
                         law=law, rank=rk,
                         nlive=int(np.median(d["nlive"][p:e + 1]))))
    return fits, why, quiet


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------
def duty(states):
    tot = max(sum(states.values()), 1)
    return {k: 100.0 * v / tot for k, v in states.items()}


def run_law(d):
    """The law the run was mostly under, over its VALID samples."""
    if not d.get("has_modal"):
        return "diag"
    v = d["valid"]
    seen = Counter(m for m, ok in zip(d["modal"], v) if ok)
    if not seen:
        return "-"
    top = seen.most_common(1)[0][0]
    frac = seen[top] / sum(seen.values())
    tag = {"1": "modal", "0": "diag"}.get(top, "?")
    return tag if frac > 0.98 else tag + "*"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv[1:]
    pats = args or ["data/*_fast_lock.csv"]
    paths = sorted({p for pat in pats for p in glob.glob(pat)})
    if not paths:
        sys.exit("  nothing matched %s" % pats)
    rungs = rung_map()

    print("  decay_scan: %d file(s). Envelope = median chN_ratio over live "
          "channels." % len(paths))
    print("  Fit windows are DAMPING with gain >= %.2f x the run's max, every "
          "sample." % GAIN_FLOOR_FRAC)
    print("  Thresholds are jerk.py's: band %.2fx quiet, trigger %.2fx, rise "
          "%.2fx, r2 >= %.2f.\n" % (BAND_MULT, TRIG_MULT, RISE_MULT, R2_FLOOR))

    hdr = ("  %-30s %-7s %-6s %8s %7s %7s %7s %6s %5s  %s"
           % ("file", "rung", "law", "rows", "damp%", "fault%", "calib%",
              "quiet", "fits", "decay 1/s (r2)"))
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    allfits, skipped, whytot = [], [], Counter()
    for p in paths:
        d = load(p)
        if d is None:
            skipped.append(p)
            continue
        fits, why, quiet = scan(d)
        whytot.update(why)
        rung = rungs.get(os.path.abspath(p), "?")
        du = duty(d["states"])
        for f in fits:
            f["file"], f["rung"] = os.path.basename(p), rung
            f["date"] = os.path.basename(p)[:8]
        allfits += fits
        shown = "  ".join("%.4f (%.2f)" % (f["rate"], f["r2"]) for f in fits[:4])
        if len(fits) > 4:
            shown += "  +%d" % (len(fits) - 4)
        print("  %-30s %-7s %-6s %8d %7.1f %7.1f %7.1f %6.3f %5d  %s"
              % (os.path.basename(p), rung, run_law(d), len(d["t"]),
                 du.get("DAMPING", 0.0), du.get("FAULT", 0.0),
                 du.get("CALIBRATING", 0.0),
                 quiet if math.isfinite(quiet) else float("nan"),
                 len(fits), shown or "-"))
        if verbose:
            for f in fits:
                print("        t=%7.1fs  peak %6.2f (%4.1fx quiet, %4.1fx rise)"
                      "  %5.1f s window  %s rank %s  %d live"
                      % (f["t"], f["peak"], f["over"], f["rise"], f["win"],
                         f["law"], f["rank"], f["nlive"]))

    if skipped:
        print("\n  SKIPPED %d file(s) -- empty, truncated, or no chN_ratio / "
              "chN_gain columns:" % len(skipped))
        for p in skipped:
            print("    %s" % os.path.basename(p))

    def summarise(title, groups):
        print("\n  %s" % title)
        print("  %-22s %5s %9s %9s %9s %9s %7s"
              % ("", "n", "median", "min", "max", "xfree", "med r2"))
        for k in sorted(groups):
            r = sorted(f["rate"] for f in groups[k])
            q = sorted(f["r2"] for f in groups[k])
            if not r:
                continue
            print("  %-22s %5d %9.4f %9.4f %9.4f %8.1fx %7.2f"
                  % (k, len(r), median(r), r[0], r[-1],
                     median(r) / FREE_PLANT, median(q)))

    if allfits:
        by = lambda key: {k: [f for f in allfits if f[key] == k]
                          for k in {f[key] for f in allfits}}
        summarise("BY LAW (the modal column over the fit window)", by("law"))
        summarise("BY RUNG (attributed from the session logs)", by("rung"))
        summarise("BY DATE", by("date"))
        print("\n  free plant, no control: %.4f /s (analysis/ringdown.md, "
              "tau > 138 s)" % FREE_PLANT)
    else:
        print("\n  NO USABLE FITS ANYWHERE.")

    if whytot:
        print("\n  REFUSED, and printed because a refusal that is invisible in "
              "the score is its own trap:")
        for k, v in whytot.most_common():
            print("    %5d  %s" % (v, k))
    print("\n  ACCIDENTAL TRANSIENTS, NOT COMMANDED KICKS. Their cause is not "
          "known and\n  their peaks are not matched across runs. Weaker "
          "evidence than a jerk.py set.")


if __name__ == "__main__":
    main()
