"""
bias_sweep.py -- does a coil's static bias move its own sensor's resting count?

CLAUDE.md item 6. Open loop, gain zero, nothing damping. Step one coil at a time
through 0.10 -> 0.40 V and record where all eight sensors come to rest.

WHAT IT DECIDES. Every channel rests ABOVE mid-scale (511.5) and clips the top
rail first: 601.2 / 630.3 / 686.5 / 681.8 counts, measured 2026-08-06 while
CALIBRATING over 22246 samples. Those were measured with BIAS = 0.25 V already
applied, and bias is a static force, so the offset may be electromagnetic rather
than mechanical.

    resting counts move with bias  ->  electromagnetic. Per-channel bias trim
                                       fixes the clipping at its source.
    resting counts do not move     ->  mechanical flag alignment. No amount of
                                       tuning helps and it is a screwdriver job.

The existing evidence is v7's adaptive trim (versions.md "v7's bias trim, on
hardware"): total offset 565 -> 455 counts, per channel a0 591->549, a1 630->603,
a2 712->670, a3 679->679. That points electromagnetic, but v7 searched by
accept/reject on the TOTAL offset, so no per-channel slope ever came out of it,
and the bias values it landed on are not recorded per step. This measures the
slope directly.

WHY THE DWELL IS 30 SECONDS AND NOT 3. The plant is effectively undamped:
Q > 433, tau > 138 s (analysis/ringdown.md). A bias step is a static force step,
so it rings the optic up and the ringing does NOT decay inside any dwell we can
afford. It does not have to. The oscillation is zero-mean about the NEW
equilibrium, so averaging over many mode periods returns that equilibrium
directly. 30 s is 21 periods of the slowest mode (0.7155 Hz, 1.398 s), so the
residual from the partial last cycle is about 1/(2*pi*21) = 0.8% of the swing.
Mode B is the largest line at 0.306 V rms on a0 (analysis/out/kalman_lines.csv),
about 88 counts peak, so that residual is under a count against the tens of
counts the trim moved. The per-level standard deviation is recorded so this is
checkable rather than assumed.

WHY IT RAMPS BETWEEN LEVELS. Same reason. A hard 0.05 V step is an impulse into
an undamped pendulum. Every transition is one 0.05 V level, ramped over 5 s
(0.01 V/s), which is slow against all three mode periods (1.398 / 1.005 /
0.610 s). Ramp samples are recorded but tagged `ramp` so the analysis drops them
without guessing where the transition ended.

WHY THE LEVEL LIST GOES UP THEN DOWN. 0.30, 0.35, 0.25 and 0.20 are each visited
twice and 0.25 three times. The spread between repeat visits is the drift and
hysteresis, measured in the same run rather than assumed absent. Any per-channel
slope smaller than that spread is not a result.

SAFETY. Open loop: nothing is damping while this runs, and the optic will ring.
Output is clamped to the controllers' own 0.0-0.5 V window. All eight coils are
returned to BIAS on exit, Ctrl+C included. Sensors that rail at a level have that
level flagged rather than averaged in, because a railed reading is pinned at the
end of the ADC range and carries no position information at all.

The 0.0-0.5 V window is itself unsourced (CLAUDE.md item 5) and this sweep stays
inside it. Note that v7 commanded 1.25 V on this hardware without incident, so a
wider sweep is available if somebody decides the window can move.

    python analysis/bias_sweep.py                    # coils 0-3, ~26 min
    python analysis/bias_sweep.py --coils 0,1,2,3,4,5,6,7   # ~51 min
    python analysis/bias_sweep.py --dwell 15 --ramp 3       # ~13 min, coarser
    python analysis/bias_sweep.py --replay data/<file>_bias_raw.csv   # no bench
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PORT = "/dev/cu.usbserial-1120"
NCOLS = 8
BIAS_V = 0.25
VMIN, VMAX = 0.0, 0.5
A_VCC, ADC_MAX_COUNTS = 5.02, 1023
COUNTS_TO_V = A_VCC / ADC_MAX_COUNTS
MID_COUNTS = 511.5
RAIL_LOW, RAIL_HIGH = 12, 1011
RAIL_FRAC_REJECT = 0.02

DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]      # sensor index -> DAC channel

LEVELS = [0.25, 0.30, 0.35, 0.40, 0.35, 0.30,
          0.25, 0.20, 0.15, 0.10, 0.25]

RAMP_S = 5.0
DWELL_S = 30.0
SETTLE_BETWEEN_COILS_S = 20.0
FLUSH_EVERY_N = 500


# The output ceiling every commanded volt is clipped to. Defaults to the
# controllers' own VMAX so the full sweep cannot leave their window by accident.
# `search()` raises it to SEARCH_HARD_CAP, because the root it is looking for is
# provably outside that window (see search()'s docstring) and a silent clamp at
# 0.5 V would have returned a flat, wrong answer for every channel rather than
# an error. Nothing else may raise it.
V_CEIL = VMAX


def clamp(v):
    return min(max(float(v), VMIN), V_CEIL)


class Recorder:
    """Every raw sample to disk as it arrives; closed in the caller's finally.

    A 26 minute sweep reduced to one slope per channel is one-shot and lossy. If
    the dwell turns out too short, or a channel rails somewhere unexpected, or
    somebody later wants the transient instead of the mean, re-running costs
    another half hour with the optic tied up. So the counts go to disk raw and
    everything below is derived from them (CLAUDE.md, standing practice).

    Buffered with an explicit flush every FLUSH_EVERY_N rows rather than
    line-buffered: the stream runs near 1 kHz here, not the controllers' 100 Hz,
    and a flush per line is 1000 syscalls a second. The exposure is the last
    0.5 s of data on a hard kill, against a `finally` that closes cleanly on
    Ctrl+C.
    """

    def __init__(self, tag="bias"):
        os.makedirs("data", exist_ok=True)
        self.path = os.path.join(
            "data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_%s_raw.csv" % tag)
        self.fh = open(self.path, "w", buffering=1 << 16)
        self.fh.write("time_s,seg,phase,coil,bias_v,"
                      + ",".join("a%d" % i for i in range(NCOLS)) + "\n")
        self.seg, self.phase, self.coil = 0, "idle", -1
        self.n = 0
        print("  raw samples -> %s" % self.path)

    def mark(self, phase, coil=-1):
        self.seg += 1
        self.phase, self.coil = phase, coil

    def write(self, t, counts, bias_v):
        self.fh.write("%.5f,%d,%s,%d,%.4f,%s\n"
                      % (t, self.seg, self.phase, self.coil, bias_v,
                         ",".join(str(int(c)) for c in counts)))
        self.n += 1
        if self.n % FLUSH_EVERY_N == 0:
            self.fh.flush()

    def close(self):
        if self.fh and not self.fh.closed:
            self.fh.flush()
            self.fh.close()
            print("\n  raw data: %d samples -> %s" % (self.n, self.path))


def hold(dac, rec, coil, seconds, v_from, v_to, t0):
    """Hold or ramp one coil for `seconds`, reading every sample that arrives.

    Returns the counts collected, as (nsamples, NCOLS). The coil is refreshed
    only when the commanded value actually changes by more than half a DAC LSB,
    so a dwell issues one write and a ramp issues about one per 2 mV. Writes are
    fire-and-forget on this board (pyDAC2 ack=False since 2026-08-06), but they
    still share the wire with the stream.
    """
    rows = []
    last_cmd = None
    t_start = time.time()
    while True:
        frac = (time.time() - t_start) / seconds
        if frac >= 1.0:
            break
        v = clamp(v_from + (v_to - v_from) * min(frac, 1.0))
        if last_cmd is None or abs(v - last_cmd) > 2.5 / 4096:
            dac.set_voltage(DAC_MAP[coil], v)
            last_cmd = v
        s = dac.read_sample(NCOLS)
        if s is not None:
            rows.append(s)
            rec.write(time.time() - t0, s, v)
    return np.asarray(rows, dtype=float)


def summarise(counts):
    """Mean, std and railed fraction per sensor for one dwell."""
    if len(counts) == 0:
        nan = np.full(NCOLS, np.nan)
        return nan, nan, np.ones(NCOLS)
    railed = ((counts <= RAIL_LOW) | (counts >= RAIL_HIGH)).mean(axis=0)
    return counts.mean(axis=0), counts.std(axis=0), railed


SEARCH_LO, SEARCH_HI = 0.10, 1.10
SEARCH_HARD_CAP = 1.25
SEARCH_TOL_COUNTS = 5.0
SEARCH_MAX_PROBES = 7


def search(coils, dwell_s, ramp_s, port, yes=False, lo=SEARCH_LO, hi=SEARCH_HI):
    """Find, per coil, the bias that brings its own sensor to mid-scale.

    The full sweep measures the whole curve. This only needs the root, so it
    goes straight there: bracket, then false position with the Illinois
    modification. The response is close to linear over this range, and false
    position exploits that where plain bisection throws it away. Budget is
    SEARCH_MAX_PROBES probes rather than the sweep's 11 levels per coil.

    WHY IT SEARCHES ABOVE THE 0.0-0.5 V WINDOW. The partial sweep of
    2026-08-07 measured coil 0 at -111.9 counts/V with a repeat spread of 0.78
    counts, and a0 rests +84 counts above mid-scale (511.5). Cancelling that
    needs +0.75 V on top of the 0.25 V operating point, so the root is near
    1.00 V and there is NO root inside 0.0-0.5. Searching only the documented
    window would return "no bracket" on every channel and measure nothing.

    That window is unsourced (CLAUDE.md item 5) and this is the first thing in
    the repo to deliberately leave it, so the justification has to be explicit:
    v7 commanded 1.25 V on this same rig with no incident (versions.md, "v7's
    bias trim, on hardware" -- ch3 -> 1.25 V, ch1 -> 0.50 V), and the DAC's own
    limit is 2.5 V. SEARCH_HARD_CAP is set at v7's 1.25 V and nothing here goes
    past it. This is still OPEN LOOP with the gain at zero: a static bias is a
    static force, and no control authority is being raised. Raising VMAX for the
    CONTROLLER is a separate decision and this measurement does not make it.

    Each coil is searched alone with the other seven held at 0.25 V, so what
    comes back is each channel's INDEPENDENT optimum. The partial sweep also
    measured coil 0 moving a1 by -54 and a2/a3 by +30/+32 counts/V, so those
    optima will fight each other and the set of them is not a shipping trim.
    Every probe records all eight sensors, so the coupling matrix falls out of
    the same file and the joint solve is an offline problem, not another run.
    """
    from pyDAC2 import FastDAC

    global V_CEIL
    hi = min(hi, SEARCH_HARD_CAP)
    V_CEIL = SEARCH_HARD_CAP
    est = len(coils) * (SEARCH_MAX_PROBES * (dwell_s + ramp_s)
                        + SETTLE_BETWEEN_COILS_S)
    print("\n  bias search: %d coils, target mid-scale %.1f counts, "
          "bracket %.2f-%.2f V" % (len(coils), MID_COUNTS, lo, hi))
    print("  <= %d probes/coil, ramp %.0f s, dwell %.0f s -> about %.0f min."
          % (SEARCH_MAX_PROBES, ramp_s, dwell_s, est / 60.0))
    print("  ABOVE the 0.0-0.5 V window, capped at %.2f V. v7 reached 1.25 V"
          % hi)
    print("  on this rig without incident. Open loop, gain zero.")
    if not yes:
        input("  Press Enter to start (Ctrl+C to stop)... ")

    dac = FastDAC(port=port)
    rec = Recorder(tag="biassearch")
    rows, results = [], {}
    t0 = time.time()
    try:
        dac.set_many(list(range(8)), [BIAS_V] * 8)
        dac.start_stream()
        rec.mark("settle", -1)
        hold(dac, rec, coils[0], SETTLE_BETWEEN_COILS_S, BIAS_V, BIAS_V, t0)

        for coil in coils:
            prev = BIAS_V
            probes = []

            def probe(v):
                nonlocal prev
                rec.mark("ramp", coil)
                hold(dac, rec, coil, ramp_s, prev, v, t0)
                rec.mark("dwell", coil)
                c = hold(dac, rec, coil, dwell_s, v, v, t0)
                prev = v
                mean, std, railed = summarise(c)
                rows.append(dict(coil=coil, level=v, n=len(c),
                                 mean=mean, std=std, railed=railed))
                err = mean[coil] - MID_COUNTS
                probes.append((v, err))
                bad = "  RAILED" if railed[coil] > RAIL_FRAC_REJECT else ""
                print("  coil %d  probe %.4f V -> a%d = %7.1f  err %+7.1f%s"
                      % (coil, v, coil, mean[coil], err, bad))
                return err

            e_lo, e_hi = probe(lo), probe(hi)
            if not np.isfinite(e_lo) or not np.isfinite(e_hi):
                results[coil] = dict(root=None, why="probe returned no data",
                                     probes=list(probes))
            elif e_lo * e_hi > 0:
                # No sign change, so mid-scale is not reachable in this range.
                # Report the extrapolated root rather than a bare failure: the
                # slope is what decides whether it is out of reach by a little
                # or by a lot, and that is the number the next decision needs.
                slope = ((e_hi - e_lo) / (hi - lo)) if hi != lo else 0.0
                ext = (lo - e_lo / slope) if slope != 0 else None
                results[coil] = dict(
                    root=None, why="no sign change in %.2f-%.2f V" % (lo, hi),
                    slope=slope, extrapolated=ext, probes=list(probes))
                print("    no bracket: err %+.1f -> %+.1f counts, slope %.1f "
                      "counts/V" % (e_lo, e_hi, slope))
                if ext is not None:
                    print("    extrapolated root %.3f V (NOT measured)" % ext)
            else:
                a, fa, b, fb = lo, e_lo, hi, e_hi
                side = 0
                root = None
                for _ in range(SEARCH_MAX_PROBES - 2):
                    c_v = b - fb * (b - a) / (fb - fa)
                    c_v = min(max(c_v, min(a, b)), max(a, b))
                    fc = probe(c_v)
                    if abs(fc) <= SEARCH_TOL_COUNTS:
                        root = c_v
                        break
                    if fc * fb < 0:
                        a, fa, side = b, fb, 0
                    else:
                        # Illinois: halve the retained endpoint's value so one
                        # stagnant end cannot hold the bracket open forever.
                        if side == 1:
                            fa *= 0.5
                        side = 1
                    b, fb = c_v, fc
                    root = c_v
                results[coil] = dict(root=root, probes=list(probes))

            rec.mark("settle", coil)
            hold(dac, rec, coil, SETTLE_BETWEEN_COILS_S, prev, BIAS_V, t0)
    except KeyboardInterrupt:
        print("\n  stopped by user; reporting on what was collected.")
    finally:
        try:
            dac.set_many(list(range(8)), [BIAS_V] * 8)
            dac.stop_stream()
            dac.close()
        except Exception as exc:
            print("  WARNING: could not return coils to bias: %s" % exc)
        rec.close()
    return rows, results, rec.path


def search_report(results):
    print("\n" + "=" * 72)
    print("  BIAS SEARCH -- bias that puts each sensor at mid-scale %.1f"
          % MID_COUNTS)
    print("=" * 72)
    print("\n  coil   root (V)   vs shipping 0.25 V   inside 0.0-0.5 V?")
    for coil in sorted(results):
        r = results[coil]
        if r.get("root") is None:
            ext = r.get("extrapolated")
            note = r.get("why", "no root")
            if ext is not None:
                print("   %2d     none      %s; extrapolates to %.3f V"
                      % (coil, note, ext))
            else:
                print("   %2d     none      %s" % (coil, note))
        else:
            v = r["root"]
            print("   %2d     %.4f      %+.4f V            %s"
                  % (coil, v, v - BIAS_V, "yes" if v <= VMAX else "NO"))
    print("\n  Each coil was searched ALONE, others held at 0.25 V. The partial")
    print("  sweep measured coil 0 moving a1 by -54 and a2/a3 by +30/+32")
    print("  counts/V, so these independent optima will fight each other.")
    print("  The joint solve is offline work on the recorded file, not a rerun.")


def run(coils, dwell_s, ramp_s, port, yes=False):
    from pyDAC2 import FastDAC

    n_dwell = len(LEVELS)
    est = len(coils) * (n_dwell * (dwell_s + ramp_s) + SETTLE_BETWEEN_COILS_S)
    print("\n  bias sweep: %d coils x %d levels %s V"
          % (len(coils), n_dwell, LEVELS))
    print("  ramp %.0f s, dwell %.0f s -> about %.0f min."
          % (ramp_s, dwell_s, est / 60.0))
    print("  OPEN LOOP. Nothing damps while this runs and the optic will ring.")
    print("  Leave the bench alone: do not kick it, this measures resting position.")
    if not yes:
        input("  Press Enter to start (Ctrl+C to stop)... ")

    dac = FastDAC(port=port)
    rec = Recorder()
    rows = []
    t0 = time.time()
    try:
        dac.set_many(list(range(8)), [BIAS_V] * 8)
        dac.start_stream()
        rec.mark("settle", -1)
        hold(dac, rec, coils[0], SETTLE_BETWEEN_COILS_S,
             BIAS_V, BIAS_V, t0)

        for coil in coils:
            prev = BIAS_V
            for level in LEVELS:
                rec.mark("ramp", coil)
                hold(dac, rec, coil, ramp_s, prev, level, t0)
                rec.mark("dwell", coil)
                c = hold(dac, rec, coil, dwell_s, level, level, t0)
                prev = level

                mean, std, railed = summarise(c)
                rows.append(dict(coil=coil, level=level, n=len(c),
                                 mean=mean, std=std, railed=railed))
                flag = "  RAILED:%s" % ",".join(
                    "a%d" % i for i in range(NCOLS)
                    if railed[i] > RAIL_FRAC_REJECT) if (
                        railed > RAIL_FRAC_REJECT).any() else ""
                print("  coil %d  %.2f V  n=%5d  self a%d = %7.1f +/- %5.1f%s"
                      % (coil, level, len(c), coil, mean[coil], std[coil], flag))

            rec.mark("settle", coil)
            hold(dac, rec, coil, SETTLE_BETWEEN_COILS_S, prev, BIAS_V, t0)
    except KeyboardInterrupt:
        print("\n  stopped by user; reporting on what was collected.")
    finally:
        try:
            dac.set_many(list(range(8)), [BIAS_V] * 8)
            dac.stop_stream()
            dac.close()
        except Exception as exc:
            print("  WARNING: could not return coils to bias: %s" % exc)
        rec.close()
    return rows, rec.path


def replay(path):
    """Rebuild the per-level table from a recorded file. No bench needed.

    Buckets by SEGMENT, not by (coil, level). The level list visits 0.25 three
    times and 0.30 / 0.35 / 0.20 twice each, and those repeats are the entire
    repeatability estimate: merging them into one bucket per level silently
    throws the drift measurement away and leaves every channel looking perfectly
    repeatable. Segment is also what the live path produces, one row per dwell,
    so replaying a file now returns exactly what the run reported.
    """
    import csv
    buckets, meta = {}, {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            if r["phase"] != "dwell":
                continue
            seg = int(r["seg"])
            meta[seg] = (int(r["coil"]), round(float(r["bias_v"]), 4))
            buckets.setdefault(seg, []).append(
                [float(r["a%d" % i]) for i in range(NCOLS)])
    rows = []
    for seg in sorted(buckets):
        coil, level = meta[seg]
        c = np.asarray(buckets[seg])
        mean, std, railed = summarise(c)
        rows.append(dict(coil=coil, level=level, n=len(c),
                         mean=mean, std=std, railed=railed))
    return rows


def report(rows, raw_path):
    """Per-channel slope in counts per volt, and whether it beats the repeats."""
    if not rows:
        print("\n  nothing collected.")
        return

    os.makedirs("analysis/out", exist_ok=True)
    out = "analysis/out/bias_sweep.csv"
    with open(out, "w") as fh:
        fh.write("coil,bias_v,n," + ",".join(
            "a%d_mean,a%d_std,a%d_railed" % (i, i, i) for i in range(NCOLS)) + "\n")
        for r in rows:
            fh.write("%d,%.4f,%d," % (r["coil"], r["level"], r["n"]))
            fh.write(",".join("%.3f,%.3f,%.4f" % (r["mean"][i], r["std"][i],
                                                  r["railed"][i])
                              for i in range(NCOLS)) + "\n")

    coils = sorted({r["coil"] for r in rows})
    print("\n" + "=" * 72)
    print("  BIAS SWEEP -- does a coil's static bias move its own sensor?")
    print("=" * 72)
    print("\n  Raw:   %s\n  Table: %s" % (raw_path, out))

    print("\n  Repeatability (spread across repeat visits to the same level).")
    print("  Any slope below this is not a result.\n")
    print("  coil  level   visits   spread of self-sensor mean (counts)")
    repeat_spread = {}
    for coil in coils:
        worst = 0.0
        for level in sorted({r["level"] for r in rows if r["coil"] == coil}):
            vs = [r["mean"][coil] for r in rows
                  if r["coil"] == coil and r["level"] == level
                  and np.isfinite(r["mean"][coil])]
            if len(vs) > 1:
                spread = max(vs) - min(vs)
                worst = max(worst, spread)
                print("   %2d   %.2f V     %d          %6.2f" %
                      (coil, level, len(vs), spread))
        repeat_spread[coil] = worst

    print("\n  Self-response: sensor a<c> against its own coil <c>.")
    print("  Slope is a straight-line fit over all non-railed dwells.\n")
    print("  coil   slope (counts/V)   over 0.10-0.40 V   repeat spread   verdict")
    for coil in coils:
        pts = [(r["level"], r["mean"][coil]) for r in rows
               if r["coil"] == coil
               and r["railed"][coil] <= RAIL_FRAC_REJECT
               and np.isfinite(r["mean"][coil])]
        if len(pts) < 3:
            print("   %2d    insufficient clean dwells (%d)" % (coil, len(pts)))
            continue
        x = np.array([p[0] for p in pts])
        y = np.array([p[1] for p in pts])
        slope, _ = np.polyfit(x, y, 1)
        swing = slope * 0.30
        spread = repeat_spread.get(coil, 0.0)
        # The repeat spread is the noise floor and it is measured, not assumed:
        # it is the disagreement between visits to the SAME level in this run,
        # so it already contains the drift, the hysteresis and the ringing that
        # survived the 30 s average. A swing has to beat 3x it to be a result.
        # No constant floor is added. An earlier version used max(3*spread, 2.0)
        # counts, and on a channel with no repeats that collapsed to a flat 2.0
        # and reported a 3.5 count swing as a real response.
        if spread <= 0.0:
            verdict = "no repeats, undecidable"
        elif abs(swing) > 3.0 * spread:
            verdict = "MOVES"
        else:
            verdict = "flat (< 3x repeat spread)"
        print("   %2d      %9.1f          %8.1f         %7.2f       %s"
              % (coil, slope, swing, spread, verdict))

    print("\n  Cross-response: coil <c> against every sensor, counts/V.")
    print("  Off-diagonal entries are the four-coils-two-DOF coupling that made")
    print("  v7 reject two of its own trim steps.\n")
    hdr = "  coil |" + "".join("   a%d  " % i for i in range(NCOLS))
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for coil in coils:
        cells = []
        for s in range(NCOLS):
            pts = [(r["level"], r["mean"][s]) for r in rows
                   if r["coil"] == coil
                   and r["railed"][s] <= RAIL_FRAC_REJECT
                   and np.isfinite(r["mean"][s])]
            if len(pts) < 3:
                cells.append("   --  ")
                continue
            slope, _ = np.polyfit([p[0] for p in pts], [p[1] for p in pts], 1)
            cells.append("%6.0f " % slope)
        print("   %2d  |" % coil + "".join(cells))

    print("\n  Resting counts at the shipping bias (0.25 V), against mid-scale")
    print("  %.1f. Positive means clipping the TOP rail first.\n" % MID_COUNTS)
    base = [r for r in rows if abs(r["level"] - BIAS_V) < 1e-6]
    if base:
        m = np.nanmean([r["mean"] for r in base], axis=0)
        print("  " + "  ".join("a%d %6.1f (%+.0f)" % (i, m[i], m[i] - MID_COUNTS)
                               for i in range(4)))
        print("  " + "  ".join("a%d %6.1f (%+.0f)" % (i, m[i], m[i] - MID_COUNTS)
                               for i in range(4, NCOLS)))

    print("\n  READING IT")
    print("  MOVES on the diagonal  -> the offset is electromagnetic. Per-channel")
    print("                            bias trim is worth building into epsilon,")
    print("                            and the slope above sizes each trim.")
    print("  flat on the diagonal   -> mechanical flag alignment. No tuning helps;")
    print("                            write it up as a request, do not tune it.")
    print("  Headroom needed: a channel at +170 counts needs its slope to reach")
    print("  -170 counts inside 0.10-0.40 V, or the window has to move first")
    print("  (CLAUDE.md item 5, the unsourced 0.0-0.5 V).")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--coils", default="0,1,2,3")
    ap.add_argument("--dwell", type=float, default=DWELL_S)
    ap.add_argument("--ramp", type=float, default=RAMP_S)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt")
    ap.add_argument("--replay", default=None,
                    help="re-report from a recorded raw CSV; no hardware")
    ap.add_argument("--full", action="store_true",
                    help="the whole %d-level sweep instead of the search"
                         % len(LEVELS))
    ap.add_argument("--lo", type=float, default=SEARCH_LO)
    ap.add_argument("--hi", type=float, default=SEARCH_HI)
    a = ap.parse_args()

    if a.replay:
        report(replay(a.replay), a.replay)
        return

    coils = [int(c) for c in a.coils.split(",") if c.strip() != ""]
    if a.full:
        rows, raw_path = run(coils, a.dwell, a.ramp, a.port, a.yes)
        report(rows, raw_path)
    else:
        rows, results, raw_path = search(coils, a.dwell, a.ramp, a.port,
                                         a.yes, a.lo, a.hi)
        report(rows, raw_path)
        search_report(results)


if __name__ == "__main__":
    main()
