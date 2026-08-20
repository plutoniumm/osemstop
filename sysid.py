"""
sysid.py -- the measurement engine behind osem.sysid.py.
========================================================
NOT a controller. It damps nothing. It drives the coils with a known excitation
and records what the OSEMs do, to recover the **actuation matrix** -- how much
each coil pushes each degree of freedom. That is `A`, and CLAUDE.md Sec 3 is
still open on its MAGNITUDES.

The front-end (`osem.sysid.py`) stays thin and the maths lives here: the
front-ends differ only in channel count and excitation, and a copy of the lock-in
per front-end would be one place per front-end for the same bug. (The rule this
once broke -- "every controller is standalone" -- was dropped repo-wide on
2026-08-17; the machinery now lives in `stdlib.py`.)

TWO METHODS
-----------
`stepped()`  one coil at a time, one frequency at a time. Nothing to
             disentangle: while coil j is driving, no other coil is. Slow, and
             completely assumption-free. This is the bench standard.

`multisine()` all coils at once, but each coil gets its own INTERLEAVED comb of
             frequency bins -- coil c drives bins c, c+C, c+2C, ... So at any
             one frequency exactly one coil is active, and the responses
             separate by frequency rather than by matrix inversion.

That second point is the whole reason this works where the previous attempt did
not. v4 drove all four coils with simultaneous PRBS and correlated at one lag.
The four drives were not independent -- in closed loop `u = -K y` with diagonal
K, and all sensors watching the same few modes, the input cross-spectrum is
near-singular. Measured on the 2026-08-03 logs: condition number over 0.4-3 Hz
had median 2.5e3 and peak 2.9e4, and the recovered LONG row disagreed with itself
between the two halves of one run, sign included. Interleaving in frequency makes
the input cross-spectrum diagonal by construction, so there is nothing to invert.

DEGREES OF FREEDOM
------------------
Four coils drive **2** DOF; eight drive **4**, the fourth being rotation. So the
measured N x N matrix is RANK-DEFICIENT on purpose, and `report()` checks that:
the singular values should show 2 significant directions for four coils and 4 for
eight. That is a real test of the measurement -- it independently reproduces v4's
"2 of 4 modes carry real motion, 98.8% / 1.2%" finding, and if the rank comes out
wrong the measurement is wrong.
"""

import json
import math
import os
import time
from datetime import datetime

import numpy as np

import stdlib

A_VCC, ADC_MAX_COUNTS = 5.02, 1023
BIAS_V = 0.25                       # coil operating point; see CLAUDE.md Sec 3
# THIS TOOL'S OWN WINDOW, and it is deliberately narrower than the ladder's. The
# rungs opened to VMIN, VMAX = 0.0, 2.5 on 2026-08-20 because a closed loop was
# clipping; this runs OPEN LOOP with nothing damping, so it stays at the
# long-standing 0.0-0.5 V until somebody measures what the coil driver tolerates
# (CLAUDE.md Sec 11 -- that circuit is not in this repo).
VMIN, VMAX = 0.0, 0.5
RAIL_LOW, RAIL_HIGH = 12, 1011      # raw counts, same as the ladder uses

COUNTS_TO_V = A_VCC / ADC_MAX_COUNTS


def park(dac, chans):
    """Every named coil back to this module's BIAS_V."""
    park_at(dac, chans, [BIAS_V] * len(chans))


def park_at(dac, chans, volts):
    """Per-coil park. `stdlib.park` owns the best-effort loop, so a closing port
    cannot raise out of a `finally` and leave the rest of the coils energised."""
    stdlib.park(dac, chans, volts)


# --------------------------------------------------------------------------
# acquisition
# --------------------------------------------------------------------------
def _clamp(v):
    return min(max(v, VMIN), VMAX)


class Recorder:
    """Streams every raw sample to disk as it arrives, and closes at the end.

    The analysis below reduces a whole run to one complex matrix. That is the
    deliverable, but it is also lossy and one-shot: a bug in the lock-in, a
    frequency list that turned out too narrow, or a drift you only notice
    afterwards would each cost another 25-50 minutes on the bench with the optic
    tied up. So the raw counts go to disk as they arrive and everything else is
    derived from them.

    Line-buffered and closed in the caller's `finally`, so a Ctrl+C or a crash
    mid-sweep keeps everything recorded up to that point -- the same guarantee
    the controllers' own CSV gives.
    """

    def __init__(self, tag, ncols, coils):
        os.makedirs("data", exist_ok=True)
        self.path = os.path.join(
            "data", datetime.now().strftime("%Y%m%d_%H%M%S") + "_%s_raw.csv" % tag)
        self.fh = open(self.path, "w", buffering=1)
        self.fh.write("time_s,seg,phase,coil,freq_hz,"
                      + ",".join("u%d" % c for c in coils) + ","
                      + ",".join("a%d" % i for i in range(ncols)) + "\n")
        self.coils = list(coils)
        self.seg, self.phase, self.coil, self.freq = 0, "record", -1, -1.0
        self.n = 0
        print("  raw samples -> %s" % self.path)

    def mark(self, phase, coil=-1, freq=-1.0):
        """Start a new segment. `phase` is settle|record so the analysis can
        drop settling without guessing where it ended."""
        self.seg += 1
        self.phase, self.coil, self.freq = phase, coil, freq

    def write(self, t, counts, u):
        # The header has one u-column per coil, but stepped() hands acquire() a
        # SINGLE coil at a time, so `u` arrives length 1 while the header expects
        # len(self.coils). Left as-is that wrote 14 fields against a 21-field
        # header and shifted every a-column left by 7 on read-back. Scatter the
        # driven value into its own slot and hold the rest at bias, which is what
        # the hardware was actually doing.
        if len(u) != len(self.coils):
            full = [BIAS_V] * len(self.coils)
            if len(u) == 1 and self.coil in self.coils:
                full[self.coils.index(self.coil)] = u[0]
            u = full
        self.fh.write("%.5f,%d,%s,%d,%.5f,%s,%s\n"
                      % (t, self.seg, self.phase, self.coil, self.freq,
                         ",".join("%.4f" % v for v in u),
                         ",".join(str(int(c)) for c in counts)))
        self.n += 1

    def close(self):
        if self.fh and not self.fh.closed:
            self.fh.close()
            print("\n  raw data: %d samples -> %s" % (self.n, self.path))


def acquire(dac, ncols, seconds, drive=None, coils=(), update_hz=50.0, rec=None):
    """Hold or modulate the coils for `seconds` and collect samples.

    `drive(t) -> list of volts (one per entry of `coils`)`, or None to hold
    everything at bias. Coils are refreshed at `update_hz`, not once per sample:
    SET shares the wire with the stream, and at 115200 baud the stream alone is
    already at capacity, so every write costs samples.

    Returns (t, counts, u) -- u being the volts ACTUALLY commanded at each
    sample, less bias. The analysis divides by the lock-in of u rather than by
    the amplitude it intended, which makes it immune to the per-tone phase
    offsets, to clamping at VMIN/VMAX, and to the coil update rate being coarser
    than the sample rate. It also makes the in-run result identical to what you
    get by re-analysing the recorded CSV, which is the point of recording it.

    Timestamps are the host's arrival times and are NOT uniform, which is why
    the lock-in integrates with real dt rather than assuming a fixed rate.
    """
    t0 = time.time()
    next_update = 0.0
    held = [BIAS_V] * len(coils)
    ts, rows, us = [], [], []
    while True:
        now = time.time() - t0
        if now >= seconds:
            break
        if drive is not None and now >= next_update:
            next_update = now + 1.0 / update_hz
            held = [_clamp(BIAS_V + v) for v in drive(now)]
            dac.set_many(coils, held)
        s = dac.read_sample(ncols)
        if s is not None:
            t = time.time() - t0
            ts.append(t)
            rows.append(s)
            us.append(list(held))
            if rec is not None:
                rec.write(t, s, held)
    return (np.asarray(ts), np.asarray(rows, dtype=float),
            np.asarray(us, dtype=float) - BIAS_V)


def railed(counts, frac=0.02):
    """Per-sensor: did this channel spend more than `frac` of the window pinned?
    A railed sensor is outside its linear range, so its magnitude and phase are
    both meaningless and the step has to be discarded rather than averaged in."""
    if len(counts) == 0:
        return np.zeros(0, bool)
    bad = (counts <= RAIL_LOW) | (counts >= RAIL_HIGH)
    return bad.mean(axis=0) > frac


def lockin(t, y, f):
    """Complex amplitude of `y` at frequency `f`, integrating with real dt.

    Synchronous detection against both quadratures at once: the real and
    imaginary parts ARE the two quadratures, so magnitude and phase both fall
    out and the sign never depends on having guessed a quadrature in advance --
    which is the specific mistake that produced the wrong actuation matrix on the
    first attempt.
    """
    if len(t) < 8:
        return 0j
    dt = np.gradient(t)
    span = t[-1] - t[0]
    if span <= 0:
        return 0j
    return 2.0 * np.sum(y * np.exp(-2j * math.pi * f * t) * dt) / span


# --------------------------------------------------------------------------
# the two methods
# --------------------------------------------------------------------------
def stepped(dac, ncols, coils, dac_map, freqs, amp, dwell, settle, log=print,
            rec=None):
    """One coil, one frequency, at a time. Returns H[nfreq, nsens, ncoil]."""
    H = np.zeros((len(freqs), ncols, len(coils)), complex)
    ok = np.ones((len(freqs), ncols, len(coils)), bool)
    total = len(coils) * len(freqs)
    n = 0
    for cj, coil in enumerate(coils):
        ch = dac_map[coil]
        for fi, f in enumerate(freqs):
            n += 1
            log("  [%d/%d] coil %d (DAC ch%d) @ %.3f Hz  settle %.0fs + dwell %.0fs"
                % (n, total, coil, ch, f, settle, dwell))
            drive = lambda tt, _f=f: [amp * math.sin(2 * math.pi * _f * tt)]
            dac.drain()
            if rec is not None:
                rec.mark("settle", coil, f)
            acquire(dac, ncols, settle, drive, [ch], rec=rec)  # transient: recorded
            if rec is not None:                                # but not integrated
                rec.mark("record", coil, f)
            t, c, u = acquire(dac, ncols, dwell, drive, [ch], rec=rec)
            if len(t) < 16:
                log("      !! only %d samples -- skipped" % len(t))
                ok[fi, :, cj] = False
                continue
            bad = railed(c)
            if bad.any():
                log("      !! railed: %s" % ", ".join("ch%d" % i
                                                      for i in np.where(bad)[0]))
            U = lockin(t, u[:, 0], f)          # what the coil was actually given
            if abs(U) < 1e-6:
                log("      !! no drive measured on the coil -- skipped")
                ok[fi, :, cj] = False
                continue
            for i in range(ncols):
                H[fi, i, cj] = lockin(t, c[:, i] * COUNTS_TO_V, f) / U
            ok[fi, :, cj] = ~bad
        park(dac, [dac_map[c] for c in coils])
    return H, ok


def multisine(dac, ncols, coils, dac_map, freqs, amp, duration, update_hz,
              settle, log=print, rec=None):
    """All coils at once, frequency-interleaved, ROTATED over C passes.

    In pass p, coil c drives bins `freqs[(c + p) % C :: C]`. No two coils ever
    share a bin, so the response at each bin belongs to exactly one coil and
    separates by frequency instead of by matrix inversion -- which is the whole
    point, and why the ill-conditioning that killed the previous attempt cannot
    arise here.

    WHY IT ROTATES, and it must. One pass measures only the column of the coil
    that happens to own each bin, leaving the rest of that frequency's matrix
    empty -- so a single pass yields a rank-1 matrix per frequency no matter how
    good the data is. Columns cannot simply be pasted together across
    frequencies either: the response at f is S.diag(G(f)).B, and G is a
    PER-DOF factor, not a scalar, so two columns measured at different
    frequencies are scaled differently and their ratio is meaningless. Rotating
    the assignment C times gives every coil a turn at every bin, so each
    frequency ends up with a complete matrix measured under one G.

    Cost is C passes rather than C x len(freqs) dwells: it stays ahead of
    stepped sine, and the margin grows with the number of frequencies.
    """
    C = len(coils)
    chans = [dac_map[c] for c in coils]
    nper = max(len(freqs[0::C]), 1)
    per = amp / max(math.sqrt(nper), 1.0)          # keeps the summed peak bounded

    H = np.zeros((len(freqs), ncols, C), complex)
    ok = np.zeros((len(freqs), ncols, C), bool)

    for p in range(C):
        plan = [freqs[(c + p) % C::C] for c in range(C)]
        log("  pass %d/%d:" % (p + 1, C))
        for c, fs in enumerate(plan):
            log("    coil %d (DAC ch%d) <- %s Hz"
                % (coils[c], chans[c], np.round(fs, 3).tolist()))

        def drive(tt, _plan=plan):
            return [sum(per * math.sin(2 * math.pi * f * tt + 0.7 * k)
                        for k, f in enumerate(_plan[c])) for c in range(C)]

        dac.drain()
        if rec is not None:
            rec.mark("settle", -1, -1.0)
        acquire(dac, ncols, settle, drive, chans, update_hz, rec=rec)
        if rec is not None:
            rec.mark("record", -1, -1.0)
        t, counts, u = acquire(dac, ncols, duration, drive, chans, update_hz,
                               rec=rec)
        park(dac, chans)
        log("    %d samples, %.1f Hz effective"
            % (len(t), len(t) / max(t[-1] if len(t) else 1.0, 1e-9)))
        bad = railed(counts)
        if bad.any():
            log("    !! railed: %s" % ", ".join("ch%d" % i for i in np.where(bad)[0]))
        for c in range(C):
            for f in plan[c]:
                fi = int(np.argmin(np.abs(np.asarray(freqs) - f)))
                U = lockin(t, u[:, c], f)      # carries this tone's own phase
                if abs(U) < 1e-6:
                    continue
                for i in range(ncols):
                    H[fi, i, c] = lockin(t, counts[:, i] * COUNTS_TO_V, f) / U
                ok[fi, :, c] = ~bad
    return H, ok


def snap_bins(lo, hi, n, duration):
    """`n` log-spaced frequencies snapped to exact bins k/duration.

    Each tone then completes a whole number of cycles inside the record, so it
    leaks no energy into a neighbouring bin -- which is what keeps one coil's
    excitation out of another coil's measurement in `multisine()`. Without this
    the interleaving is only approximate and the crosstalk it is meant to remove
    comes back in through spectral leakage.
    """
    ks = np.unique(np.round(np.geomspace(lo, hi, n) * duration).astype(int))
    return ks[ks > 0] / float(duration)


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def report(H, ok, freqs, coils, expected_dof, log=print):
    """Magnitude/phase per pair, and the rank check that says whether to believe
    any of it."""
    log("\n" + "=" * 72)
    for fi, f in enumerate(freqs):
        if not ok[fi].any():
            continue
        log("\n  %.3f Hz    magnitude (V/V), phase (deg)" % f)
        log("        " + "".join("  coil %-14d" % c for c in coils))
        for i in range(H.shape[1]):
            cells = []
            for cj in range(H.shape[2]):
                z = H[fi, i, cj]
                cells.append("  %8.4f /%+7.1f" % (abs(z), np.degrees(np.angle(z)))
                             if ok[fi, i, cj] else "        --  railed")
            log("   ch%d %s" % (i, "".join(cells)))

    log("\n" + "=" * 72)
    log("  RANK CHECK -- %d coils should drive %d DOF" % (len(coils), expected_dof))

    # The primary test STACKS every frequency: (nfreq*nsens) x ncoil. Its column
    # rank is the number of independent ways the coils can excite the optic,
    # which is the DOF count. Doing this per frequency instead would be wrong --
    # at a resonance one mode swamps the others and the matrix legitimately looks
    # rank 1, so a per-frequency test fails on perfectly good data.
    usable = [fi for fi in range(len(freqs)) if ok[fi].all()]
    if usable:
        stack = np.vstack([H[fi] for fi in usable])
        s = np.linalg.svd(stack, compute_uv=False)
        s = s / max(s[0], 1e-30)
        log("  stacked over %d usable frequencies:" % len(usable))
        log("   singular values  %s" % "  ".join("%.3f" % v for v in s))
        log("   directions above 10%%: %d      above 3%%: %d      expected %d"
            % (int((s > 0.10).sum()), int((s > 0.03).sum()), expected_dof))
        # Deliberately NOT a pass/fail. Validated against a fake board with a
        # known rank-4 matrix: the SAME correct measurement reported 3, 4 or 5
        # directions purely according to which frequencies were probed, while
        # the recovered matrix itself matched truth to corr 0.99-1.00. A DOF only
        # shows up in the singular values if the sweep actually excited it, so a
        # hard threshold here would condemn good data.
        log("\n  Read this as a diagnostic, not a verdict. A DOF only appears if the")
        log("  sweep excited it, so an UNDER-count usually means the frequency list")
        log("  missed a resonance -- widen it and re-run before doubting the rig.")
        log("  An OVER-count is the informative failure: it means energy is arriving")
        log("  by a path the model does not have -- crosstalk between coils, drift,")
        log("  or one coil's tones leaking into another's bins.")
        if int((s > 0.03).sum()) < expected_dof:
            log("\n  !! fewer than %d directions even at 3%%. Most likely the sweep did"
                % expected_dof)
            log("     not reach every resonance; next likely is a coil that never moved")
            log("     (check the map and the wiring before trusting any of the above).")
    else:
        log("  no frequency had all channels clean -- nothing to check.")

    log("\n  per frequency (diagnostic only -- rank drops to 1 near a resonance,")
    log("  where one mode dominates, and that is correct rather than a fault):")
    for fi, f in enumerate(freqs):
        if not ok[fi].all():
            continue
        s = np.linalg.svd(H[fi], compute_uv=False)
        s = s / max(s[0], 1e-30)
        log("   %6.3f Hz   %s   rank~%d"
            % (f, "  ".join("%.3f" % v for v in s), int((s > 0.10).sum())))


def save(H, ok, freqs, coils, tag, extra=None, raw_path=None):
    os.makedirs("data", exist_ok=True)
    path = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S")
                        + "_%s.json" % tag)
    with open(path, "w") as fh:
        json.dump(dict(tag=tag, freqs=list(map(float, freqs)),
                       coils=list(map(int, coils)), raw_csv=raw_path,
                       real=H.real.tolist(), imag=H.imag.tolist(),
                       ok=ok.tolist(), **(extra or {})), fh, indent=1)
    print("\n  saved %s" % path)
    return path
