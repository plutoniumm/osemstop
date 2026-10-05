"""Qualify the COILS of channels 4-7 (and re-test 5) from data already on disk.

    .venv/bin/python analysis/coil_qual.py [raw_csv]

THE QUESTION. A working sensor says nothing about its coil. The kick of
2026-08-06 moved all eight sensors -- a4 went 36 -> 879 counts peak-to-peak and
never railed -- so a4/a6/a7 are LOW-GAIN sensors, not broken ones. That settles
the sensors. It settles nothing about the coils.

WHY THE DIAGONAL CANNOT ANSWER IT. Every gain decision in this repo rests on
P_mj = R_m[j][j] = s_m[j] * a_m[j], a PRODUCT of the sensor gain and the coil
gain at channel j. A product at the noise floor does not say which factor is at
the noise floor. a5 is the standing case: in-band baseline 51% of the median and
visibly damping (0.0860 -> 0.0420 V, v10), yet its residue diagonal was rejected.

WHAT THIS FILE DOES INSTEAD -- and it needs no mode fit at all, which matters
because the mode fit is currently unusable (f = 0.7149 Hz at Q = 288 and
0.9941 Hz at Q = 2334, against a ringdown-measured Q of 20-22; see
analysis/mimo_design.md section 7). Work directly on H(f), the measured
sensor x coil response, where the rank-one structure gives two exact ratios:

    COLUMN j, over the SENSITIVE rows i:   H[i][j] / H[i][0] = a_j / a_0
    ROW    j, over the STRONG coils i:     H[j][i] / H[0][i] = s_j / s_0

s_m[i] cancels out of the first and a_m[i] out of the second, so a coil whose
own OSEM is too insensitive to see it is still qualified ON THE SENSORS THAT
ARE SENSITIVE. That is the only route by which channels 4-7 can be qualified.

THE SIGN, AND WHY IT IS ONE NUMBER PER CHANNEL. Colocation (sensor and coil are
the same physical unit, a few mm from the same flag) gives

    s_m[j] = sigma_j psi_mj      a_m[j] = g_j psi_mj
    P_mj   = sigma_j g_j psi_mj^2 = lambda_j psi_mj^2

psi^2 is a square, so sign(P_mj) = sign(lambda_j) for EVERY mode. The sign is
one per-channel constant, not a matrix -- confirmed on ch0-ch3, where the
measured residues give [-, -, +, -] in both modes, matching the DC diagonal
(-105 / -68 / +213 / -51) and matching v12's shipping Kp.

WHAT A COMPLEX RATIO MEANS HERE. a_j/a_0 is a ratio of two REAL numbers, so a
correct measurement lands at 0 deg (same sign) or 180 deg (opposite). The
measured phase is therefore a self-check, not a free parameter: a ratio sitting
at 90 deg is not a sign, it is a warning that the cell is noise or that two
modes are contributing differently to numerator and denominator. Every ratio
below is reported with its phase for exactly that reason.
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import tune                                                       # noqa: E402
import sysid                                                      # noqa: E402

DEFAULT_RAW = os.path.join(ROOT, "data", "20260806_182231_tune_raw.csv")
SENSITIVE = [0, 1, 2, 3]        # the rows with real counts-per-metre
STRONG_COILS = [0, 1, 2, 3]     # the columns with a solidly measured DC diagonal
REF_COIL, REF_SENS = 0, 0
K_ALIVE = 3.0                   # |value| > K_ALIVE * sigma to call a cell real


def _wmean_ratio(num, den, snum, sden):
    """Inverse-variance weighted mean of num/den over a set of cells.

    Weight by the ratio's own propagated variance rather than by |den|, so a
    cell whose DENOMINATOR is noisy is discounted too -- otherwise a single
    near-zero denominator dominates the mean with a huge, meaningless ratio.
    """
    num, den = np.asarray(num, complex), np.asarray(den, complex)
    snum, sden = np.asarray(snum, float), np.asarray(sden, float)
    ok = np.isfinite(num) & np.isfinite(den) & (np.abs(den) > 0)
    if not ok.any():
        return np.nan + 0j, np.inf, 0
    r = num[ok] / den[ok]
    # d(r)/r = dn/n - dd/d  ->  var(r) = |r|^2 (s_n^2/|n|^2 + s_d^2/|d|^2)
    var = np.abs(r) ** 2 * ((snum[ok] / np.abs(num[ok])) ** 2
                            + (sden[ok] / np.abs(den[ok])) ** 2)
    var = np.where(np.isfinite(var) & (var > 0), var, np.inf)
    w = 1.0 / var
    if not np.isfinite(w).any() or w.sum() <= 0:
        return np.nan + 0j, np.inf, 0
    m = float(np.sum(w * r.real) / w.sum()) + 1j * float(np.sum(w * r.imag) / w.sum())
    return m, float(1.0 / math.sqrt(w.sum())), int(ok.sum())


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else DEFAULT_RAW
    log = print
    log("coil qualification from %s" % os.path.basename(path))

    rec = tune.read_raw(path, log=log)
    log("  %d rows, %d coils, %d sensors, %d segments"
        % (len(rec["t"]), rec["u"].shape[1], rec["a"].shape[1],
           len(np.unique(rec["seg"]))))
    # The tone list is not in a `coil`/`freq_hz` column on this run: `coil` is -1
    # throughout and `freq_hz` carries a PASS INDEX 0-6, not a frequency. So the
    # tones come from the sidecar, and failing that from the plan the run used.
    meta = rec.get("meta") or {}
    tones = meta.get("freqs_hz")
    if tones is None:
        # recover from the drive itself: FFT one segment's commanded voltage and
        # take the bins that carry power. Measured from what was COMMANDED, so
        # it cannot disagree with what the lock-in will project onto.
        tones = _tones_from_drive(rec, log)
    est = tune.estimate_plant(rec, tones=tones, log=log)

    H, sig, ok = est["H"], est["sigma"], est["ok"]
    nsens, ncoil = est["nsens"], est["ncoil"]
    c2v = sysid.COUNTS_TO_V

    # ---------------------------------------------------------------- aliveness
    log("\n" + "=" * 74)
    log("1. IS THE CELL REAL?  |H| / sigma per (sensor, coil), max over tones")
    log("   A coil is ALIVE if it moves a SENSITIVE sensor. Read the columns.")
    log("=" * 74)
    snr = np.full((nsens, ncoil), 0.0)
    for i in range(nsens):
        for j in range(ncoil):
            v = np.abs(H[:, i, j]) / np.maximum(sig[:, i, j], 1e-30)
            v = v[np.isfinite(v)]
            snr[i, j] = float(v.max()) if len(v) else 0.0
    log("        " + "".join("  coil%d" % j for j in range(ncoil)))
    for i in range(nsens):
        log("   a%d   " % i + "".join("%7.1f" % snr[i, j] for j in range(ncoil))
            + ("   <- sensitive" if i in SENSITIVE else ""))
    log("\n   column max over the sensitive rows a0-a3 (this is the coil test):")
    colmax = snr[SENSITIVE].max(axis=0)
    for j in range(ncoil):
        log("     coil%d  %6.1f sigma   %s"
            % (j, colmax[j], "ALIVE" if colmax[j] > K_ALIVE else "not resolved"))

    # ------------------------------------------------------------- coil ratios
    log("\n" + "=" * 74)
    log("2. COIL GAIN AND SIGN:  a_j / a_0  from H[i][j]/H[i][0], i in a0-a3")
    log("   Model-free -- the sensor gain s[i] cancels. A real ratio must land")
    log("   at 0 deg (same sign as coil 0) or 180 deg (opposite).")
    log("=" * 74)
    log("   %-7s %11s %11s %9s %8s %7s  %s"
        % ("coil", "|a_j/a_0|", "+- ", "phase", "cells", "sign", "verdict"))
    coil_ratio = {}
    for j in range(ncoil):
        num, den, sn, sd = [], [], [], []
        for fi in range(len(est["freqs"])):
            for i in SENSITIVE:
                if ok[fi, i, j] and ok[fi, i, REF_COIL]:
                    num.append(H[fi, i, j]); sn.append(sig[fi, i, j])
                    den.append(H[fi, i, REF_COIL]); sd.append(sig[fi, i, REF_COIL])
        r, s, n = _wmean_ratio(num, den, sn, sd)
        coil_ratio[j] = (r, s, n)
        if n == 0 or not np.isfinite(r):
            log("   coil%-2d  %11s %11s %9s %8d %7s  cannot tell" % (j, "-", "-", "-", n, "-"))
            continue
        ph = math.degrees(math.atan2(r.imag, r.real))
        aligned = min(abs(ph), abs(abs(ph) - 180.0))
        sign = "+" if abs(ph) < 90 else "-"
        good = (abs(r) > K_ALIVE * s) and aligned < 45.0
        log("   coil%-2d  %11.4f %11.4f %+8.1f %8d %7s  %s"
            % (j, abs(r), s, ph, n, sign if good else "?",
               "resolved" if good else
               ("phase %.0f deg off the real axis" % aligned if abs(r) > K_ALIVE * s
                else "under %gx sigma" % K_ALIVE)))

    # ----------------------------------------------------------- sensor ratios
    log("\n" + "=" * 74)
    log("3. SENSOR GAIN:  s_j / s_0  from H[j][i]/H[0][i], i over the strong coils")
    log("   The coil gain a[i] cancels. This is the OTHER factor of P_mj, and it")
    log("   is what separates 'dead coil' from 'dead sensor'.")
    log("=" * 74)
    log("   %-7s %11s %11s %9s %8s  %s" % ("sensor", "|s_j/s_0|", "+- ", "phase", "cells", "verdict"))
    sens_ratio = {}
    for j in range(nsens):
        num, den, sn, sd = [], [], [], []
        for fi in range(len(est["freqs"])):
            for i in STRONG_COILS:
                if ok[fi, j, i] and ok[fi, REF_SENS, i]:
                    num.append(H[fi, j, i]); sn.append(sig[fi, j, i])
                    den.append(H[fi, REF_SENS, i]); sd.append(sig[fi, REF_SENS, i])
        r, s, n = _wmean_ratio(num, den, sn, sd)
        sens_ratio[j] = (r, s, n)
        if n == 0 or not np.isfinite(r):
            log("   a%-6d %11s %11s %9s %8d  cannot tell" % (j, "-", "-", "-", n))
            continue
        ph = math.degrees(math.atan2(r.imag, r.real))
        log("   a%-6d %11.4f %11.4f %+8.1f %8d  %s"
            % (j, abs(r), s, ph, n,
               "resolved" if abs(r) > K_ALIVE * s else "under %gx sigma" % K_ALIVE))

    # ------------------------------------------------------------- the product
    log("\n" + "=" * 74)
    log("4. THE PRODUCT  P_j / P_0 = (s_j/s_0)(a_j/a_0) -- the sign a GAIN needs")
    log("   Kp_j must satisfy Kp_j * P_j > 0. Kp_0 ships NEGATIVE, so a channel")
    log("   whose P_j/P_0 is POSITIVE takes a NEGATIVE gain, and vice versa.")
    log("=" * 74)
    log("   %-7s %13s %11s %9s  %s" % ("ch", "P_j/P_0", "+- ", "rel sigma", "gain sign"))
    for j in range(min(nsens, ncoil)):
        (ra, sa, na), (rs, ss, ns_) = coil_ratio.get(j, (np.nan, np.inf, 0)), \
                                      sens_ratio.get(j, (np.nan, np.inf, 0))
        if na == 0 or ns_ == 0 or not (np.isfinite(ra) and np.isfinite(rs)):
            log("   ch%-5d %13s %11s %9s  CANNOT TELL" % (j, "-", "-", "-"))
            continue
        p = ra * rs
        # relative errors add in quadrature
        rel = math.hypot(sa / max(abs(ra), 1e-30), ss / max(abs(rs), 1e-30))
        sp = abs(p) * rel
        ph = math.degrees(math.atan2(p.imag, p.real))
        decided = abs(p) > K_ALIVE * sp and min(abs(ph), abs(abs(ph) - 180)) < 45
        sgn = ("NEGATIVE" if abs(ph) < 90 else "POSITIVE") if decided else "CANNOT TELL"
        log("   ch%-5d %+13.4f %11.4f %9.2f  %s"
            % (j, abs(p) * (1 if abs(ph) < 90 else -1), sp, rel, sgn))

    # ------------------------------------------------------------ noise floors
    log("\n" + "=" * 74)
    log("5. MEASURED OFF-TONE NOISE FLOOR per sensor (volts, in-band bins)")
    log("   This is what a squelch threshold has to be built from.")
    log("=" * 74)
    nf = est["noise"]
    for i in range(nsens):
        log("   a%d  %10.6f V  (%7.2f counts)"
            % (i, nf[i], nf[i] / c2v if np.isfinite(nf[i]) else float("nan")))
    return 0


def _tones_from_drive(rec, log):
    """Recover the tone list from the COMMANDED drive.

    The run's `freq_hz` column carries a pass index, not a frequency, so there
    is nothing to read off. The drive itself is recorded, though, and taking the
    tones from what was commanded rather than from a plan file is strictly
    better: it is the same signal the lock-in projects onto, so the two cannot
    disagree, and it survives a plan file that was never written.
    """
    segs = np.unique(rec["seg"])
    m = rec["seg"] == segs[len(segs) // 2]
    t, u = rec["t"][m], rec["u"][m]
    if len(t) < 256:
        raise SystemExit("  segment too short to recover tones")
    dt = float(np.median(np.diff(t)))
    peaks = set()
    for j in range(u.shape[1]):
        x = u[:, j] - u[:, j].mean()
        F = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        f = np.fft.rfftfreq(len(x), dt)
        band = (f > 0.2) & (f < 4.0)
        if not band.any():
            continue
        thr = 0.15 * F[band].max()
        for k in np.nonzero(band & (F > thr))[0]:
            if F[k] >= F[max(k - 1, 0)] and F[k] >= F[min(k + 1, len(F) - 1)]:
                peaks.add(round(float(f[k]), 4))
    out = sorted(peaks)
    log("  tone list recovered from the COMMANDED drive: %s"
        % ", ".join("%.4f" % f for f in out))
    return out


if __name__ == "__main__":
    raise SystemExit(main())


# ===========================================================================
# The decisive test, and it does not need the in-band run at all.
# ===========================================================================
DC_MATRIX_8 = np.array([[-105, -107, -48, +19, -29, -21, -28, -26],
                        [-49, -68, -33, +18, +5, +7, +4, +8],
                        [+35, +66, +213, -141, -25, -22, -13, -26],
                        [+36, +14, +58, -51, -1, -2, +2, -1],
                        [+0, +2, +2, +1, +8, +8, +2, +1],
                        [+4, +0, +0, -6, +0, -7, -1, +8],
                        [+1, +4, +2, +3, +3, +3, +4, +6],
                        [+0, +3, +1, +0, +1, -1, +6, +7]], float)

# Measured off-tone in-band noise floor per sensor, volts, from est["noise"] of
# the 2026-08-06 multisine run (section 5 above).
NOISE_FLOOR_V = np.array([0.004668, 0.024136, 0.005924, 0.007842,
                          0.001335, 0.015529, 0.027788, 0.026468])
# Bandpassed RMS in the quiet and kick windows of
# data/20260806_190046_fast_lock.csv, 60-85 s and 85-115 s.
BP_QUIET_V = np.array([0.01460, 0.00613, 0.01126, 0.00583,
                       0.00211, 0.08193, 0.00395, 0.00111])
BP_KICK_V = np.array([0.63791, 0.38793, 0.65180, 0.44575,
                      0.07320, 0.38427, 0.22636, 0.08609])


def sec6_independence(log=print):
    """Are coils 4-7 four actuators, or one?

    THE POINT. Four coils bolted at four corners of a RIGID BODY with two
    observable degrees of freedom must produce four DIFFERENT response shapes on
    the sensitive sensors -- that is what coils 0-3 do. If four columns are
    parallel, whatever they are doing is not four independent mechanical forces.

    This runs on the DC matrix, which is an INDEPENDENT measurement from the
    multisine (one coil stepped at a time, 2026-08-04) and needs no mode fit, no
    lock-in and no assumption about the mode frequencies -- so it is unaffected
    by every caveat attached to the 2026-08-06 fit.
    """
    C = DC_MATRIX_8[:4, :]              # sensitive rows only
    log("\n" + "=" * 74)
    log("6. ARE COILS 4-7 INDEPENDENT ACTUATORS?  (DC matrix, rows a0-a3)")
    log("=" * 74)

    def cosm(g):
        return float(np.mean([C[:, j] @ C[:, k]
                              / (np.linalg.norm(C[:, j]) * np.linalg.norm(C[:, k]))
                              for j in g for k in g if k > j]))

    log("   pairwise cosine between coil columns:")
    for lab, g in (("coils 0-3", range(0, 4)), ("coils 4-7", range(4, 8))):
        log("     within %s : mean cos = %+.3f" % (lab, cosm(g)))
        for j in g:
            for k in g:
                if k > j:
                    c = float(C[:, j] @ C[:, k]
                              / (np.linalg.norm(C[:, j]) * np.linalg.norm(C[:, k])))
                    log("        coil%d vs coil%d  %+0.3f" % (j, k, c))
    log("\n   column norms, counts/V:")
    log("     " + "  ".join("coil%d %6.1f" % (j, np.linalg.norm(C[:, j])) for j in range(8)))
    for lab, sl in (("coil0-3", slice(0, 4)), ("coil4-7", slice(4, 8))):
        sv = np.linalg.svd(C[:, sl], compute_uv=False)
        log("   normalised singular values of [a0-a3 x %s]: %s"
            % (lab, np.round(sv / sv[0], 3)))
    log("\n   READ: coils 0-3 are mutually near-ORTHOGONAL (mean cos %+.3f) and span"
        % cosm(range(0, 4)))
    log("   two-to-three directions -- four independent corner forces on a body with")
    log("   two observable DOF, which is what the rank checks have always said.")
    log("   Coils 4-7 are mutually near-PARALLEL (mean cos %+.3f) and span ONE"
        % cosm(range(4, 8)))
    log("   direction. Four corner coils on a rigid body cannot do that mechanically.")
    log("   Their columns are also 3-7x SHORTER. The reading that fits is that their")
    log("   mechanical force is below a shared coupling common to all four -- so the")
    log("   response is not attributable to the individual coil.")


def sec7_squelch(log=print):
    """Would an amplitude squelch on a4/a6/a7 be sound?"""
    log("\n" + "=" * 74)
    log("7. THE SQUELCH -- what the measured floors say it would do")
    log("=" * 74)
    log("   %-5s %11s %11s %11s %9s %9s"
        % ("ch", "noise V", "quiet bp V", "kick bp V", "quiet/nf", "kick/nf"))
    for i in (4, 6, 7, 5, 0):
        log("   a%-4d %11.6f %11.5f %11.5f %9.2f %9.2f"
            % (i, NOISE_FLOOR_V[i], BP_QUIET_V[i], BP_KICK_V[i],
               BP_QUIET_V[i] / NOISE_FLOOR_V[i], BP_KICK_V[i] / NOISE_FLOOR_V[i]))
    log("\n   At a 3x-the-floor threshold the gate is CLOSED for a4/a6/a7 when quiet")
    log("   (1.58 / 0.14 / 0.04) and OPEN for a4 and a6 during the kick (54.8 /")
    log("   8.15); a7 only reaches 3.25, i.e. it grazes the threshold. So the")
    log("   squelch does mechanically what it claims.")
    log("   The objection is not that it misbehaves. See the report: it gates WHEN")
    log("   an unqualified coil acts, and concentrates that action into the moment")
    log("   of maximum stored energy and minimum headroom.")
    log("\n   Also measured, and it splits the three apart:")
    log("     a4 in-band bp RMS is %.2fx its own noise floor even when QUIET --"
        % (BP_QUIET_V[4] / NOISE_FLOOR_V[4]))
    log("        a4 is a usable low-gain SENSOR.")
    log("     a6 %.2fx and a7 %.2fx when quiet -- both BELOW their own floors, i.e."
        % (BP_QUIET_V[6] / NOISE_FLOOR_V[6], BP_QUIET_V[7] / NOISE_FLOOR_V[7]))
    log("        measuring nothing in band. And a6 RAILED in the quiet window too")
    log("        (min 9 counts, 60-85 s), so `_rail_check` demotes it before any")
    log("        gain path is reached and a squelch on a6 is moot.")
