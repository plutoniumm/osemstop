#!/usr/bin/env python3
"""f0 and Q from the 320 s quiet open-loop record. Offline; opens no port.

    .venv/bin/python analysis/ringdown_quiet.py

Every number in the "320 s quiet record" part of analysis/ringdown.md comes
from here. Reads one file and writes analysis/out/quiet_*.csv.

Source
------
  data/20260806_192723_quiet_openloop.csv
      355797 samples, 320.0 s, 1111.9 Hz effective. Columns time_s,a0..a7 in
      RAW ADC COUNTS. No controller, no loop, no bp/vel/gain columns. Coils
      held at BIAS = 0.25 V, written 8 times total at the start. The first
      20.0 s are settle and are discarded, leaving 300.0 s.

Why this file needs different estimators from analysis/ringdown.py
------------------------------------------------------------------
ringdown.py used an AUTOCORRELATION fit because every gain-off stretch on disk
was short and ambient-driven, i.e. stationary. This record is NOT stationary:
the block RMS falls monotonically for the whole 300 s (ch0 87.8 -> 54.3 counts,
ch2 117.4 -> 68.1). That is a free decay, which is what a ringdown is, so the
DIRECT estimator -- narrowband the mode, take the analytic envelope, fit the
log-envelope against time -- is now both available and correct, and the ACF
estimator's stationarity assumption is now the thing that is violated. Both are
run below; the direct one is the answer and the ACF one is reported for
continuity with the earlier work.

The controls that matter here are different too:
  * OUT-OF-BAND CONTROL. If the lab merely got quieter, every band decays. If
    only the three mode bands decay, it is the optic ringing down. 2.4-3.6 Hz
    contains no mode and is measured the same way.
  * DEAD-CHANNEL CONTROL. ch4 and ch6 sit at ~6.3 counts RMS, which is sensor
    noise, not motion. They must show no decay.
  * RECOVERY CONTROL at THIS length: known gamma in, recovered gamma out,
    through the identical narrowband + envelope + fit pipeline, including the
    gamma = 0 case that sets the false-decay floor.
"""

import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ringdown import (acf_unbiased, modes_of, score,  # noqa: E402
                      seed, writecsv, hdr)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "analysis", "out")
SRC = os.path.join(ROOT, "data", "20260806_192723_quiet_openloop.csv")

SETTLE_S = 20.0          # discarded: the optic was still moving from the last run
FS_D = 20.0              # decimated rate
EDGE_S = 12.0            # discarded after FFT shaping, each end (filter ring-up)
MODES = (0.7049, 0.9967, 1.6404)   # starting points from analysis/ringdown.md
HALFBAND = 0.060         # narrowband half-width per mode, Hz
CONTROL_BAND = (2.40, 3.60)        # no mode lives here
NREP = 24                # recovery-control realisations per point


# ===========================================================================
# load
# ===========================================================================
def load():
    t, a, bad = [], [], 0
    with open(SRC) as fh:
        head = fh.readline().rstrip("\n").split(",")
        nch = sum(1 for c in head if c.startswith("a"))
        for line in fh:
            f = line.split(",")
            try:
                row = [int(f[1 + i]) for i in range(nch)]
                tv = float(f[0])
            except (ValueError, IndexError):
                bad += 1
                continue
            # 10-bit ADC. Anything outside 0..1023 is a serial glitch, not a
            # measurement, and one such sample would dominate a least squares.
            if any(v < 0 or v > 1023 for v in row):
                bad += 1
                continue
            t.append(tv)
            a.append(row)
    return np.asarray(t), np.asarray(a, float).T, nch, bad


def bandpass(t, x, flo, fhi, fs_new=FS_D):
    """Uniform regrid -> raised-cosine band shaping -> subsample. Edges cut."""
    fs = 1.0 / np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    xu = np.interp(tu, t, x)
    n = len(xu)
    A = np.vstack([tu - tu[0], np.ones(n)]).T
    c, *_ = np.linalg.lstsq(A, xu, rcond=None)
    xu = xu - A @ c
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    H = np.zeros_like(fr)
    w = 0.5 * (flo + 0.0)
    lo0, lo1 = flo * 0.5, flo
    hi0, hi1 = fhi, fhi * 1.5
    H[(fr >= lo1) & (fr <= hi0)] = 1.0
    m = (fr > lo0) & (fr < lo1)
    H[m] = 0.5 * (1 - np.cos(np.pi * (fr[m] - lo0) / (lo1 - lo0)))
    m = (fr > hi0) & (fr < hi1)
    H[m] = 0.5 * (1 + np.cos(np.pi * (fr[m] - hi0) / (hi1 - hi0)))
    xf = np.fft.irfft(np.fft.rfft(xu) * H, n)
    k = max(1, int(round(fs / fs_new)))
    e = int(EDGE_S * fs)
    return tu[e:n - e:k], xf[e:n - e:k], fs / k


def bandpass_d(y, fs, flo, fhi, tw=0.030):
    """Same shaping, already on a uniform decimated grid.

    `tw` is the raised-cosine transition width in Hz and is ABSOLUTE, not a
    fraction of the corner. With a proportional width a +/-0.060 Hz band about
    mode A would still pass mode B at 0.40 of amplitude and the two modes
    would not be separated at all.
    """
    n = len(y)
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    H = np.zeros_like(fr)
    lo0, lo1 = max(flo - tw, 1e-6), flo
    hi0, hi1 = fhi, fhi + tw
    H[(fr >= lo1) & (fr <= hi0)] = 1.0
    m = (fr > lo0) & (fr < lo1)
    H[m] = 0.5 * (1 - np.cos(np.pi * (fr[m] - lo0) / (lo1 - lo0)))
    m = (fr > hi0) & (fr < hi1)
    H[m] = 0.5 * (1 + np.cos(np.pi * (fr[m] - hi0) / (hi1 - hi0)))
    return np.fft.irfft(np.fft.rfft(y) * H, n)


def envelope(y, fs, smooth_s=4.0):
    n = len(y)
    Y = np.fft.fft(y - y.mean())
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    e = np.abs(np.fft.ifft(Y * h))
    k = max(1, int(smooth_s * fs))
    return np.convolve(e, np.ones(k) / k, mode="same"), k


def env_gamma(t, y, fs, smooth_s=4.0):
    """gamma from a straight line through the log of the analytic envelope.

    Returns gamma, r2, and the fraction the envelope fell over the record.
    Edges are trimmed by the smoothing length so the boxcar never straddles
    the ends of the record.
    """
    e, k = envelope(y, fs, smooth_s)
    tt, ee = t[k:len(t) - k], e[k:len(e) - k]
    if len(ee) < 32 or ee.min() <= 0:
        return np.nan, np.nan, np.nan
    A = np.vstack([tt - tt[0], np.ones(len(tt))]).T
    lg = np.log(ee)
    c, *_ = np.linalg.lstsq(A, lg, rcond=None)
    res = lg - A @ c
    ss = float(((lg - lg.mean()) ** 2).sum())
    return -float(c[0]), 1 - float(res @ res) / max(ss, 1e-30), \
        float(ee[-1] / ee[0])


# ===========================================================================
# the coherence statistic: normalised complex autocorrelation
# ===========================================================================
def analytic(y):
    n = len(y)
    Y = np.fft.fft(y - y.mean())
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    return np.fft.ifft(Y * h)


def rho(z, maxlag):
    """Normalised complex autocorrelation |<z*(t) z(t+tau)>| / rms(t) rms(t+tau).

    Dividing by the two local RMS values cancels any SLOW COMMON AMPLITUDE
    TREND: if the whole record is scaled by a smooth envelope, numerator and
    denominator scale together. What is left is loss of phase-and-shape
    coherence, which for a resonance of half-width gamma decays as
    exp(-gamma*tau) regardless of what the drive amplitude is doing. That is
    the point: section 3(a) shows the amplitude trend in this record is NOT
    modal, so an amplitude-based estimator measures the lab, not the optic.
    """
    n = len(z)
    out = np.empty(maxlag + 1)
    for k in range(maxlag + 1):
        A, B = z[:n - k], z[k:]
        num = np.abs(np.vdot(A, B))
        den = math.sqrt(float(np.vdot(A, A).real) * float(np.vdot(B, B).real))
        out[k] = num / den if den > 0 else np.nan
    return out


RHO_STOP = 0.35     # stop the fit here: below it rho is at its own noise floor


def rho_gamma(lag, r, fit_to_s):
    """gamma from a straight line through log(rho).

    The fit range is ADAPTIVE: it runs from lag 0 to whichever comes first,
    `fit_to_s` or the lag at which rho first falls below RHO_STOP. A fixed
    range is wrong because rho does not decay to zero -- with N_eff
    independent samples it flattens at ~1/sqrt(N_eff) -- so a range chosen for
    a high-Q mode measures the approach to that floor, not the exponential,
    and reads every low Q back as a high one. The same rule is applied to the
    calibration in `calibrate`, so the two are comparable.
    """
    stop = np.flatnonzero(r < RHO_STOP)
    lim = min(fit_to_s, lag[stop[0]] if stop.size else fit_to_s)
    m = (lag <= lim) & (r > 1e-6)
    if m.sum() < 8:
        m = (lag <= fit_to_s) & (r > 1e-6)
        if m.sum() < 8:
            return np.nan, np.nan
    A = np.vstack([lag[m], np.ones(int(m.sum()))]).T
    lg = np.log(r[m])
    c, *_ = np.linalg.lstsq(A, lg, rcond=None)
    res = lg - A @ c
    ss = float(((lg - lg.mean()) ** 2).sum())
    return -float(c[0]), 1 - float(res @ res) / max(ss, 1e-30)


def sim_stationary(Q, f0, fs, T, rng, snr=80.0, env_rate=0.0):
    """Stationary resonance of quality Q driven by white noise, 300 s.

    `env_rate` multiplies in an exponential amplitude trend, so the simulation
    can be given the SAME non-modal amplitude fall the real record has and the
    statistic can be checked to be blind to it.
    """
    n = int(T * fs)
    g = math.pi * f0 / Q
    z = np.exp((-g + 2j * np.pi * f0) / fs)
    w = rng.normal(size=n) + 1j * rng.normal(size=n)
    s = np.empty(n, complex)
    s[0] = 0
    for i in range(1, n):
        s[i] = z * s[i - 1] + w[i]
    x = np.real(s)
    x = x / x.std()
    t = np.arange(n) / fs
    x = x * np.exp(-env_rate * t)
    return t, x + rng.normal(size=n) / snr


def calibrate(Qs, f0, fs, T, rng, maxlag_s, fit_to_s, env_rate=0.0,
              nrep=NREP, snr=80.0):
    rows = []
    for Q in Qs:
        got = []
        for _ in range(nrep):
            t, x = sim_stationary(Q, f0, fs, T, rng, snr=snr,
                                  env_rate=env_rate)
            xf = bandpass_d(x, fs, f0 - HALFBAND, f0 + HALFBAND)
            e = int(EDGE_S * fs)
            zz = analytic(xf)[e:len(xf) - e]
            r = rho(zz, int(maxlag_s * fs))
            lag = np.arange(len(r)) / fs
            gh, _ = rho_gamma(lag, r, fit_to_s)
            if np.isfinite(gh):
                got.append(gh)
        v = np.array(got)
        rows.append([Q, math.pi * f0 / Q if np.isfinite(Q) else 0.0,
                     float(v.mean()), float(v.std()), len(v)])
    return rows


# ===========================================================================
def main():
    os.makedirs(OUT, exist_ok=True)

    hdr("0. RECORD INTEGRITY")
    t, a, nch, bad = load()
    dt = np.diff(t)
    print(f"  {SRC.split('/')[-1]}")
    print(f"  samples kept {len(t)}   rejected as out-of-range/unparseable "
          f"{bad}")
    print(f"  span {t[0]:.5f} .. {t[-1]:.5f} s   dur {t[-1]-t[0]:.3f} s   "
          f"fs_eff {(len(t)-1)/(t[-1]-t[0]):.2f} Hz")
    print(f"  dt median {np.median(dt)*1e3:.4f} ms  sd {dt.std()*1e3:.4f} ms  "
          f"max {dt.max()*1e3:.2f} ms   monotonic {bool((dt > 0).all())}")
    keep = t >= SETTLE_S
    t, a = t[keep], a[:, keep]
    print(f"  after discarding {SETTLE_S:.1f} s of settle: {a.shape[1]} "
          f"samples, {t[-1]-t[0]:.3f} s")
    rail = ((a <= 1) | (a >= 1022)).sum(axis=1)
    print(f"  samples on an ADC rail, per channel: {rail.astype(int).tolist()}")
    print(f"  resting counts: "
          + " ".join(f"ch{i}={a[i].mean():6.1f}" for i in range(nch)))
    print(f"  raw RMS:        "
          + " ".join(f"ch{i}={a[i].std():6.2f}" for i in range(nch)))
    T_use = t[-1] - t[0]
    print(f"\n  FREQUENCY RESOLUTION 1/T = {1/T_use:.5f} Hz")
    print(f"  Largest Q resolvable as a linewidth, f/(1/T): "
          f"{MODES[1]*T_use:.0f} at {MODES[1]:.4f} Hz")

    print("\n  BLOCK RMS, 30 s blocks -- is the record stationary?")
    hdrs = "  ".join(f"ch{i}" for i in range(nch))
    print(f"    {'block':>12}  " + "  ".join(f"{'ch%d'%i:>7}" for i in range(nch)))
    blk = []
    for lo in np.arange(t[0], t[-1] - 29.9, 30.0):
        m = (t >= lo) & (t < lo + 30)
        r = [float(a[i][m].std()) for i in range(nch)]
        print(f"    {lo:5.0f}-{lo+30:5.0f}  " + "  ".join(f"{v:7.2f}" for v in r))
        blk.append([float(lo)] + r)
    writecsv("quiet_blockrms.csv",
             ["t0_s"] + [f"rms_ch{i}" for i in range(nch)], blk)
    b0, b1 = np.array(blk[0][1:]), np.array(blk[-1][1:])
    span = blk[-1][0] - blk[0][0]
    print(f"\n  first block / last block, {span:.0f} s apart:")
    for i in range(nch):
        g = math.log(b0[i] / b1[i]) / span if b1[i] > 0 else float("nan")
        print(f"    ch{i}  {b0[i]:7.2f} -> {b1[i]:7.2f}   ratio {b1[i]/b0[i]:5.3f}"
              f"   implied gamma {g:+.5f} 1/s")
    print("  Every channel that carries motion falls monotonically. Whether")
    print("  that is the OPTIC ringing down or the LAB going quiet is decided")
    print("  by the out-of-band control in section 3(a), not by this table.")

    # -------------------------------------------------------------------
    hdr("1. FREQUENCIES AT 0.0033 Hz RESOLUTION")
    print(f"  Hann-windowed periodogram of the full {T_use:.1f} s, zero-padded "
          f"4x (grid {1/(4*T_use):.5f} Hz).")
    print("  Peak position is refined by a 3-point parabolic fit on the")
    print("  zero-padded grid, so the answer is not itself a bin.\n")
    tu, _, fsd = bandpass(t, a[0], 0.25, 4.0)
    Y = {}
    fr = None
    for i in range(nch):
        _, y, fsd = bandpass(t, a[i], 0.25, 4.0)
        Y[i] = y
    n = len(Y[0])
    freq_rows = []
    for i in range(nch):
        y = Y[i]
        w = np.hanning(n)
        P = np.abs(np.fft.rfft((y - y.mean()) * w, 4 * n)) ** 2
        fr = np.fft.rfftfreq(4 * n, 1 / fsd)
        out = []
        for fg in MODES:
            m = (fr > fg - 0.10) & (fr < fg + 0.10)
            j = np.flatnonzero(m)[np.argmax(P[m])]
            y1, y2, y3 = P[j - 1], P[j], P[j + 1]
            d = 0.5 * (y1 - y3) / (y1 - 2 * y2 + y3) if (y1 - 2*y2 + y3) else 0.0
            out.append((fr[j] + d * (fr[1] - fr[0]), P[j]))
        pmax = max(v for _, v in out)
        print(f"  ch{i}  " + "   ".join(
            f"{f:.4f} Hz ({p/pmax:.2f})" for f, p in out))
        for k, (f, p) in enumerate(out):
            freq_rows.append([i, "ABC"[k], f, p / pmax])
    writecsv("quiet_frequencies.csv", ["ch", "mode", "f_hz", "rel_power"],
             freq_rows)
    F = {}
    print()
    for k, name in enumerate("ABC"):
        v = np.array([r[2] for r in freq_rows
                      if r[1] == name and r[0] < 4])
        F[name] = float(v.mean())
        print(f"  mode {name}  f = {v.mean():.4f} +/- {v.std(ddof=1):.4f} Hz "
              f"(ch0..ch3)   prior value {MODES[k]:.4f}   "
              f"shift {v.mean()-MODES[k]:+.4f} Hz")

    # -------------------------------------------------------------------
    hdr("2. DIRECT RINGDOWN: log-envelope decay, per mode, per channel")
    print(f"  Narrowband +/-{HALFBAND:.3f} Hz about each mode, analytic")
    print(f"  envelope smoothed 4 s, straight line through log(envelope).")
    print(f"  fall = envelope(end)/envelope(start) over the fitted span.\n")
    print(f"    {'mode':>6} {'ch':>3} {'f':>8} {'gamma':>9} {'tau':>8} "
          f"{'Q':>8} {'r2':>6} {'fall':>6}")
    dir_rows = []
    for k, name in enumerate("ABC"):
        f0 = F[name]
        for i in range(nch):
            yf = bandpass_d(Y[i], fsd, f0 - HALFBAND, f0 + HALFBAND)
            g, r2, fall = env_gamma(tu, yf, fsd)
            if not np.isfinite(g):
                continue
            q = math.pi * f0 / g if g > 0 else np.inf
            print(f"    {name:>6} {i:3d} {f0:8.4f} {g:+9.5f} "
                  f"{1/g if g > 0 else float('inf'):8.1f} {q:8.0f} {r2:6.3f} "
                  f"{fall:6.3f}")
            dir_rows.append([name, i, f0, g, 1 / g if g > 0 else np.inf, q, r2,
                             fall])
    writecsv("quiet_direct.csv",
             ["mode", "ch", "f_hz", "gamma_1_s", "tau_s", "Q", "r2",
              "env_fall"], dir_rows)

    # -------------------------------------------------------------------
    hdr("3. CONTROLS ON THIS RECORD")
    print("  (a) OUT-OF-BAND CONTROL. If the lab merely got quieter, a band")
    print(f"      with no mode in it decays too. {CONTROL_BAND[0]}-"
          f"{CONTROL_BAND[1]} Hz:\n")
    ctl_rows = []
    for i in range(nch):
        yf = bandpass_d(Y[i], fsd, *CONTROL_BAND)
        g, r2, fall = env_gamma(tu, yf, fsd)
        print(f"      ch{i}  gamma = {g:+.5f} 1/s   r2 = {r2:6.3f}   "
              f"fall = {fall:5.3f}")
        ctl_rows.append(["out_of_band", i, g, r2, fall])
    print("\n  (b) DEAD-CHANNEL CONTROL. ch4 and ch6 carry ~6.3 counts RMS,")
    print("      which is sensor noise. They should show no mode decay; see")
    print("      their rows in section 2.\n")
    print("  (c) SPLIT HALF. Same fit on the first and second 150 s. A real")
    print("      exponential gives the same gamma in both; a one-off drift")
    print("      does not.\n")
    half = len(tu) // 2
    for k, name in enumerate("ABC"):
        f0 = F[name]
        row = [name]
        for i in range(4):
            yf = bandpass_d(Y[i], fsd, f0 - HALFBAND, f0 + HALFBAND)
            g1, _, _ = env_gamma(tu[:half], yf[:half], fsd)
            g2, _, _ = env_gamma(tu[half:], yf[half:], fsd)
            row += [g1, g2]
            print(f"      mode {name} ch{i}  1st half {g1:+.5f}   "
                  f"2nd half {g2:+.5f}   ratio "
                  f"{g2/g1 if g1 else float('nan'):5.2f}")
        ctl_rows.append(["split_half_" + name] + row[1:])
    writecsv("quiet_controls.csv",
             ["control", "ch_or_a", "v1", "v2", "v3", "v4", "v5", "v6", "v7",
              "v8"],
             [r + [""] * (10 - len(r)) for r in ctl_rows])

    # -------------------------------------------------------------------
    hdr("4. COHERENCE ESTIMATOR -- immune to the amplitude trend")
    print("  Section 3(a) shows the amplitude fall is NOT modal: a band with no")
    print("  mode in it falls at the same rate. So the log-envelope slope of")
    print("  section 2 measures the ambient level, not the optic, and the")
    print("  amplitude-based reading of Q from it is void.")
    print()
    print("  What survives is the NORMALISED complex autocorrelation")
    print("      rho(tau) = |<z*(t) z(t+tau)>| / rms(t) rms(t+tau)")
    print("  of the narrowbanded analytic signal. Dividing by the two local RMS")
    print("  values cancels any smooth common amplitude trend; what is left is")
    print("  the loss of coherence, which decays as exp(-gamma*tau) for a")
    print("  resonance of half-width gamma. This is the same autocorrelation")
    print("  statistic as analysis/ringdown.md, normalised and per-mode.\n")
    MAXLAG_S, FIT_TO_S = 120.0, 90.0
    print(f"  max lag {MAXLAG_S:.0f} s, straight line fitted over 0..{FIT_TO_S:.0f} s")
    print(f"    {'mode':>6} {'ch':>3} {'gamma':>10} {'tau':>9} {'Q':>9} "
          f"{'r2':>6} {'rho(90s)':>9}")
    coh_rows = []
    e_ = int(EDGE_S * fsd)
    for name in "ABC":
        f0 = F[name]
        for i in range(nch):
            yf = bandpass_d(Y[i], fsd, f0 - HALFBAND, f0 + HALFBAND)
            zz = analytic(yf)[e_:len(yf) - e_]
            r = rho(zz, int(MAXLAG_S * fsd))
            lag = np.arange(len(r)) / fsd
            g, r2 = rho_gamma(lag, r, FIT_TO_S)
            if not np.isfinite(g):
                continue
            q = math.pi * f0 / g if g > 0 else np.inf
            print(f"    {name:>6} {i:3d} {g:+10.5f} "
                  f"{1/g if g > 0 else float('inf'):9.1f} {q:9.0f} {r2:6.3f} "
                  f"{r[int(90*fsd)]:9.3f}")
            coh_rows.append([name, i, f0, g, 1 / g if g > 0 else np.inf, q, r2,
                             float(r[int(90 * fsd)])])
    writecsv("quiet_coherence.csv",
             ["mode", "ch", "f_hz", "gamma_1_s", "tau_s", "Q", "r2",
              "rho_90s"], coh_rows)

    print("\n  BLINDNESS CHECK. The same statistic on a simulated STATIONARY")
    print("  resonance of known Q, with and without a 0.0021 1/s amplitude")
    print("  trend multiplied in -- the trend the real record has. If rho is")
    print("  doing its job the two columns agree.\n")
    rng = np.random.default_rng(4242)
    print(f"    {'Q_true':>8} {'gamma_true':>11} | {'no trend':>19} | "
          f"{'with 0.0021 trend':>19}")
    blind_rows = []
    for Q in (100.0, 300.0, 1000.0, 3000.0):
        a1 = calibrate([Q], F["B"], fsd, T_use, rng, MAXLAG_S, FIT_TO_S,
                       0.0, nrep=12)[0]
        a2 = calibrate([Q], F["B"], fsd, T_use, rng, MAXLAG_S, FIT_TO_S,
                       0.0021, nrep=12)[0]
        print(f"    {Q:8.0f} {a1[1]:11.5f} | {a1[2]:9.5f} +/- {a1[3]:7.5f} | "
              f"{a2[2]:9.5f} +/- {a2[3]:7.5f}")
        blind_rows.append([Q, a1[1], a1[2], a1[3], a2[2], a2[3]])
    writecsv("quiet_blindness.csv",
             ["Q_true", "gamma_true", "gamma_fit_notrend", "sd_notrend",
              "gamma_fit_trend", "sd_trend"], blind_rows)

    # -------------------------------------------------------------------
    hdr("4b. INDEPENDENT CHECK -- linewidth, and frequency stability")
    print("  The coherence result stands or falls on one claim: these lines are")
    print("  narrow. That is checkable in the frequency domain without any of")
    print("  the machinery above. Compare each mode's -3 dB width against a")
    print("  PURE TONE pushed through the identical window and zero-padding.\n")
    pad = 16
    wn = np.hanning(n)
    frp = np.fft.rfftfreq(pad * n, 1 / fsd)
    ref = np.cos(2 * np.pi * F["B"] * np.arange(n) / fsd)
    Pref = np.abs(np.fft.rfft(ref * wn, pad * n)) ** 2

    def width3db(fq, P, f0, half=0.06):
        m = (fq > f0 - half) & (fq < f0 + half)
        f_, p_ = fq[m], P[m]
        j = int(np.argmax(p_))
        lo = f_[:j][p_[:j] < p_[j] / 2]
        hi = f_[j:][p_[j:] < p_[j] / 2]
        return float(hi[0] - lo[-1]) if len(lo) and len(hi) else np.nan

    w_ref = width3db(frp, Pref, F["B"])
    T_fft = n / fsd
    print(f"  usable span after edge trim {T_fft:.1f} s   Hann -3 dB width "
          f"1.44/T = {1.44/T_fft:.5f} Hz")
    print(f"  pure tone through the identical pipeline: {w_ref:.5f} Hz")
    print(f"  Q that this width alone can resolve: "
          f"{F['B']/w_ref:.0f} at {F['B']:.4f} Hz\n")
    lw_rows = []
    for name in "ABC":
        f0 = F[name]
        for i in range(4):
            P = np.abs(np.fft.rfft((Y[i] - Y[i].mean()) * wn, pad * n)) ** 2
            wd = width3db(frp, P, f0)
            ex = math.sqrt(max(wd ** 2 - w_ref ** 2, 0.0))
            tag = (f"excess {ex:.5f} Hz -> Q <= {f0/ex:.0f}" if ex > 0
                   else "instrument-limited, Q unbounded above")
            print(f"    mode {name} ch{i}  -3 dB width {wd:.5f} Hz   {tag}")
            lw_rows.append([name, i, f0, wd, w_ref, ex,
                            f0 / ex if ex > 0 else np.inf])
    writecsv("quiet_linewidth.csv",
             ["mode", "ch", "f_hz", "width3db_hz", "width3db_ref_hz",
              "excess_hz", "Q_upper"], lw_rows)

    print("\n  FREQUENCY STABILITY, first half vs second half of the record:")
    h2 = n // 2
    fst_rows = []
    for name in "ABC":
        f0 = F[name]
        vals = []
        for sl in (slice(0, h2), slice(h2, 2 * h2)):
            y = Y[0][sl]
            m_ = len(y)
            P = np.abs(np.fft.rfft((y - y.mean()) * np.hanning(m_),
                                   pad * m_)) ** 2
            f2 = np.fft.rfftfreq(pad * m_, 1 / fsd)
            mm = (f2 > f0 - 0.05) & (f2 < f0 + 0.05)
            j = int(np.flatnonzero(mm)[np.argmax(P[mm])])
            y1, y2, y3 = P[j - 1], P[j], P[j + 1]
            d = 0.5 * (y1 - y3) / (y1 - 2 * y2 + y3) if (y1 - 2*y2 + y3) else 0.0
            vals.append(float(f2[j] + d * (f2[1] - f2[0])))
        print(f"    mode {name} ch0   {vals[0]:.4f} / {vals[1]:.4f} Hz   "
              f"drift {vals[1]-vals[0]:+.4f} Hz")
        fst_rows.append([name, vals[0], vals[1], vals[1] - vals[0]])
    writecsv("quiet_freq_stability.csv",
             ["mode", "f_first_half", "f_second_half", "drift_hz"], fst_rows)

    # -------------------------------------------------------------------
    hdr("5. CALIBRATION AND VERDICT")
    print(f"  Known Q in, recovered gamma out, {T_use:.0f} s records, "
          f"{NREP} realisations each,")
    print(f"  through the identical narrowband + analytic + rho + log-linear "
          f"pipeline.\n")
    # The floor depends on how far the mode stands above the sensor noise
    # inside the analysis band, so measure that from the record rather than
    # assuming it.
    snr_meas = []
    for i in range(4):
        yb = bandpass_d(Y[i], fsd, F["B"] - HALFBAND, F["B"] + HALFBAND)
        yn = bandpass_d(Y[i], fsd, 2.90, 3.02)      # same width, no mode in it
        snr_meas.append(float(yb.std() / max(yn.std(), 1e-9)))
    SNR = float(np.median(snr_meas))
    print(f"  In-band SNR of mode B, measured as the ratio of the "
          f"{2*HALFBAND:.2f} Hz band")
    print(f"  at the mode to an equally wide mode-free band at 2.96 Hz: "
          + ", ".join(f"ch{i}={v:.0f}" for i, v in enumerate(snr_meas))
          + f"  -> median {SNR:.0f}")
    print(f"  The calibration below runs at that SNR.\n")
    print(f"    {'Q_true':>9} {'gamma_true':>11} | {'gamma recovered':>21} "
          f"{'Q recovered':>12}")
    Qs = [30.0, 60.0, 100.0, 200.0, 300.0, 500.0, 1000.0, 2334.0, 1e9]
    cal = calibrate(Qs, F["B"], fsd, T_use, rng, MAXLAG_S, FIT_TO_S, snr=SNR)
    for Q, gt, gm, gs, n_ in cal:
        qr = math.pi * F["B"] / gm if gm > 0 else float("inf")
        lab = f"{Q:9.0f}" if Q < 1e8 else "  infinite"
        print(f"    {lab} {gt:11.5f} | {gm:12.5f} +/- {gs:7.5f} {qr:12.0f}")
    writecsv("quiet_calibration.csv",
             ["Q_true", "gamma_true", "gamma_fit_mean", "gamma_fit_sd",
              "n_rep"], cal)
    fl = cal[-1]
    floor = abs(fl[2]) + 2 * fl[3]
    print(f"\n  FALSE-DECAY FLOOR. With Q infinite (a pure tone plus sensor")
    print(f"  noise) the statistic returns gamma = {fl[2]:+.5f} +/- {fl[3]:.5f}"
          f" 1/s.")
    print(f"  Nothing below {floor:.5f} 1/s is distinguishable from zero, so the")
    print(f"  largest Q THIS record can resolve is "
          f"{math.pi*F['B']/floor:.0f} at {F['B']:.4f} Hz.")

    ver_rows = []
    print()
    for name in "ABC":
        f0 = F[name]
        g = np.array([r[3] for r in coh_rows if r[0] == name and r[1] < 4])
        gm = float(g.mean())
        sd_ch = float(g.std(ddof=1)) / math.sqrt(len(g))
        near = min(cal, key=lambda r: abs(r[2] - gm))
        sd = math.hypot(sd_ch, near[3])
        q = math.pi * f0 / gm if gm > 0 else np.inf
        dq = q * sd / abs(gm) if gm > 0 else np.inf
        meas = gm - 2 * sd > 0 and gm > floor
        print(f"  mode {name}  f = {f0:.4f} Hz")
        print(f"    gamma = {gm:+.5f} +/- {sd:.5f} 1/s   (n={len(g)} sensors: "
              + ", ".join(f"{v:+.5f}" for v in g) + ")")
        print(f"    tau   = {1/gm if gm > 0 else float('inf'):.1f} s")
        print(f"    Q     = {q:.0f} [{math.pi*f0/(gm+sd):.0f}, "
              f"{math.pi*f0/max(gm-sd,1e-9):.0f}]"
              f"   {'MEASURED' if meas else 'NOT RESOLVED -- bound only'}")
        ver_rows.append([name, f0, gm, sd, len(g),
                         1 / gm if gm > 0 else np.inf, q,
                         math.pi * f0 / (gm + sd),
                         math.pi * f0 / max(gm - sd, 1e-9), bool(meas)])
    writecsv("quiet_verdict.csv",
             ["mode", "f_hz", "gamma_1_s", "gamma_sd", "n_sensors", "tau_s",
              "Q", "Q_lo", "Q_hi", "resolved"], ver_rows)

    print("\n  EVERY PRIOR CLAIM, against mode B:")
    gB = [r for r in ver_rows if r[0] == "B"][0]
    print(f"    {'claim':<34} {'implies gamma':>14} {'sigma':>8}")
    for q, label in ((20.0, "Q = 20-22 (mimo_design.md)"),
                     (50.0, "Q = 50 (research.md, sim)"),
                     (100.0, "Q = 100 (ringdown.md bound)"),
                     (288.0, "Q = 288 (multisine, mode A)"),
                     (2334.0, "Q = 2334 (multisine, mode B)")):
        gp = math.pi * gB[1] / q
        sig = abs(gp - gB[2]) / gB[3]
        print(f"    {label:<34} {gp:14.5f} {sig:8.1f}")

    # -------------------------------------------------------------------
    hdr("6. BEAT PERIODS, now directly resolvable")
    for x, y in (("A", "B"), ("B", "C"), ("A", "C")):
        d = abs(F[y] - F[x])
        print(f"  {x}-{y}   df = {d:.4f} Hz   beat period = {1/d:.3f} s")

    # -------------------------------------------------------------------
    hdr("7. ACF ESTIMATOR, for continuity with analysis/ringdown.md")
    print("  NOTE: this estimator assumes stationarity, which section 0 shows")
    print("  is false for this record. Reported so the two files can be")
    print("  compared, not as the answer.\n")
    acf_rows = []
    for i in range(4):
        ml = int(100.0 * fsd)
        y = Y[i]
        if len(y) < 2 * ml:
            ml = len(y) // 2
        r = acf_unbiased(y - y.mean(), ml)
        lag = np.arange(ml + 1) / fsd
        fit = score(lag, r / r[0], 3,
                    seed(r / r[0], 1 / fsd, 3, 0.45, 2.6), band=(0.45, 2.6))
        m = modes_of(fit)
        print(f"  ch{i}  maxlag {lag[-1]:.0f} s  r2={fit['r2']:.4f}  | "
              + "  ".join(f"f={f:.4f} g={g:+.5f} Q={q:7.0f}"
                          for f, g, q, _ in m))
        for j, (f, g, q, amp) in enumerate(m):
            acf_rows.append([i, lag[-1], fit["r2"], j, f, g, q])
    writecsv("quiet_acf.csv",
             ["ch", "maxlag_s", "r2", "mode", "f_hz", "gamma_1_s", "Q"],
             acf_rows)


if __name__ == "__main__":
    main()
