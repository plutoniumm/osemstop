"""Are OSEMs a4..a7 disconnected, or just quiet?

2026-08-06. This re-tests a conclusion from 2026-08-04 -- "a4-a7 are not wired"
-- which rested on two statistics that are both WRONG for the small-signal case:

  * a DC actuation matrix (<= 8 counts/V vs 66 within the 0-3 block), which
    cannot see a sensor that works but has low gain; and
  * raw time-domain correlation against a0..a3 (|r| <= 0.12), which is dominated
    by DC offset and quantisation noise. A perfectly coherent signal of 1 count
    RMS buried under 5 counts of independent noise scores r ~ 0.2. Correlation
    on RAW counts cannot distinguish "no signal" from "small signal".

The right test is spectral. All eight OSEMs are bolted to ONE rigid body, whose
resonances are already known independently (analysis/kp040.md sec 7): 1.046 Hz
on ch0/ch2 and 1.657 Hz on ch1/ch3. A floating ADC pin cannot invent a peak at
another channel's mechanical resonance. So:

  connected but quiet  ->  narrow peak at 1.046 and/or 1.657 Hz, and coherence
                           with a0..a3 near 1 at that frequency
  floating pin         ->  broadband, no peak at either frequency, coherence at
                           the noise floor (~1/n_segments)

Coherence is the load-bearing statistic because it is amplitude-INDEPENDENT: it
normalises by each channel's own power, so a tiny signal that is genuinely the
same motion still scores high.
"""
import glob
import sys

import numpy as np

MODES = (1.046, 1.657)
BP = (0.4, 3.0)
NPERSEG = 4096


def load(path, nch=8):
    head = open(path).readline().strip().split(",")
    idx = [head.index(f"ch{i}_counts") for i in range(nch)]
    ti = head.index("time_s")
    rows = []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            p = line.rstrip("\n").split(",")
            if len(p) <= max(idx):
                continue
            try:
                rows.append([float(p[ti])] + [float(p[i]) for i in idx])
            except ValueError:
                continue
    a = np.asarray(rows)
    return a[:, 0], a[:, 1:]


def welch_cross(x, y, fs, nperseg=NPERSEG):
    """Averaged auto- and cross-spectra, Hann window, 50% overlap."""
    step, win = nperseg // 2, np.hanning(nperseg)
    Pxx = Pyy = Pxy = 0.0
    n = 0
    for s in range(0, len(x) - nperseg + 1, step):
        X = np.fft.rfft((x[s:s + nperseg] - x[s:s + nperseg].mean()) * win)
        Y = np.fft.rfft((y[s:s + nperseg] - y[s:s + nperseg].mean()) * win)
        Pxx = Pxx + np.abs(X) ** 2
        Pyy = Pyy + np.abs(Y) ** 2
        Pxy = Pxy + X * np.conj(Y)
        n += 1
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return f, Pxx / n, Pyy / n, Pxy / n, n


def peak_ratio(f, P, f0, halfwidth=0.06, side=(0.25, 0.60)):
    """Power in a narrow band at f0 over the local background beside it.
    > 1 means a peak stands above its own neighbourhood."""
    band = (np.abs(f - f0) <= halfwidth)
    near = (np.abs(f - f0) > side[0]) & (np.abs(f - f0) <= side[1])
    if not band.any() or not near.any():
        return np.nan
    return float(P[band].mean() / P[near].mean())


def main():
    paths = sorted(sys.argv[1:] or glob.glob("data/*.csv"))
    paths = [p for p in paths
             if "ch7_counts" in open(p).readline()]
    if not paths:
        sys.exit("no 8-channel logs found")

    for path in paths:
        t, c = load(path)
        if len(t) < NPERSEG * 2:
            continue
        dt = float(np.median(np.diff(t)))
        fs = 1.0 / dt
        nseg = max(1, (len(t) - NPERSEG) // (NPERSEG // 2) + 1)
        print(f"\n=== {path.split('/')[-1]}   {len(t)} samples, "
              f"fs={fs:.1f} Hz, {nseg} segments ===")
        print(f"  coherence noise floor ~ {1.0 / nseg:.3f}")
        print(f"  {'ch':>3} {'mean':>8} {'bp RMS':>8} "
              f"{'pk@1.046':>9} {'pk@1.657':>9} "
              f"{'coh@1.046':>10} {'coh@1.657':>10}  {'ref':>4}")

        f, _, _, _, _ = welch_cross(c[:, 0], c[:, 0], fs)
        bp = (f >= BP[0]) & (f <= BP[1])
        for i in range(8):
            ref = 0 if i in (1, 3, 5, 7) else 1      # compare against the OTHER mode's channel
            f, Pxx, Pyy, Pxy, n = welch_cross(c[:, i], c[:, ref], fs)
            coh = np.abs(Pxy) ** 2 / (Pxx * Pyy + 1e-30)
            # bandpassed RMS in counts, from the spectrum (Parseval, one-sided)
            rms = float(np.sqrt(np.trapezoid(Pxx[bp], f[bp]) / np.trapezoid(
                np.ones(bp.sum()), f[bp])) / NPERSEG * 2) if bp.any() else 0.0
            rms = float(np.std(c[:, i]))
            print(f"  a{i:<2} {c[:, i].mean():>8.1f} {rms:>8.2f} "
                  f"{peak_ratio(f, Pxx, MODES[0]):>9.2f} "
                  f"{peak_ratio(f, Pxx, MODES[1]):>9.2f} "
                  f"{coh[np.argmin(np.abs(f - MODES[0]))]:>10.3f} "
                  f"{coh[np.argmin(np.abs(f - MODES[1]))]:>10.3f}  "
                  f"a{ref:<3}")


if __name__ == "__main__":
    main()
