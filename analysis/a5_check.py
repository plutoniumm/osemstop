"""What exactly is a5 oscillating at, and is it the optic?

blind_check.py found a large repeatable narrowband peak on a5 near 1.046 Hz
(peak/neighbourhood 20.9x / 386.9x / 66.2x over three runs) but only middling
coherence with a0. Those two do not sit together comfortably, so this resolves
which it is by asking a sharper question than "is there a peak":

  1. WHERE exactly is a5's peak, to finer resolution than the 0.06 Hz half-width
     the peak test used? If it sits on 1.046 Hz to within a few mHz it is the
     rigid body. If it is offset by more than the mode's own width it is
     something else -- a different resonance, or electrical pickup.
  2. Is a5's peak frequency the SAME across runs? A mechanical mode is a
     property of the suspension and must be. Pickup tracks whatever produced it.
  3. What is a5's amplitude at that peak, in counts, against a0's?

Frequencies are read off a long single FFT (no segment averaging) so the bin
spacing is fs/N rather than fs/4096 -- resolution matters more than variance
here, because the whole question is whether two peaks coincide.
"""
import glob

import numpy as np

BAND = (0.7, 2.2)


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


def spectrum(x, fs):
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    # amplitude in counts, corrected for the window's coherent gain
    return f, np.abs(X) * 2.0 / (w.sum())


def peak_in(f, A, lo, hi):
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return np.nan, np.nan
    k = np.argmax(A[m])
    return float(f[m][k]), float(A[m][k])


paths = sorted(p for p in glob.glob("data/*.csv")
               if "ch7_counts" in open(p).readline())

print(f"  {'run':<26} {'ch':>3} {'peak Hz':>9} {'amp cts':>9} "
      f"{'df vs a0':>9}")
for path in paths:
    t, c = load(path)
    fs = 1.0 / float(np.median(np.diff(t)))
    f, A0 = spectrum(c[:, 0], fs)
    f0, a0amp = peak_in(f, A0, *BAND)
    name = path.split("/")[-1][:26]
    print(f"  {'-' * 60}")
    print(f"  {name:<26} {'a0':>3} {f0:>9.4f} {a0amp:>9.2f} {'--':>9}   "
          f"(bin {f[1] - f[0]:.5f} Hz)")
    for i in (2, 4, 5, 6, 7):
        f_, A = spectrum(c[:, i], fs)
        fp, amp = peak_in(f_, A, *BAND)
        print(f"  {'':<26} {'a' + str(i):>3} {fp:>9.4f} {amp:>9.2f} "
              f"{fp - f0:>+9.4f}")
