"""All eight OSEMs are connected (confirmed on a scope, 2026-08-06). So where
does a4/a6/a7's signal actually live?

The earlier pass asked only "is there a peak at 1.046 or 1.657 Hz" and answered
no for a4, a6 and a7 -- then wrongly summarised that as "noise floor". It is
not: their raw std is 13-119 counts, comparable to a1 and a3. They carry plenty
of signal. It simply is not at the two frequencies that were tested.

This asks the open question instead: for each channel, where IS the power, over
the full band, and how much of it sits in the 0.4-3.0 Hz band the controller
actually acts on? Three readings discriminate the cases that remain:

  low-frequency dominated (< 0.4 Hz)   -> drift / thermal / TIA offset wander.
                                          Real signal, no motion information in
                                          band, and invisible to a bandpassed
                                          controller.
  broadband, flat                      -> electrical noise or an out-of-shadow
                                          flag: the OSEM is powered and reading
                                          but sits outside its linear region, so
                                          it modulates with nothing.
  peaked somewhere else                -> it IS sensing motion, of a mode the
                                          other four cannot see. That would be
                                          the most interesting outcome by far.

Also reports the post-kick high-amplitude windows separately, since that is the
best SNR available: if a channel senses motion at all, it shows there.
"""
import glob

import numpy as np

BAND = (0.4, 3.0)


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


def psd(x, fs, nperseg=4096):
    step, win = nperseg // 2, np.hanning(nperseg)
    P, n = 0.0, 0
    for s in range(0, len(x) - nperseg + 1, step):
        seg = x[s:s + nperseg] - x[s:s + nperseg].mean()
        P = P + np.abs(np.fft.rfft(seg * win)) ** 2
        n += 1
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return f, P / max(n, 1)


def summarise(f, P):
    tot = P[1:].sum()
    frac = lambda lo, hi: float(P[(f >= lo) & (f < hi)].sum() / tot) if tot else 0.0
    band = (f >= BAND[0]) & (f <= BAND[1])
    k = np.argmax(P[band]) if band.any() else 0
    return frac(0.0, 0.4), frac(*BAND), frac(3.0, 20.0), float(f[band][k])


paths = sorted(p for p in glob.glob("data/*.csv")
               if "ch7_counts" in open(p).readline())

for path in paths:
    t, c = load(path)
    if len(t) < 9000:
        continue
    fs = 1.0 / float(np.median(np.diff(t)))
    print(f"\n=== {path.split('/')[-1]}   fs={fs:.1f} Hz ===")
    print(f"  {'ch':>3} {'mean':>7} {'std':>7} | {'<0.4Hz':>7} {'0.4-3':>7} "
          f"{'3-20':>7} | {'pk in band':>10}   verdict")
    for i in range(8):
        f, P = psd(c[:, i], fs)
        lo, mid, hi, pk = summarise(f, P)
        if lo > 0.80:
            v = "drift-dominated, nothing in band"
        elif mid > 0.30:
            v = "IN BAND -- senses motion"
        elif hi > 0.30:
            v = "high-frequency, above the loop band"
        else:
            v = "spread / no clear home"
        print(f"  a{i:<2} {c[:, i].mean():>7.1f} {c[:, i].std():>7.2f} | "
              f"{lo:>7.3f} {mid:>7.3f} {hi:>7.3f} | {pk:>10.3f}   {v}")
