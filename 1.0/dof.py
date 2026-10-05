"""Decompose the sensors into rigid-body degrees of freedom, from the geometry.

GEOMETRY, stated by the rig owner 2026-08-17. Picture the assembly as a head; the
plate being damped is in the middle and its normal points at the nose:

    a0-a3   back of the head, one plane, sensing along the plate NORMAL (Z)
    a4,a5   the ears, sensing left-right (X)
    a6,a7   top of the head, attached vertically to the plate (Y)

Four coplanar sensors all reading Z give exactly three rigid-body quantities plus
one that no rigid-body motion can produce:

    Z     = (a0+a1+a2+a3)/4      translation along the normal
    T1    = (a0+a1-a2-a3)/4      tilt about one in-plane axis
    T2    = (a0+a2-a1-a3)/4      tilt about the other
    WARP  = (a0+a3-a1-a2)/4      no rigid body does this

That WARP channel is the whole reason Phi has a null space, and its size is the
honest check on the rigid-body assumption. The pairs give two more each:

    X     = (a4+a5)/2   ROLLX = (a4-a5)/2
    Y     = (a6+a7)/2   ROLLY = (a6-a7)/2

Project the record onto these coordinates and see where each mode's power lands.

THE CORNER ASSIGNMENT WAS SETTLED BY THIS FILE, 2026-08-17. A rigid plate cannot
warp, so the pairing that minimises warp is the true one, and it is unambiguous:
over data/20260817_205211_status_sensors.csv the warp/rigid ratios are

    a0+a1 vs a2+a3   1.566        a0+a2 vs a1+a3   0.277
    a0+a3 vs a1+a2   0.139   <-   2x better than the next

so a0 is diagonal to a3 and a1 is diagonal to a2, which is what the DOF table
below encodes. Permuting the sensors permutes which of T1/T2/WARP is which, but
never the split between "three rigid-body" and "one warp".
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import status  # noqa: E402

DOF = {
    "Z  normal":  ([0, 1, 2, 3], [+1, +1, +1, +1]),
    "T1 tilt":    ([0, 1, 2, 3], [+1, +1, -1, -1]),
    "T2 tilt":    ([0, 1, 2, 3], [+1, -1, +1, -1]),
    "WARP  (!)":  ([0, 1, 2, 3], [+1, -1, -1, +1]),
    "X  ears":    ([4, 5], [+1, +1]),
    "RX ear diff": ([4, 5], [+1, -1]),
    "Y  top":     ([6, 7], [+1, +1]),
    "RY top diff": ([6, 7], [+1, -1]),
}


def main(path):
    rows = status.load_csv(path)
    t = np.array([r[0] for r in rows])
    c = np.array([r[7] for r in rows], float)
    fs = len(t) / (t[-1] - t[0])
    print("  %s\n  %d samples, %.0f s, %.0f Hz" % (path, len(t), t[-1] - t[0], fs))

    q = {}
    for name, (chs, w) in DOF.items():
        w = np.asarray(w, float) / len(w)
        q[name] = (c[:, chs] * w).sum(axis=1)

    print("\n  === DOF amplitudes, counts rms (mean removed) ===")
    for name in DOF:
        print("   %-13s %8.2f" % (name, q[name].std()))
    warp, rigid = q["WARP  (!)"].std(), max(q["Z  normal"].std(),
                                            q["T1 tilt"].std(), q["T2 tilt"].std())
    print("\n   WARP / loudest rigid = %.3f -- a rigid plate gives ~0; anything"
          % (warp / rigid if rigid > 0 else np.nan))
    print("   large is sensor disagreement, and it is what the out-of-mode")
    print("   residual measures.")

    nfft = 1 << int(np.log2(len(t) / 8))
    win = np.hanning(nfft)
    idx = range(0, len(t) - nfft, nfft // 2)
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    P = {}
    for name in DOF:
        x = q[name]
        F = np.array([np.fft.rfft((x[i:i + nfft] - x[i:i + nfft].mean()) * win)
                      for i in idx])
        P[name] = (np.abs(F) ** 2).mean(axis=0)
    print("\n  %d segments of %d, %.4f Hz bins" % (len(idx), nfft, f[1]))

    print("\n  === WHICH DOF IS EACH MODE? share of that mode's total power ===")
    print("   mode Hz    " + "".join("%12s" % n.split()[0] for n in DOF))
    # The mode list comes from status.MODES, never a copy of it: these numbers
    # moved 0.0170 Hz in 11 days and a stale one silently mis-bins the answer.
    for _name, target in status.MODES:
        k = int(np.argmin(abs(f - target)))
        band = slice(max(k - 2, 0), k + 3)
        tot = sum(P[n][band].max() for n in DOF)
        if tot <= 0:
            continue
        print("   %8.4f   %s" % (f[k], "".join("%11.1f%%" % (100 * P[n][band].max() / tot)
                                               for n in DOF)))

    print("\n  === do the EAR and TOP degrees of freedom resonate at all? ===")
    print("  Peaks over 6x a running median floor, 0.3-25 Hz, in their own")
    print("  coordinates rather than per sensor -- a rotation cancels in the sum")
    print("  and shows only in the difference, so per-sensor spectra can hide it.")
    print("  READ THE RATIOS AS A SCREEN, NOT A DETECTION: a running-median floor")
    print("  is a flattering denominator. The '1.43 Hz fourth mode' was quoted at")
    print("  119x here and measured 4.4 sigma as a lock-in -- withdrawn 2026-08-17.")
    band = (f > 0.3) & (f < 25.0)
    fb = f[band]
    for name in ("X  ears", "RX ear diff", "Y  top", "RY top diff", "WARP  (!)"):
        y = P[name][band]
        pad = np.pad(y, 20, mode="edge")
        floor = np.array([np.median(pad[i:i + 41]) for i in range(len(y))])
        hot = y > 6.0 * floor
        pk = []
        i = 0
        while i < len(hot):
            if hot[i]:
                j = i
                while j + 1 < len(hot) and hot[j + 1]:
                    j += 1
                kk = i + int(np.argmax(y[i:j + 1]))
                pk.append((fb[kk], y[kk] / floor[kk]))
                i = j + 1
            else:
                i += 1
        top = sorted(pk, key=lambda x: -x[1])[:6]
        print("   %-13s %s" % (name, "  ".join("%.3fHz(%.0fx)" % p for p in top)
                               or "no peak anywhere"))


if __name__ == "__main__":
    main(sys.argv[1])
