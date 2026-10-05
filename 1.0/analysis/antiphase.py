"""ON HARDWARE: why do coils 4-7 act as ONE coil?

`analysis/coil_qual.py` measured the four columns of coils 4-7 on sensors a0-a3
as 96.6% parallel in the mean (coil5 vs coil7: +0.999) at 1/4-1/7 the strength of
coils 0-3, whose columns are orthogonal (+0.016). Four coils bolted at four
corners of a rigid body cannot produce one response direction. Two explanations
fit the DC matrix equally well, and it cannot separate them because it stepped
one coil at a time:

  (a) ELECTRICAL -- the four DAC outputs reach one physical coil, or a shared
      node, so every one of them drives the same winding.
  (b) MECHANICAL -- four real coils that happen to sit where they drive the same
      observable DOF, the other directions being the 0.032 / 0.010 singular
      values that are already at noise.

ANTI-PHASE SEPARATES THEM. Drive coil4 and coil6 in opposition:

  under (a) the two contributions land on the same winding and CANCEL -- the
      response collapses toward zero;
  under (b) two opposed corner coils make a TORQUE, which is a different
      combination of the rigid body's DOF, so the response is roughly the
      DIFFERENCE of the two single-coil responses and keeps a comparable
      magnitude with a DIFFERENT SHAPE.

So the discriminator is not "is there a response" but "does anti-phase look like
A - B (mechanical) or like nothing (electrical)".

PASS 0 IS A POSITIVE CONTROL. coil0 is strong and independently characterised
(126 counts/V), so if pass 0 does not recover a clean response the method is
broken and passes 1-3 mean nothing -- the same discipline that caught the
"a4-a7 not wired" error, where the method was blind rather than the channels
dead.

DRIVE FREQUENCY. 0.45 Hz -- BELOW the lowest mode (0.7049 Hz), not above it.
The first attempt used 2.5 Hz, comfortably clear of every mode, and that was the
error: above resonance the mechanical response rolls off as 1/f^2, so the whole
plant shrank. The control coil returned 17.8 counts/V against its DC value of
126, a factor of 7 that is the roll-off almost exactly, and coils 4-7 -- already
4x weaker -- fell to 0.3-0.8 counts/V, which is noise. The recovered shape cosine
of -0.092 was noise correlated against noise.

Below the lowest mode the response is FLAT instead, approaching the DC gain: at
0.45 Hz the factor is 1/(1-(0.45/0.7049)^2) = 1.7x DC, so the control should
return ~214 counts/V and coils 4-7 ~60, which is 12 counts of signal at
AMP = 0.20 V rather than half a count. Q > 100 keeps every peak narrower than
0.017 Hz, so 0.45 Hz is still many widths clear of 0.7049 and nothing is
amplified into a rail.
"""
import os
import sys
import time
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pyDAC2 import FastDAC                                   # noqa: E402

N = 8
DAC_CHANNELS = [1, 3, 5, 7, 0, 2, 4, 6]      # logical channel -> DAC channel
BIAS = 0.25
AMP = 0.20                  # 0.05..0.45 V, inside the 0.0..0.5 window
DRIVE_HZ = 0.45
SETTLE_S, RECORD_S = 10.0, 60.0
UPDATE_HZ = 100.0
SENSE = [0, 1, 2, 3]        # the sensitive rows; a4-a7 are too low-gain to judge by

# (label, {logical coil: sign})
PASSES = [
    ("coil0 (CONTROL)", {0: +1.0}),
    ("coil4 alone",     {4: +1.0}),
    ("coil6 alone",     {6: +1.0}),
    ("coil4 - coil6",   {4: +1.0, 6: -1.0}),
]


def run_pass(dac, f, label, drive, t0):
    """One pass. Returns (complex response vector on SENSE, complex drive)."""
    print(f"\n  {label}: {RECORD_S:.0f}s at {DRIVE_HZ} Hz, "
          f"{AMP:.2f}V peak, coils {sorted(drive)}")
    tp = time.time()
    acc_y = np.zeros(len(SENSE), complex)
    acc_u = 0j
    n = 0
    next_write = 0.0
    while True:
        t = time.time() - tp
        if t >= SETTLE_S + RECORD_S:
            break
        if t >= next_write:
            next_write = t + 1.0 / UPDATE_HZ
            for c, s in drive.items():
                v = BIAS + s * AMP * np.sin(2 * np.pi * DRIVE_HZ * t)
                dac.set_voltage(channel=DAC_CHANNELS[c], voltage=float(np.clip(v, 0.0, 0.5)))
        row = dac.read_sample(N)
        if row is None:
            continue
        f.write(f"{time.time() - t0:.5f},{label},{t:.5f},"
                + ",".join(str(v) for v in row) + "\n")
        if t < SETTLE_S:
            continue
        # Lock-in against the drive ACTUALLY COMMANDED, not the one intended --
        # CLAUDE.md records that dividing by the intended amplitude put a
        # spurious per-tone phase offset into every recovered phase.
        ph = np.exp(-2j * np.pi * DRIVE_HZ * t)
        acc_y += np.array([row[i] for i in SENSE]) * ph
        acc_u += np.sin(2 * np.pi * DRIVE_HZ * t) * ph
        n += 1
    if n == 0 or abs(acc_u) == 0:
        return np.zeros(len(SENSE), complex), 0j, 0
    for c in drive:                                   # park it back at bias
        dac.set_voltage(channel=DAC_CHANNELS[c], voltage=BIAS)
    return acc_y / n, acc_u / n, n


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-1120"
    dac = FastDAC(port=port)
    for c in range(N):
        dac.set_voltage(channel=DAC_CHANNELS[c], voltage=BIAS)
    dac.start_stream()

    path = os.path.join(os.path.dirname(HERE), "data",
                        datetime.now().strftime("%Y%m%d_%H%M%S") + "_antiphase_raw.csv")
    t0 = time.time()
    f = open(path, "w", buffering=1)
    res = {}
    try:
        f.write("time_s,pass,t_pass," + ",".join(f"a{i}" for i in range(N)) + "\n")
        print(f"raw -> {path}")
        for label, drive in PASSES:
            y, u, n = run_pass(dac, f, label, drive, t0)
            h = y / u if abs(u) > 0 else y * 0
            res[label] = h
            print(f"    {n} samples;  |H| on a0-a3 = "
                  + "  ".join(f"{abs(v):8.2f}" for v in h)
                  + "   counts/V")
    finally:
        f.close()
        try:
            for c in range(N):
                dac.set_voltage(channel=DAC_CHANNELS[c], voltage=BIAS)
            dac.stop_stream()
            dac.close()
        except Exception:
            pass

    print("\n" + "=" * 70)
    c0 = res.get("coil0 (CONTROL)")
    if c0 is not None:
        print(f"  CONTROL coil0: |H| = {np.abs(c0).max():.1f} counts/V peak on a0-a3.")
        if np.abs(c0).max() < 5.0:
            print("  !! CONTROL FAILED -- the method did not see a coil known to work.")
            print("     Everything below is meaningless. Do not interpret it.")
            return
    a, b = res.get("coil4 alone"), res.get("coil6 alone")
    ab = res.get("coil4 - coil6")
    if a is None or b is None or ab is None:
        return
    pred = a - b
    print(f"  |coil4|        = {np.abs(a).max():8.2f}   (peak on a0-a3)")
    print(f"  |coil6|        = {np.abs(b).max():8.2f}")
    print(f"  |coil4-coil6|  = {np.abs(ab).max():8.2f}   MEASURED anti-phase")
    print(f"  |A - B|        = {np.abs(pred).max():8.2f}   predicted if MECHANICAL")
    denom = max(np.abs(pred).max(), 1e-9)
    ratio = np.abs(ab).max() / denom
    cos = 0.0
    if np.linalg.norm(ab) > 0 and np.linalg.norm(pred) > 0:
        cos = float(np.real(np.vdot(pred, ab)) /
                    (np.linalg.norm(pred) * np.linalg.norm(ab)))
    print(f"\n  measured/predicted = {ratio:.3f},  shape cosine = {cos:+.3f}")
    print("  ratio ~1 and cosine ~+1  -> MECHANICAL: four real coils, one shared DOF.")
    print("  ratio ~0                 -> ELECTRICAL: they land on the same winding.")
    print("  neither                  -> inconclusive; say so, do not pick one.")


if __name__ == "__main__":
    main()
