"""
v6.5 -- stepped-sine actuation matrix, EIGHT coils.
===================================================
NOT a controller. It damps nothing and closes no loop. It drives one coil at a
time with a sine and records what all eight OSEMs do, to recover the **actuation
matrix** -- the one unmet prerequisite for `research.md` item 2 (modal/MIMO
damping), and the exact thing the previous attempt got wrong.

Why stepped sine rather than something cleverer: while coil j is driving, no
other coil is. There is nothing to disentangle, no matrix to invert, and no
assumption about quadrature. Both quadratures come out of the lock-in together,
so magnitude AND phase are measured rather than inferred -- `research.md` item 2
cause 1 was a zero-lag correlation on velocity, which has no coherent term to
find for a force that acts on acceleration.

It is slow, and that is the trade. Eight coils x the frequency list x
(SETTLE + DWELL). With the defaults below that is about 40 minutes.

EIGHT COILS DRIVE FOUR DOF -- including rotation, which four cannot see at all.
So the 8x8 response matrix is rank 4 by construction. `report()` checks that, and
a rank that is not 4 means the measurement is wrong rather than the optic: too high
is crosstalk or drift, too low means a coil did not move. Channels 4..7 have never
been driven, so verify the coil map first -- see CLAUDE.md item 2.

SAFETY. Open loop -- nothing is damping while this runs, so the optic is only as
quiet as the lab. AMP is small and every output is clamped to the controllers'
own 0..0.5 V window. A railed sensor invalidates its own step and is reported
rather than averaged in. All coils are returned to bias on the way out,
including on Ctrl+C.
"""

import sys
import time

import numpy as np

import sysid
from pyDAC2 import FastDAC

PORT = "COM7"                       # bench.py overwrites this
VERSION_TAG = "v6.5"
KIND = "sysid"                      # not a controller; harness/bench skip it

NCOLS = 8                           # all eight OSEMs, one rigid body
COILS = [0, 1, 2, 3, 4, 5, 6, 7]
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]  # sensor index -> DAC channel (provenance.md)
EXPECTED_DOF = 4                    # eight coils drive four DOF, incl. rotation

FREQS = np.round(np.geomspace(0.3, 4.0, 14), 4)   # 8 coils drive 4 DOF, so the
                                                  # sweep has to reach all four
AMP = 0.05                          # volts peak on the coil, on top of BIAS
SETTLE_S = 5.0                      # discarded: Q~50 at 1 Hz rings for ~16 s
DWELL_S = 20.0                      # integrated; long dwell averages the residual


def main():
    dac = FastDAC(port=PORT)
    for c in COILS:
        dac.set_voltage(DAC_MAP[c], sysid.BIAS_V)
    print("\n  v6.5: stepped sine, %d coils x %d frequencies, amp %.3f V"
          % (len(COILS), len(FREQS), AMP))
    print("  estimated %.0f min. Nothing is damping while this runs."
          % (len(COILS) * len(FREQS) * (SETTLE_S + DWELL_S) / 60.0))
    input("  Press Enter to start (Ctrl+C to stop)... ")
    dac.start_stream()
    rec = sysid.Recorder("v65_stepped8", NCOLS, COILS)
    try:
        H, ok = sysid.stepped(dac, NCOLS, COILS, DAC_MAP, FREQS,
                              AMP, DWELL_S, SETTLE_S, rec=rec)
        sysid.report(H, ok, FREQS, COILS, EXPECTED_DOF)
        sysid.save(H, ok, FREQS, COILS, "v65_stepped8", raw_path=rec.path,
                   extra=dict(amp=AMP, dwell_s=DWELL_S, settle_s=SETTLE_S,
                              expected_dof=EXPECTED_DOF))
    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        rec.close()          # before anything that could itself fail
        for c in COILS:
            dac.set_voltage(DAC_MAP[c], sysid.BIAS_V)
        time.sleep(0.1)
        dac.stop_stream()
        dac.close()


if __name__ == "__main__":
    main()
