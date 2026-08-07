"""
v6 -- stepped-sine actuation matrix, FOUR coils.
================================================
NOT a controller. It damps nothing and closes no loop. It drives one coil at a
time with a sine and records what all four OSEMs do, to recover the **actuation
matrix** -- the one unmet prerequisite for `research.md` item 2 (modal/MIMO
damping), and the exact thing the previous attempt got wrong.

Why stepped sine rather than something cleverer: while coil j is driving, no
other coil is. There is nothing to disentangle, no matrix to invert, and no
assumption about quadrature. Both quadratures come out of the lock-in together,
so magnitude AND phase are measured rather than inferred -- `research.md` item 2
cause 1 was a zero-lag correlation on velocity, which has no coherent term to
find for a force that acts on acceleration.

It is slow, and that is the trade. Four coils x the frequency list x
(SETTLE + DWELL). With the defaults below that is about 20 minutes.

FOUR COILS DRIVE TWO DOF, so the 4x4 response matrix is rank 2 by construction.
`report()` checks that, and a rank that is not 2 means the measurement is wrong
rather than the optic: too high is crosstalk or drift, too low means a coil did
not move. That check independently reproduces v4's "2 of 4 modes carry real
motion, 98.8% / 1.2%".

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
VERSION_TAG = "sysid"
KIND = "sysid"                      # not a controller; harness/bench skip it

NCOLS = 4                           # the firmware streams 8; only A0..A3 here
COILS = [0, 1, 2, 3]                # sensor index -> its own coil
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]  # sensor index -> DAC channel (provenance.md)
EXPECTED_DOF = 2                    # four coils drive two DOF

FREQS = np.round(np.geomspace(0.3, 4.0, 12), 4)   # must SPAN every resonance:
                                                  # a DOF invisible to the sweep
                                                  # is invisible in the rank check
AMP = 0.05                          # volts peak on the coil, on top of BIAS
SETTLE_S = 5.0                      # discarded: Q~50 at 1 Hz rings for ~16 s
DWELL_S = 20.0                      # integrated; long dwell averages the residual


def main():
    dac = FastDAC(port=PORT)
    for c in COILS:
        dac.set_voltage(DAC_MAP[c], sysid.BIAS_V)
    print("\n  v6: stepped sine, %d coils x %d frequencies, amp %.3f V"
          % (len(COILS), len(FREQS), AMP))
    print("  estimated %.0f min. Nothing is damping while this runs."
          % (len(COILS) * len(FREQS) * (SETTLE_S + DWELL_S) / 60.0))
    input("  Press Enter to start (Ctrl+C to stop)... ")
    dac.start_stream()
    rec = sysid.Recorder("v6_stepped4", NCOLS, COILS)
    try:
        H, ok = sysid.stepped(dac, NCOLS, COILS, DAC_MAP, FREQS,
                              AMP, DWELL_S, SETTLE_S, rec=rec)
        sysid.report(H, ok, FREQS, COILS, EXPECTED_DOF)
        sysid.save(H, ok, FREQS, COILS, "v6_stepped4", raw_path=rec.path,
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
