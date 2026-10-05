"""
sysid: stepped-sine actuation matrix, FOUR coils. NOT a controller; it damps
nothing and closes no loop. Drives one coil at a time and records all four
OSEMs to recover A, the actuation matrix. One coil at a time means nothing to
disentangle, and the lock-in returns both quadratures, so magnitude AND phase
are measured rather than inferred. Cost is 4 coils x 12 freqs x
(SETTLE + DWELL), about 20 min.

Four coils drive two DOF, so the 4x4 matrix is rank 2 by construction.
report() checks it; rank != 2 means the measurement is wrong, not the optic
(high is crosstalk or drift, low means a coil did not move). Reproduces v4's
"2 of 4 modes carry real motion, 98.8% / 1.2%".

SAFETY: open loop, so nothing damps while this runs. Outputs are clamped to
sysid.py's own 0..0.5 V window around BIAS = 0.25. A railed sensor invalidates
its own step. All coils return to bias on exit, Ctrl+C included.

STALE, AND KNOWN TO BE: the DOF count above is the 2026-08-06 reading. The rig
was measured on 2026-08-17 to carry THREE modes, so EXPECTED_DOF here is a
2026-08-06 number and the rank line it prints is a diagnostic, not a verdict.
"""

import time

import numpy as np

import stdlib
import sysid

PORT = "COM7"                       # bench.py overwrites this
VERSION_TAG = "sysid"
KIND = "sysid"                      # not a controller; harness/bench skip it

NCOLS = 4                           # the firmware streams 8; only A0..A3 here
COILS = [0, 1, 2, 3]                # sensor index -> its own coil
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]  # sensor index -> DAC channel. Rewired
                                    # 2026-08-03; the PREVIOUS map [0,2,4,6] now
                                    # points at the coils for sensors 4-7 and
                                    # NOTHING IN SOFTWARE CAN CATCH THAT
                                    # (CLAUDE.md, "the wrong-map trap").
EXPECTED_DOF = 2                    # four coils drive two DOF
TAG = "v6_stepped4"                 # names the CSV and the JSON; kept for
                                    # continuity with data/*_v6_stepped4* on disk

FREQS = np.round(np.geomspace(0.3, 4.0, 12), 4)   # must SPAN every resonance: a
                                                  # DOF the sweep misses is also
                                                  # missing from the rank check
AMP = 0.05                          # volts peak on the coil, on top of BIAS
SETTLE_S = 5.0                      # discarded: Q~50 at 1 Hz rings for ~16 s
DWELL_S = 20.0                      # integrated; averages the residual


def main():
    # open_dac probes the baud first. Never skip that: at the wrong rate the
    # READY scan is 200 lines x 2 s = 400 s of silence with nothing printed.
    dac = stdlib.open_dac(PORT)
    chans = [DAC_MAP[c] for c in COILS]
    sysid.park(dac, chans)
    print("\n  sysid: stepped sine, %d coils x %d frequencies, amp %.3f V"
          % (len(COILS), len(FREQS), AMP))
    print("  estimated %.0f min. Nothing is damping while this runs."
          % (len(COILS) * len(FREQS) * (SETTLE_S + DWELL_S) / 60.0))
    input("  Press Enter to start (Ctrl+C to stop)... ")
    dac.start_stream()
    rec = sysid.Recorder(TAG, NCOLS, COILS)
    try:
        H, ok = sysid.stepped(dac, NCOLS, COILS, DAC_MAP, FREQS,
                              AMP, DWELL_S, SETTLE_S, rec=rec)
        sysid.report(H, ok, FREQS, COILS, EXPECTED_DOF)
        sysid.save(H, ok, FREQS, COILS, TAG, raw_path=rec.path,
                   extra=dict(amp=AMP, dwell_s=DWELL_S, settle_s=SETTLE_S,
                              expected_dof=EXPECTED_DOF))
    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        rec.close()          # before anything that could itself fail
        sysid.park(dac, chans)
        time.sleep(0.1)
        dac.stop_stream()
        dac.close()


if __name__ == "__main__":
    main()
