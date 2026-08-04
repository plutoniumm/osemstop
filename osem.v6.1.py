"""
v6.1 -- continuous multisine actuation matrix, FOUR coils.
==========================================================
Same measurement as v6, same output format, 4 passes instead of 36 dwells. NOT
a controller.

HOW IT AVOIDS WHAT KILLED THE LAST ATTEMPT. All four coils drive at once, but
each one owns its own **interleaved comb of frequency bins**, and the assignment
ROTATES over 4 passes so every coil is measured at every bin. At any one moment a
given bin belongs to exactly one coil, so the responses separate by frequency
instead of by matrix inversion. The previous attempt drove all four coils with
simultaneous PRBS and correlated at a single lag; because the four drives were not
independent, the input cross-spectrum was near-singular (measured on the
2026-08-03 logs: median condition number 2.5e3 over 0.4-3 Hz, peak 2.9e4) and the
recovered matrix disagreed with itself between the two halves of one run, sign
included. Interleaving makes that cross-spectrum diagonal by construction, so
there is nothing left to invert.

WHY IT ROTATES rather than doing a single pass. One pass only ever measures the
column of whichever coil owns each bin, so each frequency comes back rank 1 no
matter how clean the data is -- verified against a fake board with a known
matrix, where a single pass scored 0.19-0.93 correlation with every sign wrong,
and the rotated version scores 1.000 with none. Columns cannot be pasted together
across frequencies either: the response at f is S.diag(G(f)).B and G is a per-DOF
factor, not a scalar.

Every tone is snapped to an exact bin k/DURATION_S, so it completes a whole
number of cycles in the record and leaks nothing into a neighbour's bin. Without
that the interleaving is only approximate and the crosstalk comes back.

WHY THIS ONE NEEDS pyDAC2. Continuous excitation means writing all four coils
continuously. `pyDAC.set_voltage` reads and discards up to 50 stream lines
waiting for its `OK` ack, which measured out at ~24 Hz of usable samples during
DAMPING against a 348 Hz wire. Throwing away fourteen samples in fifteen destroys
the SNR and the phase this method depends on. `pyDAC2.FastDAC` writes and
returns; the acks come back interleaved in the stream and `read_sample` filters
them.

FOUR COILS DRIVE TWO DOF, so the 4x4 matrix is rank 2 and `report()` checks it.

SAFETY. Open loop; nothing damps while this runs. Tone amplitudes are scaled so
their sum stays small and every output is clamped to 0..0.5 V. Coils return to
bias on exit, including on Ctrl+C.
"""

import time

import numpy as np

import sysid
from pyDAC2 import FastDAC

PORT = "COM7"                       # bench.py overwrites this
VERSION_TAG = "v6.1"
KIND = "sysid"

NCOLS = 4
COILS = [0, 1, 2, 3]
DAC_MAP = [1, 3, 5, 7, 0, 2, 4, 6]
EXPECTED_DOF = 2

DURATION_S = 120.0                  # record length; sets the bin spacing 1/T
SETTLE_S = 20.0                     # Q~50 at 1 Hz needs ~16 s to reach steady state
UPDATE_HZ = 50.0                    # coil refresh; SET shares the wire with the stream
AMP = 0.05                          # volts peak per coil, summed over its tones
FREQS = sysid.snap_bins(0.3, 4.0, 16, DURATION_S)     # 4 bins per coil


def main():
    dac = FastDAC(port=PORT)
    for c in COILS:
        dac.set_voltage(DAC_MAP[c], sysid.BIAS_V)
    print("\n  v6.1: continuous multisine, %d coils, %d bins, %d rotation passes"
          % (len(COILS), len(FREQS), len(COILS)))
    print("  %.0fs settle + %.0fs record per pass -> %.0f min total. Nothing is damping."
          % (SETTLE_S, DURATION_S, len(COILS) * (SETTLE_S + DURATION_S) / 60.0))
    print("  bins: %s Hz" % np.round(FREQS, 3).tolist())
    input("  Press Enter to start (Ctrl+C to stop)... ")
    dac.start_stream()
    rec = sysid.Recorder("v61_multisine4", NCOLS, COILS)
    try:
        H, ok = sysid.multisine(dac, NCOLS, COILS, DAC_MAP, FREQS, AMP,
                                DURATION_S, UPDATE_HZ, SETTLE_S, rec=rec)
        sysid.report(H, ok, FREQS, COILS, EXPECTED_DOF)
        sysid.save(H, ok, FREQS, COILS, "v61_multisine4", raw_path=rec.path,
                   extra=dict(amp=AMP, duration_s=DURATION_S, settle_s=SETTLE_S,
                              update_hz=UPDATE_HZ, expected_dof=EXPECTED_DOF))
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
