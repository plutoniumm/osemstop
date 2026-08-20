"""Measure SLOPE_SIGN: which way does sensor j move when coil j pushes?

The hybrid law in osem.eta.py needs one sign per channel. For a0-a3 it is known
(STEADY_GAIN's signs; ch2 is mounted the other way round -- provenance, now in
CLAUDE.md). For a4/a6/a7 it was DEFAULTED to +1, and a wrong sign does not
under-damp, it pumps. This measures it.

A static bias step is a force step. Averaged over many periods of the slowest
mode the ring is zero-mean about the NEW equilibrium, so the mean shift is the
static response -- the same argument _dc_point and analysis/bias_sweep.py use.

    python3 slopesign.py                 # coils 4,5,6,7
    python3 slopesign.py --coils 0,1,2,3 # check against the known signs
"""

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import status  # noqa: E402

V_LO, V_HI = 0.10, 0.40        # CLAUDE.md Sec 6's own voltages
# Long enough that the ring is zero-mean about the new equilibrium: 8.7 periods
# of the slowest mode. Derived from status.MODES, not copied -- the modes moved
# 0.0170 Hz in 11 days and a stale period silently shortens the average.
DWELL = round(8.7 / min(f for _, f in status.MODES), 1)
SETTLE = 3.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--coils", default="4,5,6,7")
    ap.add_argument("--port", default=os.environ.get("PORT"))
    ap.add_argument("--dwell", type=float, default=DWELL)
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    coils = [int(x) for x in args.coils.split(",") if x.strip() != ""]

    port = status.resolve_port(args.port)
    baud = status.probe_baud(port)
    rec = status.Recorder("slopesign")
    dac = None
    try:
        dac = status.open_dac(port, baud)
        status.preflight_stream(dac)
        rdr = status.Reader(dac)
        status.park(dac)
        t, c, _, _ = status.acquire(dac, rdr, rec, 6.0, guard=False)
        print("\n  resting: %s" % " ".join("%6.1f" % v for v in c.mean(axis=0)))
        print("\n  d(counts)/d(bias), %.2f -> %.2f V, one coil at a time, %.0f s each."
              % (V_LO, V_HI, args.dwell))
        print("        " + "".join("%9s" % ("a%d" % i) for i in range(8)))
        D = np.full((8, 8), np.nan)
        blocks = {}
        for j in coils:
            got = []
            for v in (V_HI, V_LO):
                rec.mark("dc", j, -1.0, v - status.BIAS_V)
                dac.set_voltage(status.DAC_MAP[j], status.clamp(v))
                t, c, _, _ = status.acquire(dac, rdr, rec, args.dwell, None, j,
                                            guard=False, hold_v=v)
                got.append(c.mean(axis=0) if len(c) else np.full(8, np.nan))
                if len(c):
                    n = max(int(len(c) / max(args.dwell, 1)), 1)  # ~1 s blocks
                    nb = len(c) // n
                    if nb > 1:
                        blocks[(j, "hi" if v > status.BIAS_V else "lo")] = np.array(
                            [c[b * n:(b + 1) * n, j].mean() for b in range(nb)])
            status.park(dac, [j])
            D[:, j] = (got[0] - got[1]) / (V_HI - V_LO)
            print("  coil%d " % j + "".join("%+9.1f" % x for x in D[:, j]))
            status.acquire(dac, rdr, rec, SETTLE, None, None, guard=False)

        # AXIS GROUPS, from the rig geometry (owner, 2026-08-17): the plate's
        # normal points along the nose. a0-a3 sit on the back of the head in one
        # plane and sense along that normal; a4/a5 are the ears and sense
        # left-right; a6/a7 are on top, attached vertically. So a0-a3 share ONE
        # axis and a push at any of their corners produces translation plus tilt
        # that all four see -- which is why their DC matrix is not diagonally
        # dominant and why a bare diagonal is not a sign for them. Coils 4-7 push
        # along DIFFERENT axes, so their own group is where their response must
        # land, and that makes the sign meaningful.
        GROUP = {0: "normal(Z) a0-a3", 1: "normal(Z) a0-a3", 2: "normal(Z) a0-a3",
                 3: "normal(Z) a0-a3", 4: "ears(X) a4,a5", 5: "ears(X) a4,a5",
                 6: "top(Y) a6,a7", 7: "top(Y) a6,a7"}
        MEMBERS = {"normal(Z) a0-a3": [0, 1, 2, 3], "ears(X) a4,a5": [4, 5],
                   "top(Y) a6,a7": [6, 7]}
        print("\n  ==> Does each coil's response land in its OWN axis group?")
        print("   coil  group             own-group |sum|   other |sum|   ratio")
        for j in coils:
            g = GROUP[j]
            mine = [i for i in MEMBERS[g] if np.isfinite(D[i, j])]
            others = [i for i in range(8) if i not in MEMBERS[g]
                      and np.isfinite(D[i, j])]
            own = float(np.abs(D[mine, j]).sum()) if mine else 0.0
            other = float(np.abs(D[others, j]).sum()) if others else 0.0
            print("    %d    %-17s %12.1f %12.1f   %5.2f%s"
                  % (j, g, own, other, own / other if other > 0 else np.inf,
                     "" if own > other else "   <-- leaks OUT of its group"))

        print("\n  ==> SLOPE_SIGN, own sensor, with the uncertainty OF THE MEAN")
        print("   The shift is a mean over thousands of samples, so what decides")
        print("   the sign is the standard error of that mean, NOT the per-sample")
        print("   dither. Blocked at 1 s first, because the mode band is coherent")
        print("   sample-to-sample and treating samples as independent would")
        print("   understate sigma by ~20x.")
        print("   ch   counts/V     sigma   n-sigma   SIGN")
        for j in coils:
            A, B = blocks.get((j, "hi")), blocks.get((j, "lo"))
            if A is None or B is None or len(A) < 2 or len(B) < 2:
                print("   a%d   -- not enough blocks" % j)
                continue
            d = (A.mean() - B.mean()) / (V_HI - V_LO)
            se = math.sqrt(A.var(ddof=1) / len(A) + B.var(ddof=1) / len(B)) \
                / (V_HI - V_LO)
            if se <= 0:
                print("   a%d   %+9.2f       n/a       n/a   NO SIGNAL (pin at rail?)" % (j, d))
                continue
            ns = abs(d) / se
            print("   a%d   %+9.2f  %8.2f  %8.1f   %s"
                  % (j, d, se, ns, ("%+d" % np.sign(d)) if ns >= 3.0 else "UNRESOLVED"))
        print("\n  A sign is usable only if the coil's response lands in its own")
        print("  axis group AND its own sensor moves more than the dither. For")
        print("  a0-a3 it never will -- they share one axis; the known signs for")
        print("  those come from the decay fits in CLAUDE.md, not from here.")
    except KeyboardInterrupt:
        print("\n  stopped by user.")
    finally:
        if dac is not None:
            status.park(dac)
            time.sleep(0.3)
            try:
                dac.stop_stream()
                dac.close()
            except Exception:
                pass
        rec.close()
        print("  all coils returned to bias.")


if __name__ == "__main__":
    main()
