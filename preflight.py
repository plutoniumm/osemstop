"""preflight.py -- everything that must be true BEFORE a coil is energised.

Opens no serial port and needs no bench time. Run it, READ it, then start the run:

    python3 preflight.py && python3 bench.py eta

WHY THIS EXISTS. On 2026-08-20 three consecutive bench runs were started and then
watched on a timer instead of read, and in every case the controller's own first
status print already said it was failing -- once with chi2/dof = 844.70 against
~1, once with every ratio at 1.0 instead of 0.2, once with a coil already railed
out. The rig owner spotted all three before the session did. Bench time is the
scarce resource here; none of these checks needs any of it.
"""
import importlib.util
import os
import sys

# BEFORE the controller is imported: the modal gate reads it at import time.
os.environ.setdefault("OSEM_MODAL_PROVISIONAL", "1")

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

FAIL = []


def check(name, ok, detail=""):
    print("  %-4s %s%s" % ("PASS" if ok else "FAIL", name,
                           ("   " + detail) if detail else ""))
    if not ok:
        FAIL.append(name)


def load(mod, path):
    spec = importlib.util.spec_from_file_location(mod, HERE + "/" + path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


eta = load("eta", "osem.eta.py")
theta = load("theta", "osem.theta.py")

# Measured DC matrix, D[coil][sensor], all eight coils, slopesign.py 2026-08-20.
# eta carries the 4x4 driven block of it as DC_MATRIX_COUNTS_PER_V; section 4
# checks the two still agree, so this is a cross-check, not a second hand copy.
D = np.array([
    [70.4, -6.3, 155.5, -49.1, -0.6, -0.9, 4.6, 3.3],
    [-2.7, 88.9, -88.5, 61.0, -1.2, 0.4, 4.0, 3.2],
    [-40.7, 39.5, -204.7, 16.6, 1.8, 1.7, 0.2, 0.3],
    [24.1, -60.4, 50.5, -99.1, -0.3, -0.6, 0.5, 0.2],
    [-0.9, 0.4, 8.8, 3.2, 5.1, 8.7, -0.0, 0.3],
    [-5.3, 9.3, -18.4, 11.1, 6.3, -0.4, 0.7, 0.4],
    [8.8, -6.2, 13.8, -11.4, 0.7, 0.1, 10.4, 7.0],
    [2.1, -0.6, -1.1, -4.2, 0.5, -0.9, 9.1, 8.1]])
M4 = D[:4, :4].T
SLOPE = eta.SENSOR_SLOPE_COUNTS_PER_V     # read, never re-typed
# The hybrid-PID channels that are not one of the four allocated coils.
HYB = [i for i in range(eta.N) if eta.HYBRID_CHANNEL[i] and i not in eta.A_DC_COILS]

print("\n=== 1. THE DISSIPATION CONDITION, per channel ===")
print("  power = -slope x gain x vel^2, so slope x gain MUST be > 0")
prod = SLOPE[:4] * eta.STEADY_GAIN[:4]
check("slope x gain > 0 on all four driven channels", bool((prod > 0).all()),
      "products %s" % np.round(prod, 4))
check("DAMP_SIGN carries the slope sign", np.array_equal(eta.DAMP_SIGN, eta.SLOPE_SIGN))
hp = eta.SLOPE_SIGN[HYB] * np.sign(eta.HYBRID_GAIN[HYB])
check("...and so do the hybrid channels", bool((hp >= 0).all()),
      "%s %s" % ("/".join("a%d" % i for i in HYB), hp.astype(int)))

print("\n=== 2. THE MODAL CONDITION -- does the diagonal law damp EVERY mode? ===")
print("  S = A diag(g) Phi; the symmetric part must be positive definite")
Phi4 = eta.PHI_GEOM[:4, :]
A4 = np.linalg.pinv(Phi4) @ M4
S = A4 @ np.diag(eta.STEADY_GAIN[:4]) @ Phi4
w = np.linalg.eigvalsh(0.5 * (S + S.T))
check("every mode dissipates under the diagonal law", bool((w > 0).all()),
      "eigenvalues %s" % np.round(w, 4))

print("\n=== 3. eta and theta agree where they must ===")
for nm in ("STEADY_GAIN", "CAPTURE_GAIN", "KD_GAIN", "SLOPE_SIGN", "DAMP_SIGN",
           "HYBRID_GAIN", "HYBRID_COHERENT", "F_MODE_HZ", "PHI_GEOM", "BIAS",
           "KALMAN_K", "KALMAN_R", "A_DC_COUNTS_PER_V", "MODAL_KP"):
    a, b = getattr(eta, nm, None), getattr(theta, nm, None)
    check("%-20s identical" % nm,
          a is not None and b is not None and np.allclose(np.asarray(a, float),
                                                          np.asarray(b, float)))

print("\n=== 4. A_DC rows are paired with the RIGHT Phi columns ===")
import status                                              # noqa: E402
recomputed = np.array([(eta.PHI_GEOM[:, m] @ D.T) / float(eta.PHI_GEOM[:, m]
                                                          @ eta.PHI_GEOM[:, m])
                       for m in range(3)])
check("A_DC matches today's DC matrix projected onto today's Phi",
      np.allclose(recomputed, eta.A_DC_COUNTS_PER_V, atol=0.02),
      "max |diff| %.3f" % np.abs(recomputed - eta.A_DC_COUNTS_PER_V).max())
check("GEO_DOF labels match PHI_GEOM's columns",
      status.GEO_DOF == ("T2 tilt", "Z normal", "T1 tilt"),
      "%s" % (status.GEO_DOF,))
check("eta's 4-coil DC block is still this same measurement",
      np.allclose(D[:4, :4], eta.DC_MATRIX_COUNTS_PER_V),
      "max |diff| %.3f" % np.abs(D[:4, :4] - eta.DC_MATRIX_COUNTS_PER_V).max())

print("\n=== 5. modal.json, if present ===")
mp = HERE + "/data/modal.json"
if not os.path.exists(mp):
    print("  (absent -- the run will be DIAGONAL, which is a valid configuration)")
else:
    m = eta._modal(mp)
    check("modal.json loads", bool(m.ok), str(getattr(m, "why", "")))
    if m.ok:
        mask = np.zeros(eta.N, bool)
        mask[m.a_coils] = True
        rng = np.random.default_rng(0)
        worst, npump = -np.inf, 0
        Am = np.asarray(m.a)
        for _ in range(4000):
            qd = rng.normal(size=m.nm)
            u, _mk, _cm, aok = m.allocate(-eta.MODAL_KP * qd, mask)
            if not aok:
                continue
            p = float((Am @ np.asarray(u, float)) @ qd)
            worst = max(worst, p)
            npump += (p > 0)
        check("the modal law DISSIPATES over 4000 random modal velocities",
              npump == 0, "worst f.qdot %+.3g, pumping on %d" % (worst, npump))

print("\n=== 6. actuator window ===")
lo = eta.BIAS - eta.BIAS_SWING
hi = eta.BIAS + eta.BIAS_SWING
check("the window is inside the DAC's range",
      bool((lo >= eta.VMIN - 1e-9).all() and (hi <= eta.VMAX + 1e-9).all()),
      "%.2f..%.2f V" % (lo.min(), hi.max()))
# 1.10 V on all eight coils at once killed the front end on 2026-08-20; the load
# that did it is that voltage squared, summed over the channels.
KILLED_FRONT_END_V = 1.10
killed_v2 = eta.N * KILLED_FRONT_END_V ** 2
load_v2 = float((eta.BIAS ** 2).sum())
check("static load is under the %.2f that took the front end down" % killed_v2,
      load_v2 < killed_v2, "sum V^2 = %.2f (%.2f V uniform is %.2f)"
      % (load_v2, 0.25, eta.N * 0.25 ** 2))

print("\n" + ("  ALL PREFLIGHT CHECKS PASS -- safe to start a run." if not FAIL
              else "  %d FAILURE(S): %s" % (len(FAIL), ", ".join(FAIL))))
sys.exit(1 if FAIL else 0)
