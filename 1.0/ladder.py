"""The ladder: which controllers exist, and in what order.

ONE OWNER FOR THE NAMES. `harness.py` and `bench.py` both need to discover
controllers and both need them in ladder order. They used to carry a copy each
of the same regex, and the two patterns drifted: `make run` silently could not
see `osem.v5.5.py` at all while `make check` could. A version the bench cannot
select is a version that does not get run. So the list AND the directory scan
(`discover`) live here and both import them.

This module imports `glob` and `os` and NOTHING ELSE, deliberately. `bench.py`
is the hardware entry point and must not acquire numpy, `sim/` or a stubbed
`serial` by way of a name lookup.

WHY NAMES AND NOT NUMBERS. The numbered ladder ran to v13 and the numbers stopped
carrying information: v10 and v11 were both "newer than v9" and one of them damped
worse than v9 did. Numbers also imply a total order that the history does not
have -- v7 was a branch off v5, not a successor to it. Names force the question
"which one do I run" to be answered by reading, not by taking the largest number.

    epsilon  the Kalman velocity estimator replacing bandpass-and-differentiate,
             `mains-null` decimation, and Ki dropped to zero because the filter
             carries drift as a state rather than integrating it. Written
             2026-08-07. It has never run on the bench, and its suite failures
             are the worst on the ladder.
    zeta     the modal (MIMO) law, and the first rung whose control law is not
             in its own file: Phi and A live in data/modal.json, written by
             `status.py --save-modal`. With no file, a stale one, or one that
             fails its own checks, zeta runs epsilon's diagonal law and says
             which. RAN ON THE RIG 2026-08-17 and damped 5x better than the
             diagonal law by decay rate -- the first result on this rig that is
             not four independent SISO loops.
    eta      the same modal law with the velocity coming from ONE Kalman filter
             whose STATE is the modal coordinates -- 3 modes x 2 plus a DC state
             per sensor, 14 states -- instead of zeta's per-mode least squares
             over eight per-sensor filters. A demoted or railed sensor is a
             DELETED ROW, no re-weighting. The point of it is that a filter knows
             its own innovation covariance, so zeta's out-of-mode residual
             becomes a CALIBRATED statistic: chi2 per degree of freedom, order 1
             when the three-mode model fits. It is LOGGED AND NOT ACTED ON -- the
             distribution on this rig has never been measured, so a threshold
             would be a guess, and deriving one from a bench run is the next
             rung. The cost is that Phi is now inside the estimator, so a wrong
             Phi gives a wrong velocity; zeta's refusal ladder is kept whole and
             chi2 is what detects it. RAN ON THE RIG 2026-08-18 -- one diagonal
             null test and one modal run -- and is the first rung in this repo
             ever to print LOCKED (DEGRADED, 4/7, on a stored baseline).
    theta    eta's law under ONE DECLARED ACTUATOR BUDGET, divided by NEED.
             `BUDGET_V = 0.225 V` is the demand about bias a single coil may
             carry -- eta's cap, but declared once and solved on the TOTAL
             output rather than on the allocation alone, which is how coil 3
             reached 0.4229 V against a 0.250 V half-window and clipped. Four
             claimants -- the three modes and the per-channel block -- share it
             in proportion to each one's own DISSIPATION RATE, not to `ratio`:
             `ratio` is referenced to a zero-gain baseline and reads 13x apart
             for the same plate under the two laws. The split is one-poled at
             `BUDGET_TAU_S = 20 s`, slew-capped, floored, and RAMPS BACK TO FLAT
             when the statistic is unavailable rather than latching. `BUDGET_TILT`
             defaults to 0.0, at which every weight is EXACTLY 1.0 for every
             possible need vector and theta is bit-identical to eta -- the
             selftest drives both on one sensor history and compares commanded
             voltages sample by sample. An earlier draft of this rung also
             carried a per-mode FIR kernel bank; that layer was REMOVED after
             the phase was swept on the rig 2026-08-17 and zero came out the
             optimum (median `ratio` 0.155 at 0 deg against 0.862 / 0.790 /
             1.867 at -30 / +30 / -60). Written 2026-08-18, not yet on the bench.

Alphabetical order is NOT ladder order -- eta is the newest rung and sorts before
zeta. That is the whole reason this is an explicit tuple rather than a sort key.

Deleted rungs are in git history and `versions.md` says what each one was:
zero (v0), alpha (v4), beta (v9), delta (v12), and v1, v2, v3, v5, v5.5, v7, v8,
v10, v11, v13 plus the v6 sysid variants. `RETIRED` below is why typing one of
those names still gets an answer rather than "no such version".

DELTA WAS DELETED 2026-08-17 and it was the only rung ever marked validated on
hardware. What replaces it is not another file: zeta and eta both run epsilon's
diagonal law verbatim when `data/modal.json` is absent or refused, so the
diagonal control is still one run away -- that is the null test in CLAUDE.md
Sec 4. `git show b90c983:osem.delta.py` if a direct comparison is ever needed.
"""

import glob
import os

# Ladder order. Position in this tuple IS the order; nothing is derived from
# the name. Appending a name here is how a new rung becomes visible to
# `make run`, `make check` and `make list` at once.
LADDER = ("epsilon", "zeta", "eta", "theta")

# Files matching `osem.<name>.py` that are NOT rungs. They close no loop, so the
# simulator refuses them and `make run` prints a different banner. Each also
# declares `KIND` in its own source; this list is only for ordering, and the
# declaration in the file is what the tools actually gate on.
TOOLS = ("sysid",)

PREFIX = "osem."
SUFFIX = ".py"


def stem(basename):
    """'osem.eta.py' -> 'eta'. Returns None if it is not one of ours."""
    if not (basename.startswith(PREFIX) and basename.endswith(SUFFIX)):
        return None
    name = basename[len(PREFIX):-len(SUFFIX)]
    return name if name in LADDER or name in TOOLS else None


def sort_key(basename):
    """Ladder order, tools after every rung.

    Returns None for anything that is not ours, so callers can filter and sort
    in one pass. A file named `osem.something.py` that is in neither tuple is
    invisible on purpose: an unlisted controller is one nobody decided to ship,
    and silently offering it on `make run` is how the wrong thing gets flashed.
    """
    name = stem(basename)
    if name is None:
        return None
    if name in LADDER:
        return (0, LADDER.index(name))
    return (1, TOOLS.index(name))


def filename(name):
    """'eta' -> 'osem.eta.py'."""
    return PREFIX + name + SUFFIX


def discover(dirpath):
    """Every controller in `dirpath`, in ladder order, tools last.

    One copy, called by both entry points. Anything `sort_key` does not
    recognise is skipped, which is what keeps an unlisted `osem.*.py` off
    `make run`.
    """
    found = []
    for path in glob.glob(os.path.join(dirpath, PREFIX + "*" + SUFFIX)):
        k = sort_key(os.path.basename(path))
        if k is not None:
            found.append((k, path))
    return [p for _, p in sorted(found)]


def bare(text):
    """Strip the prefix and suffix: 'osem.eta.py' -> 'eta'. Not a lookup."""
    t = (text or "").strip()
    if t.startswith(PREFIX):
        t = t[len(PREFIX):]
    if t.endswith(SUFFIX):
        t = t[:-len(SUFFIX)]
    return t


def resolve_name(text):
    """Accept 'eta', 'osem.eta', 'osem.eta.py' -> 'eta', else None.

    Numbers are accepted and REDIRECTED rather than rejected, because every log,
    docstring and bench note written before the rename says 'v12'. Telling
    somebody that v12 does not exist is true and useless; telling them what
    happened to it is what they needed -- and since delta itself is now deleted,
    'v12' redirects to "that one is gone" rather than to a missing file.
    """
    if not text:
        return None
    t = bare(text)
    return t if t in LADDER or t in TOOLS else None


# The rename, for error messages. Keys are the old numbered names WITHOUT the
# leading 'v'. A version that is gone maps to None and the caller says so rather
# than pointing at a file that is not there -- which is why 0, 4, 9 and now 12
# are None and not "zero"/"alpha"/"beta"/"delta": a two-step redirect to a
# deleted file is worse than saying it is deleted.
RENAMED = {"6": "sysid",
           "0": None, "1": None, "2": None, "3": None, "4": None, "5": None,
           "5.5": None, "7": None, "8": None, "9": None, "10": None,
           "11": None, "12": None, "13": None}

# Names that were rungs and are not any more, so that `beta` -- which is what
# the notes and logs say -- gets the same answer as `v9` rather than falling
# through to "no such version".
RETIRED = ("zero", "alpha", "beta", "delta")


def redirect(text):
    """'v6' -> ('sysid', True). 'v3' / 'beta' / 'delta' -> (None, True).
    'zzz' -> (None, False).

    The second element says whether this LOOKED like a former version name,
    which is what lets a caller distinguish "that one is gone" from "no idea
    what you typed".
    """
    if not text:
        return None, False
    t = bare(text)
    if t in RETIRED:
        return None, True
    t = t.lstrip("vV")
    if t in RENAMED:
        return RENAMED[t], True
    return None, False
