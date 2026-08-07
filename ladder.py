"""The ladder: which controllers exist, and in what order.

ONE OWNER FOR THE NAMES. `harness.py` and `bench.py` both need to discover
controllers and both need them in ladder order. They used to carry a copy each
of the same regex, and `bench.py` still records what that cost: the two patterns
drifted, and `make run` silently could not see `osem.v5.5.py` at all while
`make check` could. A version the bench cannot select is a version that does not
get run. So the list lives here and both import it.

WHY NAMES AND NOT NUMBERS. The numbered ladder ran to v13 and the numbers stopped
carrying information: v10 and v11 were both "newer than v9" and one of them damped
worse than v9 did. Numbers also imply a total order that the history does not
have -- v7 was a branch off v5, not a successor to it. Names force the question
"which one do I run" to be answered by reading, not by taking the largest number.

    zero     the original. One channel. A saturation trip LATCHES: it is an
             absorbing fault state needing a manual restart. Kept as the
             baseline the ladder is measured against, NOT to be run unattended.
    alpha    graceful degradation. One blind OSEM is demoted rather than
             faulting the rig.
    beta     the supervisor, four channels. `runaway-trend` -- the breaker
             tests for GROWTH rather than level, which is what stopped the
             post-kick fault thrash. Best DAMPING fraction on record, 83.1%.
    delta    eight channels, 500000 baud, a 100 Hz control clock decoupled from
             the wire, and the fixes that made that safe: `sample-guard`,
             `decimate`, `persist-baseline`, `baseline-sanity`. The one to run.
    epsilon  the Kalman velocity estimator replacing bandpass-and-differentiate,
             `mains-null` decimation, and Ki dropped to zero because the filter
             carries drift as a state rather than integrating it. Written
             2026-08-07, `BENCH_STATUS = untested`: it has never run on the
             bench, so delta is still the one to run.

Alphabetical order is NOT ladder order -- "zero" sorts last and is first. That is
the whole reason this is an explicit tuple rather than a sort key.

Deleted rungs are in git history and `versions.md` says what each one was:
v1, v2, v3, v5, v5.5, v7, v8, v10, v11, v13, and the v6 sysid variants.
"""

# Ladder order. Position in this tuple IS the order; nothing is derived from
# the name. Appending a name here is how a new rung becomes visible to
# `make run`, `make check` and `make list` at once.
LADDER = ("zero", "alpha", "beta", "delta", "epsilon")

# Files matching `osem.<name>.py` that are NOT rungs. They close no loop, so the
# simulator refuses them and `make run` prints a different banner. Each also
# declares `KIND` in its own source; this list is only for ordering, and the
# declaration in the file is what the tools actually gate on.
TOOLS = ("sysid",)

PREFIX = "osem."
SUFFIX = ".py"


def stem(basename):
    """'osem.delta.py' -> 'delta'. Returns None if it is not one of ours."""
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
    """'delta' -> 'osem.delta.py'."""
    return PREFIX + name + SUFFIX


def resolve_name(text):
    """Accept 'delta', 'osem.delta', 'osem.delta.py' -> 'delta', else None.

    Numbers are accepted and REDIRECTED rather than rejected, because every log,
    docstring and bench note written before the rename says 'v12'. Telling
    somebody that v12 does not exist is true and useless; telling them it is
    now `delta` is what they needed.
    """
    if not text:
        return None
    t = text.strip()
    if t.startswith(PREFIX):
        t = t[len(PREFIX):]
    if t.endswith(SUFFIX):
        t = t[:-len(SUFFIX)]
    return t if t in LADDER or t in TOOLS else None


# The rename, for error messages. Keys are the old numbered names WITHOUT the
# leading 'v'. Versions that were deleted rather than renamed map to None, and
# the caller says so rather than pointing at a file that is not there.
RENAMED = {"0": "zero", "4": "alpha", "9": "beta", "12": "delta", "6": "sysid",
           "1": None, "2": None, "3": None, "5": None, "5.5": None,
           "7": None, "8": None, "10": None, "11": None, "13": None}


def redirect(text):
    """'v12' -> ('delta', True). 'v3' -> (None, True). 'zzz' -> (None, False).

    The second element says whether this LOOKED like an old version name, which
    is what lets a caller distinguish "that was renamed" from "no idea what you
    typed".
    """
    if not text:
        return None, False
    t = text.strip().lstrip("vV")
    if t in RENAMED:
        return RENAMED[t], True
    return None, False
