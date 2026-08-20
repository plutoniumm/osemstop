"""jerk.py -- kick the optic by hand, repeatedly, and time how fast it re-damps.

DAMPING fraction and lock time are whole-run properties: they mix acquisition,
faults and schedule in with the only question that separates two control laws --
given a disturbance of a given size, how fast does this law remove it? Both laws
damp; the modal claim is that it damps FASTER. A run average cannot see that.

The human is the actuator. This program launches a controller, watches its CSV,
waits for the run's own quiet held for STABLE_HOLD_S, says "jerk it" through
/usr/bin/say, then finds the peak and fits the decay back to quiet. One row per
kick. Protocol proposed by the rig's operator 2026-08-17.

A HAND KICK, NOT A COIL KICK. A coil kick excites the modes in a fixed ratio set
by one column of A -- the matrix under test. A shove excites all three in a ratio
nobody chose: less controlled, more honest.

THE KICKS ARE NOT EQUAL, so the DECAY RATE is the comparable number and the
time-to-quiet only compares at matched peak. Both are printed.

MEASURED 2026-08-17, three kicks per law, one session, `zeta`:

    law        peak ratio   decay 1/s                median   re-quiet   r2
    modal      3.96-4.70    0.0600/0.1393/0.1940     0.1393   5.3 s      0.79-0.90
    diagonal   3.66-4.44    0.0184/0.0278/0.0311     0.0278   17.4 s     0.53-0.69

Ranges do not overlap -- the worst modal kick beats the best diagonal kick by 2x,
median by 5x -- and the peaks are matched across the sets, so re-quiet is
comparable too. Against the plant's intrinsic 0.0072 /s (tau > 138 s,
`analysis/ringdown.md`) that is 4x for diagonal and 19x for modal. CAVEATS: three
kicks per law, one session, ambient drifts between sets.

2026-08-18: TWO SESSIONS, SIX REPORTED KICKS, AND NOT ONE CLEAN NUMBER. This
is the defect this file was rewritten to fix. Both runs faulted on the first
kick and never recovered; measured from the runs' own `state` columns:

    run                            state        gain    median ratio
    20260818_001841 diagonal   0.0- 20.0 s  CALIBRATING  0.0000   --
    159.6 s total             20.0- 40.9 s  DAMPING      0.0333   1.24
                              40.9-159.6 s  FAULT        0.0000   2.40
    20260818_002229 modal      0.0-  2.0 s  CALIBRATING  0.0000   --
    133.0 s total              2.0- 38.5 s  DAMPING      0.0342   0.25
                              38.5-133.0 s  FAULT        0.0000   2.25

Every gain reads +0.0000 in FAULT, so a decay fitted there is the FREE PLANT
(0.0072 /s, `analysis/ringdown.md`), not a control law. Kicks 2 and 3 of BOTH
runs landed entirely inside the faulted stretch, and both first kicks -- the
only two real ones -- had decay windows that crossed into it. So each run
yielded ONE PARTIAL kick, diagonal 0.0434 /s and modal 0.0683 /s, and neither
is a clean number.

`jerk.py` REPORTED ALL SIX AND FLAGGED NONE. The diagonal three read
0.0434 / 0.0124 / 0.0492 /s at r2 0.782 / 0.588 / 0.788 -- entirely plausible,
with nothing in the output to say the loop had been dead for two of them. THE
r2 FILTER WOULD NOT HAVE CAUGHT THEM: a free ringdown is exponential too. Only
the `state` column separates them, which is why that check is first in `grade`,
is unconditional, and abandons the window the moment it fires.

NO SESSION-TO-SESSION SCATTER IS ESTABLISHED. A "1.6x between sessions of the
same law" figure was derived from a median that included two of those invalid
fits and is withdrawn. Repeating the kick test to measure that scatter is
CLAUDE.md sec 4 and it has not been done.

WHY IT NEVER PROMPTED AGAIN, and it is mechanical:

  * `STABLE_RATIO = 2.00` / `KICK_RATIO = 2.70` were ABSOLUTE, derived from
    diagonal sessions whose quiet sits near 1.2-1.6. The modal loop holds the
    plate at 0.1185 (median of `20260818_002229_fast_lock.csv`, 25-33 s, 2514
    rows) -- 13x quieter -- so "quiet <= 2.00" and "kicked >= 2.70" no longer
    meant quiet or kicked. Thresholds are now multiples of the run's OWN quiet.
  * The faulted baseline sat at median 2.40 (diagonal) and 2.253 (modal), BOTH
    ABOVE 2.00. A dead loop does not damp, so the plate can never look quiet,
    so the quiet-hold can never complete and the cue can never fire. The
    operator stood at the table for 95 s and 119 s waiting for it.
  * Meanwhile that same baseline wandered across 2.70 twice on the modal run
    and both crossings were recorded as kicks, each with peak ratio 2.73 --
    the trigger value itself. One of those fits came out with a NEGATIVE decay
    (-0.0053 /s, the amplitude grew) at r2 0.065, and it was printed as a
    result.

RELATIVE THRESHOLDS ALONE WOULD NOT HAVE CAUGHT ANY OF THAT. On the modal run
they make arming HARDER (the quiet band becomes 0.166, which a faulted plate
never reaches) and the kick trigger EASIER (0.213, which the faulted baseline is
always above). What catches it is the `state` check, the rise test, and a FAULT
cue that is spoken out loud within 5 s.

NULL TEST. `zeta` falls back to `epsilon`'s exact diagonal law when its measured
inputs are missing, so with `data/modal.json` absent or refused the two tables
must AGREE. Run that before believing a difference.

    python3 jerk.py zeta
    python3 jerk.py eta
    python3 jerk.py --replay data/20260818_002229_fast_lock.csv   # offline, no rig
    python3 jerk.py --selftest
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ladder                                                    # noqa: E402

# ---------------------------------------------------------------------------
# thresholds
#
# `ratio` is the controller's own per-channel amplitude over its calibrated
# baseline -- the statistic its lock and runaway tests already use, so these
# numbers are in the units the controller thinks in. It is a 1.0 s sliding RMS
# (`SCHEDULE_WINDOW_S` in every controller on the ladder), which is what sets
# PREKICK_LAG_S below.
#
# THE TRIGGERS ARE MULTIPLES OF THE RUN'S OWN QUIET, not absolute levels. Four
# measured quiet levels, 2026-08-17/18, spanning 20x:
#
#   diagonal   1.58     data/signtest_20260817_192808_diagonal.log, median
#   diagonal   1.2224   20260818_001841_fast_lock.csv, 25-35 s DAMPING, 2384 rows
#   modal      0.1185   20260818_002229_fast_lock.csv, 25-33 s DAMPING, 2514 rows
#   modal      0.08     data/signtest_20260817_192808 modal +++, median
#
# so no single absolute pair can mean "quiet" and "kicked" on both.
# ---------------------------------------------------------------------------
# THE MEASURED EDGES. Every threshold below is the geometric mean of two of
# these, so the derivation is arithmetic in the file rather than a number typed
# in beside a sum -- change an edge and every threshold that leans on it moves.
WEAKEST_KICK = 2.32   # weakest measured peak-over-quiet of a REAL hand kick:
                      # 3.66/1.58, 2026-08-17 zeta session (peaks 3.66-4.70 over
                      # six kicks, data/20260817_19*_jerk_zeta.log; quiet median
                      # 1.58 from the same session's signtest). The other three
                      # measurable kicks are far above it -- 4.67/1.2224 = 3.82
                      # and 4.841/0.1185 = 40.9 on 2026-08-18, 3.96/0.08 = 49.5
                      # for zeta modal.
FALSE_KICK = 1.21     # the two FALSE kicks of 2026-08-18: peak 2.73 out of a
                      # stalled FAULT baseline of 2.253 (median of 35848 rows).
WORST_GOOD_R2 = 0.53  # weakest r2 ever accepted, diagonal 2026-08-17 (accepted
                      # population n=6: modal 0.79-0.90, diagonal 0.53-0.69).
WORST_BAD_R2 = 0.065  # the one fit known to be junk -- a 2026-08-18 baseline
                      # wander that fitted a NEGATIVE rate.

STABLE_MULT = 1.40    # quiet band = STABLE_MULT x the run's quiet, from how far
                      # a quiet plate actually wanders in TIME. p90/median over
                      # the two 2026-08-18 quiet stretches above:
                      #   diagonal  1.7153/1.2224 = 1.403   <- binds
                      #   modal     0.1331/0.1185 = 1.123
                      # The noisier law binds, which is the safe way round: a
                      # band that fits diagonal fits modal. (The old absolute
                      # 2.00 was 1.27x the 1.58 diagonal median -- tighter than
                      # this, and measured across channels rather than time.)
# kick trigger = KICK_MULT x the run's quiet: the geometric mean of the two
# facing edges, 1.29x clear of each. Same construction as eta's
# DEAD_PIN_STD_COUNTS.
KICK_MULT = math.sqrt(STABLE_MULT * WEAKEST_KICK)          # 1.80
# A kick must also peak at KICK_RISE x the level it ROSE FROM, measured over
# PREKICK_S ending PREKICK_LAG_S before the crossing. 1.39x clear of each edge.
# THIS IS NOT INDEPENDENT of KICK_MULT -- both are anchored on WEAKEST_KICK --
# so it is one measurement used twice, not two. It sits BELOW KICK_MULT
# deliberately: off a genuinely quiet plate the trigger already implies the
# rise, so this test only bites when the baseline has drifted, which is the case
# it exists for.
KICK_RISE = math.sqrt(FALSE_KICK * WEAKEST_KICK)           # 1.68
# 2.9x clear of each edge. THIS FILTER IS THE WEAKEST OF THE FOUR AND MUST NOT
# BE RELIED ON: the three 2026-08-18 diagonal fits read 0.588 / 0.782 / 0.788
# and two of them were of an UNDRIVEN plate. A free ringdown is exponential too,
# so a good r2 says the envelope decayed, not that the loop decayed it. Only
# `state` says that.
R2_FLOOR = math.sqrt(WORST_BAD_R2 * WORST_GOOD_R2)         # 0.19

# FALLBACKS, used only until the run's own quiet has been measured -- i.e. for
# the first QUIET_MIN_SPAN_S of DAMPING, before the first ask. They are the old
# absolute pair and they are wrong for any law whose quiet is not diagonal's;
# they exist so the tool has a number before it has a measurement, not because
# they are right.
STABLE_RATIO = 2.00          # diagonal quiet p90, rounded
KICK_RATIO = math.sqrt(2.01 * 3.66)   # 2.70; 2.01 is that p90 unrounded and
                                      # 3.66 the weakest 2026-08-17 kick peak

QUIET_WINDOW_S = 30.0        # trailing window the quiet median is taken over
QUIET_MIN_SPAN_S = 8.0       # ...and the least it may span before it is used.
                             # The modal run settled at 25.0 s and was kicked at
                             # 33 s, so 8 s is the shortest quiet stretch this
                             # rig has actually offered before a kick. Longer
                             # and the first ask never fires.
PREKICK_S = 5.0              # the "level it rose from" is a median over this
                             # much history: 3.6 cycles of the slowest mode
                             # (0.72194 Hz), so it is a level and not a phase.
PREKICK_LAG_S = 1.0          # ...ending this far before the crossing, because
                             # `ratio` is a 1.0 s sliding RMS and anything
                             # closer already contains the kick.
PEAK_HOLD_S = 1.0            # a peak must STAND this long before the 15% drop
                             # is believed, for the same reason: `ratio` is a
                             # 1.0 s sliding RMS, so nothing shorter than its
                             # window is resolved and a dip on the way up is not
                             # a peak. Without it the modal run's real kick was
                             # mis-peaked at 0.41 on a blip at t=33.5 while the
                             # ratio was still climbing to 4.841 at t=36.8
                             # (replay of 20260818_002229_fast_lock.csv).

STABLE_HOLD_S = 4.0          # quiet must persist this long before asking
WAIT_NOTE_S = 10.0           # progress line cadence while waiting for quiet
KICK_WAIT_S = 90.0           # give up waiting for a kick after this
DECAY_TIMEOUT_S = 180.0      # give up waiting for it to re-quiet. NOTE: a
                             # frozen `ratio` looks identical to one that never
                             # comes down -- `zeta`'s fault-clear deadlock held
                             # 3.59 for 1085 s on 2026-08-17. That case is now
                             # refused on `state` long before this fires.
SETTLE_S = 25.0              # controller time, not wall clock, before the first
                             # ask: calibration is 20 s. Using the CSV's own
                             # clock is what makes `--replay` identical to a
                             # live run.
FAULT_NOTE_S = 5.0           # say out loud that no cue is coming, this soon
                             # after the controller stops damping. 5.0 s is the
                             # controllers' own FAULT_CLEAR_SUSTAIN_S, the
                             # mandatory hold before a fault may clear, so a
                             # fault that is about to clear is never announced
                             # and one that outlives its own minimum is.
FAULT_REPEAT_S = 30.0        # ...and repeated this often while it persists.
                             # Not a measured number: it is a nag interval.
FAULT_ABORT_S = 180.0        # ...and stop after this much of it. Reuses
                             # DECAY_TIMEOUT_S's scale rather than inventing a
                             # number. The 2026-08-18 modal run sat in FAULT for
                             # 94.5 s and never cleared before the run ended, so
                             # this has never actually fired; it exists so a
                             # deadlocked rig releases the bench.
REFUSE_BUDGET = 4            # stop after `kicks + REFUSE_BUDGET` refusals. NOT
                             # a measured number: it exists so a rig that
                             # refuses everything stops instead of holding the
                             # bench forever.
STOP_WAIT_S = 90.0           # how long to wait for the controller after SIGINT.
                             # Its shutdown is BOUNDED at under 6 s of work
                             # (pyDAC2._await's 5.0 s deadline on STOPPED, four
                             # park writes, a 0.1 s settle, prints), yet the old
                             # 25 s ceiling expired anyway on 2026-08-18 and NO
                             # jerk.py run has ever captured a controller's
                             # shutdown output -- every data/*_jerk_*.log ends
                             # mid-line at a periodic print. The cause is not
                             # known, so this is generous on purpose: waiting
                             # costs bench seconds, not waiting means SIGTERM,
                             # which skips the `finally` and leaves the coils
                             # energised (CLAUDE.md sec 15, seen 2026-08-17).
POLL_S = 0.35
SPEAK = "/usr/bin/say"


def speak(text, enabled=True):
    """Say it out loud AND print it. The operator is across the room."""
    print("\n  >>> %s <<<\n" % text.upper(), flush=True)
    if enabled and os.path.exists(SPEAK):
        try:
            subprocess.Popen([SPEAK, text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print("  (could not speak: %s)" % e)


class Tail:
    """Follow a controller CSV that is still being written.

    Flushed every 200 rows, so a read can lag ~2 s at 100 Hz. Fine for a decay
    measured over tens of seconds; nothing here times anything finer.
    """

    def __init__(self, path):
        self.fh = open(path)
        self.rdr = csv.reader(self.fh)
        self.cols = next(self.rdr)
        self.idx = {k: i for i, k in enumerate(self.cols)}
        self.n = sum(1 for k in self.cols if k.endswith("_counts"))
        for k in ("time_s", "state"):
            if k not in self.idx:
                sys.exit("  %s has no `%s` column." % (path, k))
        self.has_ratio = "ch0_ratio" in self.idx
        self.has_modal = "modal" in self.idx

    def rows(self):
        """Whatever has arrived since last call, as dicts. Never blocks."""
        out = []
        for line in self.rdr:
            if len(line) < len(self.cols):
                continue                        # a half-written final line
            try:
                r = dict(t=float(line[self.idx["time_s"]]),
                         state=line[self.idx["state"]])
                live, rat = [], []
                for i in range(self.n):
                    hcol = self.idx.get("ch%d_healthy" % i)
                    if hcol is not None and line[hcol] != "1":
                        continue
                    live.append(i)
                    if self.has_ratio:
                        rat.append(float(line[self.idx["ch%d_ratio" % i]]))
                r["live"] = live
                # MEDIAN, not max: one undriven or noisy channel at a high ratio
                # would make the rig look permanently kicked.
                r["ratio"] = float(np.median(rat)) if rat else float("nan")
                if self.has_modal:
                    r["modal"] = line[self.idx["modal"]]
                    r["mrank"] = line[self.idx["mrank"]] \
                        if "mrank" in self.idx else "?"
                out.append(r)
            except (ValueError, IndexError, KeyError):
                continue
        return out


def newest_csv(before):
    """The controller CSV that appeared after we started, or None."""
    try:
        got = [os.path.join("data", f) for f in os.listdir("data")
               if f.endswith("_fast_lock.csv")]
    except FileNotFoundError:
        return None
    got = [p for p in got if p not in before]
    return max(got, key=os.path.getmtime) if got else None


def fit_decay(t, y):
    """Envelope decay rate in 1/s from a peak-to-quiet segment, or nan.

    Least squares on log(ratio) vs time. `ratio` is already an envelope (a
    windowed RMS), so no Hilbert transform. Returns the POSITIVE rate: larger is
    better. r2 is reported so a fit that is not exponential is visible.
    """
    t, y = np.asarray(t, float), np.asarray(y, float)
    m = y > 1e-9
    if m.sum() < 8:
        return float("nan"), float("nan")
    tt, ly = t[m] - t[m][0], np.log(y[m])
    A = np.column_stack([tt, np.ones_like(tt)])
    beta, *_ = np.linalg.lstsq(A, ly, rcond=None)
    pred = A @ beta
    ss = float(np.sum((ly - pred) ** 2))
    tot = float(np.sum((ly - ly.mean()) ** 2))
    return -float(beta[0]), (1.0 - ss / tot) if tot > 0 else float("nan")


def grade(k):
    """Every reason this kick is not a measurement. Empty list means keep it.

    A REFUSAL IS NOT A SILENT DROP. CLAUDE.md's sign-sweep section records the
    trap: three of the five 2026-08-17 trials scored like diagonal because the
    colocation check had refused them, and the score alone could not tell a
    refusal from a result. So every reason produced here is printed when it
    happens and again in the summary, and a refused kick does not count toward
    `--kicks`.
    """
    bad = []
    # PRIMARY AND UNCONDITIONAL. On 2026-08-18 two runs reported six kicks and
    # four of them were fitted with every gain at +0.0000; three of those four
    # had r2 0.588-0.788, so no fit-quality test would have found them. A free
    # ringdown is exponential too.
    if k["left_state"]:
        bad.append("controller left DAMPING for %s %.1f s into the %.1f s "
                   "decay window -- every gain is +0.0000 there, so this fits "
                   "the FREE PLANT (0.0072 /s), not a control law"
                   % (k["left_state"], k["left_at"], k["window_s"]))
    if k["base"] > 0 and k["peak"] < KICK_RISE * k["base"]:
        bad.append("peak %.2f is only %.2fx the %.2f it rose from, under the "
                   "%.2fx floor -- a baseline drifting across the trigger, not "
                   "a shove" % (k["peak"], k["peak"] / k["base"], k["base"],
                                KICK_RISE))
    if not math.isfinite(k["rate"]):
        bad.append("did not re-quiet within %.0f s -- no decay to fit"
                   % DECAY_TIMEOUT_S)
    else:
        if k["rate"] <= 0:
            bad.append("fitted decay %+.4f /s is not a decay: the amplitude "
                       "GREW over the window" % k["rate"])
        if not math.isfinite(k["r2"]) or k["r2"] < R2_FLOOR:
            bad.append("r2 %.3f is under the %.2f floor -- the envelope is not "
                       "an exponential, so the rate means nothing"
                       % (k["r2"], R2_FLOOR))
    return bad


class Protocol:
    """The kick protocol as a function of the controller's row stream.

    Live and `--replay` drive the SAME object, so a rule can be checked against
    a recorded run without occupying the bench -- the practice that found the
    torn-frame defect and validated `sample-guard` offline. It is also how the
    refusals below were verified against 2026-08-18's two void runs.
    """

    def __init__(self, kicks, ask=None, out=print):
        self.kicks = int(kicks)
        self.ask = ask if ask is not None else (lambda s: None)
        self.out = out
        self.kept, self.refused = [], []
        self.phase = "settle"
        self.hist = []                  # (t, ratio, state), whole run
        self.qbuf = deque()             # quiet population, trailing
        self.pbuf = deque()             # pre-kick level, trailing
        self.quiet = None               # the run's OWN quiet, or None
        self.engaged = False            # has the controller ever damped?
        self.quiet_since = self.asked_at = None
        self.peak_t = self.peak_v = self.kick_t = None
        self.kick_base = self.kick_stable = None
        self.win_left = None            # (state, seconds into the window)
        self.last_note = -1e9
        self.nondamping_since = None
        self.fault_noted = -1e9
        self.suppressed = 0             # excursions ignored outside DAMPING
        self.over = False               # ...edge detector for that count
        self.modal_seen = set()
        self.nlive = 0
        self.aborted = None

    # -- triggers, in the run's own units ---------------------------------
    @property
    def stable_trigger(self):
        return STABLE_MULT * self.quiet if self.quiet is not None else STABLE_RATIO

    @property
    def kick_trigger(self):
        return KICK_MULT * self.quiet if self.quiet is not None else KICK_RATIO

    def done(self):
        return (self.aborted is not None
                or len(self.kept) >= self.kicks
                or len(self.refused) >= self.kicks + REFUSE_BUDGET)

    # -- the run's own quiet ----------------------------------------------
    def _update_quiet(self, t, rat, state):
        """Median ratio over a trailing window of WAITING, DAMPING samples.

        Waiting only: a kick and its ring-down are never in the population, so
        the estimate does not chase what it is meant to measure. Not filtered by
        the trigger, which would ratchet the estimate downwards and never let it
        follow a room that genuinely got louder.
        """
        if state != "DAMPING" or self.phase not in ("wait_quiet", "wait_kick"):
            return
        self.qbuf.append((t, rat))
        while self.qbuf and t - self.qbuf[0][0] > QUIET_WINDOW_S:
            self.qbuf.popleft()
        if self.qbuf[-1][0] - self.qbuf[0][0] < QUIET_MIN_SPAN_S:
            return
        first = self.quiet is None
        self.quiet = float(np.median([v for _, v in self.qbuf]))
        if first:
            self.out("    this run's quiet is %.4f (median of %d samples over "
                     "%.0f s) -- quiet band <= %.3f, kick trigger >= %.3f"
                     % (self.quiet, len(self.qbuf),
                        self.qbuf[-1][0] - self.qbuf[0][0],
                        self.stable_trigger, self.kick_trigger))

    def _prekick_level(self, t):
        """Median ratio over PREKICK_S ending PREKICK_LAG_S before now."""
        v = [x for tt, x in self.pbuf if tt <= t - PREKICK_LAG_S]
        return float(np.median(v)) if v else float("nan")

    # -- fault cue ---------------------------------------------------------
    def _fault_watch(self, t, state):
        if not self.engaged or state == "DAMPING":
            self.nondamping_since = None
            return
        if self.nondamping_since is None:
            self.nondamping_since = t
        held = t - self.nondamping_since
        if held >= FAULT_NOTE_S and t - self.fault_noted >= FAULT_REPEAT_S:
            self.fault_noted = t
            # SPEAK IT. On 2026-08-18 the controller faulted, every gain went to
            # +0.0000, and the operator stood at the table for the rest of the
            # run waiting for a cue that could not come. Silence here is what
            # cost the session.
            self.ask("controller in %s -- no kick coming" % state)
            self.out("    controller has been in %s for %.0f s: every gain is "
                     "+0.0000, nothing is damping, and NO KICK WILL BE ASKED "
                     "FOR until it clears." % (state, held))
        if held >= FAULT_ABORT_S:
            self.aborted = ("controller stuck in %s for %.0f s" % (state, held))

    # -- the state machine -------------------------------------------------
    def feed(self, r):
        if not math.isfinite(r["ratio"]):
            return
        now, rat, state = r["t"], r["ratio"], r["state"]
        self.hist.append((now, rat, state))
        self.pbuf.append((now, rat))
        while self.pbuf and now - self.pbuf[0][0] > PREKICK_S + PREKICK_LAG_S:
            self.pbuf.popleft()
        self.nlive = len(r["live"])
        if "modal" in r:
            self.modal_seen.add((r.get("modal"), r.get("mrank")))
        if state == "DAMPING":
            self.engaged = True
        self._update_quiet(now, rat, state)
        self._fault_watch(now, state)
        if self.aborted:
            return

        if self.phase == "settle":
            if now >= SETTLE_S and state == "DAMPING":
                self.phase, self.quiet_since = "wait_quiet", None

        elif self.phase in ("wait_quiet", "wait_kick"):
            # NOTHING IS ARMED OUTSIDE DAMPING. In FAULT the gains are zero, so
            # a crossing is the plant, not the law, and a decay fitted to it is
            # a measurement of nothing. Counted, and reported in the summary, so
            # the suppression is never invisible.
            if state != "DAMPING":
                if rat >= self.kick_trigger and not self.over:
                    self.suppressed += 1
                self.over = rat >= self.kick_trigger
                self.quiet_since = None
                return
            self.over = False
            if self.phase == "wait_quiet":
                self._wait_quiet(now, rat)
            else:
                self._wait_kick(now, rat)

        elif self.phase in ("peaking", "decay"):
            if state != "DAMPING":
                # Abandon at once rather than after DECAY_TIMEOUT_S: the fit is
                # already void and 180 s of the operator's evening is not.
                self.win_left = (state, now - self.peak_t)
                self._finish(now, timed_out=False)
                return
            if self.phase == "peaking":
                if rat > self.peak_v:
                    self.peak_t, self.peak_v = now, rat
                elif rat < self.peak_v * 0.85 \
                        and now - self.peak_t >= PEAK_HOLD_S:
                    self.phase = "decay"
                    self.out("    peak ratio %.2f (%.2fx the %.2f it rose from)"
                             ", timing the decay"
                             % (self.peak_v,
                                self.peak_v / self.kick_base
                                if self.kick_base else float("nan"),
                                self.kick_base))
            elif rat <= self.kick_stable or now - self.peak_t > DECAY_TIMEOUT_S:
                self._finish(now, timed_out=rat > self.kick_stable)

    def _wait_quiet(self, now, rat):
        if now - self.last_note >= WAIT_NOTE_S:
            self.last_note = now
            self.out("    waiting for quiet: ratio %.3f, need <= %.3f for "
                     "%.0f s%s" % (rat, self.stable_trigger, STABLE_HOLD_S,
                                   "" if rat <= self.stable_trigger
                                   else "  (still decaying)"))
        if rat <= self.stable_trigger:
            self.quiet_since = now if self.quiet_since is None else self.quiet_since
            if now - self.quiet_since >= STABLE_HOLD_S:
                self.ask("jerk it")
                self.out("  quiet at ratio %.3f for %.1f s -- kick %d of %d"
                         % (rat, STABLE_HOLD_S, len(self.kept) + 1, self.kicks))
                self.phase, self.asked_at = "wait_kick", now
                self.peak_t, self.peak_v, self.kick_t = None, 0.0, None
        elif rat >= self.kick_trigger:
            # An UNPROMPTED kick is still a kick. The operator hit the table
            # before being asked on 2026-08-17 and the protocol threw the whole
            # event away, then waited out its decay. It is graded like any
            # other: on 2026-08-18 this branch is what recorded two baseline
            # wanders as kicks, and KICK_RISE is what now refuses them.
            self.out("    unprompted rise to %.2f (trigger %.2f) -- taking it "
                     "as a kick" % (rat, self.kick_trigger))
            self._begin_kick(now, rat)
        else:
            self.quiet_since = None

    def _wait_kick(self, now, rat):
        if rat >= self.kick_trigger:
            self.out("    kick detected at ratio %.2f (trigger %.2f)"
                     % (rat, self.kick_trigger))
            self._begin_kick(now, rat)
        elif now - self.asked_at > KICK_WAIT_S:
            self.out("    no kick within %.0f s -- asking again" % KICK_WAIT_S)
            self.phase, self.quiet_since = "wait_quiet", None

    def _begin_kick(self, now, rat):
        self.kick_t, self.phase = now, "peaking"
        self.peak_t, self.peak_v = now, rat
        self.kick_base = self._prekick_level(now)
        self.kick_stable = self.stable_trigger   # frozen: a moving finish line
        self.quiet_since = None                  # makes re-quiet meaningless
        self.win_left = None

    def _finish(self, now, timed_out):
        seg = [(t, v) for t, v, _ in self.hist if self.peak_t <= t <= now]
        if timed_out or len(seg) < 8:
            rate, r2 = float("nan"), float("nan")
        else:
            rate, r2 = fit_decay([s[0] for s in seg], [s[1] for s in seg])
        k = dict(peak=self.peak_v, t_kick=self.kick_t,
                 t_quiet=float("nan") if timed_out else now - self.peak_t,
                 rate=rate, r2=r2, nlive=self.nlive, base=self.kick_base,
                 quiet=self.quiet, stable=self.kick_stable,
                 window_s=now - self.peak_t,
                 left_state=self.win_left[0] if self.win_left else None,
                 left_at=self.win_left[1] if self.win_left else 0.0)
        bad = grade(k)
        k["why"] = bad
        if bad:
            self.refused.append(k)
            k["n"] = len(self.refused)
            # SAY IT. The operator is at the table and has just shoved it; if
            # that shove did not count they need to know now, not at the end.
            self.ask("kick refused")
            self.out("    !! KICK REFUSED (peak %.2f at t=%.1fs), %d reason(s):"
                     % (k["peak"], k["t_kick"], len(bad)))
            for b in bad:
                self.out("      - %s" % b)
            self.out("      NOT counted toward --kicks; %d of %d kept so far"
                     % (len(self.kept), self.kicks))
        else:
            self.kept.append(k)
            k["n"] = len(self.kept)
            self.out("    re-quiet in %.1f s, decay %.4f /s (r2 %.3f)"
                     % (k["t_quiet"], rate, r2))
        self.phase, self.quiet_since = "wait_quiet", None

    # -- report ------------------------------------------------------------
    def summary(self, label):
        out = self.out
        out("\n  === %s: %d kick(s) kept, %d refused ==="
            % (label, len(self.kept), len(self.refused)))
        if self.aborted:
            out("  ABORTED: %s." % self.aborted)
        if self.quiet is None:
            out("  the run's own quiet was never measured -- the fallback "
                "absolute thresholds (%.2f / %.2f) were in force the whole "
                "time, and they are diagonal's numbers."
                % (STABLE_RATIO, KICK_RATIO))
        else:
            out("  this run's quiet %.4f -> quiet band <= %.3f, kick trigger "
                ">= %.3f" % (self.quiet, self.stable_trigger, self.kick_trigger))
        if self.suppressed:
            out("  %d stretch(es) of ratio above the kick trigger were IGNORED "
                "because the controller was not DAMPING. Not kicks and not "
                "refusals -- the gains were zero, so there was nothing there to "
                "measure." % self.suppressed)

        if self.kept:
            out("\n   kick  peak ratio   re-quiet s   decay 1/s      r2   live ch")
            for d in self.kept:
                out("    %2d      %7.2f      %7.1f     %8.4f  %.3f      %d"
                    % (d["n"], d["peak"], d["t_quiet"], d["rate"], d["r2"],
                       d["nlive"]))
            rr = np.array([d["rate"] for d in self.kept])
            tq = np.array([d["t_quiet"] for d in self.kept])
            out("\n   decay rate   median %.4f /s, spread %.4f-%.4f over %d kick(s)"
                % (np.median(rr), rr.min(), rr.max(), len(rr)))
            out("   re-quiet     median %.1f s, spread %.1f-%.1f s"
                % (np.median(tq), tq.min(), tq.max()))
        else:
            out("\n   NO VALID KICK. Nothing is averaged and no rate is quoted.")

        if self.refused:
            out("\n   REFUSED -- printed because a refusal that is invisible in")
            out("   the score is its own trap (CLAUDE.md, the sign sweep):")
            for d in self.refused:
                out("    r%d  peak %.2f at t=%.1fs, %.1f s window:"
                    % (d["n"], d["peak"], d["t_kick"], d["window_s"]))
                for b in d["why"]:
                    out("        - %s" % b)

        if len(self.kept) < self.kicks:
            out("\n   ASKED FOR %d KICK(S), KEPT %d. The median above is over "
                "%d kick(s)" % (self.kicks, len(self.kept), len(self.kept)))
            out("   and IS NOT the number to compare with another run.")
        out("\n   THE COMPARABLE NUMBER IS THE DECAY RATE. Hand kicks are not")
        out("   equal, so re-quiet time only compares at matched peak ratio --")
        out("   and re-quiet now ends at each run's OWN quiet band, so it does")
        out("   NOT compare across laws whose quiet differs. The earlier")
        out("   re-quiet figures (modal 5.3 s, diagonal 17.4 s) were measured")
        out("   against the absolute 2.00 and are not comparable with these.")
        out("   ONE SESSION IS NOT AN ERROR BAR, and how big the error bar is")
        out("   HAS NOT BEEN MEASURED: no two sessions of the same law have yet")
        out("   produced two sets of valid kicks (CLAUDE.md sec 4).")
        if self.modal_seen:
            on = sorted(self.modal_seen)
            out("\n   modal flag / rank seen during the run: %s" % on)
            if all(m[0] == "0" for m in on if m[0] is not None):
                out("   MODAL NEVER ENGAGED -- this is epsilon's diagonal law. The")
                out("   controller's own preflight says why; see the console log.")


def run(version, kicks, quiet, port):
    before = set()
    if os.path.isdir("data"):
        before = {os.path.join("data", f) for f in os.listdir("data")
                  if f.endswith("_fast_lock.csv")}

    cmd = [sys.executable, "bench.py", version]
    if port:
        cmd += ["--port", port]
    log = os.path.join("data", datetime.now().strftime("%Y%m%d_%H%M%S")
                       + "_jerk_%s.log" % version)
    print("  launching: %s" % " ".join(cmd))
    print("  controller console -> %s" % log)
    fh = open(log, "w", buffering=1)
    proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL)

    tail, path = None, None
    t0 = time.time()
    while tail is None:
        if proc.poll() is not None:
            fh.close()
            print(open(log).read()[-2000:])
            sys.exit("\n  the controller exited before writing a CSV -- see above.")
        path = newest_csv(before)
        if path and os.path.getsize(path) > 0:
            try:
                tail = Tail(path)
            except StopIteration:
                tail = None
        if tail is None:
            if time.time() - t0 > 120:
                sys.exit("  no controller CSV appeared in 120 s.")
            time.sleep(0.5)
    print("  watching %s (%d channels, ratio=%s, modal=%s)"
          % (path, tail.n, tail.has_ratio, tail.has_modal))
    if not tail.has_ratio:
        sys.exit("  this controller does not log `chN_ratio`; nothing to watch.")

    p = Protocol(kicks, ask=lambda s: speak(s, not quiet))
    try:
        while not p.done():
            if proc.poll() is not None:
                print("\n  the controller exited (code %s)." % proc.returncode)
                break
            for r in tail.rows():
                p.feed(r)
                if p.done():
                    break
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        print("\n  stopped by user.")
    finally:
        # SIGINT, never SIGTERM: the controllers park their coils in a `finally`
        # and SIGTERM does not unwind. Same lesson status.py learned on 08-17.
        if proc.poll() is None:
            print("  stopping the controller (SIGINT, so it parks its coils);"
                  " up to %.0f s..." % STOP_WAIT_S)
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                print("  it did not exit in %.0f s -- SIGTERM." % STOP_WAIT_S)
                print("  !! SIGTERM SKIPS THE CONTROLLER'S `finally`, SO THE "
                      "COILS MAY STILL BE ENERGISED at whatever the loop last "
                      "set. Check them (CLAUDE.md sec 15).")
                proc.terminate()
        fh.close()

    if p.aborted:
        print("\n  %s -- the run was stopped." % p.aborted)
    p.summary(version)
    print("\n   console log: %s" % log)
    return p, path


def replay(path, kicks):
    """Grade a recorded controller CSV, offline, with no rig attached.

    Same `Protocol`, same rules, so what this prints is what the bench would
    have printed. Every program that occupies the bench streams its raw samples
    to disk for exactly this reason -- CLAUDE.md's standing practice, which has
    paid for itself three times.
    """
    tail = Tail(path)
    if not tail.has_ratio:
        sys.exit("  %s has no `chN_ratio` columns; nothing to grade." % path)
    p = Protocol(kicks)                     # no `ask`: nobody is at the table
    rows = tail.rows()
    print("  replaying %s -- %d rows, %.1f s" % (path, len(rows),
                                                 rows[-1]["t"] if rows else 0.0))
    st = {}
    for r in rows:
        st[r["state"]] = st.get(r["state"], 0) + 1
        p.feed(r)
        if p.done():
            break
    print("  states: %s" % ", ".join("%s %d (%.1f%%)"
                                     % (k, v, 100.0 * v / max(len(rows), 1))
                                     for k, v in sorted(st.items())))
    p.summary(os.path.basename(path))
    return p


# ---------------------------------------------------------------------------
# selftest
# ---------------------------------------------------------------------------
def _trace(spec, hz=347.0):
    """Rows from [(duration_s, state, f(t_local)->ratio), ...].

    347 Hz is the measured CSV row rate of 2026-08-18's runs (46 158 rows over
    133.03 s), not a round number chosen for convenience.
    """
    rows, t = [], 0.0
    dt = 1.0 / hz
    for dur, state, f in spec:
        n = int(dur * hz)
        for i in range(n):
            rows.append(dict(t=t, state=state, ratio=float(f(i * dt)),
                             live=[0, 1, 2, 3]))
            t += dt
    return rows


def selftest():
    ok = True

    def check(name, cond, note=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("  %-46s %s%s" % (name, "ok" if cond else "FAIL",
                                ("  -- " + note) if note else ""))

    # fit_decay recovers a rate it was given.
    t = np.arange(0, 20, 1 / 347.0)
    rate, r2 = fit_decay(t, 4.8 * np.exp(-0.1393 * t))
    check("fit_decay recovers 0.1393 /s", abs(rate - 0.1393) < 1e-6 and r2 > 0.999,
          "got %.5f, r2 %.4f" % (rate, r2))

    # 1. a clean modal kick, DAMPING throughout, is kept.
    q = 0.1185                                       # the measured modal quiet
    p = Protocol(1, out=lambda s: None)
    for r in _trace([(40.0, "DAMPING", lambda s: q),
                     (60.0, "DAMPING",
                      lambda s: max(q, 4.841 * math.exp(-0.1393 * s)))]):
        p.feed(r)
    check("clean modal kick kept", len(p.kept) == 1 and not p.refused,
          "kept %d refused %d" % (len(p.kept), len(p.refused)))
    if p.kept:
        k = p.kept[0]
        check("  quiet measured from the run", abs(k["quiet"] - q) < 1e-6,
              "%.4f" % k["quiet"])
        check("  decay recovered", abs(k["rate"] - 0.1393) < 0.02,
              "%.4f /s, r2 %.3f" % (k["rate"], k["r2"]))

    # 2. the same kick, but the controller faults at the peak -- the 2026-08-18
    #    case, in which even the ONE genuine kick is not a measurement.
    p = Protocol(1, out=lambda s: None)
    for r in _trace([(40.0, "DAMPING", lambda s: q),
                     (2.0, "DAMPING",
                      lambda s: max(q, 4.841 * math.exp(-0.1393 * s))),
                     (60.0, "FAULT", lambda s: 2.253)]):
        p.feed(r)
    check("kick refused when the run leaves DAMPING",
          not p.kept and len(p.refused) == 1
          and any("left DAMPING" in b for b in p.refused[0]["why"]),
          "kept %d refused %d" % (len(p.kept), len(p.refused)))

    # 3. a stalled FAULT baseline wandering across the trigger, exactly as on
    #    2026-08-18: FAULT median 2.253, crossings up to 2.73. Nothing is armed
    #    outside DAMPING, so these are counted and reported, never recorded.
    p = Protocol(3, out=lambda s: None)
    for r in _trace([(40.0, "DAMPING", lambda s: q),
                     (120.0, "FAULT",
                      lambda s: 2.253 + 0.48 * math.sin(2 * math.pi * s / 25.0))]):
        p.feed(r)
    check("faulted baseline recorded as no kick at all",
          not p.kept and not p.refused and p.suppressed > 0,
          "kept %d refused %d suppressed %d"
          % (len(p.kept), len(p.refused), p.suppressed))
    check("  and the FAULT was announced", p.fault_noted > 0,
          "first at t=%.0fs" % p.fault_noted)

    # 4. a baseline excursion across the trigger while still DAMPING -- the
    #    2026-08-18 failure with the fault taken away, so only KICK_RISE can
    #    refuse it. The peak barely exceeds the level it rose from.
    p = Protocol(1, out=lambda s: None)

    def drift(s):
        return (0.15 + 0.09 * s / 8.0 if s <= 8.0
                else max(q, 0.24 - 0.03 * (s - 8.0)))
    for r in _trace([(40.0, "DAMPING", lambda s: q),
                     (40.0, "DAMPING", drift)]):
        p.feed(r)
    check("baseline excursion refused on rise",
          not p.kept and len(p.refused) == 1
          and any("rose from" in b for b in p.refused[0]["why"]),
          "kept %d refused %d%s" % (len(p.kept), len(p.refused),
                                    "" if not p.refused
                                    else " -- " + p.refused[0]["why"][0][:60]))

    # 5. the grader itself, on fits that were printed as results.
    base = dict(left_state=None, left_at=0.0, window_s=10.0,
                peak=4.0, base=1.0)
    check("negative decay refused",
          any("GREW" in b for b in grade(dict(base, rate=-0.0053, r2=0.65))))
    check("r2 0.065 refused",
          any("r2" in b for b in grade(dict(base, rate=0.05, r2=0.065))))
    check("r2 0.53 kept (the weakest ever accepted)",
          not grade(dict(base, rate=0.0184, r2=0.53)))
    check("no re-quiet refused",
          any("did not re-quiet" in b
              for b in grade(dict(base, rate=float("nan"), r2=float("nan")))))
    check("peak 2.73 out of a 2.253 baseline refused",
          any("rose from" in b for b in
              grade(dict(base, peak=2.73, base=2.253, rate=0.05, r2=0.65))))
    check("peak 3.66 out of a 1.58 quiet kept (the weakest real kick)",
          not grade(dict(base, peak=3.66, base=1.58, rate=0.0184, r2=0.69)))
    check("a plausible-looking free-plant fit still refused on state",
          any("FREE PLANT" in b for b in
              grade(dict(base, left_state="FAULT", left_at=2.9, peak=4.67,
                         base=1.22, rate=0.0434, r2=0.782))),
          "r2 0.782 -- no fit-quality test would have caught it")

    print("\n  selftest %s" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("version", nargs="?", help="which controller (%s)"
                    % ", ".join(ladder.LADDER))
    ap.add_argument("--kicks", type=int, default=3)
    ap.add_argument("--port", default=os.environ.get("PORT"))
    ap.add_argument("--quiet", action="store_true", help="print, do not speak")
    ap.add_argument("--replay", metavar="CSV",
                    help="grade a recorded controller CSV offline, no rig")
    ap.add_argument("--selftest", action="store_true",
                    help="check the refusals against the measured populations")
    a = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    if a.selftest:
        return selftest()
    if a.replay:
        replay(a.replay, a.kicks)
        return 0
    if not a.version:
        ap.error("a version is required unless --replay or --selftest is given")
    name = ladder.resolve_name(a.version)
    if name is None:
        redirect, looked = ladder.redirect(a.version)
        sys.exit("  no controller %r.%s" % (
            a.version,
            (" It is now `%s`." % redirect) if redirect else
            (" That version was deleted; see versions.md." if looked else
             " Known: %s." % ", ".join(ladder.LADDER))))
    print("\n  jerk.py -- %s, %d kick(s). NOTHING IS AUTOMATED ABOUT THE KICK:"
          % (name, a.kicks))
    print("  when it says \"jerk it\", shove the table. Ctrl+C stops cleanly.")
    print("  Thresholds come from this run's own quiet, not from a constant.")
    run(name, a.kicks, a.quiet, a.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
