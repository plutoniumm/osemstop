# Controller versions

Every measured number here carries the file it came from. The ladder is
**epsilon, zeta, eta, theta**; older rungs -- including **`delta`, deleted
2026-08-17** -- are in git history and their sections are kept because the hardware
numbers in them are real.

**Controllers were standalone until 2026-08-17, and are not any more.** Every
`osem.*.py` used to import nothing from the others, deliberately: `bench.py` reads
the gain vectors out of the file it is about to run, so what it prints is what gets
applied, and a bench comparison of two versions was a comparison of two things that
both actually ran. On **2026-08-17** the machinery that rule duplicated eight times
was moved into `stdlib.py` and `osem.eta.py` imports it. The check survives because
`stdlib.py` holds **no constants** — every threshold is a constructor argument, so
the gain vectors are still declared in each rung's own file and are still what
`bench.py` prints; where a rung's *law* comes from another module, `bench.py` reads
`GAIN_SOURCE` and names it. Anything below still asserting the old rule is stale; the
rule existed, it was load-bearing, and it was dropped on purpose.

```
python3 harness.py --list          # what exists, controllers and bench tools
make check                         # every version the simulator can drive, asserted
make scope                         # watch all 8 channels live
```

---

## The ladder

| | channels | law | on bench | suite |
|---|---|---|---|---|
| **epsilon** | 8/8 | diagonal + Kalman velocity | **never run** | not re-run since the rename |
| **zeta** | 8/8 | **modal**, Phi/A from `data/modal.json` | **ran 2026-08-17** | 41/10 |
| **eta** | 8/8 | modal + one filter over the modal coordinates | **ran 2026-08-18, and again 2026-08-20** | 41/10, same failure set |
| **theta** | 8/8 | eta + the **actuator budget ramp** (`CLAUDE.md` § 17) | **NEVER RUN** | **never in a suite run** |

`zeta` closed a modal loop on 2026-08-17 and damped **5x faster than diagonal**
(§ *On the bench, 2026-08-17 evening*). **`eta` ran on 2026-08-18** -- one diagonal
null test and one modal run, and it is the first rung in this repo's history to
print `LOCKED` (§ *On the bench, 2026-08-18*). **It ran again on 2026-08-20** --
diagonal, 91.2 s of DAMPING, four hand kicks, **zero faults, and it PUMPED**
(§ *On the bench, 2026-08-20*). **`epsilon` has never been on the rig, and neither
has `theta`.** `delta`, the only rung that ever carried `BENCH_STATUS = validated`,
was deleted 2026-08-17 (`git show b90c983:osem.delta.py`); the diagonal control is
now `zeta`, `eta` or `theta` run with no `data/modal.json`.

**`theta` is `eta` plus the actuator budget ramp** that `CLAUDE.md` § 17 proposed on
2026-08-17: one global actuator budget, reallocated continuously between the modal
term and the per-channel terms as the measured disturbance rises and falls, rather
than a split fixed at build time. **It is BUILT and it is UNMEASURED.** Its
selftests pass **144/144**, it is in no `make check` run, and **no part of it has
been on the rig.** Every open question in § 17 is still open -- above all that **a
time-varying blend of two dissipative laws is not known to be dissipative**, and
that nothing offline catches a law that pumps. 2026-08-20 is the precedent for that
last point: a closed diagonal run pumped on hardware with every offline check
passing.

**`make check` HAS NOT BEEN RUN SINCE 2026-08-17.** The suite counts above predate
the `delta` deletion, the 2026-08-17/18 tool work, the whole of 2026-08-20, and the
addition of `theta`. What has been run on 2026-08-20 is the **per-file selftests**:
`osem.eta.py` **114/114**, `osem.theta.py` **144/144**, `status.py` all pass. A
selftest asserts a planted property against a plant known exactly; **it is not the
suite.**

**Deleted 2026-08-17, git history only:** `zero` (one channel, latching saturation
fault, the original), `alpha` (`auto-disable`), `beta` (`runaway-trend`, and still
the best raw DAMPING fraction on record at **83.1 %**, 2026-08-04). Every fix each
introduced is carried by `delta`. Their sections below stand.

### Names to numbers — read this before searching for a section

**Every section heading below is still numbered**, because that is what it was
called when the measurement was made and renaming a heading would orphan the
numbers under it. So a name has to be translated before it can be searched for:

| name | was | where its measurements are | status |
|---|---|---|---|
| `zero` | **v0** | § *v0, v1, v2 — the deleted rungs* | deleted 08-17 |
| `alpha` | **v4** | § *v4 — graceful degradation* | deleted 08-17 |
| `beta` | **v9** | § *The fault thrash, and the fix (v9)*, § *On the bench, 2026-08-04* | deleted 08-17 |
| `delta` | **v12** | § *v12 — the loop gets its own clock* | deleted 08-17 |
| `epsilon` | — | no numbered section; written after the rename | in the tree |
| `zeta` | — | § *On the bench, 2026-08-17 (evening)* | in the tree |
| `eta` | -- | § *eta*, § *On the bench, 2026-08-18*, § *On the bench, 2026-08-20* | in the tree |
| `theta` | -- | no numbered section; § *On the bench, 2026-08-20* for the session that built it | in the tree, **never run** |
| `sysid` | **v6** | § *Bench tools, not rungs* | in the tree |

`ladder.py`'s `RENAMED` is the authority for what the tools accept, and it is
narrower than this table on purpose: `LADDER = ("epsilon", "zeta", "eta", "theta")`
(`ladder.py:79`) and only **v6 → sysid** still resolves to a file from a number. Every other number --
including v0, v4, v9 and now v12, whose sections are still here — reports "that
version was deleted; see versions.md", which is what this table is for.

Deleted earlier, and their numbers were measured through `harness.py` against the
suite as it stood then, so the counts are not comparable with the table above:

| deleted | channels | I / D | known defects | ch0 ratio | lock | suite then |
|---|---|---|---|---|---|---|
| **v0** | ch0 only | zeroed | all 3 intact | 0.188 | 12.1 s | 18/18 |
| **v1** | all four | zeroed | all 3 intact | 0.025 | 10.7 s | 16/16 |
| **v2** | all four | **live** | all 3 intact | 0.028 | 10.5 s | 18/18 |
| **v4** | all four | live | + auto-disable | 0.029 | 10.6 s | 33/33 |
| **v5** | all four | live | + fast-refault | 0.029 | 10.6 s | 35/35 |
| **v5.5** | **all eight** | live | same as v5 | — | — | *skipped* |
| **v8** | all four | live | + fast-calib, warm-restart | 0.029 | 10.5 s | 36/36 |

v0-v2 went on 2026-08-04; v4, v5, v5.5 and v8 on 2026-08-06, each superseded by a
version that contains it. The `v6.1` / `v6.5` / `v6.6` sysid variants went the same
day - the multisine pair failed on hardware, and the 8-coil pair expects 4 DOF.
v3 was deleted on 08-04, restored on 08-06 as the minimal baseline, and folded away
with the rename.

"ratio" is the rolling 2 s RMS of the bandpassed signal over that channel's own
calibrated baseline - 1.0 is undamped, the lock line is 0.35. Every number in both
tables was measured through `harness.py`, not estimated. The simulator's limits
apply to all of them: see the bottom of this file.

### Bench tools, not rungs

`osem.sysid.py` declares `KIND = "sysid"`, exposes no `Controller` and damps
nothing: it drives the coils with a known excitation and records what the OSEMs do,
to recover the actuation matrix. `harness.py` and `bench.py` both check `KIND` and
refuse to simulate it; the shared lock-in and reporting live in `sysid.py`. The
deleted `v6.1` / `v6.5` / `v6.6` were the multisine and eight-coil variants.

**Three tools are not rungs at all and did the 2026-08-17 work:** `status.py` (the
census, and where Φ and A came from), `signtest.py` (which per-mode sign of A damps)
and `jerk.py` (the kick test — the only tool here that measures a dissipation rate
rather than an amplitude). `signtest.py` was DELETED later the same night: once Φ
is taken from geometry it is exact and signed, so A's sign follows from the driven
measurement and there is no sign pattern left to sweep. Its result table survives
in `CLAUDE.md` and its five records in `data/signtest_20260817_192808_*`.

v4 and v5 give the **same numbers to three decimals** in a quiet lab, on two
different plants — the rewrite is behaviour-preserving, and was verified so
before `fast-refault` was added on top. They diverge only under a *sustained*
disturbance; see § *v5*.

**Re-measured 2026-08-03**, after the simulator moved to the bench sample rate
(347.2 Hz, was 500), started modelling the ADC (per-channel DC offset, integer
counts, hard clip at 0/1023), and gave scripted shocks their own RNG so a shock's
direction no longer depends on the sample rate. Before that pass these rows read
0.197 / 12.0 s, 0.021 / 10.7 s, 0.024 / 10.5 s, 0.024 / 10.6 s, and before *that*
they were measured on a four-independent-oscillator plant and read 0.254 / 13.6 s
onward. Anything in this file quoted per channel and not re-measured (the v1
section's 0.254 / 0.249 / 0.152 / 0.275 and its "ch2 damps fastest") is from that
oldest plant; on the current one v1 reads 0.025 / 0.108 / 0.059 / 0.137 with ch0
flattest, and v0's three undriven channels read 0.333 / 0.316 / 0.253. None of
this is hardware.

---

## v0, v1, v2 — the deleted rungs

**All three deleted 2026-08-04; git history only** (`git show HEAD:osem.v0.py`).
Their measured numbers are in the ladder table above. What is still load-bearing:

- **v0** — the 2026-07-15 bench configuration and what `CLAUDE.md` § *Where the constants came from* describes.
  P-only velocity feedback, `u = -K·v`, `ENABLE_CHANNEL = [True, False, False,
  False]`. The only entry in this whole file ever confirmed against a scope.
- **v1** — one line changed: all four channels on, because `CLAUDE.md` § *One channel damps the whole mass*
  documents all four damping simultaneously (≈17 s against ≈70 s one at a time).
  v0's docstring claim that only ch0 was validated was already two days out of
  date when v0 was frozen.
- **v2** — `KI_GAIN` and `KD_GAIN` given real values for the first time. It did
  **not** damp better than v1, and on hardware it was worse: see § *On the bench,
  2026-08-03*, where v2's integrator pushed a pinned-actuator streak from 28 to
  31 samples against a threshold of 30 and latched a 50 s fault that v1 cleared
  in 5 s.

Two facts from these sections that everything since still depends on, kept here
so deleting the files did not delete the reasoning:

**ch2's `STEADY_GAIN[2] = +0.010` is positive** where the others are negative,
because that channel's coil or OSEM is mounted the other way round
(`CLAUDE.md` § *Why `STEADY_GAIN[2]` is positive*). Confirmed on hardware 2026-08-04, where it came
out the best-damping channel of the four. Do not "correct" it.

**What P, I and D actually are here.** The process variable is *velocity*, not
position, so the usual intuitions are shifted by one derivative: P is viscous
damping and **the only term that removes energy**; I integrates velocity, which
is displacement, so it is a spring that moves the resonance; D differentiates
velocity, which is acceleration, so it is added mass and the noisiest term in
the loop. All three are live from v2 onward — "P-only" describes v0 and v1 only.

---

## v3 — a proper PID, and the three defects fixed (deleted)

**File deleted 2026-08-04; git history only.** Read this section anyway: v4 is
v3 plus one change and v5 is v4 restated as arrays, so everything below is still
what those two do.

Two kinds of change, kept separate so a bench comparison can attribute an
effect to one or the other.

### The control law

A real `PID` class instead of three terms scattered through `Channel.actuate`:

- **derivative on measurement**, not on error. Algebraically identical while the
  setpoint is a constant zero, but it cannot kick if that ever changes.
- **back-calculation anti-windup** (`TRACK_TC_S = 0.5`) instead of a hard clamp
  plus conditional integration. The old scheme froze the accumulator at a limit;
  this bleeds it back at a defined time constant, so recovery from saturation is
  smooth rather than dependent on which side of a conditional the error fell on.
  `I_CLAMP_V` stays as a backstop.
- **one `reset()`**, called on every entry to a non-actuating state, so there is
  exactly one definition of bumpless re-entry.

`Channel.p_term` / `i_term` / `d_term` / `ki` / `kd` survive as properties — the
logger, the status line and the simulator's live gain editing all use those
names, and `ki`/`kd` keep working setters so retuning at runtime still takes
effect rather than silently doing nothing.

### The three defects

**1. `saturation-latch` — fixed, cleanly.**
v0 recomputed `saturated_flag` only inside `check_runaway()`, which never runs in
`FAULT`. A saturation trip set the flag, entered `FAULT`, and nothing could ever
clear it: permanent fault until a manual restart, flatly contradicting the
auto-recovery the module advertises. A *runaway* trip recovered fine, because it
never set the flag — only saturation deadlocked. The flag is now maintained in
`actuate()`, where the output is actually known, so it updates every sample in
every state including `FAULT`, where the output is held at bias and it clears on
its own. Measured: v0/v1/v2 sit in `FAULT` forever; v3 recovers.

**2. `rail-threshold` — fixed, cleanly.**
v0 used 3 counts of 1023 (~14.7 mV) and required 250 *consecutive* samples under
it. One noise sample poking above the line reset the count, so the interlock
silently stopped arming above ~1 count RMS of dark noise — and a blind channel
then reported an amplitude ratio near *zero*, reading as the steadiest axis on
the rack while the loop stayed in `DAMPING` and declared itself locked. v3 puts
the threshold at 12 counts and requires 80% of a 0.5 s window rather than an
unbroken run, so noise dilutes the evidence instead of erasing it. It also
prints once per transition instead of on every railed sample.

| sensor noise | v0/v1/v2 arms after | v3/v4/v5 arms after |
|---|---|---|
| 0 mV | 0.51 s | 0.41 s |
| 5 mV | 0.51 s | 0.41 s |
| 10 mV | **never** | 0.41 s |
| 20 mV | **never** | 0.41 s |

(0.41 s rather than 0.40 s since the simulator moved to the bench's 2.88 ms
sample spacing. `RAIL_SUSTAIN_S` is in *seconds* in every version, so unlike
`MAX_CONSECUTIVE_SATURATED` its meaning does not move with the stream rate.)

**3. `runaway-baseline` — improved, not solved. Read this one.**
v0 measured the baseline as one RMS over a single 8 s window and compared a 2 s
RMS against it for the rest of the run. v3 calibrates for 20 s and takes the
**median of 5 sub-windows**, so one bad sub-window is outvoted.

Measured on the **rigid-body plant** (simulator only, never hardware): one 6 V/s
velocity impulse inside the calibration window, worst-channel baseline skew
against an otherwise identical unshocked run, as a function of *when* it lands:

| shock at | 2 s | 4 s | 6 s | **worst** |
|---|---|---|---|---|
| **v0/v1/v2** | 39.3% | — | — | **39.3%** |
| **v3/v4/v5** | 19.7% | — | — | **19.7%** |

(Re-measured 2026-08-03 over the t = 2/4/6 s sweep the suite actually runs. The
previous full seven-column table — v0 peaking at 45.3%, v3 at 25.8% — was taken
before scripted shocks got their own RNG, so the shock *direction* depended on
the sample rate and the sweep was not a clean counterfactual. The separation is
intact and wider; the absolute numbers moved.)

Read the *spread*, not any one column. On the retired seven-column sweep v0 came
out **better** than v3 at t = 5 s and t = 7 s, so a single fixed shock time can
rank them backwards. What the median actually buys is a **bound** on the damage
wherever the transient lands: over the t = 2/4/6 s sweep the suite runs, v0/v1/v2
reach **39.3%** and v3/v4/v5 stay at **19.7%**. `harness.py` therefore sweeps the
shock across the window and asserts the worst case, with the line at 35% —
~1.1× margin on the un-fixed side, ~1.8× on the fixed side.

These numbers **replace** the 20.5% / 27.0% / 33.2% (v0) and 14.6% / 24.0% /
33.2% (v3) table that stood here before. Those were measured on the old
four-independent-oscillator plant, where a shock hit one channel; on one rigid
body a single impulse moves all four OSEMs at once, so both the magnitudes and
the v0-vs-v3 margin are different. The old numbers are not comparable and should
not be quoted.

The fix still helps against an isolated bump and **does not help** against
sustained disturbance — gain is zero during calibration, so a shock rings down
over `Q/f₀ ≈ 50 s`, longer than the whole window, and once several have landed
there is no clean sub-window left to prefer. Under continuous shocking a 20 s
window simply catches more of them than an 8 s one. Arguably that is correct —
a lab being hit that often really does have that noise floor — but this is a
partial fix and is labelled as one in the code.

A real fix needs a baseline that tracks slowly rather than being sampled once,
or a transient detector that restarts calibration. Neither is in v3, and neither
is in v4 or v5 — they inherit this fix unchanged. `CLAUDE.md` § *Why the calibration window is 20 s* is the
work.

---

## v4 — graceful degradation

**The oldest controller still in the tree.** v3 with **one** behavioural change
and nothing else touched, so a bench comparison isolates it — and since v3 was
deleted on 2026-08-04, that diff is now `git diff HEAD:osem.v3.py osem.v4.py`.
`FIXES` gains `auto-disable`. Identical damping to v3 (ch0 = 0.029, lock 10.6 s)
because in a quiet lab nothing ever demotes — the whole change is dormant until
something goes wrong.

**The problem.** Hazard 3 below: v3 reads `any_rail = any(ch.rail_fault ...)`
and freezes all four channels. One OSEM losing its flag stops *all* damping, at
exactly the moment of largest excursion — which is when the flag left range in
the first place. During a seismic event the loop gives up precisely when the
optic most needs it.

**The change.** A rail demotes that channel alone; global `FAULT` is reserved
for losing the quorum.

```
rail on ch_i  →  ch_i held at bias, ch_j≠i keep damping
global FAULT  →  only when fewer than MIN_HEALTHY_CHANNELS (= 1) remain
```

A quorum of one is not a guess. The source report measured a single OSEM/coil
pair damping the whole rigid body (≈70 s, against ≈17 s for four), and every
version runs four **independent SISO loops** — OSEM *i* drives coil *i* and
reads nothing else — so ch0 does not need ch2's sensor to compute ch0's output.
Three healthy axes are a slower rig, not a broken one.

### What is demoted and what is still global

The distinction is the design, not an implementation detail:

| | | v4 |
|---|---|---|
| **rail** | local *sensing* failure — the measurement is garbage, so anything computed from it is garbage | **demote the channel** |
| **amplitude runaway** | not localisable — four loops on one rigid body, so "amplitude is growing under feedback" does not say which loop is feeding it | global `FAULT`, as v3 |
| **actuator saturation** | local, but the measurement is *fine* and only the output is clipped — v3's back-calculation anti-windup already handles it | global `FAULT`, as v3 |

Saturation is the other obvious candidate and is deliberately **not** demoted: a
saturated channel is still pushing the right way at full authority, so parking
it at bias would remove damping at peak amplitude — the same mistake this
version exists to fix. It also avoids a limit cycle, since a channel demoted for
saturation immediately stops saturating (its output is at bias), re-arms
`REARM_SUSTAIN_S` later and saturates again.

**Calibration is not degraded either.** A rail during `CALIBRATING` is still a
whole-rig fault, because you cannot measure a baseline from a blind sensor. That
is what makes the carried-forward baseline below safe: a demoted channel was
necessarily healthy for the whole calibration window, so it always *has* one.

### Re-arming

A demoted channel returns once its rail has been clear continuously for
`REARM_SUSTAIN_S = 2.0 s`. Three things happen on the way back:

1. **Filters are not reset** — hazard 2. Re-seeding a one-pole injects a step.
   The hold does the work instead: `BP_LOW_HZ = 0.4 Hz` is τ = 0.398 s, so 2.0 s
   is 5 τ. The timer starts on **rail-clear**, not on the demotion, so the
   wash-out happens with the sensor already reading properly.
2. **The baseline is carried forward, not re-measured.** This is the one place
   v4 breaks the ladder's "always measure your own baseline, never borrow one"
   rule, and it is deliberate. Re-measuring means measuring *during* the event
   that caused the demotion, with this channel at zero gain while the others
   damp: an inflated noise floor, which then desensitises this channel's runaway
   breaker (`RUNAWAY_MULTIPLE` is relative to baseline) for the rest of the run.
   If the event was permanent rather than transient, the right recovery is a
   global recalibration — which the `FAULT` path already does.
3. **`active_gain` is zeroed on demotion**, so `GAIN_SLEW_PER_S` ramps it back
   from zero (1.75 s to capture gain). The runaway and schedule RMS windows are
   cleared too — both are short sliding windows that spent the blind period
   accumulating a flatlined bandpass near zero, and left in place that history
   would dilute a genuine excursion for a full `ENVELOPE_WINDOW_S` after
   re-entry, leaving the breaker least sensitive exactly when the channel is
   least trusted. The *filters* are the opposite case, hence (1).

### Cost of the change

Almost nothing, which is the point — v3 had already built the hard part.
`actuate()`'s else-branch was written to define bumpless re-entry for a `FAULT`
or a config-disabled channel, and a demoted channel is the same case, so
re-engagement needed no new code. The one line that puts the demotion into
effect is `if state == "DAMPING" and self.actuating:`.

New state on `Channel`, kept strictly apart from the existing flag:

| | |
|---|---|
| `enabled` | **static config** — `ENABLE_CHANNEL`, or a live edit from the simulator UI. The operator's intent. v4 never writes it. |
| `healthy` | **runtime state** — cleared on rail, restored after the hold. v4 owns it; nothing outside the module touches it. |
| `actuating` | the conjunction, and the only thing that gates output. |

Keeping them separate is what lets `sim/server.py`'s `apply_gains()` keep
assigning `ch.enabled` every frame without silently un-demoting a blind channel,
and lets a status line distinguish `ch1:off` (you turned it off) from
`ch1:DOWN 31s` (the rig did). Demotions are counted in `demote_count`, separate
from `fault_count`, so "an axis dropped out" never inflates the fault count the
other versions are scored on.

The CSV gains a 13th per-channel column, `healthy`. `rail` alone cannot answer
"how many axes were in the loop at time *t*" — a demotion outlasts the rail that
caused it by `REARM_SUSTAIN_S`.

### What v4 does *not* fix

It handles the **fully blind** case, not partial clipping. The bench measured
2.19 % of samples at a rail on v1; `RAIL_FRACTION = 0.80` of a window will not
fire on that, so a partially-clipping channel still reads as healthy and still
feeds flat-topped bandpass and spiking velocity into a live loop. Centring the
OSEM rest points is the fix for that, and it is a separate problem — see *On the
bench* below and README § *Known open problem*.

This is also the `auto-disable` block drawn in the original report's
safety-checks diagram and never implemented in v0–v3 — v4 is the first version
to build it (`CLAUDE.md` § *Two things the source report shows that were never
implemented*; it was ruled out before it was built).

---

## v5 — the same thing as arrays, plus one fix

Two changes, deliberately separable. **First, the shape**: same control law,
constants and state machine as v4; four channels are four lanes of one numpy array instead of four
objects each running identical scalar arithmetic, so every filter, the PID, the
gain schedule and each interlock is one expression rather than four.

The evidence that it *is* the same: every printed observable matches v4 exactly,
on both the old plant and the rebuilt one.

| | v4 | v5 |
|---|---|---|
| ratios (old plant) | 0.024 / 0.107 / 0.051 / 0.139 | identical |
| ratios (current plant) | 0.029 / 0.098 / 0.045 / 0.132 | identical |
| lock | 10.6 s | identical |
| peak P / I / D | 0.0881 / 0.0644 / 0.0412 V | identical |
| re-arm after un-blinding | 2.10 s | identical |

| | total | code | docstring | comments | logic | `main()` |
|---|---|---|---|---|---|---|
| v3 | 908 | 515 | 187 | 123 | 400 | 64 |
| v4 | 1172 | 577 | 308 | 199 | 460 | 64 |
| **v5** | **533** | **356** | **58** | **53** | **270** | **48** |

The weight was in the per-channel class: `Channel` went from **231 lines to 13**,
because it stopped being where state lives and became a *view* onto the
controller's arrays. That view is what keeps `sim/server.py` and `harness.py`
working unchanged — both read and write `ctl.channels[i].<field>`, including live
gain edits from the browser UI, and `__getattr__`/`__setattr__` forward those
straight through to the array. No harness or bench change was needed.

What did **not** shrink is the constants block and `main()`: those are the same
numbers and the same bench I/O, and they are now most of the file.

Two conventions worth knowing before editing it:

- **Timers that may be "not running" are `np.inf`**, so `t - timer >= X` is False
  with no branch. `_hold()` is the one place a per-lane stopwatch starts or
  clears, and both the runaway and lock timers use it.
- **`baseline` is `0.0` while uncalibrated, not `None`**, so it stays an array —
  and still reads falsy everywhere the older code tested `if baseline_rms`.

### The one thing that got worse

Speed, and in the direction you would not guess: **57.5 µs/step against v4's
42.4**. numpy's per-call overhead on a length-4 array is larger than four scalar
operations, so vectorising four lanes buys clarity, not throughput. Both are
~50× inside the 2.88 ms the bench gives you at 348 Hz, so it is irrelevant on
hardware — it only makes the simulator suite slower to run.

### The second change: `fast-refault`

v0–v4 answer *every* fault the same way — wait for all-clear, then re-calibrate.
That is `CALIBRATION_S` of **open loop, gain at zero**. Correct for a transient.
Wrong for a disturbance that has not gone away, because the optic rings straight
back up to the amplitude that faulted it and faults again on contact. The result
is a supervisor-level feedback loop that spends its life measuring instead of
damping.

Found by stressing the rebuilt simulator across clipping regimes, not by reading
the code. Over 200 s at `drive_amp = 2.5` (the ADC clips on every swing):

| | % of samples DAMPING | faults | locks |
|---|---|---|---|
| v0 / v1 / v2 | 0.6% | 1 | never — they *latch*, defect #1 |
| v3 / v4 | 4.1% | 7 | never |
| **v5** | **87.5%** | **1** | **10.2 s** |

And the optic itself, mean true displacement over 300 s (simulator ground truth,
not the controller's own estimate):

| drive | v4 | v5 |
|---|---|---|
| 2.2 | 0.550 V | **0.269 V** |
| 2.5 | 1.409 V | **0.307 V** |
| 3.0 | 1.908 V | **0.451 V** |
| 3.5 | 2.633 V | **0.773 V** |

The fix is small: if the previous engagement lasted less than `FAST_REFAULT_S`
(15 s), the cause is evidently still present, so re-engage on the baseline
already in hand rather than measuring a new one. Gain still ramps from zero
through `GAIN_SLEW_PER_S`, so re-entry is as soft as before. It is bounded by
`MAX_BASELINE_REUSE = 4`, after which it calibrates properly and accepts the
cost — a stale baseline must not be able to persist indefinitely.

This is the same reasoning already accepted for v4's re-arm path (carry the
pre-event baseline rather than measure one *during* the event), applied to the
whole rig instead of one channel.

**What it does not fix.** At `drive_amp = 3.5` — 56% of samples clipping — v5
still exhausts its four reuses and ends up calibrating. That regime is a sensor
problem, not a control problem, and no supervisor policy rescues it.

### v4 or v5?

v4 is the reference: its structure matches the deleted v0–v3, so a diff against
`HEAD:osem.v3.py` shows exactly what auto-disable cost, and it is the
pre-`fast-refault` behaviour. **v5 is the one to run and to modify.**

Both have since run on the bench — 2026-08-04, logs in `bench/20260804/` and CSVs
in `data/`, each reporting `LOCKED`. Those runs are not written up here yet and
`BENCH_STATUS` still reads `untested` in both files.

---

## v5.5 — eight OSEMs, commissioning build

v5 with `N = 8` and the full `DAC_CHANNELS = [1,3,5,7,0,2,4,6]`. **Not a new rung
of the ladder** — same control law, constants, `FIXES` and state machine.

All eight OSEMs are on **one rigid body**, so nothing about the safety semantics
needed rethinking: one pair damping the whole mass still
justifies a quorum of one, a global `FAULT` is still right because there is one
optic, and one lock claim still covers the rig. Going from 4 to 8 was a config
change rather than a rewrite only because v5 keeps channel state in arrays —
`N` is a module constant and everything derives from it.

**The simulator cannot test this file.** `sim/server.py` models one rigid body
with exactly four OSEMs: `GEOM` is four corners and `SENSE` is a 4×4 modal
projection. There is no measured geometry or coil gain for channels 4–7, and
inventing them would mean validating against fabricated physics — so `make check`
**skips** it and says so. It is a bench artefact until the stepped-sine
measurement in `CLAUDE.md` item 2b produces the real numbers; `osem.v6.5.py` and
`osem.v6.6.py` are the tools that take it.

**It has never locked.** Nothing in the ladder table's v5.5 row is measured
because there is nothing here that can measure it.

**Channels 4–7 ship disabled.** Their sign is unknown, and a wrong-signed channel
pumps its mode rather than damping it — ch2 is mounted the other way round and
that was *discovered* from bench data, not predicted. They are still filtered,
rail-checked, calibrated and logged while disabled, so the first run answers
"do the new OSEMs read real motion?" with no coil driven at all. `KI`/`KD` are
zero on them too, since all three gains share one per-channel sign.

They start at half the validated magnitude (`-0.015`). The reasoning is
deliberately hedged: eight coils at the same per-channel gain is ~2× the force of
four, which argues for halving — but the −0.040 instability was measured on v0
with a *single* channel driving, and v1 then ran four at −0.030 without trouble,
which is 4× that total authority. So −0.040 is evidently not a simple
total-authority effect and halving may be over-cautious. Nobody knows which, so
the new channels are conservative and the validated four are untouched.

---

## Adding a version

Copy the version you are starting from, bump `VERSION_TAG`, list what you fixed
in `FIXES`, and the harness picks it up with no other changes -- `--list`
and `make check` both discover `osem.v*.py` by glob.

`FIXES` is load-bearing, not documentation. The suite reads it and **inverts the
corresponding check**: claim `rail-threshold` and you must now arm the interlock
at 20 mV of noise, where v0 was asserted never to. A version that claims a fix it
did not make fails.

A file that is **not** a controller declares `KIND = "sysid"` instead. `harness.py`
reads that out of the source text without importing the module, lists it as a
bench tool and never hands it to the simulator; `bench.py` prints its excitation
parameters rather than gain vectors. That is what the four `osem.v6*.py` are.

To be runnable a controller must expose:

```
ENABLE_CHANNEL, STEADY_GAIN, CAPTURE_GAIN, KI_GAIN, KD_GAIN, BIAS,
DAC_CHANNELS, ADC_MAX_COUNTS, A_VCC, ENVELOPE_WINDOW_S, SlidingRMS,
Controller(dac, enable=, steady=, capture=, ki=, kd=)
```

and `Controller` must offer `.step(counts, volts, t, dt) -> state`,
`.drain_events()`, `.channels`, `.state`, `.lock_time`, `.fault_count`.
`VERSION_TAG` and `FIXES` are optional; absent means "v0-era, nothing fixed".

---

## What the simulator can and cannot tell you

It runs the **real** controller — every four-channel controller above is imported
and stepped, not reimplemented. Only `serial` and `DACController` are stubbed.
`osem.v10.py`, `osem.v11.py` and the `osem.v6*.py` tools are outside its reach
and are skipped rather than approximated -- the first two because they carry eight
OSEMs and this models four, which is also why `soft-saturation` and
`baseline-floor` currently have no coverage here at all.

**The control loop is not stepped at the wire rate.** `SAMPLE_HZ = 347.2` is what
the serial link delivers, but `pyDAC.set_voltage()` discards up to 50 stream
lines waiting for its `OK` ack, so every coil write costs the controller samples
it never sees. Measured 2026-08-03: 2.80 ms median between samples across a whole
run, **41 ms between consecutive `DAMPING` rows** — about 24 Hz, a factor of 14.
The simulator models this with `ack_drain` (samples lost per DAC write),
calibrated to 3.5 so the in-sim loop reads 25 Hz against the bench's 24. Set it
to 0 to model `pyDAC2.FastDAC`, which writes and returns.

It matters more than it sounds: at 24 Hz the sample-counted
`MAX_CONSECUTIVE_SATURATED = 30` means 1.2 s rather than 86 ms, so saturation
essentially stops tripping, and the actuator updates 4x less often. Twenty of the
suite's checks invert under it. `harness.py` therefore pins `ack_drain = 0` in
`BASE_PLANT` -- the suite characterises the control law, not the transport.

The plant, though, is modelled: a linear second-order oscillator per axis, with
exactly one nonlinearity — the ADC, which offsets each channel to its measured
resting count (`DC_REST_COUNTS = [615, 641, 717, 679]` of 1023), rounds to an
integer and hard-clips at 0/1023. That is why clipping now emerges on its own,
and with the bench's asymmetry: every OSEM rests *high*, so the top rail is
reached long before the bottom one, ch2 first. So:

- **It cannot reproduce the instability at Kp = −0.040.** Here more negative Kp
  is always more damping; push it to −0.6 and it just damps harder. Whatever
  turns −0.040 into instability on the real suspension is not in these
  equations. **The sim cannot tell you a safe gain.**
- `COIL_GAIN = [1.00, 0.76, -4.15, 0.86]` — per-channel loop sign and magnitude,
  **inferred** from the per-channel decay fits by subtracting the intrinsic decay
  `ω₀/2Q` from each fitted rate and dividing by that channel's |K|. The report
  characterises no actuator, and the fits behind it have R² = 0.65–0.92, so one
  significant figure is the honest precision. It is in there because assuming
  all four channels identical contradicts the bench outright — it is what makes
  ch2's +0.010 damp rather than pump.
- Q and drive amplitude are sliders because neither was characterised on the
  bench.

Two of the three v3 fixes — now carried by v4, v5 and v5.5 — are still unconfirmed
on hardware; see the bench section below for exactly which, and `CLAUDE.md` item 4
for the runs that would confirm them. They are verified against a model of the
plant and a faithful copy of the controller, which is enough to say the logic is
right and not enough to say the suspension agrees.

---

## Runtime state and hazards

Condensed from `state.md`, which was removed on 2026-08-03 (git history) — most of
it restated what the code already shows. What survives is the state machine and
the hazards.

### The state machine

Three states. One variable, `Controller.state`, a string, living in
`Controller.step()` — driven by `main()` from the serial stream and by
`sim/server.py` from a simulated plant, so the simulator exercises the shipping
code rather than a copy. It gates actuation and is written to every CSV row.

| From | To | Condition | Side effect |
|---|---|---|---|
| — | `CALIBRATING` | process start | `calib_start = now` |
| `CALIBRATING` | `DAMPING` | `now - calib_start ≥ CALIBRATION_S` | `finish_calibration()` all; `damping_start = now` |
| `CALIBRATING` | `FAULT` | `any_rail` | `fault_clear_since = None` |
| `DAMPING` | `FAULT` | `any_rail`, any runaway, or any pinned actuator | `fault_clear_since = None` |
| `FAULT` | `CALIBRATING` | `all_clear` held `FAULT_CLEAR_SUSTAIN_S` (5 s) | `reset_for_recalibration()` all |

- **`CALIBRATING`** — all gains forced to zero, outputs held at `BIAS`, accumulating
  `bp_out²` per channel. `CALIBRATION_S` was 8 s on v0–v2 and is **20 s on v3 onward**,
  so 20 s on every controller in the tree (v4, v5, v5.5). On exit it sets
  `baseline_rms`, the reference *both* the runaway breaker and the lock detector use
  for the rest of the run. Re-measured on every entry, never carried over.
- **`DAMPING`** — the only state that actuates. Per sample: schedule gains → check
  runaway → update lock → actuate. Disabled channels get `active_gain = 0.0` and
  still hold at bias.
- **`FAULT`** — all outputs to `BIAS`, `active_gain` zeroed every sample. **Not
  running here:** `update_schedule`, `check_runaway`, `update_lock`. That omission is
  hazard 1 below.

There is no transition out of `DAMPING` except into `FAULT`, and no terminal state —
`Ctrl+C` leaves through the `finally` block, which returns all four DAC channels to
bias and closes the log.

### Hazards

1. **`saturated_flag` latches (v0/v1/v2; fixed in v3).** Recomputed only inside
   `check_runaway()`, which never runs in `FAULT`. A *saturation* trip therefore pins
   `all_clear` to `False` forever and makes `FAULT` unrecoverable without a manual
   restart. A *runaway* trip recovers normally — only saturation deadlocks.
   **Confirmed on hardware 2026-08-03**, see below. v3 refreshes the flag every
   sample in `actuate()`.
2. **Filter state survives recalibration.** `reset_for_recalibration()` clears the
   PID, the RMS windows and the baseline, but *not* `hp`/`lp`/`deriv_smooth`. That is
   intentional — the filters track a continuous physical signal, and re-seeding them
   would inject a transient at the moment the loop re-arms.
3. **One rail freezes all four channels (v0–v3; fixed in v4).** Not per-axis. The
   stated reason is that all four OSEMs sit on the same rigid body, so one blind
   sensor means the optic's position is untrustworthy globally. That is true for
   *reconstructing the optic's state* — but no version does: they are four
   independent SISO loops, so ch0 never reads ch2's sensor. The effect is that all
   damping stops at the moment of largest excursion, which is what made the flag
   leave range. v4 demotes the blind channel and keeps the rest running, faulting
   only when no healthy channel is left.
4. **The rail trip point sits at the noise floor (v0/v1/v2; fixed in v3).**
   `RAIL_LOW_COUNTS = 3` of 1023 (~14.7 mV), and `check_rail` needs the signal under
   it *continuously* for 250 samples; any sample above resets `rail_since`. Simulated
   with one OSEM occluded: ≤5 mV arms in 0.51 s, 7 mV takes 6.8 s, **≥10 mV never
   fires.** v3 uses 12 counts over 80% of a 0.5 s window.
5. **`baseline_rms` is measured under whatever excitation exists at calibration
   time** — a noise floor, not an intrinsic property. Worse under broadband than
   tonal excitation: a high-Q resonance driven by noise wanders on the ringdown
   timescale (`Q/f₀ ≈ 50 s`), and the runaway breaker compares a 2 s RMS against a
   baseline from one 8 s window, so ordinary excursions read as runaways. v3's median
   over several windows is a partial fix, labelled as one.
6. **Disabled channels still hold full state.** They filter, rail-check, calibrate
   and log — they just never actuate. They can still trip the global interlock. In
   v4 a config-disabled channel is demoted like any other when its sensor rails,
   but it is not counted toward the quorum, so blinding an OSEM that was not
   driving anything no longer faults the rig. It still blocks recovery out of
   `FAULT`, because `all_clear` reads `rail_fault` on every channel — so a
   permanently blind OSEM on an unused axis will hold a fault open. Unchanged from
   v3, and worth knowing before disabling a channel whose sensor is disconnected.

---

## On the bench, 2026-08-03 — measured on v1, v2 and v3

**Those three files were deleted on 2026-08-04 and are in git history only. The
measurements below are not:** they came off the hardware, they are the only
hardware evidence in this file, and they are why v4 and v5 carry the fixes they
do. Nothing here can be re-derived from the simulator.

First hardware runs since v0. Board on `/dev/cu.usbserial-1120` (CH340), firmware
`arduino.ino` streaming 8 ADC columns at ~348 Hz, coils on the remapped
`DAC_CHANNELS = [1, 3, 5, 7]`. The first four runs (155449–160302) drove **ch0
only**; the last five (171100 onward) drove **all four coils** — corrected
2026-08-04 by reading `ch{i}_out` straight out of the logs, where ch1–ch3 have
non-zero standard deviation in exactly those five.

Two things measured from those logs afterwards, both worth knowing before
trusting any timing in this file:

- **The control loop runs at ~24 Hz during DAMPING, not 348 Hz.** Median
  inter-sample time over a whole run is 2.80 ms, matching the wire limit — but
  across consecutive `DAMPING` rows it is 41 ms. `pyDAC.set_voltage` reads and
  DISCARDS up to 50 stream lines waiting for its `OK` ack, so every coil write
  throws away samples the controller never sees. `MAX_CONSECUTIVE_SATURATED = 30`
  is therefore ~1.25 s on the bench, not the 86 ms that 348 Hz implies, and the
  simulator's uniform 347.2 Hz models a loop 14x faster than the real one.
- **The actuation matrix is not recoverable from these logs.** See below.

| | v1 | v2 | v3 |
|---|---|---|---|
| Law | P-only | full PID | full PID |
| Lock time | 13.8 s | 13.9 s | 14.5 s |
| Baseline (ch0) | 0.6316 V | 0.3158 V | 0.1804 V |
| Peak \|bp\| | 3.141 V | 2.837 V | 0.410 V |
| ADC clipped | 2.19% | 10.40% | 0.00% |
| Samples with non-zero I | 0 of 6939 | 4709 of 7513 | all |
| Longest pinned-actuator run | 28 | **31** | 0 |
| Trip type | runaway | **saturation** | none |
| Outcome | recovered in 5.0 s | **latched ~50 s** | no fault |

**Hazard 1 reproduced.** v2 hit a 31-sample pinned run against
`MAX_CONSECUTIVE_SATURATED = 30` and latched `FAULT` for the remaining 50 s of the
run. The evidence it was the stale flag and not a live condition: across 16,966
FAULT samples there is exactly **one distinct ratio value** (3.6803) — nothing is
recomputed at all. v1, on the same kind of kick, tripped the runaway breaker at a
28-sample streak and cleared itself in 5.0 s. Three samples either side of the
threshold is the whole difference between a fault that self-clears and one that
needs a human.

The integrator is what pushed it across: v1 ran `Ki = 0` and never pinned for more
than 28 samples; v2's I term was non-zero in 63% of samples and reached 31. Peak
|I| was 0.084 V against a 0.150 V clamp, so `I_CLAMP_V` never even engaged — bounded
is not the same as unwinding. Not isolated from `Kd`; a clean test would zero one
and keep the other.

**Hazard 4 also reproduced.** v1 logged 246 genuinely clipped ADC samples (2.19%)
and the rail flag never set once, because the clipping is transient and the check
needs 250 *consecutive* samples. Previously this was only ever a simulator result.

**What v3 has and has not shown.** It damps: locked in 14.5 s and held 115 s. It
has faulted and auto-recovered twice (`t=20.137→25.205` and `t=39.258→44.304`, both
~5.05 s). But both were **runaway** trips with pinned runs of 0 and 4 against a
threshold of 30 — v1 recovers from those too. The saturation-latch fix, the one
thing that distinguishes v3 from v2 in failure behaviour, is **still unexercised on
hardware**, as is the rail-threshold fix (v3's runs had 0.00% clipping). Confirming
them needs a kick hard enough to pin the actuator for 30+ consecutive samples.

**Do not compare `ratio` across runs.** It is normalised to each run's own baseline,
and the baselines above differ by 3.5×. Normalising absolute locked RMS by baseline
gives 0.139 / 0.128 / 0.115 for v1/v2/v3 — indistinguishable. These runs do not rank
the three on damping quality.

**Sensor range is the open hardware problem.** ch0 traversed the full 0–1023 ADC
range on the v1 and v2 runs, and the actuator used 100% of its ±0.25 V authority on
transients. A shadow sensor pegged at either end is outside its linear range: the
bandpass sees a flat top, the derivative sees a step, and the velocity estimate
spikes exactly when the loop is asked to push hardest. Worth fixing before any
more control work.

---

## On the bench, 2026-08-04

Board on `/dev/cu.usbserial-1120` (CH340), firmware `arduino.ino` streaming eight
ADC columns at ~348 Hz, coils on the rewired `DAC_CHANNELS = [1, 3, 5, 7]`.
Console logs in `bench/20260804/`, CSVs in `data/` (gitignored).

### Four channels, confirmed

v4's first four-channel run closed the loop on coils 3, 5 and 7 for the first
time since the rewiring, and settled three open questions at once:

- **The coil map `[1, 3, 5, 7]` is correct for all four.** Only ch0 → DAC 1 had
  ever been driven before.
- **ch2's `+0.010` is confirmed on hardware.** It ran positive while the others
  sat at `-0.030` and came out the *best-damping* channel of the four, ratio
  0.01–0.03 against ch0's 0.08 — exactly what the per-channel decay fits predicted,
  and the fact the deleted modal v4 failed to rediscover automatically.
- **Four channels lock faster than one**: 11.4 s against 14.7 s on the same rig
  twenty minutes earlier.

### The fault thrash, and the fix (v9)

The runaway breaker was a **level** test — `env > 1.8 x baseline` sustained 2 s.
A runaway is the loop pumping energy *in*, which means the envelope **growing**;
a large but decaying envelope is the loop succeeding. The level test cannot tell
them apart, so after a kick it trips on a timer all the way down the ringdown.

Measured on v8, 240 s, three kicks — ten faults, envelope falling through every
one of them:

```
re-engage t=71.0:  5.97 -> 5.25 -> 4.61 -> 4.84 -> [FAULT, ratio frozen]
re-engage t=78.2:  3.00 -> 4.29 -> 4.28 -> 3.14 -> [FAULT]
re-engage t=85.3:  2.72 -> 2.48 -> 2.07 -> 1.19 -> [FAULT]
re-engage t=92.3:  1.64 -> 1.37 -> 0.85 -> ... -> 0.21   LOCKED
```

Only the re-engagement that happened to start at 1.64, just under the line,
survived. **This defect was in every version back to v0**; v8 merely re-engaged
fast enough to hit it repeatedly.

v9's `runaway-trend` keeps the level test and adds a growth test: the envelope
must also exceed itself `RUNAWAY_TREND_LAG_S` ago by `RUNAWAY_GROWTH_FRAC`.
Both, sustained. Same three-kick schedule:

Same three-kick schedule. **These are percentages of TIME, not of CSV rows** —
see the warning below, which corrects an earlier version of this table:

| | DAMPING | FAULT | CALIB | faults | locks |
|---|---|---|---|---|---|
| v5 (control) | 81.0% | **2.1%** | 16.8% | 1 | 2 |
| v8 | 70.3% | **21.2%** | 8.4% | **10** | 3 |
| **v9** | **83.1%** | 8.5% | 8.4% | **4** | **4** |
| v10 (08-06) | 74.1% | 15.0% | 10.9% | 8 | 3 |

v9 re-locked after *every* kick (t = 31.3 / 83.8 / 151.3 / 205.3 s). The suite
confirms it still trips on a genuinely pumped resonance, so the growth gate
rejects ringdowns without letting a real runaway through. **v9 is still the best
controller measured on hardware** — v10 gives back some of it, see § *v10*.

> **Do not duty-cycle by counting CSV rows.** This table originally read
> 23.9 / 15.1 / 26.6% DAMPING because it counted rows, and that is wrong by more
> than an order of magnitude. **The sample rate depends on the state**: the loop
> runs at ~24 samples/s while actuating, because `set_voltage` blocks reading its
> ack for every coil, but ~345 samples/s in FAULT and CALIBRATING where no coil is
> written. So a second of DAMPING contributes ~24 rows and a second of FAULT ~345.
> Row-counting inflates FAULT and CALIB about **14x** and makes every version look
> as though it spends its life calibrating. Always weight by `time_s`.

### v8 on hardware: one of two mechanisms transferred

`warm-restart` fired repeatedly and correctly. **`fast-calib` did not** — it
printed *"ran the full 20 s ceiling — the floor never settled"*. `CALIB_AGREE_TOL
= 1.20` was tuned against simulator noise on one RNG seed and the bench floor
does not satisfy it. It degrades safely to v5's estimator, which is the designed
fallback, but the simulator's 20 s → 6 s saving **has not been demonstrated on
hardware**.

### Sensor centering: measured, and mostly not fixable with bias

No OSEM rests at mid-scale. At bias 0.25 V: **600 / 631 / 708 / 677** counts
against 511.5, so every channel clips its *top* rail first.

Coil bias exerts a DC force, so it moves the rest position. Per-coil DC matrix,
one coil stepped at a time, counts per volt:

```
         coil0  coil1  coil2  coil3  coil4  coil5  coil6  coil7
   a0     -105   -107    -48    +19    -29    -21    -28    -26
   a1      -49    -68    -33    +18     +5     +7     +4     +8
   a2      +35    +66   +213   -141    -25    -22    -13    -26
   a3      +36    +14    +58    -51     -1     -2     +2     -1
   a4       +0     +2     +2     +1     +8     +8     +2     +1
   a5       +4     +0     +0     -6     -0     -7     -1     +8
   a6       +1     +4     +2     +3     +3     +3     +4     +6
   a7       -0     +3     +1     +0     +1     -1     +6     +7
```

Two things fall out of that matrix:

1. **Bias cannot centre these sensors.** The least-squares solution wants
   −1.93 … **+3.77 V** of bias change and the DAC has 2.5 V total. Clamped and
   quantised it gets total offset 570 → 320 counts. The rest is mechanical or
   TIA offset.
2. **CORRECTED 2026-08-06 — all eight OSEMs are connected.** This section
   originally read "sensors a4–a7 are not connected", inferred from a DC
   actuation matrix and raw correlation. That was wrong: connection was
   confirmed directly on an oscilloscope, and the logs agree once they are asked
   the right question. The real split is not wired/unwired but **in-band /
   out-of-band**:

   | | senses suspension motion | dominated by |
   |---|---|---|
   | a0–a3 | **yes**, 77–99% of power in 0.4–3 Hz | the 1.01 / 1.66 Hz modes |
   | a5 | **yes**, 22–71% in band | 1.046 Hz, at ~1/12 of a0's gain |
   | a4, a6, a7 | **no**, 0.1–9% in band | a 6.19 Hz interference line |

   a4, a6 and a7 carry plenty of signal — raw std 13–105 counts, comparable to
   a1 and a3 — but **91–99% of it sits at 3–20 Hz**, outside the band the loop
   acts on. A single narrow line at **6.19 Hz** carries 60–87% of their 3–30 Hz
   power, with a **12.38 Hz** second harmonic on a7. a0–a3 show nothing like it
   (top five bins carry 14–26%, i.e. broadband). A sharp line plus a harmonic,
   on exactly the channels that do not sense motion, is interference: most
   likely ~350.9 Hz folded down by the 357.1 Hz sample rate, which is the 7th
   harmonic of ~50.1 Hz mains.

   **This is an alignment problem, not a wiring one.** Those OSEMs are powered
   and reading light but not modulating with pendulum motion — the flag sits
   outside the partial-shadow region where a shadow sensor is linear. a5 is the
   same story one step less severe: inside the linear region but near its edge,
   hence 1/12 the gain.

   **And it explains why v5.5 failed, correctly.** The 0.4–3 Hz bandpass already
   removes the 6.19 Hz line, so those channels calibrated baselines of ~0.002 V.
   Once 91–99% of a channel's content is filtered away, the ratio
   `env / baseline` explodes on whatever residual is left and the global
   interlock trips. Real failure, wrong diagnosis.

   See `analysis/blind_check.py`, `analysis/a5_check.py` and
   `analysis/where_is_power.py`. On a5 specifically: it tracks the
   1.01 Hz suspension mode to within one FFT bin in all three 8-channel logs
   (offsets +0.0000 / +0.0000 / +0.0116 Hz, the last at a 0.0116 Hz bin width),
   with mode amplitude 13.2 / 17.7 / 11.2 counts — larger than a0's own in the
   140508 run. A floating pin cannot peak at another channel's mechanical
   resonance three times running.

   It is a **low-gain** sensor, not an absent one, and two independent
   measurements agree on how low: the DC matrix gives a5 ≤8 counts/V against
   a0's 105 (~13×), and the passive mode amplitudes give 138 vs 13 counts
   (~10.5×). So a5 sees the same motion at roughly **1/12 of a0's
   counts-per-metre** — consistent with a flag sitting near the edge of its
   shadow, which is an alignment problem rather than a wiring one.

   **Why the original call was wrong, because the failure mode will recur.** It
   rested on a DC-response threshold and on raw time-domain correlation against
   a0–a3 (|r| ≤ 0.12). Both are blind to a small coherent signal: 13 counts of
   real motion on a 530-count DC level with drift scores near zero on raw
   correlation, because the variance is dominated by drift and by independent
   noise. **A low-gain sensor and an absent sensor are indistinguishable to
   those two tests and obvious to a spectral one.** Test for a peak at the known
   mechanical resonance, not for correlation amplitude.

Note a3: the common-mode sweep (all four coils together) read **+43** counts/V,
but coil3 → a3 alone is **−51**. Opposite sign. Common-mode slopes are not
per-coil slopes.

### v7's bias trim, on hardware

v7 trims each channel's bias one 0.25 V quantum at a time and keeps the step only
if the *total* offset across all channels improved. Against a v5 control run with
an identical kick schedule:

| | start offset | end offset |
|---|---|---|
| **v7 (trim)** | 565 | **455** counts |
| v5 (control) | 566 | 546 (drift) |

Per channel, v7 start → end: a0 591 → **549**, a1 630 → **603**, a2 712 → **670**,
a3 679 → 679. Lock times 11.2 / 11.0 / 12.4 s — the trim costs nothing.

**Two steps were rejected and frozen**, both by the total-offset test: ch1 → 0.50 V
(486 → 528) and ch3 → 1.25 V (538 → 561). Each helped its own channel and hurt the
rig, which is the four-coils-two-DOF coupling the v6 rank check measured. That
accept/revert is also what made a wrong `SLOPE_SIGN` survivable — see a3 above.

### Actuation matrix

`osem.v6.py` (stepped sine, one coil at a time) **passed its own rank check**:

```
singular values  1.000  0.154  0.058  0.043
directions above 10%: 2      expected 2
```

Four coils drive two DOF, as the geometry says. But only **4 of 12 frequencies
were usable** — ch2 railed in 17 of 48 steps — and of those four, the columns at
0.3797 and 0.4805 Hz are near-identical across coils, i.e. ambient-dominated
rather than drive-dominated. It is a valid measurement on a quarter of the sweep,
not yet a matrix to build a modal controller on.

`osem.v6.1.py` (multisine, all coils at once) **failed**: ch2 railed in all four
passes, so `RANK CHECK` reported *"no frequency had all channels clean — nothing
to check."* Its pass 1 also ran at 14.8 Hz effective against ~252 Hz for the
others, which is why that pass's bins report ~4700 V/V against ~70 elsewhere —
an aliasing artifact, not physics.

Both failures share one cause: **open loop, nothing damping, so the optic rings
up and a2 clips.** Closed-loop dither — the loop holding the optic in range while
a known signal is injected on top — is the way past it, and `sysid.run()` already
divides by the actually-commanded `u` rather than the intended amplitude, which is
most of what that needs.

### Transport defects found and fixed

- **`DACController` demanded `READY` on the first line.** Opening the port resets
  the board via DTR, but data sent *before* that reset is still buffered by the
  OS and arrives first — so a board left streaming made a healthy rig fail
  preflight with "did not send READY". Now scans up to 200 lines for it, the same
  bounded-scan shape `set_voltage` already used, then issues `STOP` to land in a
  known state.
- **`sysid.Recorder` wrote 14 fields against a 21-field header** on the stepped
  tools. `stepped()` hands `acquire()` a single coil, so `u` arrived length 1
  while the header had one column per coil, shifting every sensor column left by
  seven on read-back. It now scatters the driven value into its own slot and
  holds the rest at bias. Multisine was never affected. The `v6_stepped4` and
  `v65_stepped8` raw files recorded before this fix are misaligned; the in-run
  reports are not, since the lock-in runs off in-memory arrays.

## v10 — the merge, all eight OSEMs, and clipping stops being a fault

Written 2026-08-06. **`BENCH_STATUS = validated`** — ran on the bench the same
day on all eight channels. v9 plus v7's `bias-trim` (unchanged in behaviour),
plus `soft-saturation`, plus `baseline-floor`, plus the channel set.

It reached **40/40** in the simulator as a four-channel build, before the channel
set grew; it is skipped now (see § *The ladder*).

### `baseline-floor` — the guard for a sensor that never rails

Every test downstream of calibration is a **ratio against the baseline**: the
gain schedule (`env / baseline`), the lock detector (`< 0.35 × baseline`) and the
runaway breaker (`> 1.8 × baseline`). A channel that senses no motion calibrates
a near-zero baseline and all three then divide by it — so noise reads as a
runaway and the breaker faults the **whole rig**. `auto-disable` cannot catch it:
that keys on a channel *railing*, and a signal-less OSEM does not rail. It sits
mid-scale and flat, which an RMS interlock reads as perfect stability.

Measured, v5.5, `bench/20260804/v55_all8.log`: a4/a6/a7 calibrated
0.0014 / 0.0050 / 0.0020 V against ch0's 0.3109 V; the rig faulted **ten times in
145 s**, nine of them naming a4, a6 or a7, while ch0–ch3 damped at ratio
0.31–0.39. At the first fault a4 reported ratio **2.73 on a bandpassed signal of
+0.000 V**.

**The rule.** At the end of every calibration, a channel whose baseline is under
`BASELINE_FLOOR_FRAC = 0.10` of the **median baseline across the enabled
channels** is demoted exactly as `auto-disable` demotes a railed one — held at
bias, out of the interlocks, the rest keep damping. Relative, not a level in
volts, because the floor scales with the lab *and* with alignment (counts-per-
metre is where the flag sits in its shadow — a5 is at ~1/12 of a0's). The median
buys a safety property outright: the largest enabled baseline is ≥ the median >
the threshold, so **at least one enabled channel always survives**, with no
quorum arithmetic. And if *most* of the rig is signal-less the median is itself a
dead channel, the threshold collapses, and the guard does nothing — deliberate,
because a reference that has been captured cannot tell which half is broken.

Bench numbers, 2026-08-06, `data/20260806_161715_fast_lock.csv`, as % of the
median over the five enabled channels:

| | a0 | a1 | a2 | a3 | a5 | a4 | a6 | a7 |
|---|---|---|---|---|---|---|---|---|
| baseline, V | 0.3104 | 0.1745 | 0.4436 | 0.1559 | 0.0896 | 0.0052 | 0.0090 | 0.0022 |
| % of median | 178 | 100 | 254 | 89 | **51** | **3** | **5** | **1** |
| verdict | keep | keep | keep | keep | keep | DEMOTE | DEMOTE | DEMOTE |

The 10% line sits in a **10× empty gap** — 5.1% (worst dead) to 51.4% (a5) — with
2.0× and 5.1× of margin, and it is biased toward *keeping*: demoting a good
sensor costs one loop, keeping a dead one costs the whole rig.

### v10 on the bench, 2026-08-06 — eight channels, 60 s, no kicks

**Zero faults.** 20 s of calibration then 40 s of unbroken `DAMPING`, against
v5.5's ten faults on the same eight channels.

| bandpassed RMS, V | a0 | a1 | a2 | a3 | a5 | a4 | a6 | a7 |
|---|---|---|---|---|---|---|---|---|
| first 5 s of DAMPING | 0.1714 | 0.0986 | 0.2171 | 0.0672 | 0.0860 | 0.0149 | 0.0140 | 0.0046 |
| last 5 s | 0.0651 | 0.0235 | 0.0179 | 0.0113 | **0.0420** | 0.0152 | 0.0144 | 0.0040 |

a4/a6/a7 are **flat to three decimals** through 40 s that took a factor of 3–12
out of every real channel. They are not watching this optic.

**a5's sign prediction was right.** `STEADY_GAIN[5] = -0.015` came from a5's own
diagonal in the per-coil DC matrix, −7 counts/V — under 2× the noise of the block
it sits in, and a wrong sign *pumps*. It damped, 0.0860 → 0.0420 V. The magnitude
can be raised toward −0.030 next.

**What it cost: the rig did not announce LOCKED.** a5 sat at ratio 0.45 against
the 0.35 lock line, so its `locked` flag never set. Not a fault and not something
to "fix" in the file: `ratio` is normalised by each channel's *own* baseline, and
a5's is ~1/12 of a0's, so the same absolute motion reads larger there. a5's
absolute envelope is comparable to ch0's and falling. Realigning a5's flag fixes
it at the source; lowering `LOCK_RMS_FACTOR` would weaken the claim for all eight.

### A rejected design worth recording — an absolute floor in volts

The first shape of this guard was `baseline < SOME_MV`. It cannot work, and the
reason is the same one that makes the whole problem interesting: the number it
would have to be depends on the seismic background *and* on each OSEM's
alignment. a5 calibrates 0.0896 V and is real; a6 calibrates 0.0090 V and is not;
on a quieter night both fall together. Any constant is wrong the first time
either changes, and the failure is silent in the dangerous direction.

### `soft-saturation`

v9 faulted the whole rig after `MAX_CONSECUTIVE_SATURATED = 30` consecutive
samples with an output pinned against its rail. But clipping removes authority in
**one direction only** — an output pinned at `vmax` still pulls down at full
strength — so a clipped loop is a weakened loop, not a broken one, and it is
usually the thing bringing the optic back. The fault now also requires the
envelope **not to be falling**. Same level-vs-trend correction `runaway-trend`
made to the runaway breaker, one interlock over.

Measured on the suite's own over-gain scenario (ch0 at Kp = −0.600, one kick,
90 s). Both versions clip; only the verdict differs:

| | faults | clipped | ch0 ratio | peak | % DAMPING | lock |
|---|---|---|---|---|---|---|
| v9 | 1 | yes | 0.090 | **1.527** | 87.8% | 11.6 s |
| **v10** | **0** | yes | 0.086 | **0.853** | **93.3%** | 14.8 s |

Freezing a winning actuator let the optic ring **1.8× higher**. Note this is a
simulator result and the simulator is linear apart from the ADC, so −0.600 there
is aggressive damping rather than an instability — which is exactly why v9's trip
was a false positive.

The interlock is not merely switched off, and the suite asserts both halves on
two different runs: the wrong-signed-ch2 scenario also clips, but leaves the
optic near full amplitude, so it is clipped-and-*not*-winning and still faults
(4 faults, ch2 peak 1.99).

### A rejected design worth recording

The first draft also gated the fault on `|clip_excess| > SAT_EXCESS_V`, reading
that as "still demanding more than the rail can give". It is wrong for a
structural reason: **back-calculation anti-windup exists precisely to drive that
residual toward zero**, so under sustained clipping it vanishes exactly when
saturation is worst. The suite caught it as *"over-gain plus a disturbance trips
a fault — 0 fault(s)"*: a pumped loop nothing stopped. An anti-windup residual
cannot measure unmet demand.

### Merging the trim was not a paste

A bias step is a force step: it rings the pendulum at ~1 Hz, inside
`BP_LOW_HZ..BP_HIGH_HZ` — the band both *trend* tests read. Suspending the
interlocks for a ringdown would open a ~16 s hole in the runaway breaker every
`TRIM_PERIOD_S = 15 s`, i.e. open nearly always. Instead a step **invalidates the
history** it would corrupt: `env_hist` cleared, `excess_since` and `sat_streak`
reset. The level halves keep working throughout; the trend halves resume one lag
later.

The **baseline is deliberately not invalidated**, though the first draft did that
too and it cost four tests. The baseline is a *bandpassed* RMS floor and a bias
step is DC, so the bandpass removes it. The second-order effect is real — an OSEM
is a shadow sensor, so moving the flag changes counts-per-metre — but a quantum
is ~25–50 counts of 1023 against thresholds like `RUNAWAY_MULTIPLE = 1.8`.
Refusing reuse on every trim step just disabled `warm-restart` outright.

---

## v11 — all eight, and the transport stops throttling the loop

Written 2026-08-06, **`BENCH_STATUS = untested`**, skipped by the simulator.
v10 plus three changes that are all about one thing: an eight-channel rig runs a
loop a four-channel rig does not. No gain and no threshold is retuned.

**REQUIRES A REFLASH — `make arduino`.** The board must be at 230400 baud before
v11 runs, and a host at the wrong baud fails preflight with "never sent READY".
That is the intended failure; there is no version negotiation on this link.

### 1. `sat-window` — the saturation interlock was unreachable

`MAX_CONSECUTIVE_SATURATED = 30` counted **consecutive iterations**, and both
halves of that are wrong:

- *Samples, not seconds.* The controller only iterates on stream samples it
  actually sees, and every DAC write eats some, so one "sample" is not a fixed
  amount of time. Measured (`analysis/out/loop_rate.csv`, and confirmed on v10's
  own 08-06 run): **73.0 Hz** driving 1 coil, **23.5 Hz** on 4, **18.2 Hz** on 5,
  **12.5 Hz** on 8. So 30 samples meant 0.41 s, 1.28 s, 1.65 s and 2.40 s — the
  interlock got *less* sensitive exactly as the rig got harder to control.
- *An unbroken run.* One clean sample — one count of ADC noise, one slew step
  landing a microvolt inside the rail — erased all accumulated evidence.

Together, that is why **`soft-saturation` never fired once** in v10's 240 s
three-kick bench run of 2026-08-06: all eight faults were the runaway breaker.

The fix is the shape the *rail* interlock has always had — a fraction of a window
in seconds, `_sat_check` being `_rail_check` line for line, "so one noise sample
dilutes the evidence instead of erasing it". `SAT_SUSTAIN_S = 1.30 s` is chosen
to be **behaviour-preserving at the four-coil rate** (30 × 42.6 ms = 1.278 s), so
this is a bug fix rather than a retune; `SAT_FRACTION = 0.80` is the rail check's
own number. It goes **first** because the other two changes both move the sample
rate, and `CLAUDE.md` item 2c says not to move it under a sample-counted
threshold.

### 2. `fast-transport` — `pyDAC2.FastDAC` instead of `DACController`

`DACController.set_voltage()` writes `SET` then reads up to 50 lines waiting for
the board's `OK`, and every one is a stream sample the controller never sees.
Measured on v10's 08-06 run, one number: **357.1 Hz while CALIBRATING** (no coil
writes) against **18.2 Hz while DAMPING** (5 coils). A factor of 19.6, and it is
a function of how many coils you drive — a strange thing to be true of a control
system. At the 8-coil 12.5 Hz the phase margin at 3.75 Hz is **45.5°** against
**73.5°** at four coils (`analysis/out/phase_budget.csv`), and the −0.040 rail
onset is still uncharacterised, so there is no measured margin to spend.

**Taken, and the ack was worth giving up.** `ERR` comes back for exactly two
things — a malformed command and a channel outside 0–7 — and `FastDAC` rejects
both locally before writing; the firmware clamps voltage itself; and
`Actuator.send()` has *always* treated the ack as optional, catching the
RuntimeError and moving on because "the next sample resends". Nothing in any
controller has ever branched on an `OK`. What it costs is real and handled: the
board still *sends* those replies and they now arrive interleaved with the data
rows, so `read_sample()` filters on the first character. The fallback is two
lines — swap the import and `main()`'s constructor back — and that everything
else works unmodified against either transport is itself the evidence.

### 3. Baud 230400

Eight columns is ~33 bytes, and 33 bytes at 115200 (10 bits/byte) is **2.86 ms
against the 2.88 ms actually measured** between samples: the link was *at*
capacity with nothing left for the `SET` commands sharing it, while 8 coils
throttled at 10 ms want ~12 kB/s against a total of 11.5 kB/s. At 230400 a row
costs 1.43 ms and the budget doubles. One line in `arduino.ino`, and the default
in `pyDAC.py` and `pyDAC2.py` — all three must agree, so **every** controller
moves with the board.

`pyDAC2.FastDAC` also picked up the bounded 200-line `READY` scan and the `STOP`
that `DACController` already had, because a controller re-opens the port right
after a preflight that just left the board streaming — the exact case the
demand-READY-on-line-1 version fails.

### All eight enabled, and what that means

`ENABLE_CHANNEL` is all eight. a4/a6/a7 ship at **zero gain**: they join as
*sensors* — rail interlock, runaway breaker, quorum, lock claim, CSV — and
contribute no force. Their own diagonals in the DC matrix are +8 / +4 / +7
counts/V and the mean |response| anywhere in that block is **4 counts**, so those
signs are coin flips dressed as measurements, and a wrong-signed channel pumps.
`osem.v6.5.py` / `v6.6.py` are the measurement that turns them into numbers.

**Expect three demoted channels on the first run.** a4/a6/a7 carry no in-band
signal, so `baseline-floor` will demote them at the end of calibration, by name
and with the numbers printed. That is the designed outcome, not a failure — and
it is what makes `ENABLE_CHANNEL` stop being a constant somebody maintains and
start being a measurement the rig makes each time it calibrates. Realign a4's
flag and it joins on the next calibration with no edit to the file.

One thinner margin to know about: with all eight in the median, three signal-less
channels pull the reference *down*, so on the 08-06 baselines the worst dead
channel sits at 7.3% of the median against v10's 5.1% — 1.4× under the 10% line
where v10 had 2.0×. Adequate on the measured numbers, and the reason the
reference is the *enabled* set rather than a global one.

---

### Still open

1. ~~**Realign a4, a6 and a7.** They are connected and reading; their flags sit
   outside the partial shadow, so they modulate with nothing (0.1–9% of power in
   band, lock-in SNR 1.1–1.5). `baseline-floor` keeps them from taking the rig
   down, but only a screwdriver makes them useful. a5 is the same problem one
   step less severe — at ~1/12 gain it damps, but it also holds the rig out of
   `LOCKED` at ratio 0.45.~~
   **SUPERSEDED twice.** The "flags outside the partial shadow" premise was
   refuted on 2026-08-06 by the resting counts (§ *Resting counts, 2026-08-06*),
   and the whole item was superseded on **2026-08-17**: a4 grades GOOD at SNR
   107.2 and is the **strongest** channel on the rig; a5 reads a hard 0.0 counts
   with variance exactly zero; a6/a7 are **bottom-railed** and are **vertical**
   sensors, so an in-band grade was never the right test for them. See
   § *On the bench, 2026-08-17*, and `CLAUDE.md` § *Hardware requests*.
2. **`soft-saturation` and `baseline-floor` have no simulator coverage.** Both
   are declared only by v10 and v11, which are eight-channel and therefore
   skipped, so the suite now only asserts the *un-fixed* side of them on
   v3/v7/v9. The checks are still in `harness.py` and both fixes have bench
   evidence behind them (`baseline-floor` on 08-06, `soft-saturation` none — it
   has never fired on hardware, see `sat-window`), but nothing off the bench
   would catch a regression in either.
3. **v9's remaining 36% in FAULT** is `FAULT_CLEAR_SUSTAIN_S = 5 s` x 4 recoveries
   plus the gain ramp — no longer thrash. Gating re-engagement on amplitude rather
   than a fixed 5 s would cut it further.
4. **Bench-tune `CALIB_AGREE_TOL`** from the nine logs already recorded.
5. **v10 has never run on hardware.** Both of its mechanisms are simulator-only,
   and the simulator models clipping but not the coil driver behind it.

---

## v12 — the loop gets its own clock, and the wire stops being trusted

The shipping controller as of 2026-08-06. `BENCH_STATUS = "validated"`. Five
changes, none of them in the control law: `STEADY_GAIN`, `KI_GAIN` and `KD_GAIN`
on a0–a3 are v3's to the last digit, ch2's opposite sign included.

1. **`sample-guard`.** `read_sample()` accepted any row starting with a digit that
   split into ≥ N fields, so a **torn serial line passed as data**. Harmless while
   the ack was being drained; `fast-transport` stopped draining it, so `OK ch=..`
   replies now interleave with data rows and a buffer boundary splices two fields'
   digits together. v11's own 240 s log (`data/20260806_171500_fast_lock.csv`)
   carries **ten rows with a count outside 0..1023** — 5659, 65690, 522676 — and
   **zero** such rows appear in any of the four slower logs from the same session.
   The speed-up created the defect. One of them costs +45.6 V into the bandpass,
   ringing down over the 0.4 Hz highpass (τ = 0.398 s) with the velocity estimate
   peaking at **1401 V/s**; P = 0.030 × 1401 = 42 V demanded into a 0.5 V rail.
   Four of v11's twelve entries to FAULT follow a torn line within 2 s.
   Fix: a count outside `0..ADC_MAX_COUNTS` is a framing error, dropped like an
   `OK` line. Replayed offline on the same log, ch0's velocity estimate goes
   rms **13.109 → 5.413 V/s**, peak **1441 → 41 V/s**.
2. **`decimate`.** Read the wire at full rate; run the control step at a fixed
   `CONTROL_HZ = 100` on the mean of whatever arrived since the last one. The
   arrival rate moved four times in three days (24 → 12.5 → 18 → 1111 Hz) and
   every gain, filter and interlock here was tuned against one of them. Replayed
   with the guard: rms **5.413 → 4.732 V/s**, p99 27.0 → 23.7. **Without** the
   guard it makes things worse (13.1 → 23.4) because a boxcar spreads one bad
   sample across a whole step — which is why the two ship together.
   Measured on the bench: `1024–1113 Hz` wire, `avg10–avg12` samples per step.
3. **`persist-baseline`.** The noise floor is a property of the room, not of the
   run, so it is written to `data/baseline.json` and reused across restarts,
   behind a config fingerprint, an age limit and a start-up sanity check.
   Cold-start calibration 20 s → `BASELINE_WARMUP_S = 2.0` s.
4. **`baseline-sanity`.** Persistence needs a way to refuse a bad floor, because
   stationarity is not quietness: a calibration taken while the optic is ringing
   measures the ringing. Caught on hardware — a fresh floor **11.5×** the trusted
   one was refused and the trusted one kept. `MAX_BASELINE_REFUSALS = 3` stops it
   refusing forever if the room genuinely got louder.
5. **`RUNAWAY_TREND_LAG_S` 2.0 → 10.0** — see below. This is the change that made
   v12 damp.

### The runaway breaker was reading the beat, not the envelope

`runaway-trend` (v9) tests whether the envelope is *growing*:
`env > env(t − LAG) × 1.02`. `LAG = 2.0 s` was chosen before the mode
frequencies were known. They are now measured to ±0.0001 Hz — **0.7155 /
0.9949 / 1.6396 Hz** (`analysis/ringdown.md`) — and the beat periods between
them are **3.578 s (A–B), 1.551 s (B–C), 1.082 s (A–C)**. A 2 s lag sits inside
all three, so the test was sampling the beat and calling it growth.

The other half: a genuine ringdown at the measured τ = 56–130 s falls only
**1.5–3.6 % in 2 s**, under the 2 % threshold. So the breaker could not tell a
real decay from a beat.

`LAG = 10.0 s` is 2.8× the longest beat and 5.6–13× shorter than τ. Measured
back-to-back on the bench the same evening, same gains, hand kicks:

| run | DAMPING | FAULT | CALIB | FAULT entries |
|---|---|---|---|---|
| `20260806_200158`, LAG = 2.0  | 54.1 % | 25.2 % | 20.7 % | 10 |
| `20260806_200822`, LAG = 10.0 | **77.5 %** | 14.4 % | 8.2 % | 7 |

The remaining 14.4 % FAULT is **not** thrash: 7 entries × `FAULT_CLEAR_SUSTAIN_S
= 5 s` = 35 s = 14.3 % of 244.5 s. It is the mandatory hold, not the breaker
re-tripping. `analysis/out/plots/v12_fixed_ch03.png` shows each kick decaying
inside the damping region instead of the near-solid fault bands of the run
before it.

For context, the same time-weighted measure on the earlier rungs
(240 s, three kicks):

| | DAMPING | FAULT | CALIB | faults |
|---|---|---|---|---|
| v9  (4ch, 23 Hz)   | 83.1 % |  8.5 % |  8.4 % |  4 |
| v10 (8ch, 18 Hz)   | 74.1 % | 15.0 % | 10.9 % |  8 |
| v11 (8ch, 1111 Hz) | 42.3 % | 25.3 % | 32.4 % | 11 |
| v12 (8ch, 100 Hz)  | 77.5 % | 14.4 % |  8.2 % |  7 |

**Do not read v12 < v9 off that table.** v9's three kicks were scripted; v12's
five were by hand at unrecorded strength, and they were hard enough to clip every
channel. The comparison that is controlled is v12-vs-v11 and v12-vs-itself across
the lag change, both above.

**All four channels still clip**, which is the open problem underneath every
number here. Counts at or beyond the rail (≤1 or ≥1022) over the 251 280 samples
of `20260806_200822`:

| | ch0 | ch1 | ch2 | ch3 |
|---|---|---|---|---|
| railed samples | 7386 | 1958 | **9025** | 2327 |
| fraction | 2.9 % | 0.8 % | **3.6 %** | 0.9 % |

The `rail` interlock fired on only ch0 (136 samples) and ch2 (51), because
`RAIL_SUSTAIN_S` demands the pin be *sustained* — so 97 % of this clipping is
invisible to the interlocks and shows up only as a corrupted velocity estimate at
exactly the moment the loop is pushing hardest. ch2 is worst, as it has been
since 08-03. This is the sensor-range problem in README § 5, not a v12 defect.

### v12 never announced `LOCKED`, and neither did v10 or v11

The one that matters, because README § 4 says the lock time is the deliverable.
Zero `*** LOCKED ***` lines in **every** run of the 2026-08-06 session.

**Fixed by 2026-08-18**: `eta` printed it, DEGRADED, 4/7 healthy, 20.7 s after gain
(§ *On the bench, 2026-08-18*). The diagnosis below is what got there.

The cause is the lock quorum, not the damping. `self.locked[live].all()` where
`live = enabled & healthy`, and `ENABLE_CHANNEL` is `True` on all eight. a4, a6
and a7 demote themselves to `NOSIG` and drop out of `live`. **a5 does not** — it
stays healthy, so it is in the quorum — but `STEADY_GAIN[5] = 0`, so nothing
drives it, and it sits at ratio **1.26–2.53** against `LOCK_RMS_FACTOR = 0.35`.
One undriven channel vetoes the announcement for the whole rig, permanently.

a0–a3 *do* reach lock together: the CSV's four `ch{i}_locked` flags are
simultaneously 1 for **6 distinct episodes** in `20260806_200822`, and **3** in
the pre-fix run. The rig locks; it just cannot say so.

This is a v13 fix, not a v12 one, and it is one line either way — drop
zero-gain channels from the quorum, or set `ENABLE_CHANNEL[5] = False`. It is
listed first under *Still open* because it is the deliverable.

### Resting counts, 2026-08-06 (`CALIBRATING`, 22 246 samples, outputs at bias)

| | ch0 | ch1 | ch2 | ch3 | ch4 | ch5 | ch6 | ch7 |
|---|---|---|---|---|---|---|---|---|
| mean | 601.2 | 630.3 | 686.5 | 681.8 | 567.5 | 554.8 | 860.6 | 734.7 |
| min–max | 443–807 | 559–698 | 511–894 | 627–741 | 552–582 | 476–652 | 85–875 | 557–740 |

Two things to take from it. Every channel still rests **above** mid-scale
(511.5) and clips the top rail first, unchanged since 08-03. And **a4 rests at
567.5 with a 30-count swing** — near mid-scale, not pinned — which does *not*
support the "flags outside the partial shadow" story told earlier in this file.
What is established is that a4/a6/a7 have very low in-band gain and only show
suspension motion on a hard kick (confirmed on an oscilloscope 08-06); *why*
is not established, and the resting points argue against gross misalignment.

**Superseded 2026-08-17.** a4 is now the strongest channel on the rig and a5/a6/a7
have moved from mid-scale to the **bottom** rail (0.0 / 4.3–7.4 / 8.8–13.2
counts). The eight-channel comparison table is in
§ *On the bench, 2026-08-17* § 4.

### MIMO is closed, by measurement — SUPERSEDED on the sensor side, 2026-08-17

**Read this first.** Every measurement below stands for the data it was taken on,
and the reasoning was correct for that data. What changed on **2026-08-17** is
that **a1 grades GOOD** (SNR 68.2 / 48.3 / 54.0) and **five** sensors carry all
three modes, so Φ need not come out square and the sensor-side blocker is
cleared — while **the Φ gate has still never run on a valid record and A is
unmeasured**. Full numbers in § *On the bench, 2026-08-17*; the priority queue is
`CLAUDE.md`. Two further corrections: **square was never a property of the
geometry**, only of having determined three rows (the reported layout predicts a
non-empty null space), and **the 8-coil matrix below was measured at frequencies
that have since moved**, so its phases are wrong by up to 84°.

Recorded here so it is not re-attempted. A full fixed-frequency refit of the
8-coil dither run (`analysis/mimo_design.md`, `analysis/refit_fixed.py`):

- Only **three** sensors have determined residue rows — a0, a2, a3. a1 never
  reaches the 6 usable tones it needs (noise floor **0.024 V** against a0's
  **0.0047 V**). With three modes, Φ comes out **square**.
- A square Φ kills the one argument that survived everything else: drop any
  single sensor and it can no longer span the modes. It also deletes the
  sensor-disagreement check, since a square Φ reproduces any reading exactly —
  measured `resid = 1.8e-16`. And it would fall back exactly when needed: a hard
  kick railed all three of those sensors.
- **It is a sensor problem, not a coil problem.** The allocator is fine — A is
  3×4, cond **1.41**, singular values 1.000 / 0.885 / 0.711. Coils 0–3 drive all
  three modes with a spare direction.
- Coils 4–7 are separately useless, for an unrelated reason: pairwise cosine
  **+0.966**, one shared direction at ¼–⅐ the strength of coils 0–3
  (`analysis/coil_qual.py`). An anti-phase discriminator was run to separate
  "one physical winding" from "four coils on one DOF" and was **inconclusive** —
  ratio 0.579, shape cosine −0.488, with coils 4/6 returning 0.2–0.9 counts/V
  against the control coil's 44 (`analysis/antiphase.py`).

Unblocking it, if ever revisited, means determining a1's row: 6 usable tones,
currently 2–5. More dwell or more drive on the tones a1 responds to. A bench
session, not a hardware change.

### Still open after v12

1. **`LOCKED` is never announced** — a5 vetoes the quorum, above. One line.
2. **Diagonal reallocation is closed. It was never available.** Recorded as a
   correction: this file previously claimed ratios `[1.00, 0.83, −3.82, 0.90]`
   worth 1.6–1.8×. Those numbers are in no measurement file. `gains.json`'s
   optimiser proposes **flat ±0.035**, which is what `delta` already ships.
   It is structurally impossible, not merely unmeasured: `observable_dof = 2`
   against `free_gains_per_term = 4`, so a **two-dimensional family of gain
   vectors damps identically** and the proposal picks its point *"by minimum
   actuator effort, not by measurement"*. Flat already reaches 90 % of the
   achievable maximum at the ceiling (0.0739 /s worst mode).
   The 1.36× that does exist (added damping 0.344 → 0.468 /s) is the
   −0.030 → −0.035 change plus the Ki/Kd resizing, **already shipping**. Its
   real result is budget: peak actuator 0.2998 → **0.200 V** against a 0.250 V
   half-window, with Ki's peak going 0.1135 → 0.0056 V.
3. **Kalman velocity estimator** — now unblocked. Frequencies known to ±0.0001 Hz;
   Q no longer matters, because τ > 138 s intrinsic against a 2.9–4.7 s closed
   loop means the plant is effectively undamped on control timescales.
   Per-channel measurement noise is measured, which is the R it needs.
4. **`LOCK_SUSTAIN_S = 5.0`** is five of the ten-second lock budget spent
   confirming a lock that already happened. Deliberately not cut in v12 so the
   decimation result stayed readable. Cut it next, on its own.
5. **`FAULT_CLEAR_SUSTAIN_S = 5 s` is now the whole fault cost** — 35 of v12's
   35.2 fault-seconds. Gating re-engagement on amplitude rather than a fixed 5 s
   is the next real gain.
6. **The suite: 220 passed, 11 failed** (679 s). `sim/server.py` now carries a
   measured *eight*-OSEM body (`SIM_CHANNELS = (4, 8)`, built from the 08-04 and
   08-06 logs), so v10, v11 and v12 are simulated rather than skipped; only
   `osem.v6.py` and `osem.v13.py` are skipped, both by `KIND`. `sample-guard`,
   `decimate` and all five `persist-baseline` assertions pass.
   **v3, v7, v9 and v10 are clean.** All 11 failures are on v11 (3) and v12 (8).
7. ~~**a4, a6, a7.** Connected, near mid-scale, ~1/100 the in-band gain of
   a0–a3. Cause not established.~~ **SUPERSEDED 2026-08-17.** a4 is the strongest
   channel on the rig (SNR 107.2 / 75.1 / 94.0, grade GOOD); a6/a7 are
   bottom-railed and are *vertical* sensors. Part of the historical deficit may be
   a frequency-error artefact — re-analysing one 2026-08-17 record at stale vs.
   corrected frequencies moves a0's mode-C SNR 12.1 → 50.5, a factor 4.2 — but the
   08-06 record has **not** been re-analysed, so that is a hypothesis.
   § *On the bench, 2026-08-17* § 4.
8. **The mode frequencies used everywhere in this file are stale.** They moved
   between 2026-08-06 and 2026-08-17 by up to 0.0170 Hz, i.e. 9.1 half-widths at
   Q = 433. `F_MODE_HZ` in `epsilon` and `zeta` needs updating and `zeta` will
   refuse a Φ measured today until it is. Highest-value open item;
   § *On the bench, 2026-08-17* § 2.

---

## v13 — a skeleton, not a version

`osem.v13.py` exists in the tree and is **not a controller**. It declares
`KIND = "skeleton"`, which is what keeps it out of `make list`, `make check` and
`make run` — the same mechanism `osem.v6.py` uses. `main()` raises `SystemExit`
with the reasons.

It was written to carry a three-mode modal law. That law is unreachable: `MODAL`
is `None` and `mimo_ready()` refuses, because MIMO is closed by measurement
(above). What is left is v12's control law plus a modal *observer* logging at
zero force.

**Two things about it that were wrong until 2026-08-06**, both worth recording
because both were invisible:

1. **It was runnable.** The refusal was defined as `_unused_main` and never
   called; the live `main()` ran v12's loop. `make run V=v13` would have driven
   the optic from a file whose own docstring says it must not be. The names are
   now swapped: `main()` refuses, `_run_as_v12()` is the deliberate path.
2. **Loading it crashed the harness.** It does not define the module-level
   constants a controller must, so `S.load()` raised `AttributeError` on
   `ENVELOPE_WINDOW_S` and took `make list` down with it. Fixed by the `KIND`
   declaration, and `harness.declared()` now reads the
   `VERSION_TAG, BENCH_STATUS = "vN", "status"` tuple form so a file that is
   never loaded can still report its own status.

`_run_as_v12` does `v12.Controller = Controller` — it **mutates the imported
module**. That was against the standalone rule as it then stood, because `bench.py`
prints the gain vectors out of the file about to run and a controller importing its
body from another file defeats that check. **SUPERSEDED 2026-08-17:** the standalone
rule was dropped for `stdlib.py`, and `bench.py` now handles the case explicitly —
`KIND = "skeleton"` plus `GAIN_SOURCE`, so it names the module the law came from.
Mutating an imported module is still the wrong shape; importing from one is not.

### What the lag change cost, measured

`RUNAWAY_TREND_LAG_S = 10.0` is not free, and the suite priced it. Running v12
against the whole suite at each value, everything else identical:

| `RUNAWAY_TREND_LAG_S` | passed | failed |
|---|---|---|
| 2.0 (v9's value) | 43 | 7 |
| **10.0 (shipped)** | 42 | **8** |

**Exactly one assertion changed**, and it is the wrong-sign detector:

> `a wrong-signed channel turns a fault-free run into a repeatedly faulting one`
> `b["faults"] >= 3 and a["faults"] == 0`

With ch2's gain inverted over a 120 s run, a 2 s lag trips the breaker **3+**
times and a 10 s lag trips it **2**. The check wants at least 3, so it fails.

**The wrong sign is still caught.** The very next assertion,
`the global interlock trips on it` (`b["faults"] > 0`), passes at both values.
What moved is how *many* times it trips, and the threshold of 3 was written when
2.0 s was the only lag anyone had used.

This is the honest trade of the fix: a breaker that no longer mistakes a 1.55 s
beat for growth is also a breaker that fires less often on something that
genuinely is growing. Two trips in 120 s is still a trip. Whether 10.0 s is the
right point on that curve is not settled by one bench evening, and the threshold
of 3 should probably be re-derived rather than assumed.

The other seven v12 failures are present at **both** lag values, so they are not
from this change. They are `soft-saturation` (2), `auto-disable` on the
eight-OSEM body (2), `warm-restart` (2) and baseline-reuse (1). v11 fails three
of the same family. **None of them has been attributed to a specific commit**,
because v12 has never been committed and there is no baseline to diff against.
Do that before trusting any of them.

---

## On the bench, 2026-08-17 (afternoon) — `status.py` only, no loop closed

The afternoon session was the census tool. **No controller ran and no gain was
applied.** The evening session closed a modal loop and is written up separately
below. Records in `data/`:

| record | samples | length | rate | verdict |
|---|---|---|---|---|
| `20260817_172150_status_sensors.csv` | 37 619 | 90 s | 418 Hz | **CLEAN.** The sensor census and the frequency re-measurement |
| `20260817_174825_status_phi.csv` | 96 883 | 229 s | 424 Hz | **DISTURBED** — the table was being physically worked on. Useless as a quiet record, and the definitive a5/a6/a7 evidence |
| `20260817_175912_status_sensors.csv` | 26 092 | 60 s | 435 Hz | **CLEAN.** The a5/a6/a7 retest after a physical check |
| `20260817_171104_status_coils.csv` | — | partial | — | coils 0–1 only, driven at the **stale** frequencies. **Do not build an actuation matrix from it** |

### 1. The board changed, and it runs at 115200 baud

**The attached board is not the board this repo was written against.** It is an
official **Arduino Mega 2560 R3** — `/dev/cu.usbmodem11101`, USB VID:PID
**2341:0042**, manufacturer string `Arduino (www.arduino.cc)`, and the only USB
device on the machine with a vendor ID. The port recorded throughout this repo,
`/dev/cu.usbserial-1120`, is a CH340/FTDI-bridge name: **a different physical
board**, a clone.

**It runs at 115200 baud, not the 500000 that `pyDAC.BAUD`, `pyDAC2.BAUD` and
`arduino.ino`'s `BAUD_HZ` all declare.** Measured: at 500000 the wire returns
framing garbage (`b'\x80\x80xx\x00x\x00x\x00x\x80xx\x00\x80x'`); at 115200 it
returns a clean `b'READY\r\n'`. `arduino.ino` carries a `BAUD` command precisely
so the rate can be changed without a reflash, so **the live rate is a property of
what was last done to that board and nothing in this tree can tell you what it
is.**

Achieved rate at 115200 with 8 ASCII channels: **418–435 Hz** across the six
records. For comparison, 1024–1113 Hz was measured at 500000 on 2026-08-06 (v11
and v12, four runs) and ~348 Hz at 115200 on 08-03/04 — on the other board. Why
the same baud gives 418–435 Hz now and 348 Hz then is **not established**.

420 Hz is fine: 4.2x the 100 Hz control clock, and inside the range `delta`'s
`decimate` was validated at (347 Hz).

**Why this cost hours, and it is the part worth keeping.**
`pyDAC2.FastDAC.__init__` sleeps 2 s and then scans up to 200 lines for `READY`
with a 2.0 s serial timeout. At the wrong baud that is up to **400 s of total
silence with nothing printed**, which is indistinguishable from a hung program.
The fix is a standing practice, now in `CLAUDE.md`: **probe the baud, never assume
it.** `status.py` has `probe_baud`.

### 2. The mode frequencies moved

**The highest-value finding of the session.** Re-measured from
`20260817_172150_status_sensors.csv`, consensus over the five sensors above SNR 8
(a0–a4), inter-sensor spread **0.0005 Hz**:

| mode | 2026-08-06 | 2026-08-17 | shift | half-widths at Q = 433 | on-peak response left |
|---|---|---|---|---|---|
| A | 0.7154396674928515 | **0.72294** | **+0.00750** | 9.1 | 11 % |
| B | 0.9949288053475371 | **0.99193** | **−0.00300** | 2.6 | 36 % |
| C | 1.6395739504316790 | **1.65657** | **+0.01700** | 9.0 | 11 % |

Half-width is f/(2Q) at Q's measured 1σ floor of **433** (`analysis/ringdown.md`).
A drive at the old frequency reaches 1/√(1+n²) of the on-peak response at n
half-widths — the last column — **and its phase is wrong by arctan(n), up to
84°**. Phase is half of what an actuation matrix exists to produce. This is a
milder instance of the exact defect that invalidated `gains.json`, which was
59–68 half-widths off.

**Consequence.** `F_MODE_HZ = np.array([0.7155, 0.9949, 1.6396])` at
`osem.epsilon.py:163` and `osem.zeta.py:211` is **stale**. `epsilon`'s Kalman
filter models three undamped oscillators at those frequencies. `zeta` validates a
loaded `modal.json`'s frequencies against `F_MODE_HZ` to 0.005 Hz, so it will
**correctly refuse any Φ measured today** until the array is updated. The
2026-08-06 8-coil actuation matrix (717 420 samples, 880 Hz) was measured at those
frequencies too, so its magnitudes are attenuated and its phases are wrong: **A is
effectively a hole again.**

**The day-to-day stability of these frequencies is NOT established.** One 11-day
interval is all the evidence there is, and it is not enough to state a drift rate.

**Why nobody saw it earlier.** § *Frequencies* in `analysis/ringdown.md` records
"same frequencies on all three days, 2026-08-03 / 04 / 06, to within one zero-pad
bin" — and that bin is **0.0139 Hz**. It cannot resolve a 0.0030 or 0.0075 Hz
shift at all; only mode C's +0.0170 Hz would have exceeded it. The stability check
was real and too coarse.

**New standing practice:** re-measure the mode frequencies at the *start* of every
session, before driving anything. `osem.epsilon.py`'s docstring already asked for
this; **2026-08-17 was the first time anybody did it.**

### 3. Off-resonance drive makes the anti-phase unwind PUMP

`status.py`'s active pass reverses each drive with an anti-phase segment on a
shared time origin. Driven at the stale frequencies **that stopped cancelling and
started adding energy.** From the aborted coil pass
(`20260817_171104_status_coils.csv`):

- residual grew **0.1883 → 0.2774 → 0.4218 V** against a **0.2590 V** drive peak;
- a later point reached **0.3321 V** residual against a **0.1067 V** peak —
  ringing at **3.1x** the drive.

**Mechanism.** 0.010–0.017 Hz of frequency error accumulates 2π·Δf·T of phase over
a 60 s drive-plus-unwind: **216–367°**. Past anti-phase the "unwind" is
re-driving. So a stale frequency does not merely attenuate the measurement — it
leaves the optic **ringing into the next point** and contaminates it. The coil pass
was aborted after **2 of 8** coils for this reason, which is why there is no
actuation matrix from this session.

### 4. The sensor census

Measured at the **corrected** frequencies, `20260817_172150_status_sensors.csv`.
`status.py`'s grades: **GOOD** = all three modes above SNR 8 with headroom and
mechanical coherence; **OK** = damps but cannot carry a Φ row; **DEAD** = railed,
flat, blind, or reading another channel.

| ch | grade | resting counts | SNR A / B / C | coupling | note |
|---|---|---|---|---|---|
| a0 | GOOD | 699.2 | 66.1 / 72.0 / 50.5 | MECHANICAL | |
| a1 | GOOD | 585.4 | 68.2 / 48.3 / 54.0 | MECHANICAL | the channel that closed MIMO |
| a2 | OK | 637.5 | 79.1 / 87.4 / 70.0 | AMBIGUOUS | § 6 |
| a3 | GOOD | 630.1 | 90.6 / 63.1 / 79.6 | MECHANICAL | |
| a4 | GOOD | 482.6 | 107.2 / 75.1 / 94.0 | MECHANICAL | **strongest channel on the rig** |
| a5 | DEAD | 0.0 | 0.0 / 0.0 / 0.0 | BLIND | std **exactly** 0.00 over 96 883 samples |
| a6 | DEAD | 7.4 | 3.8 / 3.3 / 2.0 | BLIND | bottom-railed 89 % of record |
| a7 | DEAD | 13.2 | 3.7 / 4.4 / 2.9 | BLIND | bottom-railed 69 % of record |

**This refutes a standing claim in this file.** § *v12* and § *Still open* ask why
a4, a6 and a7 have ~1/100 the in-band gain of a0–a3. **a4 is now the strongest
channel on the rig** at SNR 107.2 on mode A, where on 2026-08-06 it read
**3.0 / 28.9 / 10.1**. Whatever was wrong is not wrong now.

**Part of the historical deficit may be a frequency-error artefact — hypothesis,
with evidence, not a result.** Re-analysing the *same* 2026-08-17 record at stale
vs. corrected frequencies moves a0's mode-C SNR from **12.1 to 50.5**, a factor
**4.2**, and every 2026-08-06 number was computed at the stale frequencies. **The
08-06 record has not been re-analysed at corrected frequencies**, so this is not
settled. Doing that re-analysis is offline work and costs no bench time.

Resting counts, 2026-08-06 → 2026-08-17, against mid-scale 511.5,
`RAIL_LOW = 12`, `RAIL_GUARD_LOW = 60`:

| record | ch0 | ch1 | ch2 | ch3 | ch4 | ch5 | ch6 | ch7 |
|---|---|---|---|---|---|---|---|---|
| 08-06 `CALIBRATING` | 601.2 | 630.3 | 686.5 | 681.8 | 567.5 | 554.8 | 860.6 | 734.7 |
| 08-17 `172150` | 699.2 | 585.4 | 637.5 | 630.1 | 482.6 | **0.0** | **7.4** | **13.2** |
| 08-17 `175912` | 680.3 | 585.4 | 636.2 | 699.6 | 558.6 | **0.0** | **4.3** | **8.8** |

a0–a4 still all rest above mid-scale and clip the **top** rail first, unchanged
since 08-03 — but **a5/a6/a7 went from mid-scale or above to the bottom rail.**
The offset moved for exactly the three channels that stopped working and for none
of the five that did not; no cause is asserted. Note also that **ch3 moved
+69.5 counts and ch4 +76.0 between two clean records 37 minutes apart**, so the
resting points drift within a session, not just between them.

### 5. a5/a6/a7 are a signal-path failure, not a motion or alignment failure

Tested directly, and the test is decisive. During `20260817_174825_status_phi.csv`
the table was being physically worked on — which makes the record worthless for
what it was taken for and ideal for asking "does this sensor see anything at
all". Per-channel motion over the record and over its loudest 5 s window:

| ch | whole-record std | whole-record p2p | loudest-5 s p2p | share of a0–a4 p2p |
|---|---|---|---|---|
| a0 | 70.68 | 854 | 738 | 85.02 % |
| a1 | 48.40 | 523 | 388 | 44.70 % |
| a2 | 75.00 | 765 | 658 | 75.81 % |
| a3 | 90.18 | 871 | **868** | 100.00 % |
| a4 | 69.26 | 711 | 661 | 76.15 % |
| a5 | **0.00** | **0** | **0** | 0.00 % |
| a6 | 0.78 | 32 | 6 | 0.69 % |
| a7 | 0.89 | 75 | 6 | 0.69 % |

a0–a4 moved **868 counts peak-to-peak** in the loudest 5 s — **85 % of the full
1023-count ADC range**. In that same window a5 moved **exactly 0** and a6/a7 moved
**6**. "There was no motion" cannot explain it: five sensors on the same optic
registered near-full-scale motion at the same instant.

**a5's std of exactly 0.00 over 96 883 samples is the clincher.** A live ADC input
always carries at least ±1 count of dither, so that line reads as **hard ground**.

**a6/a7 are railed, and a rail is not a measurement.** Clipping is nonlinear: it
folds signal into harmonics and biases the lock-in, so even the 6 counts they show
cannot be trusted.

**The retest, after the rig was physically checked** —
`20260817_175912_status_sensors.csv`, 26 092 samples, 60 s, 435 Hz, clean.

a5 is **unchanged**: over 26 092 samples it takes exactly **one distinct value,
0.0**, variance exactly **0.000000e+00 counts²**. Confirmed now on two independent
records and across a physical check.

a6/a7 are **bottom-railed on 100 % of samples**, past `RAIL_LOW = 12`, resting 4.3
and 8.8 counts at std 0.56 and 0.48 — sub-LSB. Per-band share of 0.05–60 Hz power,
with a0 and a4 for contrast:

| ch | resting | std | DC–0.5 | 0.6–1.8 | 1.8–3 | 3–6 | 6.19 line | 6.5–10 | 10–20 | 20–60 |
|---|---|---|---|---|---|---|---|---|---|---|
| a0 | 680.3 | 54.41 | 1.8 | **92.1** | 4.3 | 0.6 | 0.0 | 0.1 | 0.2 | 0.7 |
| a4 | 558.6 | 49.37 | 0.4 | **98.5** | 0.6 | 0.1 | 0.0 | 0.0 | 0.0 | 0.1 |
| a6 | 4.3 | 0.56 | 12.7 | 7.6 | 1.7 | 5.3 | 1.1 | 6.4 | 13.7 | **51.8** |
| a7 | 8.8 | 0.48 | 11.4 | 10.9 | 2.4 | 5.6 | 1.0 | 7.9 | 16.5 | **44.0** |

- **a6/a7's high-frequency power is bandwidth, not signal.** 20–60 Hz is 67 % of
  the analysed band, 10–20 Hz 17 %, 6.5–10 Hz 6 %, and their measured shares track
  those almost exactly. White noise — the ADC's own dither on a pinned line.
- **No 6.19 Hz line in them on this record**: 1.0–1.1 % of power in a band that is
  1 % of the bandwidth. So this record does **not** support the hypothesis that
  6.19 Hz is the vertical resonance (§ 8) — **and cannot refute it either, because
  a railed sensor cannot show its resonance.**
- **One encouraging number:** a6/a7 put **7.6 %** and **10.9 %** of their power in
  the 0.6–1.8 Hz pendulum band against a **2 %** bandwidth share — a **4–5x
  excess**, consistent with **wired but biased onto the floor** rather than
  disconnected. Actionable: fix the DC operating point and re-test. a5 has no
  excess anywhere, because it has no variance at all.
- a0–a4 put **92.1–98.5 %** in the pendulum band, which is what a working in-band
  sensor looks like on this rig.

Per this repo's software-only rule, this is written up as a request rather than
attempted: `CLAUDE.md` § *Hardware requests*, REQUEST 1.

### 6. a2 is mechanically sound; its problem is a shared 6.19 Hz line

a2 grades **OK** not because it is blind but because its in-band/out-of-band
coherence contrast is **1.9x**. Breaking that down:

- a2's **mode-band** coherence with a0/a1/a3 is **0.577–0.667** — healthy.
- At the **6.19 Hz** line, a2–a0 = **0.335** and a2–a1 = **0.326**, against a bias
  floor of 1/K = **0.125**.

So a2 watches the optic **and** shares an electrical line with a0 and a1. A
different and milder fault than being blind. The 6.19 Hz line is already on record
in this repo as a known narrow feature (`analysis/kalman.md`).

### 7. MIMO: the sensor-side blocker is cleared, and nothing else is

Five sensors (a0–a4) carry all three modes at SNR **48–107**. Three modes need
**≥ 4** determined rows for a tall Φ, so the sensor-side condition
§ *MIMO is closed, by measurement* named for reopening modal control **is met** —
counted conservatively, exactly: four grade GOOD and a2 grades OK. It does **not**
depend on a5/a6/a7; those were never going to carry a row.

**Not established as of the afternoon — BOTH SUPERSEDED the same evening,
2026-08-17:**

- ~~**The Φ gate has never run on a valid record.**~~ It needs **≥ 192 s
  undisturbed** — 6 averaging windows of `PHI_WINDOW_S = 32 s` for the jackknife.
  The 90 s clean record gives **2** windows and `status.py` correctly **refused** to
  produce an answer; the 229 s record is disturbed. **SUPERSEDED: Φ measured at
  18:20 on a 300 s passive record, 4×3, rank 3, null space 1.**
- ~~**A is unmeasured.**~~ The coil pass aborted at 2 of 8 coils, at stale
  frequencies (§ 3). **SUPERSEDED: A measured at 18:27 over coils 0–3, rank 3 of
  3** — with its phase contaminated, which is a live caveat and not a supersession.

So *"the sensor-side blocker is cleared"* was true, and ***"MIMO works" was not
established as of the afternoon.*** **It was measured the same evening: modal damps
5x faster than diagonal.**

**On the old closure.** Everything in § *MIMO is closed, by measurement* was
correct for the data it had, including the square-Φ argument and the out-of-mode
residual — **measured on hardware 2026-08-07 as 7.4e-16 over 137 528 damping
samples**. What has changed is that a1 now grades GOOD. And **square was never a
property of the geometry** (§ 8): it was a consequence of only ever determining
three rows.

**One thing to be careful about.** `CLAUDE.md` opened with "MIMO IS CLOSED. DO NOT
TOUCH IT." until 2026-08-17. That instruction now contradicts a shipping
controller — `osem.zeta.py`, written 2026-08-15, which implements the modal law
and refuses safely when its measured inputs are absent or stale — and a session
following it literally would **delete** that file. `CLAUDE.md` has been rewritten
to keep the closure as history and state what changed.

### 8. The sensor geometry — reported, not measured

Stated by the person who built the rig, 2026-08-17, and **recorded as reported
design intent rather than a measurement**, because this repo has been burned twice
by unmeasured statements that read as facts:

| | where |
|---|---|
| a0–a3 | the four **in-plane (XY)** sensors, at the four **corners** |
| a4, a5 | the two **side** sensors, one either side |
| a6, a7 | **VERTICAL** |

**Which sensor sits at which corner is not established** — illustrative
coordinates were offered and withdrawn as "just a vibe". Also reported: a6/a7 are
vertical-motion sensors "capable of correcting up to ~10 Hz by design, because the
element is a steel bar with some spring-damping property". Reported, not measured;
**no vertical mode frequency has ever been measured on this rig.**

Three consequences.

1. **Four corner sensors span three DOF with one direction left over.** Four
   sensors at the corners of a square give a sum, two differences and one **warp**
   combination that no rigid-body motion of the optic can produce. **The
   permutation does not matter** — permuting corners permutes columns and changes
   no rank — so the layout *itself* predicts Φ is 4×3 (5×3 with a4), rank 3, with
   a genuine null space, and that leftover direction **is** the out-of-mode
   residual. **A prediction the Φ gate can check, not a result.**
2. **The census tests every sensor against the horizontal modes**, i.e. lock-in
   SNR at 0.72 / 0.99 / 1.66 Hz plus coherence with a0–a3 in 0.6–1.8 Hz. **A
   sensor watching an orthogonal DOF grades DEAD by construction**, however
   healthy. So a6/a7's DEAD grade is evidence about their DC operating point and
   **not** about their health as vertical sensors. This is a real methodological
   defect in the tool, in the same family as "a4–a7 are not wired" and "the flags
   are misaligned". Fixing it needs a vertical band, and the vertical frequencies
   are not measured.
3. **The 6.19 Hz line becomes a candidate for the vertical resonance.**
   Hypothesis, with a test: ring the vertical mode and see whether *unrailed*
   a6/a7 respond at 6.19 Hz. Counter-evidence in § 5, which is not a refutation.

### 9. Defects found in `status.py`, all fixed except the last

Recorded because each was caught by a specific measurement and would otherwise
recur.

1. **`mimo_gate`'s σ on Φ was optimistic by √n_coils = 2.83** — it used
   `sqrt(sum sigma^2)/ncoils`. It declared a sensor row "determined at 4.1σ" that
   had been **planted at exactly zero** in a synthetic test. A row determined by
   nothing but its own noise is exactly how MIMO gets reopened on a sensor that
   cannot see. Fixed by propagating properly:
   `var(phi_i) = sum_j sigma_ij^2 * |a_j|^2`.
2. **a6 graded OK on a marginal SNR of 4.6** at mode B while its in-band coherence
   with every sensor on the optic sat at the bias floor. The coherence test now
   outranks the lock-in for that case.
3. **The "where is this mode actually" fine scan was ±0.010 Hz.** On a 90 s record
   that is under one resolution element (1/T = 0.011 Hz), **and mode C's peak
   landed exactly on the upper edge** — so the scan railed and under-reported C's
   drift by nearly 2x: reported **+0.0100**, true **+0.0170**. Widened to
   ±0.05 Hz, and hitting the edge is now reported. General lesson: **a peak at the
   edge of its search window is not a peak that has been found.**
4. **The census lock-in was evaluated at the stale frequency**, understating every
   SNR — a0's mode C read **12.1** instead of **50.5**. Now evaluated at the
   measured peak.
5. **A per-coil "independent fraction"** measured as the residual against the span
   of all *other* coils is **identically zero whenever there are more coils than
   modes**, because the response has exactly `MODES_N` degrees of freedom. It
   looks like a strong statement and is arithmetic. Replaced by pairwise cosine
   plus leave-one-out conditioning.
6. **`status.py`'s `PORT` carried the comment "bench.py's autodetect overrides
   this", which was false** — `bench.py` launches controllers and never sees
   `status.py`, so the tool was pinned to whatever tty the board enumerated as on
   one particular day. It now reuses `bench.choose_port`.
7. **Python block-buffers stdout when it is not a terminal**, so every print was
   invisible in 8 kB chunks the moment output was piped or captured — which is how
   a bench log gets kept. Fixed with line buffering.
8. **The acquisition loop printed nothing while running**, so a dead stream and a
   healthy one looked identical for 60 s — and would for 36 min on the coil pass.
   **Three header-only recordings were made before anyone could tell.** Now prints
   a live rate-and-counts line every 5 s, plus a 2 s stream preflight before any
   coil is touched.
9. **STILL OPEN: a plain `SIGTERM` (`pkill`) skips `status.py`'s `finally`**, so
   the coils are left at whatever voltage the drive last set. **This happened on
   2026-08-17** when the coil pass was stopped, and the coils had to be parked by
   hand afterwards.

### 10. Two control-design points worth keeping

- ~~**Colocation cuts the cost of measuring A by 3x.**~~ **SUPERSEDED 2026-08-17
  (evening): colocation does not hold on this rig.** The claim was that each OSEM's
  coil and sensor are colocated, so `A[m,j] = lambda_j * Phi[j,m]` — 8 unknowns
  rather than 24, mostly signs, and 12 min of bench time instead of 36. It is the
  wrong identity: the rig owner states **four coils point the same way, two are
  diametrically opposed, three different planes**, and two measurements agree — two
  independent coil passes gave λ signs `(+,+,+,+)` vs `(+,-,-,+)`, and a
  colocation-constrained rank-1 fit came out **second/first singular value 0.67**
  where colocation predicts ~0. A must be measured over every coil used, at all
  three modes. `zeta`/`eta`'s colocation sign check is therefore gating on an
  identity that does not hold, which is why it refused a clean coil pass.
- **Reordering a matrix is a no-op.** A permutation matrix is orthogonal, so
  permuting rows or columns leaves **rank, condition number and achievable modal
  force identically unchanged**; it only relabels which coil is which. Recorded
  because it was proposed as an alternative to measuring A and cannot be one.
  Choosing a **subset** of coils (pivoted QR / subset selection) is a real
  technique but still requires knowing A. **Rank reduction** is also real and
  `osem.zeta.py` does **not** do it: `Modal.allocate` refuses the entire mask when
  `cond > MODAL_COND_MAX = 12` and drops the whole loop back to diagonal, where
  truncating the smallest singular value would still damp the two well-conditioned
  modes modally. **Open improvement.** *(Done since this was written: both `zeta`
  and `eta` truncate in `Modal._build_tables` — `keep = sum(sv >= sv[0]/12)` — and
  refuse only the directions that are actually unreachable.)*

---

## On the bench, 2026-08-17 (evening) — `zeta` closes a modal loop, and it wins

**The first modal run in this repo's history, and the first result that is not four
independent SISO loops.** `zeta` only; `eta` first ran the following night
(§ *On the bench, 2026-08-18*). Records are `data/20260817_19*` — five `signtest.py`
trials, five `jerk.py` sets, and the coil pass at 19:47.

### 1. Modal damping beats diagonal by 5x, measured as a decay rate

Two runs, same session, same rig, one law swapped, three hand kicks each (`jerk.py`
asks out loud; a human shoves the table), decay fitted to the envelope after each
kick.

| law | peak ratios | decay 1/s | median | re-quiet | r² |
|---|---|---|---|---|---|
| **modal** | 3.96–4.70 | 0.0600 / 0.1393 / 0.1940 | **0.1393** | 5.3 s | 0.79–0.90 |
| diagonal | 3.66–4.44 | 0.0184 / 0.0278 / 0.0311 | 0.0278 | 17.4 s | 0.53–0.69 |

**The ranges do not overlap: the worst modal kick beats the best diagonal kick by
2x.** Peaks are matched across the two sets, so the re-quiet times are comparable
too. Against the plant's intrinsic **0.0072 /s** (τ > 138 s, `analysis/ringdown.md`)
that is **~4x for diagonal and ~19x for modal**.

**Caveats, and they are not a formality: three kicks per law, one session, ambient
drifts between the sets, and the two laws ran as blocks rather than interleaved.**

This is the number to quote rather than § 2's 0.08 — a decay rate is a dissipation
measurement, an amplitude is not.

### 2. The sign sweep, and the colocation check as a complete discriminator

Four per-mode sign patterns of A plus the diagonal control, 70 s each, scored by
median live-channel `ratio` over t ≥ 30 s (`data/signtest_20260817_192808_*`):

| law | median | p90 | faults | modal % |
|---|---|---|---|---|
| **modal +++** | **0.08** | 0.17 | 0 | 97 % |
| modal +-- | 1.35 | 1.55 | 0 | 0 % |
| modal ++- | 1.51 | 1.94 | 0 | 0 % |
| modal +-+ | 1.54 | 1.99 | 0 | 0 % |
| diagonal | 1.58 | 2.01 | 0 | 0 % |

The last three ran at **modal 0 %** because the colocation sign check **refused
them**, so they are diagonal runs and land on diagonal's number. **On this data that
check was a complete discriminator of wrong sign patterns**, not the partial one its
own docstring predicted. It is also a warning: **a refusal is invisible in the
score** — only the `modal` column distinguishes those trials from a diagonal run.

**0.08 is 12x quieter than the calibration window, and that deserves suspicion.**
`ratio` is an amplitude, so **holding** the optic and damping it are identical in it.
§ 1 shows the modal law does dissipate faster; it does **not** establish that the
static 0.08 is dissipation. **Not excluded.**

### 3. The sign of A is physics

On resonance the response lags the drive by 90°, so with Φ rotated real the whole of
A sits in the imaginary part with a determined sign:
`A = -Im(a * conj(rot_phi))`. Verified numerically against this repo's own lock-in —
an undamped oscillator driven on resonance from rest gives `H = -1.2024i` for
`Φ·A > 0` and `+1.2024i` for `Φ·A < 0`. `status.py --save-modal` does that.

**The earlier attempt picked the per-mode sign by maximising colocation consistency
and PUMPED: median ratio 1.5 diagonal → 2.3 modal.** Nothing offline catches it — a
wrong-signed A is still exactly inverted by the allocator, so the force is right in
the model and backwards in the plant. The selftest, the gate and the colocation
check all pass while the loop adds energy.

### 4. Colocation does not hold on this rig, and it is geometry not miswiring

See § *10. Two control-design points*, first bullet, now marked superseded.
Reported by the rig owner: **four coils point the same way, two are diametrically
opposed, three different planes.** Measured: two independent coil passes gave λ signs
`(+,+,+,+)` vs `(+,-,-,+)`; a colocation-constrained rank-1 fit gave second/first
singular value **0.67** where colocation predicts ~0.

**The `DAC_MAP` "coil j dominates sensor j" heuristic cannot work either** for a
suspended rigid body — one coil moves the whole optic. Coil 1 gave a0 **+37.2** and
a1 **+36.1** counts/V, two sensors nearly equal. **Weak evidence, not a wiring
verdict**, and it is recorded that way because "only the bench catches a wrong coil
map" is still true and this is the bench check.

### 5. Φ is measured and TALL

**4×3, rows a0–a3, null space dimension 1, and dropping any single row still leaves
rank 3.** `data/20260817_182021_status_phi.csv`, 113 412 samples, 300 s, ambient,
nothing driven.

The rows being exactly the four **in-plane corner sensors** matches the geometric
prediction: four corners span three rigid-body DOF plus one **warp** direction that
no rigid-body motion produces, and **the warp is the out-of-mode residual.**

### 6. A is measured, and its phase is contaminated

Coils 0–3, modal matrix **rank 3 of 3**, `data/20260817_182701_status_coils.csv`.

**CAVEAT, and it is live:** that pass ran with the defective unwind (§ 7), so
off-quadrature came out **0.736 / 2.088 / 0.812** where resonance wants small —
**A's phase is contaminated.** The sign survives; the magnitudes within each row do
not, and the 5x rests on them.

**A's SIGNS were settled on 2026-08-18** — 12 of 12 across three determinations that
share no estimator — and its **magnitudes were not**. § *On the bench, 2026-08-18*.

A cleaner pass at **19:47** was **REJECTED** by the colocation check on 2 of 4 coils
(`MODAL_MIN_COILS = 3`), which § 4 now explains as the check being wrong rather than
the data. **`data/modal.json` currently holds the 18:27 A — the one the 5x was
measured with.**

### 7. The unwind bug, in two halves, both fixed

`status.py` reversed each drive with an anti-phase segment and **retried** on a
residual test.

- **The retry had no feedback.** It reissued the same command against a state now
  near rest, i.e. a fresh excitation. **Residuals grew 0.188 → 0.277 → 0.422 V
  against a 0.259 V peak.**
- **The comparison was broken.** `peak` was a lock-in over the **whole ramping
  drive**, which averages to about half the end-of-drive amplitude, so
  `resid <= 0.25*peak` was really `<= 0.125*final` and failed points that had
  cancelled fine.

Both fixed — `UNWIND_EXTRA = 0`, and `peak` measured over the last 8 s of the drive.
**The next pass ran with ZERO retries.**

### 8. The fault-clear deadlock — the night's worst bug

The runaway breaker **freezes all channels**, and the per-channel amplitude `ratio`
**stops being recomputed** — but clearing the fault is gated on amplitude coming
down. Measured: **`ratio = 3.59` identical across every print for 1085 s**, gains
zero, nothing damping. **One hand kick killed the entire run.**

**Freezing the actuators is the safety action. Freezing the measurement is the bug.**

**Outcome, 2026-08-18: the frozen measurement is fixed and the deadlock is not.**
`ratio` is now live during FAULT — 1.4–3.8 print to print — and both runs that night
still faulted on their first hand kick and never cleared, under **both** laws.
§ *On the bench, 2026-08-18* and `CLAUDE.md` § 1.

`jerk.py` cannot distinguish a frozen `ratio` from one that never comes down — it
records `nan` after `DECAY_TIMEOUT_S` either way.

### 9. Residual motion, per mode and per axis

Over **28 775 modal damping samples**. Modal velocity rms:

| mode A | mode B | mode C |
|---|---|---|
| 0.64 | **2.61** | **3.15** |

**THAT READING IS WITHDRAWN.** It said "B and C are 4–5x worse than A while all three
run the same gain — the first real argument for a per-mode gain". Two things kill it:
the run it came from (`data/20260817_201454`) was in **FAULT for 1085 of its 1168 s**
and holds only **14 s** of damping, and a residual with no open-loop reference cannot
tell a badly-damped mode from a hard-driven one. Measured against a control
(`CLAUDE.md` § *Per-mode residual*), **B and C are the modes modal damping wins on** —
5.9x and 6.7x — and **A** is the one it does not. The rms numbers above stand; the
inference did not.

Per-sensor residual counts rms:

| a0 | a1 | a2 | a3 | a4 (side) | a6 (vert) | a7 (vert) |
|---|---|---|---|---|---|---|
| 86.2 | 57.9 | 96.0 | 92.4 | 43.4 | **61.3** | 32.6 |

a0–a3 are the four rows of Φ. **a4, a6 and a7 have no Φ row, so the DOF they watch
are unobserved and undamped: a6 carries 61 counts rms that nothing acts on.** The
vertical mode frequencies have never been measured.

### 10. Actuator saturation — coil 3 clips, and that voids the guarantee

Measured demand about bias on coils 0–3:

| | coil 0 | coil 1 | coil 2 | coil 3 |
|---|---|---|---|---|
| rms V | 0.0273 | 0.0328 | 0.0255 | **0.0819** |
| max V | 0.1863 | 0.2239 | 0.1949 | **0.4229** |

**Coil 3 reached 169 % of the 0.25 V half-window and clipped**, while the modal
allocation stayed inside its own **0.20 V** cap the whole time — **the derivative
term is added after the cap.** Once a coil clips the realised force is no longer
`A_C^+ f` and the dissipation guarantee is void. **The 5x was measured with this
happening.**

Compare § *Diagonal reallocation* in `CLAUDE.md`, where the term eating the headroom
was Ki (peak 0.1135 → 0.0056 V). It is now Kd.

### 11. a5 is a disconnected pin; a6/a7 came back with no cause -- SUPERSEDED 2026-08-20

**SUPERSEDED 2026-08-20 ON a5. It came back and `ENABLE_CHANNEL[5]` is `True`
again** -- std 9.16 counts, 36 distinct values, coherence 0.00 -> 0.70, and **its
own coil still does not move it** (−0.39 ±0.93 counts/V, 0.4 sigma). § *On the
bench, 2026-08-20*. The measurement below was right for the data it had and no
cause is recorded for the change.

**a5: exactly one distinct value, 0.0 counts, variance exactly zero, across 236 387
samples in three records**, while a0–a4 ran std 41–90 counts. A live ADC input
carries ±1 count of dither. **Disabled in the controller on that basis.**

**a6/a7 were bottom-railed at 4.3 / 8.8 counts (std 0.48–0.56) at 17:59 and read
591.3 / 566.0 at 18:20. They came back and NO CAUSE IS RECORDED.**

### 12. The mode frequencies are stable on an hours timescale

Re-measured **0.72294 / 0.99193 / 1.65657 Hz** at 17:21 and **confirmed within
0.001 Hz at 19:50**. So they are stable within a session even though they moved up to
**0.0170 Hz — 9 half-widths — over the 11 days** from 2026-08-06. One 11-day interval
is still not enough to state a drift rate.

### 13. The board runs at 115200 baud, not the 500000 this tree declares

`pyDAC2.probe_baud` / `resolve_baud` now probe it for every tool. **At the wrong rate
both transports scan 200 lines at a 2 s timeout — up to 400 s of silence with nothing
printed**, which is indistinguishable from a hung program.

### 14. a4, a6 and a7 DO see the optic — this reverses § 4 of the afternoon

**Measured once, on one ambient record, and untested on the bench.** Multiple
coherence against a0–a3 over 0.6–1.8 Hz, from
`data/20260817_205211_status_sensors.csv` (113 418 samples, 300 s, 378 Hz, 26
segments, multiple-coherence bias floor 3/26 = **0.12**), bias-corrected usable
fraction:

| a0 | a4 (side) | a6 (vert) | a7 (vert) |
|---|---|---|---|
| 1.00 | **0.46** | **0.26** | **0.50** |

**Roughly half of a4's and a7's in-band motion is the optic.**

**Why the driven test said otherwise, and it is the important part.** The coil pass
gave those channels |H| **0.02–0.39 V/V** at SNR **0–6**, which was read as blind
sensors. It is not: **coils 0–3 barely push the side and vertical DOF**, so a weak
driven response says the **coils** do not reach those degrees of freedom, not that
the **sensors** cannot see. This is the same family of error as "a4–a7 are not
wired" and "the flags are misaligned" — a test that cannot distinguish a broken
sensor from one pointed where the actuator is not.

`eta` is being given a **per-channel PID on those three, at gain scaled by the
coherent fraction.** The argument: dissipation is linear in gain while injected
noise is quadratic, so there is always a gain at which noisy feedback is net
dissipative, and gain proportional to the coherent fraction is the Wiener-optimal
weight rather than a fudge. **Untested on the bench.**

### 15. The suite, after the 2026-08-17 deletions

`make check` on the final code: **161 passed / 40 failed in 513 s**, `sysid` skipped.

| | passed | failed |
|---|---|---|
| delta | 42 | 8 |
| epsilon | 37 | 12 |
| zeta | 41 | 10 |
| eta | 41 | 10 — **failure set identical to `zeta`'s** |

`delta`'s 8 match what was recorded for v12, so the rename and the deletions cost
nothing there. `epsilon`'s 12 are the worst on the ladder and it has never been on
the rig. All of them are **unattributed**: none of these files has been committed,
so there is no baseline to diff against. Commit, then bisect.

**That table is stale in two rows and `make check` HAS NOT BEEN RE-RUN SINCE.**
`delta` has since been deleted; `theta` did not exist and has never appeared in a
suite run. Nothing after 2026-08-17 is represented -- not the deletions, not the
`status.py` / `stdlib.py` / `jerk.py` work of 2026-08-17/18, and not the whole of
2026-08-20. The only thing run on 2026-08-20 was the **per-file selftests**:
`osem.eta.py` **114/114**, `osem.theta.py` **144/144**, `status.py` all pass, and a
selftest is not the suite.

### What this session did NOT establish

Recorded here so the next session does not assume otherwise: **A's phase**;
**holding vs damping** for the 0.08; **the χ²/dof distribution** on this rig; **the
vertical mode frequencies**; **why a6/a7 recovered**; **why colocation fails, in
detail** — that it fails is measured, but no coil→DOF geometry exists to replace it
with; and **whether feeding back on a4/a6/a7 at coherence-scaled gain helps**, which
is one ambient record and an argument.

---

## On the bench, 2026-08-18 (00:00–00:30) — `eta`'s first run, A's signs, and two faults

**`eta`'s first time on the rig**, and the first modal loop closed on a **geometric**
Φ. Two runs: a diagonal null test and a modal run. Records are
`data/20260818_001819_jerk_eta.log` + `data/20260818_001841_fast_lock.csv` and
`data/20260818_002207_jerk_eta.log` + `data/20260818_002229_fast_lock.csv`; the coil
work is `data/20260817_231251_status_coils.csv` and
`data/20260817_235950_status_coils.csv`.

### 1. A's SIGNS are settled — 12 of 12, three independent determinations

Sign of `A[mode, coil]` over coils 0–3, from static steps at **23:12**, static steps
at **23:59**, and the **resonant lock-in** on the 23:12 file. All three agree on all
twelve:

| mode | coil 0 | coil 1 | coil 2 | coil 3 |
|---|---|---|---|---|
| A (T1) | −1 | +1 | +1 | +1 |
| B (Z) | −1 | +1 | −1 | −1 |
| C (T2) | −1 | −1 | −1 | +1 |

**Why it is stronger than three repeats.** The static and the resonant
determinations **share no estimator** — a 20 s mean against a lock-in at the mode
frequency — so they cannot fail the same way. And **a static step is immune to the
leftover-ring contamination that ruins the driven phase** (§ *The 23:12 coil pass* in
`CLAUDE.md`), because the ring is zero-mean about the new equilibrium.

**A's MAGNITUDES are NOT settled.** The two DC passes reproduce individual entries
only to a factor **0.23–1.8**, and the driven pass is off-quadrature **0.624 / 0.749
/ 1.196** where resonance wants ~0.

`data/modal.json` now holds **geometric Φ** and the driven 23:12 A, `basis:
"geometric"`, written 00:15:54, accepted by `stdlib.Modal`.

### 2. The multisine is a NEGATIVE RESULT on resonance

`status.py coils --multisine` drives all three modes at once, one point per coil,
**9.9 min instead of 18**.

| | one-at-a-time | multisine |
|---|---|---|
| per-coil phase spread | 73–89° | **31.6 / 36.7 / 7.8°** |
| signs agreeing with the two DC passes | **12 of 12** | **7 of 12** |

**The reason is structural.** An on-resonance response **ramps**, so it has broad
1/f² spectral skirts rather than a clean line and the loudest mode leaks into the
quietest mode's bin. Mode A is strongest (**|H| 8.58**), mode B weakest (**|H|
3.03**), and **mode B is exactly where the multisine signs go wrong.** The flag
exists and stays; it is not the right trade on resonance.

### 3. Both runs faulted on their first hand kick — state blocks, measured

| run | file | length | CALIBRATING | DAMPING | FAULT |
|---|---|---|---|---|---|
| diagonal | `data/20260818_001841_fast_lock.csv` | 160 s | 0.0–20.0 s | **20.0–40.9 s**, gain 0.0333, median ratio **1.24** | **40.9–159.6 s**, gain 0.0000, median ratio **2.40** |
| modal | `data/20260818_002229_fast_lock.csv` | 133 s | 0.0–2.0 s | **2.0–38.5 s**, gain 0.0342, median ratio **0.25** | **38.5–133.0 s**, gain 0.0000, median ratio **2.25** |

**The null test confirmed the refusal path and nothing else.** With
`data/modal.json` moved aside `eta` runs `epsilon`'s diagonal law and **modal never
engages** — `modal` flag / rank `[('0','0')]` across the whole run, banner
`[modal] REFUSED -- running DIAGONAL`. **It did not produce a comparable decay set.**

**One partial kick per law, and neither is clean:**

| law | peak | decay 1/s | r² | why partial |
|---|---|---|---|---|
| diagonal | 4.67 | **0.0434** | 0.782 | its 16.2 s decay window crosses the fault boundary at 40.9 s |
| modal | 4.73 | **0.0683** | 0.670 | 9.6 s re-quiet, and the run entered FAULT at 38.5 s |

**Every later "kick" in both runs was fitted to an UNDRIVEN plate** — gains exactly
0.0000 from the trip to the end — so those fits measure neither control law; one
comes out at **−0.0053 /s, r² 0.065**.

**WITHDRAWN the same night:** a "median 0.0434 /s over three kicks, 1.6x the
2026-08-17 diagonal median" was derived here first and is **false**, two of the three
fits being to an undriven plate. **There is no measured session-to-session scatter of
the diagonal law.** And 2026-08-18 **neither reproduced nor refuted the 5x** — leave
§ 1 of the 08-17 evening section standing with its own caveats.

### 4. The runaway breaker trips on a normal hand kick under EITHER law

- **Diagonal**: quiet ratio **1.24**, kick **4.67** — **3.8x**, against the **1.8x**
  breaker.
- **Modal**: quiet **0.25** over the DAMPING block, **0.117** per channel in the
  quiet periods, against the diagonal run's quiet periods of **1.449 / 1.832 /
  1.769** — about **13x** lower. Kick **4.73**.

**So it is not a modal-specific mis-scaling.** Modal's lower operating point makes it
worse; the defect is present under the diagonal law too. Both faults then persisted
**95–119 s to the end of the run** and neither cleared: ch3 held at **2.1–3.2**
against `FAULT_CLEAR_RATIO = 1.4`.

**Half of the 08-17 deadlock is fixed:** `ratio` is **live during FAULT**, varying
1.4–3.8 print to print, where 2026-08-17 measured it frozen at **3.59 for 1085 s**.
**The deadlock is not.** Every threshold on this rig — the breaker at 1.8x, the clear
gate at 1.4, the lock detector at 0.35x, and `jerk.py`'s own triggers — is an absolute
multiple of a baseline measured with the loop **off**. `CLAUDE.md` § 1.

**A fix is in flight in `osem.eta.py` / `stdlib.py` / `jerk.py`. Nothing here claims
it is done and its design is not described here.**

### 5. `LOCKED`, for the first time in this repo's history

    *** LOCKED -- 20.7s after gain was applied (t=22.7s total), 4/7 channels
        driven and quiet -- DEGRADED, 4/7 healthy ***

`data/20260818_002207_jerk_eta.log`, on the modal run. Read the qualifiers with it:
**DEGRADED** (a4/a6/a7 demoted by `inband-floor` before the announcement), and the
run **engaged on the stored baseline** after a 2.0 s warm-up rather than a fresh 20 s
calibration, so 22.7 s is not comparable with a cold start. It faulted 12 s later.
**Anything in this tree still saying no rung has ever printed `LOCKED` is stale.**

### 6. The hybrid PID channels have never run

Measured across **both** runs: **ch4, ch6 and ch7 carried `|gain|` exactly 0.0000 for
100 % of all DAMPING samples.** They were demoted at engage, both times:

    !! ch4, ch5, ch6, ch7 demoted -- in-band 0.0027 / 0.0000 / 0.0026 / 0.0024V,
       under 10% of the 0.0834V median across enabled channels

`inband-floor` measures in-band amplitude at the three **horizontal** mode
frequencies, and **a4 senses X while a6/a7 sense Y** — axes orthogonal to the damped
Z / T1 / T2 — so their in-band amplitude is small **by construction**, not because
they are blind. Their measured coherence with the optic is **0.46 / 0.26 / 0.50**.

**This is the fourth instance of one error family**, after "a4–a7 are not wired",
"the flags are misaligned", and "a weak driven response says the coils do not reach
those DOF, not that the sensors cannot see". A fix is being attempted in
`osem.eta.py`; **nothing here claims it is done.**

### 7. First χ²/dof numbers off the rig — six prints, not a distribution

Modal run, `dof = 4`: **8.00 / 2.10 / 2.90 / 2.31 / 0.32 / 0.03** while DAMPING,
**888.74** on the kick that tripped the breaker, then **3662.82 / 563.59 / 226.61 /
54.71 / 97.48** through FAULT. These are the 5 s status prints, and there is **no
`CALIBRATING` population at all** because the run engaged on a stored baseline. **No
threshold follows from six numbers** — § *eta* and `CLAUDE.md` § 5 stand.

### 8. Tooling, and what was deleted

`status.py` gained **geometric Φ as the default export basis** (`--phi geo|svd`),
`phase_consistency`, `phase_compare`, `--multisine`, `--settle`, `--hand-damp`, and
**`MODES` updated to the 840 s empty-room values 0.72194 / 0.99193 / 1.65607 Hz** —
they had been the **2026-08-06** values, up to **0.0170 Hz** stale.

**`settle` — damp between coil points with a colocated diagonal velocity loop. It is
NEUTRAL, not damping, and OFF by default.**

- **The first version PUMPED.** It ran at the **379 Hz** sample rate, about
  **182 kbit/s** of `SET` traffic against a **115200** baud link, so the wire
  saturated and the applied voltage lagged by a growing delay: in-band rms **40 → 84
  counts**. **This is exactly what `delta`'s `decimate` exists for, and the lesson
  did not transfer until it was repeated.**
- **After decimating to a 100 Hz control clock it is neutral**: **55.45 → 53.73
  counts over 45 s.** An open item, not a solution.

**Deleted in this session** (the rest of the tree dates these to 2026-08-17; the
session spans midnight): **`osem.delta.py`** — retired to git at `b90c983`, ladder now
**epsilon → zeta → eta**; **`signtest.py`** — a geometric Φ leaves no per-mode sign to
search, its five records survive as `data/signtest_20260817_192808_*`; and
**`build/`**, gitignored staging regenerated by `bench.py --flash`.

### What this session did NOT establish

**A's magnitudes**; **any session-to-session scatter of the diagonal law**; **whether
the 5x reproduces**; **whether the hybrid PID on a4/a6/a7 helps** — it has still
never run; **the χ²/dof distribution**; and **whether `settle` can be made to damp**.
`osem.eta.py` still declares `BENCH_STATUS = "untested"`, which is now wrong.

---

## On the bench, 2026-08-20 -- the coils were re-seated, a slope flipped sign, and the loop pumped

**The largest uncontrolled change to this plant since the 2026-08-03 rewiring.** A
bias raise took the front end down at midday; the hardware team **re-seated the
coils**; every slope was re-measured afterwards and **one had changed sign**. One
closed-loop run followed and it **pumped**. Records:

| record | what it is |
|---|---|
| `data/20260820_125342_status_sensors.csv`, `data/20260820_125525_status_sensors.csv` | the dead front end -- all eight channels at exactly 0.0 counts |
| `data/20260820_155638_status_slopesign.csv` | the 8-coil DC pass, 0.10 -> 0.40 V, 12 s dwell |
| `data/20260820_162534_status_sensors.csv` | 300 s census, 105 013 samples, no drive |
| `data/20260820_171227_jerk_eta.log` + `data/20260820_171242_fast_lock.csv` | the one closed-loop run, `eta`, diagonal |

**The rig runs on a 3 A power supply** (rig owner, 2026-08-20). Nothing computable
from this repo bounds the coil current -- the coil driver is not in the tree.

### 1. a3's slope CHANGED SIGN, and nothing in software could catch it

All eight coils, one at a time, **0.10 -> 0.40 V, 12 s dwell**, counts/V on each
channel's own sensor, uncertainty on the **mean** over 1 s blocks (`slopesign.py`):

| ch | counts/V | | ch | counts/V | |
|---|---|---|---|---|---|
| a0 | **+70.50** ±5.63 | 12.5σ | a4 | **+5.06** ±0.46 | 10.9σ |
| a1 | **+88.87** ±10.15 | 8.8σ | a5 | **−0.39** ±0.93 | 0.4σ, **UNRESOLVED** |
| a2 | **−205.94** ±30.46 | 6.8σ | a6 | **+10.37** ±0.30 | 34.9σ |
| a3 | **−98.79** ±19.81 | 5.0σ | a7 | **+8.07** ±0.17 | 46.2σ |

**a3 was +81.08 counts/V before the re-seating and is −98.79 after**, so
`STEADY_GAIN[3]` is now **+0.035** and a0-a3 read **−0.035 −0.035 +0.035 +0.035**
(`osem.eta.py:221`, `osem.theta.py:351`). Until it was flipped, that channel's
shipped gain was **positive feedback**.

**NOTHING IN SOFTWARE COULD HAVE CAUGHT IT** -- same class as the wrong-`DAC_MAP`
trap and the 2026-08-17 sign-of-A failure. **Only a DC step on the bench sees it.**

**a6/a7 reproduce their 2026-08-17 values inside 1 sigma** (+10.37 against +10.07,
+8.07 against +8.05), so the estimator is confirmed and a3's change is a change in
the **rig**. **a4 is now resolved** at 10.9σ where it was 2.9σ, same sign.

**CONSEQUENCE FOR A, and it is an argument from a measurement rather than a
re-measurement.** Every determination of A in this repo -- including the twelve
signs settled 12-of-12 on 2026-08-18 -- predates the re-seating. **A DC slope on
this rig has now been observed to flip sign across a re-seating**, so
`data/modal.json` may not be assumed to describe this plant. **A has not been
re-measured.** `CLAUDE.md` § 3.

### 2. A SIGN BUG: `SLOPE_SIGN` held two conventions at once

`SLOPE_SIGN` was carrying two different quantities in one array: **a0-a3's entries
were copied from `STEADY_GAIN`** (the **negated** slope, never measured as a slope)
and **a4/a6/a7's came from a real DC step** (the **raw** slope). So
`HYBRID_GAIN = HYBRID_KP * SLOPE_SIGN * HYBRID_COHERENT` gave **a4/a6/a7 positive
feedback**.

**It never fired, and only because of an unrelated defect**: `inband-floor` demoted
those three at engage on both 2026-08-18 runs, so `|gain|` was exactly 0.0000 for
100 % of DAMPING. **A latent positive-feedback path was saved by a defect this repo
lists as a defect.** Neither the selftests, nor the modal gate, nor the colocation
check saw it.

**The fix** (`osem.eta.py:480-496`, `osem.theta.py:632-648`): `SLOPE_SIGN` is now the
**physical, measured** slope sign and nothing else; **`DAMP_SIGN = -SLOPE_SIGN`** is
the separate quantity the gains use; and `HYBRID_GAIN` is additionally masked by
**`HYBRID_CHANNEL`**, so a non-hybrid channel carries an **identically zero** gain
rather than a nonzero one held off by a gate.

**The `DAMP_SIGN` negation is EMPIRICAL and the file says so.** The four gains that
have actually damped on this rig are the negated slope on all four, and that is what
produced every decay number in the repo -- but working the sign through the code
gives the **opposite** answer, because `Pid.terms` sets `p = gain x (−vel)`. **One
of those is wrong and it is not the four hardware confirmations.** There is an
unfound inversion between `bp`, `vel` and the coil force; if anyone finds it, the
negation has to come out at the same time.

### 3. a5 is ALIVE -- and its own coil still does not move it

| | before | 2026-08-20 |
|---|---|---|
| std, quiet record | **0.00 counts** (variance exactly zero, 236 387 samples, 3 records) | **9.16 counts** |
| distinct values | **1** | **36** |
| multiple coherence with a0-a3 | **0.00** | **0.70** |

`ENABLE_CHANNEL[5]` is now **`True`**. Independently, over the 300 s 16:25 census
(105 013 samples) a5 reads **mean 537.4, std 8.02 counts, 82 distinct values** --
same verdict, different record.

**ITS GAIN STAYS ZERO, AND THE 0.70 IS NOT A REASON TO TURN IT ON.** **Coil 5 does
not move a5**: **−0.39 ±0.93 counts/V, 0.4 sigma**, the only unresolved slope on the
rig. The same drive moves **a1 +9.30, a2 −18.39, a3 +11.11** counts/V. So a5 into
coil 5 is a **NON-COLOCATED** loop, and **the whole free-dissipation argument for
the per-channel term is colocation** -- sensor j and coil j on the same coordinate,
so `−Kp x velocity` opposes motion whatever the plant does, at any positive gain,
without a model. Non-colocated rate feedback has no such guarantee; it needs A, and
A's magnitudes are not established. **`HYBRID_CHANNEL[5] = False`**
(`osem.eta.py:558`).

**a5 is a good SENSOR with no actuator of its own.** The place to use it is a Φ row,
and nothing has tested that. **No cause is recorded for it coming back.**

### 4. Mode frequencies re-measured, and THE TWO TILTS SWAPPED

**0.71519 / 0.99231 / 1.65307 Hz**, 16:25, 300 s, 105 013 samples, no drive,
consensus over a0-a3 which agree to **0.00050 / 0.00050 / 0.00000 Hz**. Shifts
against 2026-08-06: **−0.00675 / +0.00038 / −0.00300 Hz**. **a5 deliberately
excluded** -- its mode-C peak is **0.0345 Hz out, 41 half-widths**. Alive is not the
same as belonging in a consensus.

**AND THE TILT LABELS REVERSED:**

| | 2026-08-17 | 2026-08-20 |
|---|---|---|
| lowest mode | 0.7283 Hz -> **T1**, 90.3 % | 0.7264 Hz -> **T2**, 81.6 % |
| middle | 0.9941 Hz -> Z, 88.2 % | 0.9828 Hz -> Z, 73.7 % |
| highest | 1.6530 Hz -> **T2**, 89.4 % | 1.6665 Hz -> **T1**, 89.9 % |

Recomputed directly from the 16:25 record at the **consensus** frequencies over
±0.02 Hz, the same reversal and the same sizes: **T2 81.9 % / Z 73.9 % / T1 90.2 %**.

**The geometric Φ's two tilt COLUMNS are therefore exchanged** in `osem.eta.py`,
`osem.theta.py` and `status.py`. Z is unchanged.

**Every per-mode statement in this file that names a tilt was written under the old
labels** -- § *On the bench, 2026-08-17 (evening)* § 9 and `CLAUDE.md`
§ *Per-mode residual*. **Those measurements are still valid; their LABELS are not**,
and nothing has re-derived which physical tilt each conclusion belongs to.

### 5. The corner assignment survives; the warp grew

Candidate warp vectors on the 16:25 record, counts rms: **`a0+a1 vs a2+a3` 48.17,
`a0+a2 vs a1+a3` 20.09, `a0+a3 vs a1+a2` 12.61**. `[+1,−1,−1,+1]` is still smallest,
so **a0 is still diagonal to a3** -- but it wins by **1.6x** where it won by **2x**.

**Warp GREW: warp over the loudest rigid DOF, 0.139 -> 0.262.** Recorded as
unexplained when measured, and **RESOLVED later the same day: warp is per-sensor
gain mismatch**, so a growth across a re-seating is a change in the sensor gains.
The plate is 2-inch steel (rig owner), so its warp is far below the noise floor
and `warp = 0` is a HARD constraint, one equation per coil -- 8 equations in 4
unknowns, over-determined, against the 3-in-4 fit that failed on 2026-08-17.
Fitted `h = 1/g = [1.6057, 1.0667, 0.4761, 0.8514]`, which takes rms warp over rms
response **0.6224 -> 0.0322, a 19.3x reduction**, and correlates **+0.960** with
`1/|diagonal slope|` that it never saw. MEASURED BUT NOT SHIPPED: folding `h` into
Phi's rows alone took cond 3.32 -> 9.41 and chi2/dof 3.47 -> 844.70 and the run
pumped, because `KALMAN_R` is a variance in RAW volts -- applying it properly needs
the READINGS normalised and `R` rescaled by `h^2` together. See `CLAUDE.md`
Sec *WARP IS CALIBRATION*.

### 6. Coherence re-measured, and the caveat IS the result

24 Hann segments of 8192, bias floor **4/24 = 0.167**, raw -> bias-corrected, with
the in-band rms:

| ch | raw | corrected | in-band rms |
|---|---|---|---|
| a4 | 0.240 | **0.09** | 0.115 counts |
| a5 | 0.751 | **0.70** | 0.237 counts |
| a6 | 0.271 | **0.12** | 0.038 counts |
| a7 | 0.209 | **0.05** | 0.034 counts |

**THIS IS NOT EVIDENCE a4/a6/a7 WENT BLIND**, and recording it as such would be the
**fifth** instance of the error family `CLAUDE.md` § 6 documents four of. Their
in-band rms is **0.034-0.115 counts, at or under ADC dither** -- a coherence
estimate on quantisation noise. The 2026-08-17 numbers (**0.46 / 0.26 / 0.50**) came
from a different record with different ambient and **nothing reconciles the two.**
Both are single ambient records. **a5's 0.70 is the one number that moved for an
understood reason.**

### 7. TWO BIAS RAISES FAILED, and a centring vector is a negative result

**ATTEMPT 1, midday: 1.10 V on all eight coils at once.** **Every analog input went
to zero and STAYED there** through reverts to 0.50 V and then 0.25 V --
`data/20260820_125342_status_sensors.csv` and `data/20260820_125525_status_sensors.csv`
read **mean 0.0, std 0.00 counts on all eight** over 38 726 rows each. The board kept
streaming at **822 Hz with zero dropped frames**, so **the transport was healthy and
the front end was not.** The coils were reset by hand and re-seated.

**ATTEMPT 2, evening: 0.75 V bias with `BIAS_SWING` also raised 0.25 -> 0.75.** The
swing is what matters -- it sets how much current a coil can pull, and tripling it
triples the demand. **The rig owner stopped the run with the power supply audibly
alarming at its limit**, and `data/20260820_171242_fast_lock.csv` agrees:

- common-mode rms over all eight sensors **15.4 -> 132.3 counts**;
- on **a4-a7, at ZERO gain with coils pinned at exactly 0.750 V for every sample**,
  **78-83 %** of their variance was that common mode. **Channels nothing drives
  cannot move mechanically** -- that is the shared rail feeding the OSEM LEDs;
- every DC level fell **together** against the census 47 minutes earlier:
  **−185 −197 −286 −251 −247 −242 −238 −237 counts**.

**`BIAS` now ships at 0.50 V uniform with `BIAS_SWING` back at 0.25**
(`osem.eta.py:194`, `osem.eta.py:428`), and **`VMIN, VMAX` is now `0.0, 2.5`** where
this repo elsewhere still says `0.0, 0.5`. **0.50 V IS NOT ESTABLISHED AS SAFE** --
it sits between 0.25 V, which has run for months, and values measured to fail. The
test is 30 s at bias with the gains at zero, watching common-mode rms and whether
the DC levels fall as a block.

**A PER-CHANNEL CENTRING VECTOR IS A NEGATIVE RESULT.**
`[0.250, 0.494, 0.750, 0.750]` was solved from the DC matrix to pull a2 off the top
rail. **Stepping coils 2/3 to 1.25 V did NOTHING**: a2 sat at **845 / 857 / 859 /
861 / 860 counts across five steps** where the linear model predicted **762**, and
its railed fraction went the wrong way, **5.8 % -> 10.8 %**. **The DC matrix does not
extrapolate past the 0.10-0.40 V range it was fitted over.** The vector survives as
`status.py:187` `BIAS_CH`; **the record of the five steps is not named anywhere in
this repo.**

### 8. The 17:12 run PUMPED, and it is a PHASE error, not a sign error

**A DIAGONAL run** -- `[modal] REFUSED -- running DIAGONAL`, no `data/modal.json`,
the only A available being the provisional DC fallback. 20.0 s CALIBRATING, then
**DAMPING from 20.01 s to 111.21 s (91.2 s) with ZERO FAULT samples.**

Dissipation measured directly as `slope x (u − bias) x velocity`, band-limited
**0.5-2.0 Hz**, common mode removed. Fraction of steps on which the applied force
was dissipating:

| | a0 | a1 | a2 | a3 |
|---|---|---|---|---|
| dissipating steps | **24.2 %** | **41.4 %** | **49.1 %** | **46.1 %** |

**WITHDRAWN THE SAME SESSION, AND THE NUMBERS ABOVE ARE WRONG.** The
discriminator is right -- a wrong sign gives ~0 %, a 90 deg lag gives 50 % -- but
the fractions were computed on **WIRE-RATE rows**. The CSV logs every sample at
581 Hz while `bp`, `vel` and `out` change once per control step at 100 Hz, so in
the file they are **staircases**, and correlating against a staircase understates
the velocity term. Re-evaluated at the control clock, same file:

| | a0 | a1 | a2 | a3 |
|---|---|---|---|---|
| dissipating, wire rate (WRONG) | 24.2 % | 41.4 % | 49.1 % | 46.1 % |
| **dissipating, control clock** | **5.4 %** | **11.0 %** | **22.6 %** | **12.3 %** |
| corr(u, vel) | +0.919 | +0.624 | -0.678 | -0.623 |
| corr(vel, d(bp)/dt) | +0.782 | +0.703 | +0.722 | +0.763 |

**5-23 % IS AN INVERTED SIGN.** The last two rows also kill the hypotheses built on
the wrong numbers: the Kalman filter WAS producing velocity (0.70-0.78) and the
control law WAS feeding it back (0.62-0.92). **`slope x gain` was negative on all
four channels**, where dissipation requires it positive. Correcting it took the
dissipating fraction to **93.0 / 90.1 / 50.4 / 96.6 %** and printed `LOCKED`
(`data/20260820_180344_fast_lock.csv`). See `CLAUDE.md` § *THE GAIN SIGN WAS
INVERTED ON EVERY DRIVEN CHANNEL*.

Differential rigid DOF, common mode removed, CALIBRATING -> DAMPING, counts rms:

| | Z | T1 | T2 | WARP |
|---|---|---|---|---|
| CALIBRATING | 9.47 | 48.11 | 10.82 | 7.76 |
| DAMPING | **46.52** | **131.85** | **67.05** | **53.68** |

**WARP pumped 6.9x and no gain can damp it, because it is not a rigid-body DOF.**
**a2 railed 21.2 % of DAMPING** -- 15.4 % at the top rail, 5.8 % at the bottom,
counted from the raw counts in the CSV.

**TWO THINGS THIS RUN DOES NOT ESTABLISH.** It does **not** show the diagonal law
pumps in general: it ran at 0.75 V bias with a tripled swing, on re-seated coils, at
226 Hz. And **no decay measurement is obtainable from it**, so the kick test is no
closer.

### 9. The cause: the sample rate halves under load, and the old wire budget was wrong

**MEASURED: 351.9 -> 226.5 Hz at the exact instant DAMPING began**, and it stayed
there. Gap **p50 5.40 ms, p99 5.60 ms, ZERO gaps over 50 ms across 91 s** -- a clean
step is **contention**, not supply jitter, which would show as tail.

**THE MECHANISM, and the first reading of it was wrong.** It is **NOT** that `SET`
competes with the stream for wire. **The UART is FULL DUPLEX**, so host->board bytes
cost the board CPU time to parse but take **no** board->host bandwidth -- `pyDAC2`'s
own header has said so since 2026-08-06. **What binds is that the ASCII stream is
already nearly the whole downstream on its own.** A row measured off the wire is
`563,632,914,668,670,534,588,586\r\n`, **exactly 32 bytes**, so 352 Hz is
**11.3 kB/s against 11.52 kB/s at 115200 8N1 -- about 98 %**. The board has no
slack, and once four coils start writing it must **also** parse ~400 `SET`/s and
format eight integers per sample. **The sampling loop is what gives.**

**`CLAUDE.md`'s wire-budget paragraph counted only the `SET` traffic and has been
corrected.** `osem.zeta.py:1376` and `osem.epsilon.py:821` still count only that
direction. `pyDAC2.py` cited the same 50 kB/s figure and **was corrected on
2026-08-20** to the measured 11.52 kB/s each way, full duplex.

**THE FIX IS BUILT, IT NEEDED A REFLASH, AND IT IS NOW ON.**
`BINARY_TRANSPORT = True`: a fixed **20-byte frame -- 7.0 kB/s at 352 Hz, 61 % of
the link instead of 98 %** -- costing no `itoa` per channel per sample, and `seq`
makes lost frames **countable**.

Probed BEFORE flashing, **`MODE BIN`, `MODE ASCII`, `VER` and `INFO` each returned
`ERR unknown command`**, with **no 0xA5 0xC3 sync in a full second** after forcing
`MODE BIN` -- **the board was running an older sketch than `arduino.ino`**, which
the baud had already implied. After `bench.py --flash` the probe returns
`OK mode=bin` and `INFO nch=8 mode=ascii presc=32 ack=1 frame=20`.

**MEASURED AFTER: 581 Hz, HELD THROUGH DAMPING**, where ASCII stepped
351.9 -> 226.5 Hz the instant the coils started writing. `data/20260820_180344`:
41 352 raw samples, wire 581 Hz, 5.8 averaged per control step, **0 frames dropped,
0 torn rows**.

**A SECOND BUG BLOCKED IT AND FAILED SILENTLY.** `stdlib.SampleGuard.read` was
**ASCII-only** while the binary framing lived in `pyDAC2.FastDAC._read_binary`, and
every controller reads through `SampleGuard`. Turning the transport binary left the
controller decoding frames as UTF-8: `read` returned `None` forever, the CSV got a
header and **no rows**, and the controller sat in `CALIBRATING` printing nothing --
the **fourth header-only recording** in this repo. Fixed by delegating to
`FastDAC.read_sample`, with selftest coverage.

### 10. The runaway breaker did NOT trip on kicks that killed the 2026-08-18 runs

**The breaker was restructured**: `~receded` moved **out of the latch and into
`run_trip` as a VETO**, so a recession from the envelope's own recent peak now
vetoes the trip rather than resetting the sustain (`stdlib.py:1183-1305`, marked
*"WHOLE OF THIS CHANGE -- 2026-08-20"*). And **the breaker and the fault-clear gate
now scale off `max(baseline, the running loop's own measured floor)`**, while the
gain schedule and `LOCKED` still scale off the zero-gain baseline:

    [quiet] the running loop's own floor is measured: ch0=0.3598V (1.44x
    baseline), ch1=0.5161V (1.94x baseline), ch2=1.1545V (4.52x baseline),
    ch3=0.8709V (3.19x baseline) -- 20th percentile over 30s of DAMPING.

**THE EVIDENCE IS ONE RUN.** Four hand kicks over 91.2 s of DAMPING; per-channel
`ratio` reached **5.69** at t = 30 s and **6.80** at t = 70 s against a **1.8x**
breaker; **the run never left DAMPING.** Both 2026-08-18 runs died on their **first**
kick at **3.8x**.

**IT IS NOT A DEMONSTRATION THAT A FAULT CLEARS.** No fault occurred, so the clear
gate was never asked to open, and **no clear time exists**. `CLAUDE.md` § 1 stays
open on exactly that.

### 11. The hybrid PID channels have STILL never run -- three runs, two sessions

    !! demoted: ch4, ch5, ch6, ch7 -- raw counts std 1.304 / 1.815 / 0.971 /
       1.056, under the 1.30-count dead-pin line.

All four sat at `NOSIG` for the whole run, commanded output **exactly 0.750 V,
min = max, over every DAMPING sample**, so `|gain|` was again exactly 0.0000.

**The demoting check was `dead-pin` this time, not `inband-floor`, and that is a
different thing.** `dead-pin` grades a channel on **whether it MOVES**, in raw
counts, which is the right test for a sensor whose axis has no Φ row. **A defensible
refusal is still an unmeasured hybrid PID.**

**An unexplained tension: a5's std was 1.815 counts at engage and 8.02-9.16 counts
on the quiet censuses**, with `DEAD_PIN_STD_COUNTS = 1.3` sitting in the middle of
that gap.

### 12. Tooling

- **`status.py` gained a per-channel bias**: `BIAS_CH` (`status.py:187`) and
  `bias_of()` (`status.py:190`), so a DC pass no longer assumes one bias for all
  eight coils.
- **A LIVE DEFECT WAS FIXED IN THE EXPORT PATH.** `a_from_dc` silently fell back to
  **`DC_SEED`, the hardcoded 2026-08-04 matrix** (`status.py:343`), because
  `_EXPORT` never carried a `"dcm"` key -- so a run that had **just measured** its own
  DC matrix could export an A built from a two-week-old constant and say nothing.
  `_EXPORT["dcm"]` is now populated (`status.py:3626`) and read (`status.py:3163`).
- **`SIGTERM` and `SIGHUP` are routed to the park path**, so a plain `kill` returns
  the coils to bias. The run announces it. **Nobody has killed a run and read the
  coil voltages back**, so the handler is claimed, not verified.
- **`pyDAC2.BAUD` and `arduino.ino`'s `BAUD_HZ` are now 115200**, both having been
  500000, and **`BAUD_CANDIDATES` tries 115200 first** (`pyDAC2.py:82`).

### What this session did NOT establish

**A -- signs OR magnitudes**, since every determination predates the re-seating and
nothing has re-measured it. **Whether 0.50 V of bias is safe on the 3 A supply.**
**The vertical mode frequencies**, still never measured. **Whether the hybrid PID
helps**, still never run. **The χ²/dof distribution** -- the run was diagonal, so
the modal Kalman path did not execute at all. **Any decay rate**, because the run
pumped. **Whether a fault clears**, because no fault occurred. **Why warp grew**,
**why a5 came back**, and **which physical tilt each earlier per-mode conclusion
belongs to.**

`osem.eta.py` still declares `BENCH_STATUS = "untested"` and has now run three
times; `osem.theta.py` declares the same and it is correct.

---

## eta — the modal velocity comes from ONE filter over the modal coordinates

`osem.eta.py`, written 2026-08-17. It is `zeta`'s modal law with `zeta`'s gains,
`zeta`'s refusal ladder and `epsilon`'s supervisor, unchanged, and **one thing
replaced: the estimator behind the modal velocity.** A bench run therefore compares
two *estimators* and not two gain vectors.

**It ran on 2026-08-18** — § *On the bench, 2026-08-18*. **The file still declares
`BENCH_STATUS = "untested"`, which is now wrong.**

**The 0.08 median `ratio` measured on 2026-08-17 is `zeta`'s, not `eta`'s** —
`signtest.py` (since deleted) launched `bench.py zeta`. **`eta`'s first runs did not
reproduce or refute it**: the modal run held at per-channel `ratio` **0.117** in its
quiet periods, about **13x** below the diagonal run's 1.449 / 1.832 / 1.769, but it
faulted at 38.5 s of 133 s and never recovered, so there is no 70 s scored window to
compare with. See README § 8 for the signtest table and its caveats.

Run it with `make run V=eta`. `python3 osem.eta.py --selftest` is where the modal
math and the modal filter are asserted; `make check` cannot reach either, because
`sim/server.py` models **two** modes and this runs **three**.

### What changed against `zeta`

**1. `ModalKalman` — 14 states, and Phi is now inside the estimator.**

    x    = [q_A, v_A, q_B, v_B, q_C, v_C, d_0 .. d_7]      2x3 modes + 8 sensor DC
    y_i  = SUM_m Phi[i,m] q_m + d_i + n_i                  n_i ~ N(0, KALMAN_R[i])
    qdot = [v_A, v_B, v_C]

`zeta` estimated each mode by an inverse-variance scalar least squares across
**eight independent 7-state per-sensor filters**; `eta` runs **one** filter whose
state *is* the modal coordinates, plus one DC/drift state per sensor because each
OSEM has its own slowly-moving rest offset. Same plant assumption as `epsilon` and
`zeta` and it is measured: **Q > 433 at 1σ, τ > 138 s** (`analysis/ringdown.md`)
against a 2.9–4.7 s closed loop, so the modes are undamped oscillators on the
timescales the filter runs at.

**A demoted, railed or absent sensor is a DELETED ROW, never a re-weighting.** It
leaves `H`, it leaves `R`, the dof falls by one, and nothing else changes. A
sensor that comes back re-primes its own DC state (its covariance row and column
cleared, reset to `R_i`) and the modal states are untouched, because the other
sensors were measuring them throughout. That is the property row deletion buys and
channel-blanking does not, which is why `RAIL_BLANK` stays `False`.

**No new tuning constants.** `R` is the measured `KALMAN_R`; the per-mode process
noise is `q_modal[m] = SUM_{i: rowok[i,m]} KALMAN_Q[i,m]` — 24 measured numbers and
`T_AMP_S`, with the `w_m^2` cancelling exactly because Phi's columns are unit-norm
— and `Q_DC = KALMAN_R / T_DC_S` as before.

**The gain is NOT precomputed, unlike `KALMAN_K`, and cannot be.** `K` depends on
Phi, which arrives at *run time* from `data/modal.json`, and on *which sensors are
live*, which changes mid-run. So the filter carries its own covariance, in
**Joseph form** — `(I-KH) P (I-KH)^T + K R K^T` — because the short form is only
PSD at exactly the optimal gain and this loop runs for hours with rows appearing
and disappearing. There is deliberately no flag to switch it.

**2. chi² per degree of freedom — the residual becomes calibrated.**

    e    = y_live - H xpred
    S    = H Ppred H^T + diag(R_live)
    chi2 = e^T S^-1 e ,   dof = |live|

`zeta`'s out-of-mode residual can only say "the sensors disagree by this
fraction"; `S` is the filter's own belief about how big its innovations should be,
so **chi²/dof is order 1 exactly when the readings fit three oscillators at
`F_MODE_HZ` to within the MEASURED per-channel noise**, and it says so *in sigma
of measured noise*. It is the check that catches a bad sensor which is neither
railed nor signal-less — the failure every other interlock here is blind to.

**It is LOGGED AND NOT ACTED ON, deliberately.** The distribution on this rig has
never been measured, so any threshold would be a guess, and adding an unmeasured
fault source to a rig whose fault history is the main thing wrong with it is the
trade `analysis/mimo_closed.md` § 4.5 already refused. `eta` adds the columns
(`chi2`, `ndof`) on every CSV row, accumulates the distribution **per state** —
`CALIBRATING` is undriven and is the cleanest population a threshold should be
derived from — and prints a summary at exit. **That distribution is the
deliverable of the run**; the lock time is `delta`'s and is unchanged.

**3. Both estimators are carried, and the per-sensor one is not a fallback.** It
owns `bp = displacement()`, and `bp` is what the gain schedule, the runaway
breaker, the lock detector and `baseline-floor` all read — so it owns **every
per-channel interlock and the baseline**. That is why `_fingerprint` is unchanged
and a `zeta`-measured floor is legitimately reusable here, and it is what makes
the fallback cheap: with Phi refused the modal filter is never built at all
(`self.mkf is None` *is* the fallback) and what runs is `epsilon`'s diagonal law on
`epsilon`'s estimator. `zeta`'s least squares also still runs, logged only, as
`qls*`, so one bench run compares the two estimators on identical samples.

### What it costs, stated rather than buried

`Modal.sense`'s docstring is explicit that `zeta` avoids any matrix inverse **on
purpose**, so that `cond(Phi)` is not load-bearing in the estimator. **`eta`
reverses that**, and the reversal is the whole risk: a wrong Phi now gives a wrong
modal velocity with no per-sensor modal velocity to retreat to. Two mitigations,
both in the file: `zeta`'s refusal ladder kept whole, and **chi², which is the
statistic that detects a wrong Phi**. The new risk and its detector arrive
together. The only thing inverted is `S = H P H^T + R`, which is `|live|×|live|`
and bounded below by **measured** `R` rather than by a ridge somebody chose
(`mimo_closed.md` § 3.5 had to pick `eps = (2 σ̄)²` by hand for exactly this).

A Kalman filter is also **more** sensitive to a wrong frequency than a bandpass
is, because modelling the oscillator is the point of it. `F_MODE_HZ` is the
re-measured 2026-08-17 set and `Modal._load` enforces 0.005 Hz against the file.
**How much chi² rises per Hz of frequency error is NOT established** — no
measurement of it exists here or in `analysis/kalman.md`, and the selftest does not
cover it.

### `dead-pin` — the one interlock change, and it is why a bench run is possible

A rail during `CALIBRATING` is a whole-rig fault by design, correctly: it normally
means the optic is against a mechanical stop. But on 2026-08-17 **a5 was measured
to be a disconnected pin** — exactly one distinct value, 0.0 counts, variance
exactly zero, over three records totalling 236 387 samples — which is below
`RAIL_LOW = 12`, and **the rig sat in FAULT for 50 s with every gain at zero
because of one bad solder joint**. `ENABLE_CHANNEL[5] = False` handled a5.

**SUPERSEDED 2026-08-20 on a5 only: it is ALIVE** -- std 9.16 counts, 36 distinct
values, coherence with a0-a3 0.00 -> 0.70, and `ENABLE_CHANNEL[5]` is back to
`True` (§ *On the bench, 2026-08-20*). **`dead-pin` itself is unaffected and is what
demoted ch4-ch7 on the 2026-08-20 run.**

**a6/a7 are the case that config cannot handle.** At 17:59 they sat *bottom-railed
at 4.3 and 8.8 counts with std 0.48–0.56*, and at 18:20 they read 591.3 (std
24.23) and 566.0 (std 17.76) — they came back, and **no cause is recorded
anywhere**. So their state at the start of a session is unknown, and while they are
railed neither `zeta` nor `eta`-as-first-written gets past calibration.

The test is: **railed AND essentially motionless → demote that one channel; railed
and still moving → fault the whole rig, exactly as before.** What changed on
2026-08-17 is *which statistic*, and the reason is measured:

- **The first version used peak-to-peak SPAN over 64 control samples of the
  decimated mean, against 1.0 count**, on the argument that a5's variance is
  exactly zero so any sub-LSB number would do. True of a5, false of a6/a7.
- **A bigger threshold could not fix it.** Replaying
  `data/20260817_175912_status_sensors.csv` (26 092 samples, 60.0 s, 434.85 Hz
  wire) through `eta`'s own decimation and mains-null, span per 64-sample window:
  a6 **0.925 / 1.292 / 2.900** and a7 **0.900 / 1.583 / 4.625**
  (median / p99 / MAX), against a0–a4 at 95.5–142 median. The simulator's occluded
  channel — the only *railed-but-moving* example in evidence — runs **2.71 / 3.96 /
  5.17** on the same statistic. **Those overlap**, and the overlap is decisive
  because `rail_now` is evaluated every control step: one window over the line
  faults the rig.
- **The statistic is now the std of the RAW counts over `RAIL_SUSTAIN_S`, at the
  wire rate** — the rail interlock's own window, rate and samples. A span is set by
  the single most extreme sample in its window; a std over the 217–555 samples that
  window holds concentrates to 3–5%, which is what turns a 4× difference in σ into
  two populations that do not touch. It is also available exactly when `rail` arms
  and not one sample later.
- **Both edges of the threshold are measured on that statistic.** Bench, 8625
  windows: a5 exactly **0.0000**, a6 **0.377 / 0.497 / 0.621**, a7 **0.263 / 0.407
  / 0.853**; live a0–a4 never below **4.80** in *any* window. Simulator's occluded
  channel, 2257 windows: **1.998 MIN**, 2.397 median. Threshold =
  `sqrt(0.853 × 1.998) = 1.31` → **`DEAD_PIN_STD_COUNTS = 1.3`**: 1.52× above the
  loudest dead window measured, 1.53× below the quietest moving one, and 3.7× below
  the quietest live window.

Two guards on top, both because this test must not weaken the fault path:

- **A witness is required.** If the demotion would leave *no* enabled channel
  behind, nobody is demoted and the rail fault stands — every channel railed and
  flat at once is a dead ADC, an unplugged loom or an optic hard against a stop,
  not eight dead pins. Same shape as `lock-quorum` and `runaway-quorum`.
- **It is announced loudly, once per channel, with the measured std**, so the
  number to change is in the log rather than in somebody's head.

**Known limit, stated rather than discovered later:** this is a test over *one*
rail window, so an optic parked hard against a stop for longer than
`RAIL_SUSTAIN_S` with under 1.3 counts of its own noise is, over that window,
indistinguishable from an unwired pin. What covers that case instead is
`baseline-floor`, which demotes a channel that calibrates a floor under 10% of the
median, and the witness rule, which faults if the condition takes every channel
with it.

**`osem.zeta.py` still carries `DEAD_PIN_SPAN_COUNTS = 1.0`** and was deliberately
not touched. If `zeta` is run tonight with a6/a7 bottom-railed, it will fault
through calibration.

### One more thing to check before every modal run: the export itself

`eta` loaded and gated both 2026-08-17 exports of `data/modal.json`, and **they do
not agree**:

| written | source | verdict |
|---|---|---|
| 19:26:47 | `20260817_182701_status_coils.csv` | **LOADED** — all 4 coils kept, full mask cond **1.97**, rank 3, 80 of 256 masks run MIMO. Off-quadrature 0.736 / 2.088 / 0.812. This is the pair `signtest` scored **0.08** on |
| 20:05:41 | `20260817_194726_status_coils.csv` | **REFUSED** — only **2 of 4** coils pass the colocation sign check (`MODAL_MIN_COILS = 3`); coils 1 and 2 dropped on `A/Phi` sign disagreement. Off-quadrature **1.662 / 0.737 / 1.181** |

So the refusal ladder is **live**, not decorative, and "there is a `data/modal.json`
on disk" is not the same statement as "this is a modal run". Read the `[modal]`
banner; it prints before any coil is energised, and `Modal.report()` prints Φ, A,
the surviving coils, the cond per mask and the per-mode gain with it.

### `--selftest`, 2026-08-17 — all pass

Every check asserts a *planted* property against a plant known exactly; nothing
asserts what the code printed.

| | measured |
|---|---|
| filter shape | 14 states = 2×3 modes + 8 sensor DC |
| `q_modal` = summed measured modal variances | `[0.00491, 0.55021, 0.10104]` |
| exact-ZOH conserves oscillator energy, 1e5 steps | drift **1.03e-11**; a Euler step at the same dt drifts **3.3e+89** |
| recovers a planted modal velocity, all rows | rel err ≤ **0.0042** |
| …with a0 deleted / a0+a1 deleted | **0.0042** / **9.7e-4** |
| …with two rows leaving mid-run at t=30 s | **6.8e-4**, still finite |
| chi²/dof when the model fits | mean **1.0111**, max 4.71 over 6 dof |
| P stays positive definite (Joseph form) | min eigenvalue **9.1e-07** |
| …and stays order 1 as rows go | 5 rows **1.006**, 4 rows **1.011** |
| 3σ / 5σ extra noise on one sensor | **2.341** / **4.754** |
| one sensor's gain wrong by 1.5× | **38.4** |
| one sensor's sign inverted | **595.8** |
| one sensor stuck at a constant | **149.6** |
| the same fault on a quieter sensor | a0 (√R 0.068 V) **2.00** vs a3 (0.007 V) **38.37** |
| a STATIC offset is invisible — the DC state absorbs it | **1.0111** against 1.0111 baseline. Designed, not a gap |
| a STEP offset is visible, then absorbed | **3.681** for 0–40 s, **0.985** after |
| the filter's own force dissipates against TRUE qdot | mean f·qdot **−1.43e-01**, negative on **100.0%** of steps |
| cost of one modal step | **84.9 µs = 0.85%** of the 10 ms period (asserted < 20%) |
| a6/a7 bottom-railed as measured | std 0.624 / 0.575 counts → both demoted, **0 faults**, still `CALIBRATING` |
| the running-sum std vs a direct one | agree to **8.9e-13** over 551 samples |
| a railed channel that is still MOVING | std 152.4 counts → **1 fault**, state `FAULT` |
| every enabled channel railed and motionless | **1 fault**, **0 demotions**, no-witness announced |
| every refusal (schema, frequencies, age, gauge, provisional A, sign-flipped A, missing A, missing file) | refuses |

Those chi² figures are **synthetic** — the readings are generated from the filter's
own `R` and `Q`, so order 1 is order 1 *by construction*. **What chi²/dof
distributes as on the real rig is unmeasured, and that is exactly why nothing in
the file acts on it.**

### Suite

`python3 harness.py --test eta` → **41 passed, 10 failed** (181–228 s, 2026-08-17),
and **`zeta` is 41/10 with a byte-identical failure set**: the rigid-body
undriven-decay check, 4 in the wrong-sign / `soft-saturation` / `saturation-latch`
group, `baseline-floor`'s known out-of-band-interference limit, `fast-refault`,
2 × `warm-restart`, and the baseline-reuse bound. Verified again *after* the
`dead-pin` change: same 41/10, same set, and `a rail during CALIBRATION is still a
whole-rig fault` still reports **1 fault, 0 demotions, state=FAULT**. The fix does
not swallow the case it must not swallow, and that is measured rather than argued.

The 10 failures are **unattributed** and are the same ones `zeta` carries; neither
has ever been committed, so there is no baseline to diff against. Commit, then
bisect. Note that these are the eight-OSEM body, so `eta`'s *supervisor* and
*diagonal* law are simulated; its **modal** path is not reachable from
`make check` at all (two modes in the simulator, three in the controller).
