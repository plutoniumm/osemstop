# Controller versions

Each `osem.vN.py` is a **complete, standalone controller**. None of them import
from each other and none are patches — you can flash the Arduino, point any one
of them at `COM7` and run it. That is deliberate: the bench comparison between
two versions has to be a comparison of two things that both actually ran.
(`osem.v6*.py` are the exception and are not controllers at all — see *Bench
tools* below.)

**v0, v1, v2 and v3 were deleted from the working tree on 2026-08-04** and exist
only in git history: `git show HEAD:osem.v0.py`, or `git checkout HEAD -- osem.v0.py`
to bring one back. Their sections below are kept, because v4 is v3 plus one
change, because the suite still measures itself against their behaviour, and
because the hardware numbers in § *On the bench* were measured on them and are
real. **v5 is the current starting point** — the newest thing that closes a loop
on four channels, and what `README.md` § 0 tells you to run; v4 is the reference
it is compared against.

```
python3 harness.py --list          # what exists, controllers and bench tools
make test                          # pick one, watch it run, press x to kick it
make check                         # every version the simulator can drive, asserted
make sim V=v4                      # one of them in a browser
```

---

## The ladder

In the tree, as of 2026-08-06.

| | channels | I / D | fixes | on bench | suite |
|---|---|---|---|---|---|
| **v3** | 4/4 | live | all 3 defects fixed | validated 08-03 | 24/24 |
| **v7** | 4/4 | live | + auto-disable, fast-refault, **bias-trim** | validated 08-04 | 36/36 |
| **v9** | 4/4 | live | + fast-calib, warm-restart, **runaway-trend** | validated 08-04 | 37/37 |
| **v10** | **5/8** | live | v9 + trim + **soft-saturation**, **baseline-floor** | **validated 08-06** | *skipped* |
| **v11** | **8/8** | live | + **sat-window**, **fast-transport** | **untested** | *skipped* |

All five ship the **same control law** — identical `STEADY_GAIN`, `KI_GAIN` and
`KD_GAIN` on a0–a3, ch2's `+0.010` included. Everything that separates them is
supervisor, and from v10 also the channel set. v3 is kept as the minimal baseline
precisely because of that: it is the only one with essentially no supervisor, so a
supervisor change can be A/B'd against it.

**v10 is the one to run on hardware; v11 needs a reflash first.** v10 went on the
bench 2026-08-06 on all eight OSEMs and locked into 40 s of unbroken damping with
zero faults. v11 raises the baud rate to 230400 and switches to `pyDAC2.FastDAC`,
so the board must be reflashed (`make arduino`) before it will run at all.

**v10 and v11 are eight-channel, so `harness.py` skips both** — `sim/server.py`
models a 4-OSEM rigid body. The suite still runs in full on v3/v7/v9 (which is
what the v9 column above is), but the FIXED-side assertions for
`soft-saturation` and `baseline-floor` are now declared only by versions the
simulator cannot drive, so those two fixes have **no simulator coverage** even
though their checks are still in `harness.py` and still assert the un-fixed
behaviour on v3/v7/v9. `bias-trim` is unaffected — v7 still declares it.

`osem.v6.py` is a **bench tool**, not a rung: it drives coils and records, closes
no loop, and exposes no `Controller`. The suite skips it loudly, with a reason
printed.

Deleted, and in git history only. Their numbers were measured through
`harness.py` while they were still in the tree, against the suite as it stood
then — so the counts are not comparable with the table above:

| deleted | channels | I / D | known defects | ch0 ratio | lock | suite then |
|---|---|---|---|---|---|---|
| **v0** | ch0 only | zeroed | all 3 intact | 0.188 | 12.1 s | 18/18 |
| **v1** | all four | zeroed | all 3 intact | 0.025 | 10.7 s | 16/16 |
| **v2** | all four | **live** | all 3 intact | 0.028 | 10.5 s | 18/18 |
| **v4** | all four | live | + auto-disable | 0.029 | 10.6 s | 33/33 |
| **v5** | all four | live | + fast-refault | 0.029 | 10.6 s | 35/35 |
| **v5.5** | **all eight** | live | same as v5 | — | — | *skipped* |
| **v8** | all four | live | + fast-calib, warm-restart | 0.029 | 10.5 s | 36/36 |

v0–v2 went on 2026-08-04; v4, v5, v5.5 and v8 on 2026-08-06, each superseded by a
version that contains it. The `v6.1` / `v6.5` / `v6.6` sysid variants went the
same day — the multisine pair failed on hardware, and the 8-coil pair expects
4 DOF, which cannot be seen until a4–a7 are wired. v3 was deleted on 08-04 and
**restored on 08-06** as the minimal baseline.

"ratio" is the rolling 2 s RMS of the bandpassed signal over that channel's own
calibrated baseline — 1.0 is undamped, the lock line is 0.35. Every number in
both tables was measured through `harness.py`, not estimated. The simulator's
limits apply to all of them: see the bottom of this file.

### Bench tools, not rungs

`osem.v6.py`, `osem.v6.1.py`, `osem.v6.5.py` and `osem.v6.6.py` declare
`KIND = "sysid"`, expose no `Controller` and damp nothing. They drive the coils
with a known excitation and record what the OSEMs do, to recover the actuation
matrix (`research.md` item 2, `CLAUDE.md` item 2b). `harness.py` and `bench.py`
both check `KIND` and refuse to simulate them; the shared lock-in and reporting
live in `sysid.py`. v6/v6.1 are the four-coil pair (stepped sine / interleaved
multisine), v6.5/v6.6 the eight-coil pair.

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

- **v0** — the 2026-07-15 bench configuration and what `provenance.md` describes.
  P-only velocity feedback, `u = -K·v`, `ENABLE_CHANNEL = [True, False, False,
  False]`. The only entry in this whole file ever confirmed against a scope.
- **v1** — one line changed: all four channels on, because `provenance.md` §4
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
(`provenance.md` § *Table 1*). Confirmed on hardware 2026-08-04, where it came
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
is in v4 or v5 — they inherit this fix unchanged. `research.md` item 6 is the
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

A quorum of one is not a guess. `provenance.md` §3 measured a single OSEM/coil
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
to build it (`provenance.md`, *What the report does NOT contain*, and
`research.md` § *Deliberately cut*, where it was ruled out before it was built).

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
needed rethinking: `provenance.md` §3 (one pair damps the whole mass) still
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
in `FIXES`, and the harness picks it up with no other changes — `--list`,
`make test` and `make check` all discover `osem.v*.py` by glob.

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
`BASE_PLANT` — the suite characterises the control law, not the transport — while
`make sim` and `make test` run at the honest default so what you watch is the
loop the hardware actually has.

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
  **inferred** from `provenance.md` Table 1 by subtracting the intrinsic decay
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
  0.01–0.03 against ch0's 0.08 — exactly what `provenance.md` Table 1 predicted,
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

1. **Realign a4, a6 and a7.** They are connected and reading; their flags sit
   outside the partial shadow, so they modulate with nothing (0.1–9% of power in
   band, lock-in SNR 1.1–1.5). `baseline-floor` keeps them from taking the rig
   down, but only a screwdriver makes them useful. a5 is the same problem one
   step less severe — at ~1/12 gain it damps, but it also holds the rig out of
   `LOCKED` at ratio 0.45.
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

### v12 has never announced `LOCKED`, and neither did v10 or v11

The one that matters, because README § 4 says the lock time is the deliverable.
Zero `*** LOCKED ***` lines in **every** run of the 2026-08-06 session.

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

### MIMO is closed, by measurement

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
7. **a4, a6, a7.** Connected, near mid-scale, ~1/100 the in-band gain of a0–a3.
   Cause not established.

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

`_run_as_v12` does `v12.Controller = Controller` — it **mutates the v12 module**.
That is fine for a skeleton driven deliberately and is not the shipping form:
this repo's rule is that every controller is standalone, because `bench.py`
prints the gain vectors out of the file that is about to run, and a controller
importing its body from another file defeats exactly that check. **Flatten it
before treating it as a version.**

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
