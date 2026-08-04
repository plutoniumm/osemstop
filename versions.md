# Controller versions

Each `osem.vN.py` is a **complete, standalone controller**. None of them import
from each other and none are patches — you can flash the Arduino, point any one
of them at `COM7` and run it. That is deliberate: the bench comparison between
two versions has to be a comparison of two things that both actually ran.

`v0` is the frozen baseline and does not change. Everything else is measured
against it.

```
python3 harness.py --list          # what exists
make test                          # pick one, watch it run, press x to kick it
make check                         # all four, headless, asserted
make sim V=v2                      # one of them in a browser
```

---

## The ladder

| | channels | I / D | known defects | ch0 ratio | lock | suite |
|---|---|---|---|---|---|---|
| **v0** | ch0 only | zeroed | all 3 intact | 0.188 | 12.1 s | 18/18 |
| **v1** | all four | zeroed | all 3 intact | 0.025 | 10.7 s | 16/16 |
| **v2** | all four | **live** | all 3 intact | 0.028 | 10.5 s | 18/18 |
| **v3** | all four | live | **all 3 fixed** | 0.029 | 10.6 s | 18/18 |
| **v4** | all four | live | all 3 fixed, + auto-disable | 0.029 | 10.6 s | 28/28 |
| **v5** | all four | live | + **fast-refault** | 0.029 | 10.6 s | 30/30 |
| **v5.5** | **all eight** | live | same as v5 | — | — | *skipped* |

"ratio" is the rolling 2 s RMS of the bandpassed signal over that channel's own
calibrated baseline — 1.0 is undamped, the lock line is 0.35. Every number in
this file was measured through `harness.py`, not estimated. The simulator's
limits apply to all of them: see the bottom of this file.

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

## v0 — the frozen baseline

What ran on the bench on 2026-07-15 and what `provenance.md` describes. Velocity
feedback (cold damping), `u = -K·v`, P-only. `ENABLE_CHANNEL = [True, False,
False, False]`.

It differs from the original `osem_fast_lock (2) (1).py` in exactly two ways,
both behaviour-preserving:

- the PID scaffolding (`KI_GAIN`, `KD_GAIN`, `D_SMOOTH_HZ`, `I_CLAMP_V`) is
  present but **both gains are 0.0**, which collapses the sum to the P-only law
  that was validated;
- `Controller` is extracted from `main()`. Pure refactor. It is what lets the
  simulator drive the real state machine instead of a copy of it.

**Do not fix anything in v0.** Its three defects are the reference point that
makes v3's fixes verifiable, and the suite asserts them from the broken side.

---

## v1 — all four channels

One line changed: `ENABLE_CHANNEL = [True, True, True, True]`.

**Why.** `provenance.md` §4 documents all four channels damping simultaneously on
the bench, with per-channel exponential fits (τ = 4.55, 5.49, 3.58, 5.05 s for
A0–A3) and the optic settling in ≈17 s against ≈70 s for one channel at a time.
v0 shipped with three of those disabled, which predates that run. v0's docstring
claim that "only ch0 is experimentally validated" was two days out of date.

**A2 keeps `STEADY_GAIN[2] = +0.010`,** positive where the others are negative.
Not a typo. The report has A2 damping *fastest* of the four at that gain, which
is only possible if its coil or OSEM is mounted the other way round. The
simulator models this as `COIL_GAIN[2] < 0` (see below).

Measured: all four damp (0.254 / 0.249 / 0.152 / 0.275), locks at 15.0 s, and
ch2 damps fastest — the same ordering the report reports.

---

## v2 — I and D switched on

`KI_GAIN` and `KD_GAIN` given real values. The structure was already there from
v0; until now it was inert.

```python
KI_GAIN = np.array([-0.040, -0.040, +0.040, -0.040])
KD_GAIN = np.array([-0.0015, -0.0015, +0.0015, -0.0015])
```

All three gains follow **one sign convention per channel**, because they share
one loop — negative on ch0/1/3, positive on the inverted ch2. Flipping only Kp
would make the spring repulsive and the mass negative on that channel.

**What they actually are.** The process variable is velocity, so the names
mislead:

| term | on velocity | physically |
|---|---|---|
| P | proportional | cold damping — **the only term that removes energy** |
| I | ∫velocity = displacement | an added **spring**; shifts the resonance |
| D | d(velocity)/dt = acceleration | added **mass**; also differentiates the sensor twice |

So do not expect v2 to damp better than v1 because it has "more PID." It damps
slightly better (0.232 vs 0.254) and locks a second sooner, but neither new term
dissipates; what they buy is a stiffer, heavier effective plant. Measured term
sizes at full swing: **P = 0.154 V, I = 0.066 V, D = 0.045 V** — P still
dominates, which the suite asserts.

`kd` must stay far below `1/k_act`. The D term feeds acceleration back into
acceleration; around kd ≈ 0.05 the effective mass goes to zero and the loop runs
away.

---

## v3 — a proper PID, and the three defects fixed

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

Read the *spread*, not any one column. At t = 5 s or t = 7 s v0 comes out
**better** than v3, so a single fixed shock time ranks them backwards. What the
median actually buys is a **bound**: v0's damage depends entirely on where the
transient lands and reaches 45%, v3's stays inside a 19–26% band. `harness.py`
therefore sweeps the shock across the window and asserts the worst case, with
the line at 35%.

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
or a transient detector that restarts calibration. Neither is in v3.

---

## v4 — graceful degradation

v3 with **one** behavioural change and nothing else touched, so a bench
comparison isolates it. `FIXES` gains `auto-disable`. Identical damping
(ch0 = 0.024, lock 10.6 s) because in a quiet lab nothing ever demotes — the
whole change is dormant until something goes wrong.

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
safety-checks diagram and never implemented in any version (`provenance.md`,
*What the report does NOT contain*).

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

v4 is the reference: its structure matches v0–v3, so a diff against v3 shows
exactly what auto-disable cost, and it is the pre-`fast-refault` behaviour.
**v5 is the one to run and to modify.** Neither has been on hardware.

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
measurement in `CLAUDE.md` §2b produces the real numbers.

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
at 20 mV of noise, where v0 is asserted never to. A version that claims a fix it
did not make fails.

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

It runs the **real** controller — every version above is imported and stepped,
not reimplemented. Only `serial` and `DACController` are stubbed.

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

Two of the three v3 fixes are still unconfirmed on hardware -- see the bench
section below for exactly which. They are verified against a model of the plant
and a faithful copy of the controller, which is enough to say the logic is right
and not enough to say the suspension agrees.

---

## Runtime state and hazards

Condensed from `versions.md`, which was removed on 2026-08-03 — most of it restated
what the code already shows. What survives is the state machine and the hazards.

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
  `bp_out²` per channel. `CALIBRATION_S` is 8 s on v0–v2, 20 s on v3. On exit it sets
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

## On the bench, 2026-08-03

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
