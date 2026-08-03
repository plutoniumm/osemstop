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
| **v0** | ch0 only | zeroed | all 3 intact | 0.197 | 12.0 s | 18/18 |
| **v1** | all four | zeroed | all 3 intact | 0.021 | 10.7 s | 16/16 |
| **v2** | all four | **live** | all 3 intact | 0.024 | 10.5 s | 18/18 |
| **v3** | all four | live | **all 3 fixed** | 0.024 | 10.6 s | 18/18 |

"ratio" is the rolling 2 s RMS of the bandpassed signal over that channel's own
calibrated baseline — 1.0 is undamped, the lock line is 0.35. Every number in
this file was measured through `harness.py`, not estimated. The simulator's
limits apply to all of them: see the bottom of this file.

**Re-measured on the rigid-body plant.** These four rows, and the
`runaway-baseline` table further down, were previously 0.254 / 13.6 s,
0.254 / 15.0 s, 0.232 / 14.0 s, 0.230 / 14.1 s — measured when the plant was
four independent oscillators. With one rigid body every coil pushes on the mass
all four OSEMs watch, so the four-channel versions damp far harder than they
used to and v0's single channel drags the other three down with it. Anything
else in this file quoted per channel (the v1 section's 0.254 / 0.249 / 0.152 /
0.275 and its "ch2 damps fastest") is from the same old plant and has **not**
been re-measured; on the current one v1 reads 0.021 / 0.116 / 0.066 / 0.144 and
ch0 is flattest. None of this is hardware — see the bottom of this file.

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

| sensor noise | v0/v1/v2 arms after | v3 arms after |
|---|---|---|
| 0 mV | 0.51 s | 0.40 s |
| 5 mV | 0.51 s | 0.40 s |
| 10 mV | **never** | 0.40 s |
| 20 mV | **never** | 0.40 s |

**3. `runaway-baseline` — improved, not solved. Read this one.**
v0 measured the baseline as one RMS over a single 8 s window and compared a 2 s
RMS against it for the rest of the run. v3 calibrates for 20 s and takes the
**median of 5 sub-windows**, so one bad sub-window is outvoted.

Measured on the **rigid-body plant** (simulator only, never hardware): one 6 V/s
velocity impulse inside the calibration window, worst-channel baseline skew
against an otherwise identical unshocked run, as a function of *when* it lands:

| shock at | 1 s | 2 s | 3 s | 4 s | 5 s | 6 s | 7 s | **worst** |
|---|---|---|---|---|---|---|---|---|
| **v0/v1/v2** | 40.1% | 45.3% | 29.3% | 28.7% | 16.5% | 19.7% | 11.2% | **45.3%** |
| **v3** | 20.4% | 25.0% | 22.4% | 18.8% | 25.8% | 22.1% | 19.6% | **25.8%** |

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

The plant, though, is modelled: a linear second-order oscillator per axis. So:

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
3. **One rail freezes all four channels.** Not per-axis. All four OSEMs sit on the
   same rigid body, so one blind sensor means the optic's position is untrustworthy
   globally.
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
   and log — they just never actuate. They can still trip the global interlock.

---

## On the bench, 2026-08-03

First hardware runs since v0. Board on `/dev/cu.usbserial-1120` (CH340), firmware
`arduino.ino` streaming 8 ADC columns at ~348 Hz, coils on the remapped
`DAC_CHANNELS = [1, 3, 5, 7]`, **ch0 only** enabled on every run.

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
