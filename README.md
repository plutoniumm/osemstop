# OSEM suspension damping

A three-layer control stack that damps a suspended optic using OSEM (Optical Sensor
and Electro-Magnetic actuator) shadow sensors. Eight OSEMs are connected; as of
the 2026-08-17 census **five** (a0–a4) carry usable in-band signal, and the four
coils behind a0–a3 push back. The whole thing is one closed loop split across a
language boundary:

```
arduino.ino (Arduino firmware)     <-- SPI --> AD5628 octal DAC --> coils
   ^  |                                                              |
   |  | serial, rate set on the BOARD (115200 on 08-17)              v
   |  v                                                      suspended optic
pyDAC.py (DACController: transport)                                   |
   ^  |                                                               |
   |  v                                                    OSEMs --> A0..A7
osem.<name>.py (control law, safety, logging) <-- analogRead ----------'
```

## What each rung actually measured

Re-judged 2026-08-18 with `jerk.py`'s replay and its state check. **A decay rate is
the only dissipation measurement here** — a `ratio` is an amplitude and a % is a
duty cycle, and the duty cycles this table used to carry measured uptime, not
damping. Where a rung has no number it says so rather than borrowing one.

**Two columns, and they are not the same class of evidence.**

- **Commanded** — `jerk.py` asked for a shove, timed the decay, graded it and
  printed its refusals. Peaks are matched across a set and every fitted sample is
  DAMPING with a live gain.
- **Accidental** — `analysis/decay_scan.py`, written 2026-08-18. It reads every
  `data/*_fast_lock.csv` (90 files, 3.6 GB, ~6 M rows, in 70 s) and fits the amplitude
  excursions that happened to occur *and* decayed while the loop was driving.
  **Nobody asked for these transients and nothing records what caused them** —
  a bump, a fault clearing, a gain ramp — so their peaks are not matched and the
  population is whatever the room did. **Weaker evidence than a `jerk.py` set,
  and it must not be quoted as if it were one.** It is graded against the only
  fits known to be valid: over the four files `jerk.py --replay` accepts kicks
  in, it independently finds **7 of the 8** and agrees to within **10 %** on 4 of
  them and **33 %** on all 7. It misses one (2.1 s of decay, under its floor).

| rung | what it was | commanded kicks (`jerk.py`) | accidental transients (`decay_scan.py`) |
|---|---|---|---|
| `zero` (v0) | one channel, latching saturation fault | none — predates `jerk.py`. **Simulator** ratio **0.188**, lock **12.1 s** (`harness.py`, `versions.md` § *The ladder*) | **not measured** — no bench CSV in `data/` is attributable to it |
| `alpha` (v4) | + `auto-disable` (demote one blind OSEM) | none — predates `jerk.py`. **Simulator** ratio **0.029**, lock **10.6 s** | **0.4675 /s**, r² 0.97 — but **n = 1**, one 6.8 s transient at peak `ratio` 2.57 in `data/20260804_111807_fast_lock.csv`. See the era caveat below before believing it |
| `beta` (v9) | + `runaway-trend` (growth gate on the breaker) | none. **83.1 %** of a 240 s run in DAMPING — **bench 2026-08-04**, still the best duty cycle on record, and it is uptime | **no usable fit.** 18 excursions in `data/20260804_135853_fast_lock.csv`: 11 quieter than the plate's own undriven motion, 4 that never rose, **3 that left DAMPING mid-decay**. Its loudest reached `ratio` 6.37 and only 5.46 of that while driving |
| `delta` (v12) | 8 ch, 100 Hz clock off the wire, `decimate` | none. **77.5 %** of a 244 s run in DAMPING — **bench 2026-08-06**. Never printed `LOCKED` | **no usable fit** over 3 runs and 971 283 rows (`data/20260806_190046`, `_200158`, `_200822`): 60 excursions — 45 sub-baseline, 6 that never rose, **5 that left DAMPING**, 3 with no valid baseline behind them, 1 that never re-quieted. Its transients reached `ratio` 9.69 — they exist, and the breaker ate them |
| `epsilon` | + Kalman velocity, Ki = 0 | **not measured** — never on the rig | **not measured** — never on the rig |
| `zeta` | the modal law, Φ/A from `data/modal.json` | **modal 0.0197 /s**, r² 0.33, 1 kick (`data/20260817_193855`, 0 % FAULT). **Diagonal 0.0253 / 0.1072 / 0.0305 /s**, median **0.0305**, r² 0.57–0.78, 3 kicks (`data/20260817_194128`, 98.7 % DAMPING, **0 % FAULT**). Both sets pass the state check | modal **0.0133** (r² 0.23); diagonal **0.0278 / 0.0305** (r² 0.61 / 0.78) — the same events, found independently |
| `eta` | modal + one 14-state filter whose state is the modal coordinates | **median 0.1093 /s** of 0.0624 / 0.1093 / 0.1098, r² 0.42–0.92, matched peaks 6.33 / 6.52 / 7.33 — 2026-08-18 02:43, `data/20260818_024240_jerk_eta.log`, **the first complete valid kick set on this rig**: 7 channels live, no faults, one further kick correctly refused. **15x the free plant.** Also **0.1355 /s**, r² 0.944, 1 kick of 4 on `data/20260817_213642` (56 % FAULT). First rung ever to print `LOCKED` (20.7 s, DEGRADED, warm start) | **median 0.0950 /s**, spread 0.0241–0.1609 over **11** transients in 5 runs, median r² 0.83 — much the largest usable population on the rig, and it brackets the commanded number |
| `theta` | the same law as a bank of causal FIR kernels | **not measured** — never on the rig. At `KERNEL_PHASE_DEG = [0,0,0]` it is bit-identical to `eta`, max \|du\| **0.000e+00 V** over 30 000 samples (simulator) | **not measured** — never on the rig |

**Best measured on this rig is `eta`'s commanded 0.1093 /s, and it is modal.** The
free plant is 0.0072 /s (τ > 138 s, `analysis/ringdown.md`), so that is **15x**.

### Two corrections the scan forced

**The 2026-08-17 21:36 run was `eta`, not `zeta`.** `data/20260817_213620_jerk_eta.log`
line 16 prints `osem.eta` and its fix list includes `modal-kalman`, which `zeta`
does not have. The 0.1355 /s this table used to credit to `zeta` is an `eta`
number and has been moved.

**A valid diagonal kick set does exist**, and the earlier "there is none at all"
was too strong. `jerk.py --replay data/20260817_194128_fast_lock.csv` keeps
**3 kicks** — 0.0253 / 0.1072 / 0.0305 /s — off a run that is **98.7 % DAMPING and
0 % FAULT**, with `modal` never engaged, i.e. `epsilon`'s diagonal law throughout.
**It still does not rescue "modal damps 5x faster".** It points the other way: in
that same `zeta` session modal measured **0.0197 /s** against diagonal's median
**0.0305 /s**. The 5x quoted in § 8 remains **SUPERSEDED AND UNSUPPORTED** — its
original diagonal half came from runs **78–99.7 % in FAULT with every gain at
+0.0000**, which fits the free plant and not a control law. It is left in place
below, marked, because the correction is the point.

### What the accidental fits cannot settle

**They do not separate the laws.** Held to one evening, so the rig is fixed and
only the law changed, the two evenings that ran both disagree in *direction* —
and neither difference is bigger than the spread inside its own cell:

| evening | diagonal | modal |
|---|---|---|
| 2026-08-17 | **0.0291** /s (n=2, 0.0278–0.0305) | **0.0542** /s (n=2, 0.0133–0.0950) |
| 2026-08-18 | **0.1198** /s (n=3, 0.0241–0.1578) | **0.0702** /s (n=8, 0.0324–0.1609) |

(2026-08-17 spans both `zeta` and `eta`; 08-18 is `eta` apart from one
unattributed run. `decay_scan.py`'s own *BY SESSION AND LAW* block prints this.)

**And there is a 3–10x era gap nobody has explained**, on both laws. Median
decay by session: 2026-08-04 **0.3774**, 08-06 **0.2534**, 08-07 **0.2137** /s
(n = 13, median r² 0.92–0.97) against 08-17 **0.0291** and 08-18 **0.0844** /s
(n = 15). `ratio` is the same quantity in both eras — a
1.0 s sliding RMS over `SCHEDULE_WINDOW_S` divided by that run's own calibration
baseline, unchanged from `osem.v9.py` through `osem.eta.py` — so the definition is
not the cause. Candidates, none measured: the rig lost a5/a6/a7 and the modes
moved 0.0170 Hz between the eras; the old runs are 3–5 live channels against 7;
and `ratio` is normalised to each run's own 20 s `CALIBRATING` window, which is
not the same room twice. **Until that gap is understood, no accidental fit should
be compared across sessions**, which is most of what this column could otherwise
be used for.

**Nothing here separates damping from holding** either — a loop that pins the
optic and one that dissipates its energy are identical in `ratio` (`CLAUDE.md`
§ *Not established*).

---

**Sensor layout**, reported by the person who built the rig 2026-08-17. Picture the
assembly as a head, with the damped plate in the middle and its normal pointing at
the nose:

| | where | senses |
|---|---|---|
| a0–a3 | back of the head, one plane, at the four **corners** | along the plate **normal (Z)** |
| a4, a5 | the **ears** | left–right (**X**) |
| a6, a7 | the **top**, attached vertically to the plate | up–down (**Y**) |

**The corner assignment IS established** — `dof.py` on
`data/20260817_205211_status_sensors.csv`: the pairing that minimises warp is
`a0+a3` vs `a1+a2` at warp/rigid **0.139**, 2x better than the next
(0.277, 1.566). **a0 is diagonal to a3 and a1 is diagonal to a2.** So the rigid-body
coordinates are `Z = (a0+a1+a2+a3)/4`, `T1 = (a0+a1−a2−a3)/4`,
`T2 = (a0−a1+a2−a3)/4`, `WARP = (a0−a1−a2+a3)/4`, and **the three modes are
T1 / Z / T2** at 0.72194 / 0.99193 / 1.65607 Hz (`CLAUDE.md` § *The geometry*).

That last row of the table matters more than it looks: a vertical sensor cannot be
graded by its response to the three horizontal pendulum modes, which is what every
tool in this repo does (`CLAUDE.md` § 10). **a4/a6/a7 do not carry degrees of
freedom of their own** — no peak anywhere in 0.3–25 Hz in their own coordinates over
an 840 s silent-room record — so what they see is cross-coupling from the same three
rigid modes.

**Before you open the port: probe the baud rate.** The board attached on
2026-08-17 runs at **115200**, not the 500000 that `pyDAC.BAUD`, `pyDAC2.BAUD`
and `arduino.ino`'s `BAUD_HZ` all declare — and at the wrong baud
`pyDAC2.FastDAC.__init__` can sit silent for **400 s** before it says anything.
`CLAUDE.md` § *Read this before opening the port* has the whole story; it cost
most of an afternoon.

---

# Bench quickstart

**Read this before flashing anything.**

## 0. Which version to run

The ladder is **named, not numbered**, and as of 2026-08-18 it is three rungs:
`epsilon`, `zeta`, `eta` — `ladder.py` says `LADDER = ("epsilon", "zeta", "eta")`
and `delta` was retired to git. Numbers stopped carrying information once v10
and v11 were both "newer than v9" and one of them damped worse than v9 did; they
also imply a total order the history does not have, since v7 was a branch off v5
rather than a successor. The order lives in `ladder.py` and nothing is derived
from a filename. `make list` prints the table; old numbered names still work on
the command line and are redirected with a note.

**`zero`, `alpha` and `beta` were deleted 2026-08-17** and are in git history.
They were the four-channel supervisor lineage — a latching saturation fault,
`auto-disable`, `runaway-trend` — and every fix each one introduced is carried by
`delta`. `versions.md` says what each was and what it measured.

**Controllers are no longer standalone, and that reverses a long-standing rule.**
Until 2026-08-17 every `osem.*.py` imported nothing from the others, deliberately:
`bench.py` reads the gain vectors out of the file it is about to run, so what it
prints is what gets applied. The machinery that rule duplicated eight times now
lives in **`stdlib.py`**, and `osem.eta.py` imports it. The check survives because
**`stdlib.py` holds no constants** — every threshold is a constructor argument, so
the gains are still declared in each rung's own file. Where a rung's *law* comes
from another module, `bench.py` reads `GAIN_SOURCE`, names it, and says to verify
the gains there.

`epsilon` and `zeta` run the **same diagonal control law** the deleted `delta` did:
independent SISO velocity-feedback PID loops, one per channel, with ch2's deliberate
opposite sign. `zeta` and `eta` add the modal law on top of it (§ 8).

- **`osem.delta.py` was the diagonal baseline and the only rung ever validated on
  hardware. DELETED 2026-08-17** (`git show b90c983:osem.delta.py`); the diagonal
  control is now zeta or eta run with no `data/modal.json`. It was eight channels,
  500000 baud, and a **100 Hz control clock decoupled from the wire**, plus the
  four fixes that made that safe: `sample-guard`, `decimate`, `persist-baseline`,
  `baseline-sanity`. Time-weighted on the bench 2026-08-06 it damped **77.5 %**
  of a 244 s five-kick run. The single change that did it was the runaway
  breaker's lag: 2 s sits inside all three measured mode-beat periods (3.578 /
  1.551 / 1.082 s), so the breaker was reading the beat as growth. At 10 s it is
  not. (Recomputed from the 2026-08-17 frequencies those beats are 3.718 /
  1.505 / 1.071 s — arithmetic, not a new measurement. The argument is unchanged:
  2 s is still inside all three and 10 s is still outside.) **It has never announced `LOCKED`** — see § 4; that is a quorum bug, not a
  damping one.
- **`osem.epsilon.py`** — `delta` plus the Kalman velocity estimator replacing
  bandpass-and-differentiate, `mains-null` decimation, and Ki dropped to zero
  because the filter carries drift as a state instead of integrating it. Written
  2026-08-07, **`BENCH_STATUS = untested`** — it has never run on the bench.
  **Its `F_MODE_HZ` is stale**: the Kalman filter models three undamped
  oscillators at the 2026-08-06 frequencies, and those moved (`CLAUDE.md` item 1).
- **`osem.zeta.py`** — the modal (MIMO) law, and the first rung whose control law
  is not in its own file: Φ and A are **measured**, live in `data/modal.json`, and
  are written by `status.py --save-modal`. With no file, a stale one, or one that
  fails the colocation sign check it runs `epsilon`'s diagonal law and says which.
  Written 2026-08-15. **It closed a modal loop on the rig 2026-08-17 and damped
  5x faster than the diagonal law** (§ 8) — **that 5x is superseded and
  unsupported as of 2026-08-18; see the headline table at the top.** **Do not delete it** — an earlier
  `CLAUDE.md` said MIMO was closed and a session following that literally would.
- **`osem.eta.py`** — `zeta`'s modal law with the modal velocity coming from
  **one Kalman filter whose state IS the modal coordinates**: 3 modes × 2 plus one
  DC state per sensor, **14 states**, replacing `zeta`'s per-mode least squares
  over eight per-sensor filters. A demoted or railed sensor is a **deleted row**,
  never a re-weighting. The point of it is that a filter knows its own innovation
  covariance, so `zeta`'s out-of-mode residual becomes a **calibrated** statistic:
  **χ² per degree of freedom**, order 1 when the three-mode model explains the
  readings to within the measured per-channel noise. It is **logged and
  deliberately not acted on** — the distribution on this rig has never been
  measured, so a threshold would be a guess, and the point of the first run is to
  produce one. Both estimators are carried and the per-sensor one is *not* a
  fallback: it still owns `bp`, and therefore the gain schedule, the runaway
  breaker, the lock detector and `baseline-floor`. It is also the only rung whose
  `dead-pin` test can get past calibration with a6/a7 bottom-railed (see below).
  Written 2026-08-17. **It ran on the rig 2026-08-18** — one diagonal null test and
  one modal run (`data/20260818_0018*`, `data/20260818_0022*`), and it is the first
  rung in this repo's history to print `LOCKED` (§ 4). Both runs faulted on their
  first hand kick and neither fault cleared (§ 7). **Its `BENCH_STATUS` still reads
  `untested` and that is now wrong.** Its module docstring carries a **PRE-FLIGHT**
  section written to be read standing at the bench.
- **`osem.sysid.py`** — stepped-sine actuation-matrix measurement. It damps
  nothing and closes no loop.
- **`stdlib.py`** — the machinery every controller used to carry its own copy of.
  Not a rung; nothing runs it directly.
- **`status.py`**, **`jerk.py`**, **`slopesign.py`**, **`dof.py`** — the bench
  tools. Not rungs and not controllers; see the Reference table and § 8.
  (`signtest.py` was deleted 2026-08-17: with Φ taken from geometry there is no
  per-mode sign left to search.)

**In git history only** (`git log --diff-filter=D --name-only`): `zero`, `alpha`
and `beta` (deleted 2026-08-17), and the numbered rungs the named ladder replaced
— v1, v2, v3, v5, v5.5, v7, v8, v10, v11, v13, and the v6.1 / v6.5 / v6.6 sysid
variants. `versions.md` says what each was and why it went. **`osem.v13.py` was
deleted 2026-08-07**: it existed only to carry a modal law, and modal control was
closed by measurement at the time. It stays deleted — `osem.zeta.py` replaced it
2026-08-15 with a modal law that loads a *measured* Φ, and that law was measured
working on 2026-08-17 (§ 8).

**All eight OSEMs are wired.** The 2026-08-17 census
(`data/20260817_172150_status_sensors.csv`, 37 619 samples, 90 s, 418 Hz, clean),
measured at the **re-measured** mode frequencies, is the current answer:

| ch | grade | resting counts | SNR A / B / C | note |
|---|---|---|---|---|
| a0 | GOOD | 699.2 | 66.1 / 72.0 / 50.5 | |
| a1 | GOOD | 585.4 | 68.2 / 48.3 / 54.0 | the channel that closed MIMO in August |
| a2 | OK | 637.5 | 79.1 / 87.4 / 70.0 | watches the optic, shares a 6.19 Hz line |
| a3 | GOOD | 630.1 | 90.6 / 63.1 / 79.6 | |
| a4 | GOOD | 482.6 | 107.2 / 75.1 / 94.0 | **strongest channel on the rig** |
| a5 | DEAD | 0.0 | 0.0 / 0.0 / 0.0 | hard zero — a disconnected pin, see below |
| a6 | DEAD | 7.4 | 3.8 / 3.3 / 2.0 | bottom-railed, **vertical**, and **not blind** |
| a7 | DEAD | 13.2 | 3.7 / 4.4 / 2.9 | bottom-railed, **vertical**, and **not blind** |

**a6 and a7's DEAD grade is wrong, and it is measured wrong.** Multiple coherence
against a0–a3 over 0.6–1.8 Hz from `data/20260817_205211_status_sensors.csv`
(113 418 samples, 300 s, 378 Hz, 26 segments, bias floor 3/26 = 0.12) gives
bias-corrected usable fractions **a0 1.00, a4 0.46, a6 0.26, a7 0.50** — roughly
**half** of a4's and a7's in-band motion is the optic. The grade is a statement about
their DC operating point and about the census scoring the *horizontal* modes, not
about whether they see. Measured once, on one ambient record.

**And the driven test that agreed with the DEAD grade was also misread.** The coil
pass gave those three |H| 0.02–0.39 V/V at SNR 0–6, taken as blind sensors. **Coils
0–3 barely push the side and vertical DOF**, so a weak driven response says the
*coils* do not reach those degrees of freedom — not that the *sensors* cannot see.

**SUPERSEDED, and kept because the correction is instructive.** This section used
to give an *in-band vs out-of-band* split — a0–a3 at 77–99 % of power in
0.4–3 Hz, a5 in band at ~1/12 of a0's gain, and **a4/a6/a7 "no", 0.1–9 % in
band, ~1/100 of a0–a3's in-band gain**, dominated by a 6.19 Hz line. Three things
went wrong with that:

1. **a4 is now the strongest channel on the rig.** On 2026-08-06 it read SNR
   3.0 / 28.9 / 10.1; on 2026-08-17, 107.2 / 75.1 / 94.0.
2. **Part of the historical deficit may be a frequency-error artefact — a
   hypothesis, not a result.** The 2026-08-06 numbers were computed at
   frequencies that have since moved, and re-analysing the *same* 2026-08-17
   record at stale vs. corrected frequencies moves a0's mode-C SNR from **12.1 to
   50.5**, a factor **4.2**. The 08-06 record has **not** been re-analysed at
   corrected frequencies, so this is not settled.
3. **a6/a7 are vertical**, so their in-band grade was never the right test —
   a sensor watching an orthogonal DOF scores blind by construction
   (`CLAUDE.md` § 10).

**That is three separate wrong answers about the same three channels** — "not
wired", "flags misaligned", "blind" — each of them a test that could not tell a
broken sensor from a sensor pointed where the actuator is not. `CLAUDE.md` § 6.

And the explanation before *that* — "the flags sit outside the linear
partial-shadow region", i.e. an alignment job — was already refuted on 2026-08-06
by the resting counts: a4 sat at 567.5 against mid-scale 511.5 with a 30-count
swing, inside its linear range, not pinned.

**a5, a6 and a7 are a signal-path / DC-operating-point failure, not a motion or
alignment failure, and the test is decisive.** During
`data/20260817_174825_status_phi.csv` the table was being physically worked on.
In its loudest 5 s window a0–a4 moved up to **868 counts peak-to-peak — 85 % of
the 1023-count ADC range** — while **a5 moved exactly 0** counts and a6/a7 moved
**6**. Five sensors on the same optic saw near-full-scale motion at the same
instant, so "there was no motion" cannot explain it. On the clean retest
(`data/20260817_175912_status_sensors.csv`, 26 092 samples, 60 s, 435 Hz) **a5
takes exactly one distinct value, 0.0, with variance exactly 0.000000e+00
counts²** — a live ADC input always carries ±1 count of dither, so that reads as
hard ground — and **a6/a7 are bottom-railed on 100 % of samples** past
`RAIL_LOW = 12`, at std 0.56 and 0.48 counts. On 2026-08-06 the same three rested
at 554.8 / 860.6 / 734.7. **This is a screwdriver job and it is written up as a
request in `CLAUDE.md` § *Hardware requests*.**

**And until somebody does it, a dead input stops the rig — which is a software
problem, not a wiring one.** 0 counts is below `RAIL_LOW = 12`, and a rail during
`CALIBRATING` is a whole-rig fault by design (correctly: it normally means the
optic is against a mechanical stop). Measured 2026-08-17: **the rig sat in FAULT
for 50 s with every gain at zero because of one bad solder joint.**
`ENABLE_CHANNEL[5] = False` in `zeta` and `eta` handles a5. **a6/a7 are the case
config cannot handle** — they were bottom-railed at 17:59 and reading 591.3 and
566.0 at 18:20, with no recorded cause, so their state at the start of a session is
unknown. `eta` separates the two cases by **variance**: railed *and* essentially
motionless is an unwired input and demotes one channel; railed and still **moving**
faults the whole rig exactly as before. The threshold is the std of the raw counts
over the rail interlock's own window and **both its edges are measured** — dead
a6/a7 reach 0.853 counts, the quietest railed-*and-moving* channel in evidence is
1.998, and the line sits at the geometric mean, 1.3. `eta`'s `--selftest` asserts
both directions and the suite still reports *a rail during CALIBRATION is a
whole-rig fault* with 0 demotions. **`zeta` still carries the old 1.0-count span
test and will fault through calibration** if a6/a7 are railed when it starts. Full
derivation in `versions.md` § *eta* and at `DEAD_PIN_STD_COUNTS` in the file.

One encouraging number for a6/a7, and one discouraging one: they put **7.6 %** and
**10.9 %** of their power in the 0.6–1.8 Hz pendulum band against a **2 %**
bandwidth share — a 4–5x excess, consistent with **wired but biased onto the
floor** — while the rest of their spectrum tracks bandwidth exactly (20–60 Hz is
67 % of the band and carries 44–52 % of their power), which is ADC dither on a
pinned line. a5 has no excess anywhere, because it has no variance at all.

Methodological note worth keeping: the original call came from a DC-response
threshold and raw time-domain correlation, and **both are blind to a small
coherent signal**. Test for a peak at the known mechanical resonance instead —
and make sure you know where that resonance actually is.

Each controller declares a `BENCH_STATUS` and `make list` prints it, but
**`bench.py` does not gate on it** — the statuses went stale faster than they were
updated. The preflight and the printed gain vectors are the load-bearing checks;
with `stdlib.py` in the picture `bench.py` follows `GAIN_SOURCE` to the module the
gains come from and names it. `delta` ran 2026-08-06 and 08-07 (console logs in
`bench/20260806/`); **`zeta` ran 2026-08-17** — three `jerk.py` kick sets and five
`signtest.py` trials, `data/20260817_19*`; **`eta` ran 2026-08-18**, one diagonal and
one modal run, `data/20260818_00*`. `epsilon` has **never been on the rig**.

## 1. Flash the Arduino

`arduino.ino` is the firmware. arduino-cli will not look at a sketch until it is
named `<dir>/<dir>.ino`, so one command does the copy, the compile and the upload:

```bash
make arduino                                   # defaults to arduino:avr:mega
make arduino FQBN=arduino:sam:arduino_due_x    # if it is a Due
make arduino PORT=/dev/cu.usbmodem11101        # skip auto-detection
```

Board is an Arduino **Mega or Due** — `CS` is on pin 53, which is Mega/Due specific.

**The board attached on 2026-08-17 is an official Arduino Mega 2560 R3**:
`/dev/cu.usbmodem11101`, USB VID:PID **2341:0042**, manufacturer string
`Arduino (www.arduino.cc)`, and the only USB device on the machine with a vendor
ID. **This is not the board the rest of this repo was written against** — the
recorded port `/dev/cu.usbserial-1120` is a CH340/FTDI-bridge name, i.e. a
different, cloned board. It also **runs at 115200 baud**, because `arduino.ino`
carries a `BAUD` command that changes the rate without a reflash and somebody
used it. The live rate is a property of the board, not of this repo; probe it.

`setup()` zeroes all eight DAC channels before anything else runs, and enables the
DAC's internal 2.5 V reference. Opening the serial port toggles DTR and resets the
board, so **connecting is itself a safing action** — the coils are de-energised
before `READY` is sent.

## 2. Host setup

```bash
conda activate ligo        # numpy 2.5.1 + pyserial 3.5; the only env with both
```

Without it `python3` resolves to a Homebrew interpreter with neither dependency,
and every target dies at `import numpy` with a traceback out of
`sim/server.py:load()` that looks like a repo bug and is not.

You do **not** need to edit `PORT`. It is still `COM7` (a Windows name) in every
controller, but `make run` auto-detects the board and sets the port on the loaded
module, so every controller stays byte-for-byte what was validated. `make ports`
shows what it can see; `make run PORT=...` overrides it.

**The baud rate is now probed, not assumed.** `pyDAC.BAUD`/`pyDAC2.BAUD` still
declare 500000 and the attached board runs at **115200**, which returns framing
garbage (`b'\x80\x80xx\x00x\x00x\x00x\x80xx\x00\x80x'` measured 2026-08-17) rather
than an error — and the `READY` scan then burns up to **400 s in silence** before
failing. `pyDAC2.probe_baud` / `resolve_baud` own the candidate list and every
tool in the tree calls them. Anything that does not repeats that afternoon.

## 3. Run

```bash
cd /path/to/this/repo          # CSV goes to ./data/ RELATIVE TO CWD
make run                       # a picker over the ladder
make run V=eta                 # the modal rung; needs data/modal.json
# for a diagonal control, move data/modal.json aside and run V=eta again
```

**For a modal run (`zeta`, `eta`) the order is fixed and the first step is not
optional:**

```bash
python3 status.py sensors                 # re-measure the mode frequencies FIRST
python3 status.py coils --save-modal      # A -> data/modal.json; Φ is GEOMETRIC
python3 osem.eta.py --selftest            # the modal math and filter, offline
make run V=eta
python3 jerk.py eta                       # and this is how you find out if it worked
```

**Φ now comes from geometry, not from a measurement.** `--phi geo|svd` defaults to
`geo`: the exact signed rows `[1,1,1,1]`, `[1,1,−1,−1]`, `[1,−1,1,−1]` under the
measured corner assignment. That removes the per-mode sign gauge that **pumped the
optic on hardware** on 2026-08-17 (median channel ratio 1.5 → 2.3), which is why
`signtest.py` no longer exists. The measured Φ is now a *check* on the geometry —
|cos| **0.956 / 0.942 / 0.948** on the 23:12 pass — and with `--phi geo` the run's
own Φ-vs-geometry banner is **vacuous** and says so.

**Do not add `--multisine` on resonance.** It is faster (9.9 min against 18) and it
improves the per-coil phase spread (73–89° → 31.6 / 36.7 / 7.8°), but it **broke the
signs of A, 7 of 12 against 12 of 12** for the one-at-a-time pass, because an
on-resonance response ramps and its 1/f² skirts leak the loudest mode into the
quietest mode's bin (`CLAUDE.md` § 3).

`status.py sensors` before anything is driven, every session: the modes moved
0.0170 Hz — 9 half-widths — in eleven days, and a Kalman filter that models the
oscillator treats a frequency error as a **model** error rather than a detune.
`eta` refuses a `data/modal.json` whose frequencies disagree with its own by more
than 0.005 Hz, and prints which of its checks refused before any coil is
energised. `eta`'s module docstring has a **PRE-FLIGHT** section: what must be
true, what to watch on the console, and what means stop.

**Nothing on the ladder prompts.** The prompt survives only when
`sys.stdin.isatty()`, so a run goes unattended — which also means **the coils are
energised the moment it starts**, with no Enter key between you and that.

`make run` **preflights** the port — opening it through the real `DACController`,
which raises unless the sketch answers `READY`, so "wrong port" and "board not
flashed" are distinguishable before the optic is swinging — then prints the bench
status and the gain vectors it is about to apply, and hands over to that version's
`main()`. The controller then calibrates with the gain at zero before engaging:
the long quiet stretch at the start of a run is that calibration, not a hang.
**20 s**, and that length is measured — `CLAUDE.md` § *Why the calibration window
is 20 s*. It is a *ceiling*: `fast-calib` exits as soon as the floor settles, but
that has only ever paid off in the simulator; on the bench it ran the full ceiling
every time and fell back to the old estimator, which is the designed failure mode.

## 4. What you should see

```
[CALIBRATING] measuring baseline noise for 20s (outputs held at bias)...
[DAMPING] baseline set (ch0=0.1804V, ...). Gain will schedule between -0.035 and -0.030
*** LOCKED -- 14.5s after gain was applied (t=34.5s total) ***
```

The **lock time is the deliverable**. A run that never prints `LOCKED` did not
work, whatever the traces look like.

**It has now been printed exactly once**, 2026-08-18, by `eta` on the modal law
(`data/20260818_002207_jerk_eta.log`):

```
*** LOCKED -- 20.7s after gain was applied (t=22.7s total), 4/7 channels
    driven and quiet -- DEGRADED, 4/7 healthy ***
```

Read the qualifiers with it: **DEGRADED** — a4/a6/a7 were demoted by `inband-floor`
before the announcement — and the run engaged on the **stored** baseline after a
2.0 s warm-up rather than a fresh 20 s calibration, so the 22.7 s total is not
comparable with a cold start. **Anything in this tree still saying no rung has ever
printed `LOCKED` is stale.**

The bug that had blocked it for the whole history of the repo was a quorum bug, not
a damping failure. The announcement needs every `enabled & healthy` channel quiet,
and `ENABLE_CHANNEL` was `True` on all eight; a4/a6/a7 demote to `NOSIG` and drop
out, but **a5 stayed healthy while `STEADY_GAIN[5] = 0`**, so nothing drove it and it
sat at ratio 1.26–2.53 against a threshold of 0.35. One undriven channel vetoed the
whole rig, permanently. a0–a3 did reach lock together throughout — 6 simultaneous
episodes in `20260806_200822`. a5 is now a disconnected pin with
`ENABLE_CHANNEL[5] = False`, and zero-gain / no-signal channels drop out of the
quorum. **What is still owed is a `LOCKED` on a full quorum from a cold
calibration** (`CLAUDE.md` § 7).

**The other thing that stops a run, and it is worse.** A normal hand kick faults the
rig and the fault does not clear. Measured **twice on 2026-08-18**, once under each
law: the diagonal run's quiet sat at ratio **1.24** and the kick reached **4.67** —
**3.8x**, against the runaway breaker's **1.8x** — while modal's quiet sat at
**0.25** and its kick reached **4.73**. Both runs then sat in FAULT with every gain
at **0.0000** for **95–119 s, to the end of the run**, ch3 holding at **2.1–3.2**
against a clear gate of **1.4**. **It is not a modal-specific mis-scaling: the
diagonal law does it too.** The frozen-measurement half is fixed — `ratio` is now
live during FAULT, varying 1.4–3.8 print to print, where 2026-08-17 measured it
frozen at **3.59 for 1085 s** — but **the deadlock is not** (`CLAUDE.md` § 1).

`data/<YYYYMMDD_HHMMSS>_fast_lock.csv` gets 13 columns per channel plus `time_s`
and `state`, the run-level `ctl` and `n_avg` (the decimation's samples-per-step),
and on `zeta`/`eta` the `modal` and `mrank` columns — **read `modal` before
believing any modal result; a refusal is invisible in every other number.** `eta`
adds `chi2` and `ndof`. Line-buffered and flushed every 200 rows, so it **survives
an unclean kill**. `data/` is gitignored.

## 5. On the scope, watch for

- **Clean, unclipped convergence.** Amplitude falling, no flat tops.
- **The DAC output staying inside 0–0.5 V.** Thirty consecutive samples against
  either limit trips a saturation fault.
- **`RAIL!` messages** — a fully occluded OSEM pins the ADC at 0 and the bandpassed
  signal flatlines, which an RMS-only check reads as *perfect stability* while the
  sensor is blind. That is why the interlock reads raw counts.

**And the DAC output does NOT stay inside 0–0.5 V under the modal law.** Measured
demand about bias over the 2026-08-17 modal damping run:

| | coil 0 | coil 1 | coil 2 | coil 3 |
|---|---|---|---|---|
| rms V | 0.0273 | 0.0328 | 0.0255 | **0.0819** |
| max V | 0.1863 | 0.2239 | 0.1949 | **0.4229** |

**Coil 3 reached 169 % of the 0.25 V half-window and clipped**, while the modal
allocation stayed inside its own 0.20 V cap the whole time — **the derivative term
is added after the cap**. Once a coil clips, the realised force is no longer
`A_C^+ f` and the dissipation guarantee is void. The 5x result in § 8 was measured
with this happening (`CLAUDE.md` § 2).

**Known open problem: the OSEM signal clips, because no OSEM rests at mid-scale.**
Mean resting counts over the nine bench logs, taken from each run's `CALIBRATING`
window (gain zero, outputs at bias), against a mid-scale of 511.5:

| | ch0 | ch1 | ch2 | ch3 |
|---|---|---|---|---|
| resting counts | 615 | 641 | **717** | 679 |
| headroom up / down | 408 / 615 | 382 / 641 | **306 / 717** | 344 / 679 |

So every channel clips the **top** rail long before the bottom, and **ch2 is the
worst by a wide margin**. A sensor pegged at either end is outside its linear range
and the velocity estimate spikes exactly when the loop is asked to push hardest
(`versions.md` § *On the bench*). Centring the flags is worth 1.7× the tight-side
headroom on ch2 alone, but ch0's own resting point moved 599 → 635 counts *between
runs*, so this drifts and a one-off mechanical centring will not hold. **Fix the
sensor range before trusting any tuning done on top of it** — `CLAUDE.md` § 13 is
the bias sweep that would tell you whether the offset is electrical or mechanical.

**Still true for a0–a4 on 2026-08-17, and reversed for a6/a7.** a0–a4 rest at
680.3 / 585.4 / 636.2 / 699.6 / 558.6 counts, all above mid-scale, still clipping
the top rail first. **a6 and a7 now rest at 4.3 and 8.8 counts — on the bottom
rail** — where on 2026-08-06 they rested at 860.6 and 734.7, and a5 reads a hard
0.0 where it rested at 554.8. So whatever sets the per-channel DC offset changed
for exactly the three channels that stopped working and for none of the five that
did not. **No cause is asserted**; the bias sweep is still the experiment.

And the drift is faster than "between runs": **ch3 moved +69.5 counts and ch4
+76.0 counts between two clean records 37 minutes apart** on the same evening
(`data/20260817_172150_status_sensors.csv` → `..._175912_...`).

## 6. Safety limits — do not exceed without a scope on it

**`Kp = −0.040` is the documented instability / rail onset.** `-0.030` damps
cleanly; `-0.040` does not. **Do not raise gain past `-0.035`** (the `CAPTURE_GAIN`
already in use, itself not independently validated) **unattended.**

Be aware: **that −0.04 limit has almost no recorded evidence.** Damping worked at
−0.030 and instability / rail onset were seen at −0.040 on **2026-07-15 run 3**, and
that sentence lived only in the docstring of `osem.v0.py`/`osem.zero.py`, both now
deleted. The written report two days later says nothing about it, and the simulator
provably cannot reproduce it. Treat it as real and uncharacterised — `CLAUDE.md`
§ 12 is the bench task, and § *Where the constants came from* is now the only
non-history record.

**`STEADY_GAIN[2] = +0.010` is positive** where the others are negative. Not a
typo: A2 damped *fastest* of the four at that gain (γ 0.2796 /s against
0.2197 / 0.1821 / 0.1982), which only holds if its coil or OSEM is mounted the
other way round. **Do not "correct" it.** `CLAUDE.md` § *Why `STEADY_GAIN[2]` is
positive*.

**The coils were rewired 2026-08-03.** Sensor index `i` (0–3) → analog pin `A<i>` →
DAC channel `DAC_CHANNELS[i] = [1, 3, 5, 7]`. The full eight-pair map is
`[1, 3, 5, 7, 0, 2, 4, 6]` (`CLAUDE.md` § *The coil map, and the wrong-map trap*);
the controllers take the first four because only A0–A3 have OSEMs on them. The
previous map, `[0, 2, 4, 6]`, now points at the coils for sensors 4–7 — a controller
still holding it closes every loop onto the wrong actuator, and **neither the
simulator nor the interlocks can tell. Only the bench catches a wrong map**, and
`status.py`'s DC pass is weak evidence at best: one coil moves the whole suspended
body, so coil 1 gave a0 +37.2 and a1 +36.1 counts/V — two sensors nearly equal.

## 7. If something trips

The state machine is `CALIBRATING → DAMPING → FAULT → (auto-recover) →
CALIBRATING`. A fault zeroes all gains and holds every output at bias. If
everything stays clear for 5 s it re-calibrates and resumes on its own.

**`auto-disable` demotes one blind OSEM rather than freezing the rig.** It parks
that channel (`ch1:DOWN`), keeps the rest damping, and re-arms 2 s after the sensor
comes back — a global fault needs *every* channel blind. A rail during
`CALIBRATING` is global; you cannot calibrate a blind sensor.

**Except that it does not clear.** 2026-08-17: the runaway breaker froze all
channels and **stopped recomputing the `ratio` that clearing the fault is gated
on** — `ratio = 3.59` for 1085 s, gains zero. That half is fixed. **2026-08-18: it
still does not clear**, because the gate is an absolute multiple (`FAULT_CLEAR_RATIO
= 1.4`) of a baseline measured with the loop **off**, and a live `ratio` above a
gate that cannot be reached is the same deadlock. Both runs that night ended in
FAULT — 95–119 s, every gain 0.0000. `CLAUDE.md` § 1.

**And a channel demoted at engage never damps at all.** Measured across both
2026-08-18 runs: **ch4, ch6 and ch7 had `|gain|` exactly 0.0000 for 100 % of all
DAMPING samples**, demoted by `inband-floor` on in-band amplitude measured at the
three **horizontal** mode frequencies — while a4 senses X and a6/a7 sense Y. Their
in-band amplitude is small **by construction**, not because they are blind
(coherence with the optic 0.46 / 0.26 / 0.50). The coherence-scaled hybrid PID in
§ 8 has therefore **never run on the rig** (`CLAUDE.md` § 6).

**A blind channel is also still not fully handled.** `auto-disable` keys on *rail*,
and a disconnected OSEM does not always rail — it can sit mid-scale and flat, then
calibrate a near-zero baseline, so any noise reads as a runaway and trips the
*global* interlock. That killed the eight-channel v5.5 run: ch0–ch3 damping at ratio
0.07–0.26 while ch4–ch7, calibrated at ~0.002 V, faulted the whole rig six times in
120 s. `baseline-floor` is the guard for that; `eta`'s `dead-pin` is the guard for
the railed-and-motionless case.

## 8. MIMO / modal control — ruled out 2026-08-06, measured working 2026-08-17

The diagonal ladder is **independent per-channel loops**. The modal law
diagonalises the plant and damps the modes directly. It was designed, measured and
**ruled out on 2026-08-06**; on **2026-08-17** the sensor blocker cleared, Φ and A
were measured, and the modal law was run against the diagonal one on the rig.

**SUPERSEDED 2026-08-18 — the 5x is not supported by valid data.** Re-judging every
recorded kick run with `jerk.py`'s replay and its state check found **no valid
diagonal kick measurement at all**: the diagonal rows below came from runs that were
**78–99.7 % in FAULT with every gain at +0.0000**, so those decays were fitted to the
free plant rather than to a control law. The modal rows are also mostly invalid.
What survives is in the headline table at the top of this file — best measured is
`eta`'s modal **0.1093 /s**, 15x the free plant, and the diagonal comparison has yet
to be made. The table below is kept as written.

**The headline number is a decay rate, not an amplitude: modal damps 5x faster.**
Three hand kicks per law, same session, same rig, one law swapped, decay fitted to
the envelope after each kick (`jerk.py`):

| law | peak ratios | decay 1/s | median | re-quiet | r² |
|---|---|---|---|---|---|
| **modal** | 3.96–4.70 | 0.0600 / 0.1393 / 0.1940 | **0.1393** | 5.3 s | 0.79–0.90 |
| diagonal | 3.66–4.44 | 0.0184 / 0.0278 / 0.0311 | 0.0278 | 17.4 s | 0.53–0.69 |

**The ranges do not overlap** — the worst modal kick beats the best diagonal kick by
2x. Peaks are matched across the sets, so the re-quiet times are comparable too.
Against the plant's intrinsic **0.0072 /s** (τ > 138 s) that is ~4x for diagonal and
~19x for modal. **Caveats: three kicks per law, one session, ambient drifts between
the sets.** Every date and number here is a measurement, not a preference, and the
caveats below are not a formality.

**2026-08-18 neither reproduced nor refuted it.** Both runs that night — one per law
— faulted on their **first** hand kick and never came out, so each produced **one**
partial kick whose decay window spans the transition into FAULT: diagonal
**0.0434 /s**, modal **0.0683 /s**. Every later fit in both runs was to an
**undriven** plate, gains 0.0000. **No session-to-session scatter of the diagonal law
is established**, and no repeat kick test is obtainable until `CLAUDE.md` § 1 is
fixed.

### What is true now

**Five sensors carry all three modes at SNR 48–107** — a0, a1, a2, a3, a4, from
the census above. Three modes need **≥ 4** determined rows for a tall Φ, so the
sensor-side condition is met; counted conservatively it is met *exactly*, since
four grade GOOD and a2 grades OK (it damps, but a shared 6.19 Hz line means it is
not cleared to carry a row). **a1 — the channel that closed MIMO — grades GOOD.**

The reported geometry predicts the same thing independently: four sensors at the
four corners of a square span three rigid-body DOF plus one **warp** combination
that no rigid-body motion can produce, so Φ should be **4×3 (5×3 with a4), rank
3, with a genuine null space** — and that leftover direction *is* the out-of-mode
residual. The permutation does not matter, so this holds without knowing which
sensor is at which corner. **A prediction the Φ gate can check, not a result.**

### The matrices, and the amplitude score

**SUPERSEDES the three bullets that were here** (the Φ gate had never run on a
valid record; A was unmeasured; `F_MODE_HZ` was stale). All three were closed on
2026-08-17.

**Φ is measured and it is TALL.** From `data/20260817_182021_status_phi.csv`
(113 412 samples, 300 s, passive, nothing driven): **4×3 on rows a0–a3, cond 2.44
raw / 1.63 with unit-norm columns, null space dimension 1**, and **dropping any
single row still leaves rank 3 at cond 2.43–3.46**. That last clause is exactly
the property the 2026-08-06 closure said did not exist. The geometric prediction
above is confirmed rather than assumed. **Reproduced on the 23:12 coil pass**
(`data/20260817_231251_status_coils.csv`): cond **2.13**, null space 1, any single
row droppable at rank 3, SNR on a0–a3 **54–398**, and **|cos| 0.956 / 0.942 / 0.948**
against pure geometry. Φ is the half of that pass that came out well; A is not.

**A is measured** over coils 0–3, modal matrix rank 3, from
`data/20260817_182701_status_coils.csv`.

**AND ITS SIGNS ARE SETTLED — 12 of 12, 2026-08-18.** Three determinations agree on
every entry of `A[mode, coil]`: static steps at 23:12
(`data/20260817_231251_status_coils.csv`), static steps at 23:59
(`data/20260817_235950_status_coils.csv`), and the **resonant lock-in** on the first
of those files.

| mode | coil 0 | coil 1 | coil 2 | coil 3 |
|---|---|---|---|---|
| A (T1) | −1 | +1 | +1 | +1 |
| B (Z) | −1 | +1 | −1 | −1 |
| C (T2) | −1 | −1 | −1 | +1 |

That is stronger than three repeats: the static and the resonant determinations
**share no estimator** — a 20 s mean against a lock-in at the mode frequency — so
they cannot fail the same way, and **a static step is immune to the leftover-ring
contamination that ruins the driven phase**, because the ring is zero-mean about the
new equilibrium. `data/modal.json` now carries geometric Φ and the driven 23:12 A,
`basis: "geometric"`, written 00:15:54 and loaded by `eta` with all four coils
CONSISTENT. **A's magnitudes are still open** — the two DC passes reproduce
individual entries only to a factor 0.23–1.8.

**`signtest.py` then ran the diagonal law and all four per-mode sign patterns of A
back to back on the rig, 70 s each** — the tool was **deleted 2026-08-17**, because
a geometric Φ is exact and signed and leaves no per-mode sign to sweep; its five
records survive as `data/signtest_20260817_192808_*` — scored by the median
live-channel `ratio`
over t ≥ 30 s — the controller's own amplitude-over-baseline statistic, so 1.0
means "as quiet as its own calibration window":

| law | median ratio | p90 | faults | modal % |
|---|---|---|---|---|
| **modal +++** | **0.08** | 0.17 | 0 | **97 %** |
| modal +-- | 1.35 | 1.55 | 0 | 0 % |
| modal ++- | 1.51 | 1.94 | 0 | 0 % |
| modal +-+ | 1.54 | 1.99 | 0 | 0 % |
| diagonal | 1.58 | 2.01 | 0 | 0 % |

**0.08 against diagonal's 1.58 is a 95 % reduction, with the modal law engaged on
97 % of rows and zero faults.** `signtest.py` launches `bench.py zeta`, so every
row is **`zeta`** — `eta`'s estimator has not been on the rig, and 0.08 is the
number it has to reproduce rather than one it inherits. The three losing patterns
are **not** measurements
of the modal law: `zeta`'s colocation sign check **refused all three**, so they
silently ran the diagonal law — which is why their `modal %` is 0 and why they all
land on diagonal's number. That is a result *about the check*: on this data it was
a **complete** discriminator, not the partial one its own docstring predicts. It is
also a warning, because **a refusal is invisible in the score** — only the `modal`
column in the CSV tells those three trials apart from a diagonal run.

**The sign of A is physics, and getting it wrong is how a modal loop pumps.** On
resonance the response lags the drive by 90°, so once Φ is rotated real the whole
of A sits in the imaginary part with a determined sign: `A = -Im(a * conj(rot_phi))`.
Verified numerically against this repo's own lock-in — an undamped oscillator
driven on resonance from rest gives `H = -1.2024i` for `Φ·A > 0` and `+1.2024i` for
`Φ·A < 0`. `status.py --save-modal` now does exactly that. The version before it
picked the per-mode sign by maximising **colocation** consistency, and that loop took
the median ratio from 1.5 to **2.3** — it added energy. **Nothing offline catches
it**: a wrong-signed A is still exactly inverted by the allocator, so the demanded
modal force is realised perfectly in the model and backwards in the plant.

### Colocation does not hold on this rig

`A[m,j] = lambda_j * Phi[j,m]` is **the wrong identity**, so the "8 unknowns not 24,
12 minutes of bench time instead of 36" shortcut that used to be in this section is
**void**. Reported by the rig owner 2026-08-17: **four coils point the same way, two
are diametrically opposed, three different planes.** Two measurements agree — two
independent coil passes gave **different λ signs** (`+,+,+,+` vs `+,-,-,+`), and a
colocation-constrained rank-1 fit came out **second/first singular value 0.67**
where colocation predicts ~0. A must be measured over every coil used, at all three
modes.

The `DAC_MAP` "coil j dominates sensor j" heuristic cannot work either, for the same
reason: one coil moves the whole suspended rigid body. Weak evidence, not a wiring
verdict (§ 6).

**So `zeta`/`eta`'s colocation sign check is gating on an identity that does not
hold.** On the 19:28 data it happened to be a complete discriminator of wrong sign
patterns, and it **refused a cleaner 19:47 coil pass on 2 of 4 coils** — now
understood as the check being wrong rather than the data. Do not remove it without a
replacement, and do not read a refusal as evidence against a record.

### What "MIMO works" still needs before it is quoted as settled

- **Three kicks per law in one session**, run as blocks rather than interleaved.
  **Repeat it — and as of 2026-08-18 a repeat is not obtainable.** The null test ran
  and confirmed the refusal path (with `data/modal.json` moved aside, `eta` runs
  `epsilon`'s diagonal law and `modal` never engages: flag / rank `[('0','0')]` over
  the whole run), but both runs faulted on their first kick, so each yielded one
  partial kick and no comparable table. `CLAUDE.md` § 1 blocks this.
- **0.08 means the rig was 12× quieter than its own calibration window**, which is
  good enough to deserve suspicion. The kick test shows modal *dissipates* 5x faster
  (**superseded — see the headline table at the top**),
  so the loop does damp — but that the static 0.08 is damping rather than the loop
  **holding** the optic has **not** been excluded, and those are identical in
  `ratio`.
- **Coil 3 clips at 169 % of the half-window** (§ 5), so the dissipation guarantee
  was void for part of the run the 5x was measured on.
- **WITHDRAWN: "two modes are 4–5x worse than the third at the same gain."** The
  modal velocity rms A **0.64**, B **2.61**, C **3.15** over 28 775 samples is real,
  but the run it came from was in FAULT for 1085 of its 1168 s and holds only 14 s of
  damping, and a residual with no open-loop reference cannot tell a badly-damped mode
  from a hard-driven one. **Against a control it is the other way round**: modal wins
  **5.9x on B** and **6.7x on C**, and on **A** it is 6.31 against a best diagonal of
  6.86 — inside the run-to-run scatter of 2–3 (`CLAUDE.md` § *Per-mode residual,
  measured against a control*).
- **Three DOF are unobserved.** a4, a6 and a7 have no Φ row, so nothing acts on what
  they see: per-sensor residual counts rms a0–a3 **86.2 / 57.9 / 96.0 / 92.4**, a4
  side **43.4**, a6 vertical **61.3**, a7 vertical **32.6**. **a6 carries 61 counts
  rms that no loop touches**, and no vertical mode frequency has ever been measured.
  Those three are **not blind** — coherent fraction with the optic 0.46 / 0.26 / 0.50
  — so `eta` is being given a per-channel PID on them at gain **scaled by that
  fraction**: dissipation is linear in gain and injected noise quadratic, so some gain
  is always net dissipative, and the coherent fraction is the Wiener-optimal weight.
  **One ambient record, and it has still never run**: across both 2026-08-18 runs
  those three carried `|gain|` **exactly 0.0000 for 100 % of all DAMPING samples**,
  demoted at engage by `inband-floor` — which measures in-band amplitude at the three
  **horizontal** mode frequencies, where a4 senses X and a6/a7 sense Y. **Fourth
  instance of the same error family** (§ 7, `CLAUDE.md` § 6).
- **A's phase is contaminated — its SIGNS are not, as of 2026-08-18** (above: 12 of
  12 across three determinations). What follows is about the **magnitudes**, which
  are still open, and about the leftover-ring mechanism that produced the phase
  scatter. **The multisine named at the end of this bullet as the fix was built on
  2026-08-18 and is a NEGATIVE RESULT on resonance** — better phase spread,
  broken signs. That coil pass ran with the old anti-phase unwind,
  which pumped — residuals grew **0.188 → 0.277 → 0.422 V** against a 0.259 V drive
  peak — so A's off-quadrature fraction per mode came out **0.736 / 2.088 / 0.812**
  where on resonance it should be small. The sign survived; the magnitudes within
  each row are suspect, and the 5x rests on them. **Two defects in that unwind, both
  now fixed.** Retrying it *pumps* — there is no feedback in the retry, so the same
  command is reissued against a state now near rest, i.e. a fresh excitation; and the
  comparison was broken, because `peak` was a lock-in over the whole ramping drive
  and therefore read about half the end-of-drive amplitude, making `resid <= 0.25*peak`
  really `<= 0.125*final` and failing points that had cancelled fine. `UNWIND_EXTRA`
  is now 0 and `peak` is measured over the last 8 s of the drive — and **the 23:12
  pass ran with zero retries and is still unusable**: off-quadrature
  **0.652 / 0.830 / 1.234**, with the per-coil response phases scattered by
  **73–89°** where one mode has one phase up to a sign
  (`data/20260817_231251_status_coils.csv`). The cause is measured: τ > 138 s against
  a 74 s point, so **3.5–417 %** of the previous drive is still ringing when the next
  one starts, and a leftover ring is the *same mode*, so it leaves the response
  matrix **exactly rank 1** — which is why a 0.997+ rank-1 fraction did not catch it.
  **Four rescues have now failed**: subtracting the 6 s `pre` phasor
  (0.65/0.83/1.23 → 1.26/1.10/1.19), fitting the demodulated drive window to
  `c + b·t` (→ 0.87/1.19/1.25), the same subtraction at the multisine's 30 s
  `MULTI_PRE_S` (worst phase spread 41.3° → 36.7°, under the tool's own 5 °
  "no difference" line), and the **multisine itself** — 9.9 min instead of 18 and
  phase spread 31.6 / 36.7 / 7.8°, but only **7 of 12** signs agreeing with the DC
  passes, because an on-resonance response ramps and mode A (|H| 8.58) leaks into
  mode B's bin (|H| 3.03), which is exactly where the signs go wrong. **Do not retry
  any of them on resonance** (`CLAUDE.md` § 3). **And check the export,
  do not assume it**: the
  `data/modal.json` written at 20:05 on 2026-08-17 from
  `20260817_194726_status_coils.csv` is **refused** by `eta` — only 2 of 4 coils
  pass the colocation sign check (3 needed), off-quadrature 1.662 / 0.737 / 1.181 —
  while the 19:26 export from `20260817_182701_status_coils.csv`, the pair that
  scored 0.08, keeps all four coils at cond 1.97, rank 3. **Two exports one evening
  apart and only one of them runs the modal law**, so read the `[modal]` banner
  every time.
- **`F_MODE_HZ` is current in `zeta` and `eta`** (0.72294 / 0.99193 / 1.65657 Hz,
  consensus of 5 sensors, spread 0.0005 Hz) and **still stale in `epsilon`**. The
  modes moved up to 0.0170 Hz — **9 half-widths** — since 2026-08-06, so re-measure
  with `status.py sensors` at the start of every session, before driving anything.
  **`status.py`'s own `MODES` were stale until 2026-08-18** — they were the
  2026-08-06 values, up to 0.0170 Hz out, and now carry the 840 s empty-room set
  **0.72194 / 0.99193 / 1.65607 Hz**. The constant is a default, not a measurement.

### The 2026-08-06 closure, kept because its measurements stand

- Modal control needs Φ *taller* than wide or a single sensor loss makes the
  modes unobservable. On that data only **three** sensors had a determined row —
  a0, a2, a3 — because **a1's noise floor was 0.024 V against a0's 0.0047 V** and
  it resolved 2–5 of the 6 tones it needed. With three modes Φ came out
  **square**, which deletes graceful degradation *and* the sensor-disagreement
  check: a square Φ reproduces any reading exactly, **measured 7.4e-16 over
  137 528 damping samples on hardware 2026-08-07**. It would also have failed
  exactly when wanted — a hard kick railed all three of those sensors at once.
  **Square was never a property of the geometry**, only of having determined three
  rows.
- **It was never a coil problem.** Coils 0–3 are fine: the allocator is 3×4,
  **cond 1.41**, singular values 1.000 / 0.885 / 0.711. Coils 4–7 have pairwise
  column cosine **+0.966** — one shared direction at **¼–⅐** the strength of
  coils 0–3 — and an anti-phase test meant to separate "they share one winding"
  from "four real coils on one DOF" came back **inconclusive** (ratio 0.579,
  shape cosine −0.488, coils 4/6 returning 0.2–0.9 counts/V against the control
  coil's 44). MIMO never wanted those four.

### One design point, kept because it was proposed and cannot work

**Reordering a matrix is a no-op.** A permutation matrix is orthogonal, so permuting
rows or columns leaves rank, condition number and achievable modal force identically
unchanged — it only relabels which coil is which, and it is not an alternative to
measuring A. **Subset selection** (pivoted QR) is real but still needs A. **Rank
reduction** is real and **both `zeta` and `eta` do it** — `Modal._build_tables` keeps
`sum(sv >= sv[0]/MODAL_COND_MAX)` directions and refuses only what is genuinely
unreachable, instead of throwing a whole mask back to diagonal because one modal
direction is badly conditioned. On the measured A the full coil set runs cond 1.97
at rank 3, and **80 of 256 coil masks run MIMO**.

### Convolution kernels — the general form of every law here, and what blocks it

Proposed by the rig owner 2026-08-17, ~23:50: *"right now we are cascaded, i would
eventually consider making this just straight up conv kernels."* **Nothing is
built, nothing is measured, and this is a TODO and not a plan.**

The general LTI MIMO controller is a bank of convolution kernels, one from every
sensor to every coil:

    u_i(t) = Σ_j ∫ h_ij(τ) y_j(t−τ) dτ

**Every control law in this repo is a special case of it.** The diagonal law is a
per-channel kernel — bandpass-and-differentiate, or `epsilon`'s Kalman velocity
estimator, times a scalar gain. The modal law is `A_C⁺ K Φᵀ` wrapped around a
velocity estimator, i.e. a **rank-3** kernel bank whose three kernels are narrow
resonant filters at the mode frequencies. Writing it as kernels is not a new
control idea; it is dropping the cascade and stating the same object directly.

**What it buys that the present form cannot: per-mode phase shaping.** The only
per-mode freedom in the shipping law is a scalar gain — `MODAL_KP` is flat
`[0.035, 0.035, 0.035]` (`osem.zeta.py:476`, `osem.eta.py:364`). A kernel sets
magnitude **and** phase per mode.

**Why it is blocked, and this is the important half.** A kernel bank has far more
free parameters than three gains, and **nothing offline catches a wrong one.** That
is not a general caution — it is the failure measured on this rig on 2026-08-17.
The per-mode sign of A was picked wrong; the selftest, the modal gate and the
colocation check all passed; and the loop **pumped**, median channel `ratio` 1.5
diagonal → 2.3 modal. A wrong-signed A is inverted *exactly* by the allocator, so
the demanded force is realised perfectly in the model and backwards in the plant.
**A kernel designed against a contaminated plant model is that same bug with more
degrees of freedom.** So it is blocked on A being measured cleanly — and as of
2026-08-18 **it is not**. A's **signs** are settled (12 of 12, three determinations),
but its **magnitudes** are not: the 23:12 pass ran with zero retries and still came
out off-quadrature 0.652 / 0.830 / 1.234 with per-coil phases scattered 73–89°, the
export now in `data/modal.json` is off-quadrature **0.624 / 0.749 / 1.196**, the two
DC passes agree on entries only to a factor 0.23–1.8, and the multisine that was to
fix it broke the signs (above, and `CLAUDE.md` § 3). A kernel sets **phase** per
mode, so a plant whose magnitudes and phases are the unmeasured half is exactly the
wrong input for one.

**It is not the feedforward canceller that was ruled out**, and the distinction is
worth stating because the two look alike written down. That canceller predicted the
disturbance and injected anti-phase; it was ruled out on 31 logs, where it removes
**43–62 %** of the in-band power from open-loop data and **−11 % to +13 %** on top
of the existing velocity damper, because the damper has already taken the
predictable part. A kernel bank is **inside** the feedback loop. `CLAUDE.md`'s
closed section says of exactly that case: *"Not ruled out: a resonant /
internal-model term inside the feedback loop. That needs the plant known, not the
disturbance predictable."* **Convolution kernels are the general form of that
not-ruled-out idea**, and "the plant known" is precisely the clean A that blocks
them. Ranked low in `CLAUDE.md` (§ 16) because it is blocked, not because it is
small.

### Actuator budget ramping — one budget, reallocated as the disturbance dies

Proposed by the rig owner 2026-08-17, ~23:55: *"we can have a global budget value,
and as corrections happen, we can reallocate budget to other parts of the system...
when MIMO is done, we allocate more to gains, and if the disturbance gets high again
we switch them back to mimo... a small budget ramp calculation, where as the
disturbance dies we slowly ramp down MIMO and ramp up the other gains."* **Nothing
is built, nothing is measured, and this is a TODO and not a plan.**

Hold one **global** actuator budget and reallocate it between control terms as
conditions change, instead of fixing the split at build time: as the disturbance
falls, ramp the modal term down and the per-channel gains up; as it rises, ramp back
toward modal. A **ramp**, keyed on a measured disturbance level — not a switch.

**The budget is real and already overspent, so this is a fix for a defect that
exists rather than a new feature.** § 5: coil 3 reached **0.4229 V — 169 % of the
0.25 V half-window** — and clipped, while the modal allocation stayed inside its own
**0.20 V** cap the whole time, because the derivative term is added after the cap.
The split between the modal and per-channel terms is today implicit and unmanaged.
**And modal's value is not uniform across the modes**: against the best of four
diagonal runs it wins **5.9x on B** and **6.7x on C**, but on **A it is 6.31 against
6.86 — inside the run-to-run scatter of 2–3** (`CLAUDE.md` § *Per-mode residual,
measured against a control*). There is also a bottom to the ramp: the residual
**0.0089 V** is the actuator's 0.5 mV deadband and 10 ms throttle, not sensing
(0.0089 / 0.0087 / 0.0094 V at 20 / 5 / 0 mV of injected sensor noise, simulator),
and budget spent below it buys nothing.

**Three constraints, and they are arguments rather than results.** The ramp must be
**slow against the slowest mode** — 0.72194 Hz is a 1.4 s period, so a time constant
of **10 s or more**; faster modulates the loop gain inside the control band and is a
new disturbance source. **Two separately dissipative laws do not make a
time-varying blend of them dissipative** — that is open, and nothing offline catches
a law that pumps (the sign-of-A failure above passed every offline check). And it
needs a disturbance statistic: the per-channel `ratio` is the obvious one, and it is
**referenced to a baseline measured with the loop off**, so the better the loop damps
the larger it reads for the same physical motion. It also used to **freeze on a
runaway trip** — 3.59 identical for 1085 s — which is fixed as of 2026-08-18, while
the zero-gain-reference defect is not (`CLAUDE.md` § 1).

**Independent of the convolution kernels above** — the rig owner's claim, not a
measurement — and implementable before or after them. It is not blocked on a clean
A, because it reallocates between laws that already exist. `CLAUDE.md` § 17.

Details in `versions.md` § *On the bench, 2026-08-17*, § *On the bench, 2026-08-18*,
§ *eta*, `analysis/mimo_closed.md`, `analysis/coil_qual.py`, `analysis/antiphase.py`,
and `jerk.py`'s docstring. The ranked queue is `CLAUDE.md`: frequencies, Φ and A's
signs are settled, and **what is left is A's magnitudes, the hand-kick fault that
does not clear (`CLAUDE.md` § 1, and it blocks the repeat kick test), the hybrid PID
channels that have never run, and coil 3's clipping**.

---

# Off the bench

A simulator runs **this code** — not a reimplementation — so behaviour can be
checked without hardware.

```bash
make scope            # 8-channel scope: the real board, or a live run's CSV
make check            # the behavioural suite, headless, every version
make test             # interactive terminal runner: pick a version, watch it
                      # damp, press x to kick the optic
make check            # the behavioural suite, headless, every version
make list             # what versions exist
```

`make check` runs every controller. Only `osem.sysid.py` is skipped — a
measurement tool with no `Controller` in it — and the skip prints its reason.
**Eight-channel controllers are no longer skipped**: `sim/server.py` carries a
measured eight-OSEM body as well as the four (`SIM_CHANNELS = (4, 8)`), built
from the 2026-08-04 and 2026-08-06 logs, so the eight-channel rungs are scored
against it. It models **two** modes, though, and the ringdown measured **three**.
What is stubbed and what is modelled is in `versions.md`
§ *What the simulator can and cannot tell you*. What it **cannot** tell you, and
neither can the interlocks:

- **A safe gain.** The plant is linear apart from the ADC, so it will not reproduce
  the instability at Kp = −0.040 — here, more negative gain is monotonically more
  damping all the way to −0.6. The ADC model does not bring that any closer; it
  reproduces clipping, not instability.
- **A wrong coil map.** `sim/server.py` round-trips its held-voltage dict through
  whatever map the controller declares. Only the bench can catch one.
- **The wrong baud rate, or the wrong board.** It never opens a port. The board
  attached on 2026-08-17 is a different physical board from the one this repo was
  written against, at a different baud — see § 2.
- **Where the modes are.** Its two frequencies are the 2026-08-06 values, and
  those moved by up to 0.017 Hz by 2026-08-17. A simulator cannot tell you that the
  plant drifted.
- **Anything modal.** It models **two** modes and the controllers run **three**, so
  `eta`'s modal path is not reachable from `make check` at all. `osem.eta.py
  --selftest` is where the modal math and filter are asserted.

---

# Reference

| | |
|---|---|
| `osem.epsilon.py` … `osem.eta.py` | the three controllers, see `versions.md`. `zeta` ran on the rig 2026-08-17 and `eta` 2026-08-18; **`epsilon` has not**. `zero`, `alpha`, `beta`, `delta` and every numbered rung are in git history |
| `stdlib.py` | the machinery the controllers used to each carry a copy of. **New 2026-08-17**, and it ends the standalone rule — `bench.py` follows `GAIN_SOURCE` to the module the gains come from |
| `ladder.py` | the one owner of the ladder names and their order — `harness.py` and `bench.py` both import it |
| `status.py` | the bench census tool: probe baud, re-measure the modes, grade all eight sensors, drive the coils, run the Φ gate, write `data/modal.json`. Φ and A came out of this. **2026-08-18: `--phi geo\|svd` (geometric by default), `phase_consistency` / `phase_compare`, `--multisine` (a negative result on resonance), `--settle` (neutral, off by default), `--hand-damp`, and `MODES` refreshed off the 840 s empty-room record** |
| `dof.py` | projects a record onto the rigid-body coordinates Z / T1 / T2 / warp. The corner assignment came out of this, by minimising warp |
| `slopesign.py` | which way a sensor moves when its own coil pushes — the sign the hybrid PID needs. a6/a7 settled at 5.0σ and 6.5σ; **a4 is still open at 2.9σ** |
| `jerk.py` | the kick test: a human shoves the table when it asks, and it fits the decay after each kick. **The only tool here that measures a dissipation rate rather than an amplitude** — it is what separates damping from holding, and where the 5x comes from |
| `data/modal.json` | Φ and A, written by `status.py --save-modal` and read by `zeta`/`eta`. **Not** in any controller: with it absent, stale, or failing a check they run the diagonal law and say which. Since 2026-08-18 it carries `basis: "geometric"` — Φ from geometry, A driven from `20260817_231251_status_coils.csv` |
| `tune.py` | on-hardware gain identification: dither, lock-in, sign de-rotation |
| `analysis/` | offline analysis and one-off bench experiments, each with its own `.md` |
| `osem.sysid.py` | stepped-sine actuation-matrix measurement tool — `KIND = "sysid"`, no control loop |
| `sysid.py` | the lock-in, rank check and recording behind it |
| `pyDAC.py` | serial transport (`DACController`) — waits for its `OK` ack |
| `pyDAC2.py` | `FastDAC`: writes and returns, for the continuous-excitation tools. `probe_baud`/`resolve_baud` own the baud probe for the whole tree |
| `arduino.ino` | Arduino sketch: ADC stream + AD5628 SPI |
| `bench.py` | the on-hardware entry point: `make run`, `make arduino`, `make ports` |
| `harness.py` | one entry point for everything off the bench |
| `sim/`, `test/` | simulated plant + browser UI; interactive runner |
| `bench/`, `data/` | bench-session scripts and console logs; raw CSV/JSON from every run |
| `versions.md` | what each version changes, the state machine and hazards, and the bench results |
| `CLAUDE.md` | pending bench work, what is **not** established, and where the constants came from. `provenance.md` and `research.md` were folded into it and deleted 2026-08-17 |

## Serial protocol

Changing any line here means editing `arduino.ino` and `pyDAC.py` together.

| Direction | Message | Meaning |
|---|---|---|
| MCU → host | `READY` | sent once after `setup()` |
| host → MCU | `SET <ch> <volts>` | sets DAC channel 0–7; replies `OK ch=.. v=..` or `ERR ..` |
| host → MCU | `STREAM` / `STOP` | toggles sampling; replies `STREAMING` / `STOPPED` |
| MCU → host | `a0,...,a7` | free-running ADC counts 0–1023, one line per loop, only while streaming |

The stream is **eight columns**. Measured rate — serial print dominates, and
**the baud rate is a property of the board, not of this repo**, because
`arduino.ino` has a `BAUD` command:

| baud | rate | measured |
|---|---|---|
| 115200 | **~348 Hz** (2.88 ms/sample) | 2026-08-03/04, on the clone board |
| 500000 | **1024–1113 Hz** | 2026-08-06, four `delta`-lineage runs, clone board |
| 115200 | **418–435 Hz** | **2026-08-17**, six `status.py` records, Mega 2560 R3 |

The 08-17 rate is 4.2x the 100 Hz control clock and inside the range `delta`'s
`decimate` was validated at (347 Hz), so 115200 is usable — it just is not what
any constant in the tree says. Why 418–435 Hz at the same baud that gave 348 Hz
in August is **not established**; it is a different board.

**A row that parses is not a row that is valid.** `read_sample` accepted anything
starting with a digit that split into ≥ N fields, and at 500000 baud with the ack
no longer being drained, an `OK ch=..` reply landing on a buffer boundary splices
two fields' digits together into a row that parses fine. One 240 s log has
**ten** rows with a count outside 0..1023 — 5659, 65690, 522676 — against zero in
every slower log from the same session. One of them is +45.6 V into the bandpass
and a velocity estimate peaking at **1401 V/s** into a 0.5 V rail. `delta`'s
`sample-guard` range-checks every count and drops the row. **Any new tool reading
this stream at 500000 needs that check**; `pyDAC2.FastDAC` does not do it for you.

`SET` and the stream share one wire, so an `OK` arrives buried in sample lines.
`set_voltage` reads up to 50 lines looking for it, and `RateLimitedActuator.send`
swallows the `RuntimeError` when it never arrives, on purpose: the next sample
resends.

**Exactly one threshold used to be counted in *samples* rather than seconds:**
`MAX_CONSECUTIVE_SATURATED = 30`, which is 86 ms at 348 Hz and 27 ms at 1111 Hz —
so its wall-clock meaning moved with the stream rate, and at 500000 baud it became
unreachable. `sat-window` made it a time window, and `decimate` depends on that:
**no threshold on any current rung is counted in samples**, which is what makes
decimating the control step to a fixed 100 Hz safe. The rail window was never one
of them — `RAIL_SUSTAIN_S` has always been in seconds.

## Two voltage scales — do not conflate them

- **Sensing:** ADC reference `A_VCC = 5.02` V over `ADC_MAX_COUNTS = 1023`.
- **Actuation:** DAC full scale **2.5 V** (internal reference). Firmware clamps to
  0–2.5, `set_voltage` rejects outside that, and the controller further restricts
  itself to 0–0.5 V around `BIAS = 0.25` — that is the ±0.25 V of authority the
  saturation interlock in § 5 is counting against.
