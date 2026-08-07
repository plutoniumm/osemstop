# OSEM suspension damping

A three-layer control stack that damps a suspended optic using OSEM (Optical Sensor
and Electro-Magnetic actuator) shadow sensors. Eight OSEMs are connected; four of
them (a0–a3) carry usable in-band signal and their four coils push back. The whole
thing is one closed loop split across a language boundary:

```
arduino.ino (Arduino firmware)     <-- SPI --> AD5628 octal DAC --> coils
   ^  |                                                              |
   |  | serial @500000                                               v
   |  v                                                      suspended optic
pyDAC.py (DACController: transport)                                   |
   ^  |                                                               |
   |  v                                                    OSEMs --> A0..A3
osem.<name>.py (control law, safety, logging) <-- analogRead ----------'
```

---

# Bench quickstart

**Read this before flashing anything.**

## 0. Which version to run

The ladder is **named, not numbered**: `zero`, `alpha`, `beta`, `delta`, and
`epsilon` when it is written. Numbers stopped carrying information once v10 and
v11 were both "newer than v9" and one of them damped worse than v9 did; numbers
also imply a total order the history does not have, since v7 was a branch off v5
rather than a successor. The order lives in `ladder.py` and nothing is derived
from a filename. `make list` prints the table; old numbered names still work on
the command line and are redirected with a note.

They all run the **same control law**: independent SISO velocity-feedback PID
loops, one per channel, with ch2's deliberate opposite sign. They differ in the
supervisor wrapped around it and in the transport underneath it.

- **`osem.delta.py` — run this one, and develop against it.** Eight channels,
  500000 baud, and a **100 Hz control clock decoupled from the wire**, plus the
  four fixes that made that safe: `sample-guard`, `decimate`, `persist-baseline`,
  `baseline-sanity`. Time-weighted on the bench 2026-08-06 it damped **77.5 %**
  of a 244 s five-kick run. The single change that did it was the runaway
  breaker's lag: 2 s sits inside all three measured mode-beat periods (3.578 /
  1.551 / 1.082 s), so the breaker was reading the beat as growth. At 10 s it is
  not. **It has never announced `LOCKED`** — see § 4; that is a quorum bug, not a
  damping one.
- **`osem.beta.py`** — the last four-channel rung, and still the best raw DAMPING
  fraction on record (**83.1 %**, 2026-08-04). Where `runaway-trend` was
  introduced: the breaker tests for *growth* rather than level, which is what
  stopped the post-kick fault thrash.
- **`osem.alpha.py`** — where **`auto-disable`** was introduced: one blind OSEM is
  demoted and parked rather than faulting the whole rig.
- **`osem.zero.py`** — the original, and the baseline the ladder is measured
  against. **One channel**, and a saturation trip **latches**: FAULT is an
  absorbing state needing a manual restart. Do not run it unattended.
- **`osem.sysid.py`** — stepped-sine actuation-matrix measurement. It damps
  nothing and closes no loop.

**In git history only** (`git log --diff-filter=D --name-only`): the numbered
rungs that the named ladder replaced or superseded — v1, v2, v3, v5, v5.5, v7,
v8, v10, v11, v13, and the v6.1 / v6.5 / v6.6 sysid variants. `versions.md` says
what each one was and why it went. **`osem.v13.py` was deleted 2026-08-07**: it
existed only to carry a modal law, and modal control is closed by measurement
(§ 8).

**All eight OSEMs are connected** — confirmed on an oscilloscope. Earlier notes
in this repo said a4–a7 were unwired; that was wrong. The real split is
**in-band vs out-of-band**:

| | senses suspension motion | dominated by |
|---|---|---|
| a0–a3 | **yes**, 77–99% of power in 0.4–3 Hz | the 1.01 / 1.66 Hz modes |
| a5 | **yes**, 22–71% in band | 1.046 Hz, at ~1/12 of a0's gain |
| a4, a6, a7 | **no**, 0.1–9% in band | a 6.19 Hz interference line |

a4, a6 and a7 carry real signal (raw std 13–105 counts) but 91–99% of it is at
3–20 Hz, with a narrow 6.19 Hz line carrying most of that. They **do** move on a
hard kick — seen on an oscilloscope 2026-08-06 — so they are in the loop, just at
roughly 1/100 of a0–a3's in-band gain.

**Why is not established.** This file previously said "the flags are outside the
linear partial-shadow region", i.e. an alignment job. The 2026-08-06 resting
counts do not support that: **a4 sits at 567.5 against a mid-scale of 511.5 with
a 30-count swing** — inside its linear range, not pinned at an end. a7 is at
734.7 and a6 at 860.6, higher but still off the rail. Treat the low gain as
measured and the cause as open (`versions.md` § *v12*).

Methodological note worth keeping: the original call came from a DC-response
threshold and raw time-domain correlation, and **both are blind to a small
coherent signal**. Test for a peak at the known mechanical resonance instead.

Each controller declares a `BENCH_STATUS` and `make list` prints it, but
**`bench.py` does not gate on it** — the statuses went stale faster than they were
updated, and a gate whose data is wrong only teaches you to click through it. The
preflight and the printed gain vectors are the load-bearing checks, because both
come from the file that is about to run. `zero` ran 2026-08-03, `alpha` and
`beta` 2026-08-04, `delta` 2026-08-06 and 08-07 (`versions.md` § *On the bench*
and § *delta*, console logs in `bench/20260804/` and `bench/20260806/`).

## 1. Flash the Arduino

`arduino.ino` is the firmware. arduino-cli will not look at a sketch until it is
named `<dir>/<dir>.ino`, so one command does the copy, the compile and the upload:

```bash
make arduino                                   # defaults to arduino:avr:mega
make arduino FQBN=arduino:sam:arduino_due_x    # if it is a Due
make arduino PORT=/dev/cu.usbserial-1120       # skip auto-detection
```

Board is an Arduino **Mega or Due** — `CS` is on pin 53, which is Mega/Due specific.

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

## 3. Run

```bash
cd /path/to/this/repo          # CSV goes to ./data/ RELATIVE TO CWD
make run V=delta               # or bare `make run` for a picker
```

**`delta` does not prompt.** Every earlier version blocks on `input()` after
the biases are applied. `delta` keeps the prompt only when `sys.stdin.isatty()`, so it
runs unattended — which also means **the coils are energised the moment it
starts**, with no Enter key between you and that.

`make run` **preflights** the port — opening it through the real `DACController`,
which raises unless the sketch answers `READY`, so "wrong port" and "board not
flashed" are distinguishable before the optic is swinging — then prints the bench
status and the gain vectors it is about to apply, and hands over to that version's
`main()`. The controller then calibrates with the gain at zero before engaging:
the long quiet stretch at the start of a run is that calibration, not a hang.
**20 s on v3 and v7.** On v9 the 20 s is a *ceiling* — `fast-calib` exits as soon
as the floor settles — but that has only ever paid off in the simulator; on the
bench it ran the full ceiling every time and fell back to v7's estimator, which is
the designed failure mode. See `versions.md`. (The deleted v0–v2 calibrated 8 s.)

## 4. What you should see

```
[CALIBRATING] measuring baseline noise for 20s (outputs held at bias)...
[DAMPING] baseline set (ch0=0.1804V, ...). Gain will schedule between -0.035 and -0.030
*** LOCKED -- 14.5s after gain was applied (t=34.5s total) ***
```

The **lock time is the deliverable**. A run that never prints `LOCKED` did not
work, whatever the traces look like.

**With one caveat you will hit immediately: v10, v11 and v12 never print it.**
Not once in the whole 2026-08-06 session. This is a quorum bug, not a damping
failure. The announcement needs every `enabled & healthy` channel quiet, and
`ENABLE_CHANNEL` is `True` on all eight; a4/a6/a7 demote themselves to `NOSIG`
and drop out, but **a5 stays healthy while `STEADY_GAIN[5] = 0`**, so nothing
drives it and it sits at ratio 1.26–2.53 against a threshold of 0.35. One
undriven channel vetoes the whole rig, permanently. a0–a3 do reach lock together
— 6 simultaneous episodes in `20260806_200822` — so until this is fixed, read
the four `ch{i}_locked` columns in the CSV rather than the console.

`data/<YYYYMMDD_HHMMSS>_fast_lock.csv` gets 13 columns per channel plus `time_s`
and `state` — the 13th is `healthy`, added by v4 and kept by v7 and v9; v3 wrote
12, since it has no per-channel health. v11 and v12 add two run-level columns,
`ctl` and `n_avg` (the decimation's samples-per-step). It is line-buffered and flushed
every 200 rows, so it **survives an unclean kill** and is enough to reconstruct the
run offline. `data/` is gitignored.

## 5. On the scope, watch for

- **Clean, unclipped convergence.** Amplitude falling, no flat tops.
- **The DAC output staying inside 0–0.5 V.** Thirty consecutive samples against
  either limit trips a saturation fault.
- **`RAIL!` messages** — a fully occluded OSEM pins the ADC at 0 and the bandpassed
  signal flatlines, which an RMS-only check reads as *perfect stability* while the
  sensor is blind. That is why the interlock reads raw counts.

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
sensor range before trusting any tuning done on top of it** — `CLAUDE.md` item 1 is
the bias sweep that would tell you whether the offset is electrical or mechanical.

## 6. Safety limits — do not exceed without a scope on it

**`Kp = −0.040` is the documented instability / rail onset.** `-0.030` damps
cleanly; `-0.040` does not. **Do not raise gain past `-0.035`** (the `CAPTURE_GAIN`
already in use, itself not independently validated) **unattended.**

Be aware: **that −0.04 limit has no recorded evidence anywhere.** It is one
sentence in the docstring of `osem.v0.py` — deleted 2026-08-04, readable as
`git show HEAD:osem.v0.py` — written two days before a report that never mentions
it, and the simulator provably cannot reproduce it. Treat it as real and
uncharacterised — `research.md` item 3 and `CLAUDE.md` item 5 are the bench task
to characterise it.

**`STEADY_GAIN[2] = +0.010` is positive** where the others are negative. That is not
a typo: A2 damped *fastest* of the four at that gain, which only holds if its coil or
OSEM is mounted the other way round. **Do not "correct" it.** See `provenance.md`
§ *Table 1*.

**The coils were rewired 2026-08-03.** Sensor index `i` (0–3) → analog pin `A<i>` →
DAC channel `DAC_CHANNELS[i] = [1, 3, 5, 7]`. The full eight-pair map is
`[1, 3, 5, 7, 0, 2, 4, 6]` (`provenance.md` § *Coil wiring*); the controllers take
the first four because only A0–A3 have OSEMs on them. The previous map,
`[0, 2, 4, 6]`, now points at the coils for sensors 4–7 — a controller still holding
it closes every loop onto the wrong actuator, and **neither the simulator nor the
interlocks can tell. Only the bench catches a wrong map.**

## 7. If something trips

The state machine is `CALIBRATING → DAMPING → FAULT → (auto-recover) →
CALIBRATING`. A fault zeroes all gains and holds every output at bias. If
everything stays clear for 5 s it re-calibrates and resumes on its own.

**v7 and v9 demote one blind OSEM rather than freezing the rig.** They park that
channel (`ch1:DOWN`), keep the rest damping, and re-arm it 2 s after the sensor
comes back — a global fault needs *every* channel blind. A rail during
`CALIBRATING` is global in both; you cannot calibrate a blind sensor. **v3 has no
`auto-disable` and freezes all four on any one rail** — expected, and one of the
things it is the baseline for.

**A blind channel is still not fully handled, in any version.** `auto-disable`
keys on *rail*, and a disconnected OSEM does not rail — it sits mid-scale and
flat. It then calibrates a near-zero baseline, so any noise reads as a runaway and
trips the *global* interlock. That is what killed the eight-channel v5.5 run:
ch0–ch3 were damping at ratio 0.07–0.26 while ch4–ch7, calibrated at ~0.002 V,
faulted the whole rig six times in 120 s. A minimum-plausible-baseline check is
the missing guard.

**One thing that no longer bites, but that the docs and the suite are written
around:** in v0/v1/v2 a *saturation* trip latched `FAULT` permanently and needed a
manual restart while a runaway trip recovered fine (deliberate, fixed in v3,
reproduced on hardware 2026-08-03). See `versions.md` § *Hazards*; it is behaviour
you would get back by restoring one of those files from git.

## 8. MIMO / modal control — blocked on better sensors and coils

Everything above is **four independent SISO loops**. The obvious next step is to
diagonalise the plant and damp the modes directly instead of the channels. It was
designed, measured and **ruled out on 2026-08-06**. Not on preference — on
numbers. It needs hardware this rig does not have:

- **Better sensors.** Modal control needs the sensor→mode matrix Φ, and Φ has to
  be *taller* than it is wide or a single sensor loss makes the modes
  unobservable. There are three modes (0.7155 / 0.9949 / 1.6396 Hz) and only
  **three** sensors with a determined row — a0, a2, a3. **a1 never gets there:
  its noise floor is 0.024 V against a0's 0.0047 V**, so it resolves 2–5 of the
  6 tones it needs. Φ comes out **square**, which deletes graceful degradation
  *and* the sensor-disagreement check (a square Φ reproduces any reading exactly
  — measured residual `1.8e-16`). And it would fail exactly when wanted: a hard
  kick railed all three good sensors at once.
- **Better coils.** Coils 0–3 are actually fine for this — the allocator is 3×4,
  **cond 1.41**. The problem is the other four. Coils 4–7 have pairwise column
  cosine **+0.966**: they push in one shared direction, at **¼–⅐** the strength
  of coils 0–3. Four coils on a rigid body cannot physically do that, so
  something is wrong upstream of the control law. An anti-phase test meant to
  separate "they share one winding" from "four real coils on one DOF" came back
  **inconclusive** (ratio 0.579, shape cosine −0.488, coils 4/6 returning
  0.2–0.9 counts/V against the control coil's 44).

So: **eight OSEMs, but four usable sensor rows and four usable coils, and the
four of each do not overlap enough to build an over-determined modal loop.**
Details and the scripts in `versions.md` § *v12*, `analysis/mimo_design.md`,
`analysis/coil_qual.py`, `analysis/antiphase.py`. What *is* worth doing without
new hardware is in `versions.md` § *Still open after v12* — diagonal
reallocation, then a Kalman velocity estimator.

---

# Off the bench

A simulator runs **this code** — not a reimplementation — so behaviour can be
checked without hardware.

```bash
make sim              # browser simulator, http://localhost:8770
make sim V=v9         # a specific version
make test             # interactive terminal runner: pick a version, watch it
                      # damp, press x to kick the optic
make check            # the behavioural suite, headless, every version
make list             # what versions exist
```

`make check` runs every controller. `osem.v6.py` and `osem.v13.py` are skipped —
the first is a measurement tool with no `Controller` in it, the second a
skeleton — and both skips print their reason. **Eight-channel controllers are
no longer skipped**: `sim/server.py` now carries a measured eight-OSEM body as
well as the four (`SIM_CHANNELS = (4, 8)`), built from the 2026-08-04 and
2026-08-06 logs, so v10, v11 and v12 are scored against it. It models **two**
modes, though, and the ringdown has since measured **three**.
What is stubbed and what is modelled is in `versions.md`
§ *What the simulator can and cannot tell you*. What it **cannot** tell you, and
neither can the interlocks:

- **A safe gain.** The plant is linear apart from the ADC, so it will not reproduce
  the instability at Kp = −0.040 — here, more negative gain is monotonically more
  damping all the way to −0.6. The ADC model does not bring that any closer; it
  reproduces clipping, not instability.
- **A wrong coil map.** `sim/server.py` round-trips its held-voltage dict through
  whatever map the controller declares. Only the bench can catch one.

---

# Reference

| | |
|---|---|
| `osem.v3.py` … `osem.v12.py` | the controllers — complete and standalone, see `versions.md`. **v12 is the one to run.** v0–v2, v4, v5, v5.5 and v8 are in git history |
| `osem.v13.py` | skeleton only — `main()` raises `SystemExit`. Not a controller yet |
| `tune.py` | on-hardware gain identification: dither, lock-in, sign de-rotation |
| `analysis/` | offline analysis and one-off bench experiments, each with its own `.md` |
| `osem.v6.py` | stepped-sine actuation-matrix measurement tool — `KIND = "sysid"`, no control loop |
| `sysid.py` | the lock-in, rank check and recording behind it |
| `pyDAC.py` | serial transport (`DACController`) — waits for its `OK` ack |
| `pyDAC2.py` | `FastDAC`: writes and returns, for the continuous-excitation tools |
| `arduino.ino` | Arduino sketch: ADC stream + AD5628 SPI |
| `bench.py` | the on-hardware entry point: `make run`, `make arduino`, `make ports` |
| `harness.py` | one entry point for everything off the bench |
| `sim/`, `test/` | simulated plant + browser UI; interactive runner |
| `bench/`, `data/` | bench-session scripts and console logs; raw CSV/JSON from every run |
| `versions.md` | what each version changes, the state machine and hazards, and the bench results |
| `provenance.md` | where the constants came from, and the coil wiring map |
| `research.md` | ranked queue of what to try next, and what was ruled out |
| `CLAUDE.md` | pending bench work — the experiments that need the board connected |

## Serial protocol

Changing any line here means editing `arduino.ino` and `pyDAC.py` together.

| Direction | Message | Meaning |
|---|---|---|
| MCU → host | `READY` | sent once after `setup()` |
| host → MCU | `SET <ch> <volts>` | sets DAC channel 0–7; replies `OK ch=.. v=..` or `ERR ..` |
| host → MCU | `STREAM` / `STOP` | toggles sampling; replies `STREAMING` / `STOPPED` |
| MCU → host | `a0,...,a7` | free-running ADC counts 0–1023, one line per loop, only while streaming |

The stream is **eight columns**. Measured rate: **~348 Hz** at 115200 baud
(2.88 ms/sample) and **1024–1113 Hz** at 500000 (v11 and v12, measured over four
runs on 2026-08-06) — serial print dominates.

**A row that parses is not a row that is valid.** `read_sample` accepted anything
starting with a digit that split into ≥ N fields, and at 500000 baud with the ack
no longer being drained, an `OK ch=..` reply landing on a buffer boundary splices
two fields' digits together into a row that parses fine. v11's 240 s log has
**ten** rows with a count outside 0..1023 — 5659, 65690, 522676 — against zero in
every slower log from the same session. One of them is +45.6 V into the bandpass
and a velocity estimate peaking at **1401 V/s** into a 0.5 V rail. v12's
`sample-guard` range-checks every count and drops the row. **Any new tool reading
this stream at 500000 needs that check**; `pyDAC2.FastDAC` does not do it for you.

`SET` and the stream share one wire, so an `OK` arrives buried in sample lines.
`set_voltage` reads up to 50 lines looking for it, and `RateLimitedActuator.send`
swallows the `RuntimeError` when it never arrives, on purpose: the next sample
resends.

**Up to v10, exactly one** threshold was counted in *samples* rather than
seconds: `MAX_CONSECUTIVE_SATURATED = 30`, which is 86 ms at 348 Hz and 27 ms at
1111 Hz — so its wall-clock meaning moved with the stream rate, and at 500000
baud it became unreachable. v11's `sat-window` made it a time window, and v12
depends on that: **no threshold in v11 or v12 is counted in samples**, which is
what makes decimating the control step to a fixed 100 Hz safe. The rail window
was never one of them: `RAIL_SUSTAIN_S` is in seconds in every version. (v0's
docstring describes it as "250 consecutive samples", which is 0.5 s expressed at the
*simulator's* old rate; on the bench the same 0.5 s is 174 samples.)

## Two voltage scales — do not conflate them

- **Sensing:** ADC reference `A_VCC = 5.02` V over `ADC_MAX_COUNTS = 1023`.
- **Actuation:** DAC full scale **2.5 V** (internal reference). Firmware clamps to
  0–2.5, `set_voltage` rejects outside that, and the controller further restricts
  itself to 0–0.5 V around `BIAS = 0.25` — that is the ±0.25 V of authority the
  saturation interlock in § 5 is counting against.
