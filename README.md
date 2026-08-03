# OSEM suspension damping

A three-layer control stack that damps a suspended optic using OSEM (Optical
Sensor and Electro-Magnetic actuator) shadow sensors. Four OSEMs sense position;
four coils push back. The whole thing is one closed loop split across a language
boundary:

```
arduino.ino (Arduino firmware)     <-- SPI --> AD5628 octal DAC --> coils
   ^  |                                                              |
   |  | serial @115200                                               v
   |  v                                                      suspended optic
pyDAC.py (DACController: transport)                                   |
   ^  |                                                               |
   |  v                                                    OSEMs --> A0..A3
osem.vN.py  (control law, safety, logging)  <-- analogRead ------------'
```

---

# Bench quickstart

**Read this before flashing anything.**

## 0. Which version to run

There are four controllers. They are **not** patches — each is a complete,
standalone program that runs on the hardware. Higher number does **not**
automatically mean better.

```
$ make list
version    on bench    channels  I/D      fixes
osem.v0    validated   1/4       zeroed   -
osem.v1    reported    4/4       zeroed   -
osem.v2    untested    4/4       live     -
osem.v3    untested    4/4       live     saturation-latch, rail-threshold, runaway-baseline
```

| status | meaning |
|---|---|
| `validated` | confirmed on hardware with a scope |
| `reported` | the behaviour is documented, but *this file* has never run |
| `untested` | never been on hardware |

**`osem.v3.py` is the one to run.** As of 2026-08-03 v1, v2 and v3 have all run
on the bench on ch0; v3 locked in 14.5 s, held for 115 s, and auto-recovered from
two faults. v2 locked just as fast and then **latched a permanent FAULT** — a
known defect that v3 fixes. `versions.md` § *On the bench* has the full numbers.

v0 remains the frozen reference: one channel, P-only, the 2026-07-15 configuration
(ch0 at Kp = −0.030, real damping, no rail clipping).

## 1. Flash the Arduino

`arduino.ino` is the firmware. arduino-cli will not look at a sketch until it is
named `<dir>/<dir>.ino`, so one command does the copy, the compile and the upload:

```bash
make arduino                                   # defaults to arduino:avr:mega
make arduino FQBN=arduino:sam:arduino_due_x    # if it is a Due
make arduino PORT=/dev/cu.usbserial-1120       # skip auto-detection
```

Board is an Arduino **Mega or Due** — `CS` is on pin 53, which is Mega/Due
specific. `arduino-cli` 1.5.1 with the `arduino:avr` core is installed on the dev
machine and the firmware compiles clean for the Mega.

`setup()` zeroes all eight DAC channels before anything else runs, and enables the
DAC's internal 2.5 V reference with an `0x08` frame. Opening the serial port
toggles DTR and resets the board, so **connecting is itself a safing action** —
the coils are de-energised before `READY` is sent.

## 2. Host setup

```bash
conda activate ligo        # numpy 2.5.1 + pyserial 3.5; the only env with both
```

Without it `python3` resolves to a Homebrew interpreter with neither dependency,
and every target dies at `import numpy` with a traceback out of
`sim/server.py:load()` that looks like a repo bug and is not.

You do **not** need to edit `PORT`. It is still `COM7` (a Windows name) in all
four files, but `make run` auto-detects the board and assigns the port on the
loaded module, so every controller stays byte-for-byte what was validated.
`make ports` shows what it can see; `make run PORT=...` overrides it.

## 3. Run

```bash
cd /path/to/this/repo          # CSV goes to ./data/ RELATIVE TO CWD
make run V=v3                  # or bare `make run` for a picker
```

It resolves the port, **preflights** it — opening through the real
`DACController`, which raises unless the sketch answers `READY`, so "wrong port"
and "board not flashed" are distinguishable before the optic is swinging — then
prints the bench status and the actual gain vectors it is about to apply, asks for
confirmation on anything not `validated`, and hands over to that version's
`main()`. From there it is the controller: calibrate with the gain at zero
(**8 s on v0–v2, 20 s on v3**), then engage. The long quiet stretch at the start
of a v3 run is that calibration, not a hang.

## 4. What you should see

```
[CALIBRATING] measuring baseline noise for 20s (outputs held at bias)...
[DAMPING] baseline set (ch0=0.1804V, ...). Gain will schedule between -0.035 and -0.030
*** LOCKED -- 14.5s after gain was applied (t=34.5s total) ***
```

The **lock time is the deliverable**. A run that never prints `LOCKED` did not
work, whatever the traces look like.

`data/<YYYYMMDD_HHMMSS>_fast_lock.csv` gets 12 columns per channel plus `time_s`
and `state`, line-buffered and flushed every 200 rows, so it survives an unclean
kill and is enough to reconstruct the run offline. `data/` is gitignored.

## 5. On the scope, watch for

- **Clean, unclipped convergence.** Amplitude falling, no flat tops.
- **The DAC output staying inside 0–0.5 V.** It is biased at 0.25 V and clipped to
  ±0.25 V around that. Thirty consecutive samples against either limit trips a
  saturation fault.
- **`RAIL!` messages** — a fully occluded OSEM pins the ADC at 0, and the
  bandpassed signal then flatlines, which an RMS-only check reads as *perfect
  stability* while the sensor is actually blind. That is why the interlock reads
  raw counts.

**Known open problem: the OSEM signal clips.** Measured 2026-08-03, ch0 traversed
the full 0–1023 ADC range and the actuator used 100% of its ±0.25 V authority on
transients. A shadow sensor pegged at either end is outside its linear range — the
bandpass sees a flat top, the derivative sees a step, and the velocity estimate
spikes exactly when the loop is asked to push hardest. Fix the sensor range before
trusting any tuning done on top of it.

## 6. Safety limits — do not exceed without a scope on it

**`Kp = −0.040` is the documented instability / rail onset.** `-0.030` damps
cleanly; `-0.040` does not. Do not raise gain past `-0.035` (the `CAPTURE_GAIN`
already in use, itself not independently validated) unattended.

Be aware: **that −0.04 limit has no recorded evidence anywhere.** It is one
sentence in `osem.v0.py`'s docstring, written two days before a report that never
mentions it, and the simulator provably cannot reproduce it. Treat it as real and
uncharacterised — `research.md` item 3 is the bench task to characterise it.

`STEADY_GAIN[2] = +0.010` is **positive** where the others are negative. That is
not a typo: A2 damped *fastest* of the four at that gain, which only holds if its
coil or OSEM is mounted the other way round. See `provenance.md`. Do not
"correct" it.

## 7. If something trips

The state machine is `CALIBRATING → DAMPING → FAULT → (auto-recover) →
CALIBRATING`. A fault zeroes all gains and holds every output at bias. If
everything stays clear for 5 s it re-calibrates and resumes on its own.

**One exception, in v0/v1/v2:** a *saturation* trip latches `FAULT` permanently
and needs a manual restart. A runaway trip recovers fine. Observed on hardware
2026-08-03: v2 latched for 50 s after a 31-sample pinned run, while v1 cleared a
runaway trip in 5.0 s. Deliberate, and fixed in v3. See `versions.md`.

---

# Off the bench

There is a simulator that runs **this code** — not a reimplementation — so
behaviour can be checked without hardware.

```bash
make sim              # browser simulator, http://localhost:8770
make sim V=v0         # a specific version
make test             # interactive terminal runner: pick a version, watch it
                      # damp, press x to kick the optic
make check            # the behavioural suite, headless, every version
make list             # what versions exist
```

`make check` is currently **70 passed, 0 failed**.

Three things are faked: `serial` (so `pyDAC` imports without a port — nothing in
the stub is ever called), `DACController` → a `FakeDAC` with identical validation,
and the `time` module inside the controller's namespace, so the actuator's 10 ms
write throttle follows sim time instead of the wall clock. Everything else is the
shipping code executing.

**What it cannot tell you:**

- The plant is linear, so it will not reproduce the instability at Kp = −0.040.
  Here, more negative gain is monotonically more damping all the way to −0.6.
  **The simulator cannot tell you a safe gain.**
- **It cannot catch a wrong coil map.** `sim/server.py` builds its held-voltage
  dict from `enumerate(DAC_CHANNELS)` and reads it back with the same index, so it
  round-trips through whatever map the controller declares. Only the bench can.

---

# Files

| | |
|---|---|
| `osem.v0.py` … `osem.v3.py` | the controllers — complete and standalone, see `versions.md` |
| `pyDAC.py` | serial transport (`DACController`) |
| `arduino.ino` | Arduino sketch: 8-column ADC stream + AD5628 SPI |
| `bench.py` | the on-hardware entry point: `make run`, `make arduino`, `make ports` |
| `harness.py` | one entry point for everything off the bench |
| `sim/`, `test/` | simulated plant + browser UI; interactive runner |
| `versions.md` | what each version changes, the state machine and hazards, and the bench results |
| `provenance.md` | where the constants came from, and the coil wiring map |
| `research.md` | ranked queue of what to try next, and what was ruled out |

# Serial protocol

Changing any line here means editing `arduino.ino` and `pyDAC.py` together.

| Direction | Message | Meaning |
|---|---|---|
| MCU → host | `READY` | sent once after `setup()` |
| host → MCU | `SET <ch> <volts>` | sets DAC channel 0–7; replies `OK ch=.. v=..` or `ERR ..` |
| host → MCU | `STREAM` / `STOP` | toggles sampling; replies `STREAMING` / `STOPPED` |
| MCU → host | `a0,...,a7` | free-running ADC counts 0–1023, one line per loop, only while streaming |

The stream is **eight columns**; only A0–A3 carry OSEMs, so `read_sample` requires
`len(parts) >= 4` and takes the leading four. Measured rate: **~348 Hz** (2.88
ms/sample) — serial print dominates, so four columns would roughly double it. Two
thresholds are counted in *samples*, not seconds (`MAX_CONSECUTIVE_SATURATED = 30`
and the rail window), so their wall-clock meaning moves with the stream rate.

`SET` and the stream share one wire — the firmware keeps streaming while it
services commands, so an `OK` arrives buried in sample lines. `set_voltage` reads
up to 50 lines looking for it, and `RateLimitedActuator.send` swallows the
`RuntimeError` when it never arrives, on purpose: the next sample resends.

# Two voltage scales — do not conflate them

- **Sensing:** ADC reference `A_VCC = 5.02` V over `ADC_MAX_COUNTS = 1023`.
- **Actuation:** DAC full scale **2.5 V** (internal reference). Firmware clamps to
  0–2.5, `set_voltage` rejects outside that, and the controller further restricts
  itself to 0–0.5 V around `BIAS = 0.25`.

# Channel mapping

Sensor index `i` (0–3) → analog pin `A<i>` → DAC channel `DAC_CHANNELS[i] =
[1, 3, 5, 7]`.

**Rewired 2026-08-03.** The full eight-pair map is `[1, 3, 5, 7, 0, 2, 4, 6]`
(`provenance.md`); the controllers take the first four because only A0–A3 have
OSEMs on them. The previous map, `[0, 2, 4, 6]`, now points at the coils for
sensors 4–7 — a controller still holding it closes every loop onto the wrong
actuator, and neither the simulator nor the interlocks can tell.
