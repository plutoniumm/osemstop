# OSEM suspension damping

A three-layer control stack that damps a suspended optic using OSEM (Optical
Sensor and Electro-Magnetic actuator) shadow sensors. Four OSEMs sense position;
four coils push back. The whole thing is one closed loop split across a language
boundary:

```
thing.c  (Arduino firmware)        <-- SPI --> AD5628 octal DAC --> coils
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

There are five controllers. They are **not** patches — each is a complete,
standalone program that runs on the hardware. Higher number does **not** mean
better, and the newest one is broken.

```
$ make list
version    on bench    channels  I/D      fixes
osem.v0    validated   1/4       zeroed   -
osem.v1    reported    4/4       zeroed   -
osem.v2    untested    4/4       live     -
osem.v3    untested    4/4       live     saturation-latch, rail-threshold, runaway-baseline
osem.v4    broken      4/4       live     ... , modal
```

| status | meaning |
|---|---|
| `validated` | confirmed on hardware with a scope |
| `reported` | `report.pdf` documents the behaviour, but *this file* has never run |
| `untested` | never been on hardware |
| `broken` | does not damp — **do not flash** |

**Run `osem.v0.py`.** It is the only configuration confirmed on hardware
(2026-07-15 run 3: ch0 at Kp = −0.030, real damping, no rail clipping). Once
that reproduces, `osem.v1.py` is the natural next step — `report.pdf` §4
describes all four channels damping together, settling in ≈17 s against ≈70 s
for one at a time.

**Do not run `osem.v4.py` on hardware.** Its system identification does not
converge; it drives the coils from a wrong actuation matrix.

## 1. Flash the Arduino

`thing.c` is an Arduino **sketch**, not compilable C — it uses `SPI.h`, `String`
and `Serial`. To build it:

```bash
mkdir -p osem_stream && cp thing.c osem_stream/osem_stream.ino
arduino-cli compile --fqbn arduino:avr:mega osem_stream
arduino-cli upload  --fqbn arduino:avr:mega -p /dev/ttyACM0 osem_stream
```

Board is an Arduino **Mega or Due** — `CS` is on pin 53, which is Mega/Due
specific. Neither the IDE nor `arduino-cli` is installed on the dev machine, so
this step happens on the lab machine.

`setup()` zeroes DAC channels 0, 2, 4, 6 before anything else runs, and enables
the DAC's internal 2.5 V reference with an `0x08` frame.

## 2. Host setup

```bash
pip install numpy pyserial
```

Then **edit the port** at the top of the version you are running:

```python
PORT = "COM7"          # Windows. On Linux/Mac: "/dev/ttyACM0", "/dev/ttyUSB0", ...
```

## 3. Run

```bash
cd /path/to/this/repo          # CSV goes to ./data/ RELATIVE TO CWD
python osem.v0.py
```

It prints `READY`, calibrates for 8 s with the gain held at zero, then engages.
`DACController.__init__` raises if `READY` never arrives — that means the sketch
is not running or the port is wrong.

## 4. What you should see

```
[CALIBRATING] ...
[DAMPING] baseline set (ch0=0.4542V, ...). Gain will schedule between -0.035 ... and -0.030 ...
*** LOCKED -- 13.6s after gain was applied (t=21.6s total) ***
```

The **lock time is the deliverable**. A run that never prints `LOCKED` did not
work, whatever the traces look like.

`data/<YYYYMMDD_HHMMSS>_fast_lock.csv` gets 12 columns per channel plus `time_s`
and `state`, line-buffered and flushed every 200 rows, so it survives an unclean
kill and is enough to reconstruct the run offline.

## 5. On the scope, watch for

- **Clean, unclipped convergence.** Amplitude falling, no flat tops.
- **The DAC output staying inside 0–0.5 V.** It is biased at 0.25 V and clipped
  to ±0.25 V around that. Thirty consecutive samples against either limit trips a
  saturation fault.
- **`RAIL!` messages** — a fully occluded OSEM pins the ADC at 0, and the
  bandpassed signal then flatlines, which an RMS-only check reads as *perfect
  stability* while the sensor is actually blind. That is why the interlock reads
  raw counts.

## 6. Safety limits — do not exceed without a scope on it

**`Kp = −0.040` is the documented instability / rail onset.** `-0.030` damps
cleanly; `-0.040` does not. Do not raise gain past `-0.035` (the `CAPTURE_GAIN`
already in use, itself not independently validated) unattended.

Be aware: **that −0.04 limit has no recorded evidence anywhere.** It is one
sentence in `osem.v0.py`'s docstring. `report.pdf`, written two days later, never
mentions it, and the simulator provably cannot reproduce it. Treat it as real and
uncharacterised — see `research.md` item 3, which is a bench task.

`STEADY_GAIN[2] = +0.010` is **positive** where the others are negative. That is
not a typo: `report.pdf` Table 1 has A2 damping *fastest* of the four at that
gain, which only holds if its coil or OSEM is mounted the other way round. Do not
"correct" it.

## 7. If something trips

The state machine is `CALIBRATING → DAMPING → FAULT → (auto-recover) →
CALIBRATING`. A fault zeroes all gains and holds every output at bias. If
everything stays clear for 5 s it re-calibrates and resumes on its own.

**One exception, in v0/v1/v2:** a *saturation* trip latches `FAULT` permanently
and needs a manual restart. A runaway trip recovers fine. This is a known defect,
preserved on purpose, and fixed in v3. See `versions.md`.

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

Only two things are faked: `serial` (so `pyDAC` imports without a port — nothing
in the stub is ever called) and `DACController` → a `FakeDAC` with identical
validation. Everything else is the shipping code executing.

**What it cannot tell you:** the plant is linear, so it will not reproduce the
instability at Kp = −0.040. Here, more negative gain is monotonically more
damping all the way to −0.6. **The simulator cannot tell you a safe gain.**

---

# Files

| | |
|---|---|
| `osem.v0.py` … `osem.v4.py` | the controllers — complete and standalone, see `versions.md` |
| `pyDAC.py` | serial transport (`DACController`) |
| `thing.c` | Arduino sketch: ADC stream + AD5628 SPI |
| `harness.py` | one entry point for everything off the bench |
| `sim/`, `test/` | simulated plant + browser UI; interactive runner |
| `versions.md` | what each version changes, with measured before/after |
| `state.md` | every piece of state in all four layers, and the known hazards |
| `research.md` | ranked queue of what to try next, and what was ruled out |
| `report.pdf` | Weekly Report-11, S. Mithiya, CQuICC / IIT Madras, 17 Jul 2026 |
| `CLAUDE.md` | orientation for Claude Code |

# Serial protocol

Changing any line here means editing `thing.c` and `pyDAC.py` together.

| Direction | Message | Meaning |
|---|---|---|
| MCU → host | `READY` | sent once after `setup()` |
| host → MCU | `SET <ch> <volts>` | sets DAC channel 0–7; replies `OK ch=.. v=..` or `ERR ..` |
| host → MCU | `STREAM` / `STOP` | toggles sampling; replies `STREAMING` / `STOPPED` |
| MCU → host | `a0,a1,a2,a3` | free-running ADC counts 0–1023, one line per loop, only while streaming |

`SET` and the stream share one wire — the firmware keeps streaming while it
services commands, so an `OK` arrives buried in sample lines. `set_voltage` reads
up to 50 lines looking for it.

# Two voltage scales — do not conflate them

- **Sensing:** ADC reference `A_VCC = 5.02` V over `ADC_MAX_COUNTS = 1023`.
- **Actuation:** DAC full scale **2.5 V** (internal reference). Firmware clamps
  to 0–2.5, `set_voltage` rejects outside that, and the controller further
  restricts itself to 0–0.5 V around `BIAS = 0.25`.

Sensor index `i` (0–3) → analog pin `A<i>` → DAC channel `DAC_CHANNELS[i] =
[0, 2, 4, 6]`. Even channels only.
