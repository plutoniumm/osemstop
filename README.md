# OSEM plate damping

Active damping for a suspended plate: eight shadow sensors (a0..a7) read by an Arduino
Mega, eight coils driven through an SPI DAC. The controller estimates how fast each of
the plate's modes is moving and pushes against it.

Everything current lives in `2.0/`. The earlier controllers are in `1.0/` with their own
[README](1.0/README.md). The suspension this rig follows is described in
[`2.0/design.pdf`](2.0/design.pdf).

## Setup

```
conda activate ligo            # Python 3.14 with numpy, scipy, rich, pyserial
cd 2.0
```

`flash.py` also needs `arduino-cli` with the `arduino:avr` core.

## Everyday use

```
python run.py bench             # close the loop; Ctrl+C parks the coils and stops
python run.py bench --kicks 3   # same, then says "jerk it" three times and stops
```

That is the whole routine once the rig is calibrated. The other three commands are for
when something changed:

| command | when | time |
|---|---|---|
| `python debug.py` | something looks wrong, or the board was unplugged | 2.5 min |
| `python run.py calib` | anyone touched the hardware, or `bench` says the calibration is stale (over 7 days) | 10 min |
| `python flash.py` | `bench` says the board's firmware version does not match | 1 min |

Leave the table alone during `calib` and `debug.py`.

## Reading `bench`

```
rig     calibrated 0.2 days ago; modes 0.733 / 0.997 / 1.645 / 0.991 / 5.948 Hz; 8 coils in the loop
checks  5 of 5 pass
board   firmware 3 on /dev/cu.usbserial-1120
log     ../data/20261008_171125_v2.csv

how much each sensor moves, in counts rms (1 count = 5 mV); smaller is better
   time  state         a0     a1     a2     a3     a4     a5     a6     a7   quieter
gain on at 1.5 s
    5 s  damping     54.3   22.0   43.6   24.7    2.8    2.2    1.8    0.7   1x
LOCKED 6.8 s after gain
   10 s  locked       6.2    3.8    4.3    2.7    2.5    1.7    0.5    0.3   12x
```

- **One number per sensor:** how much it moved over the last five seconds. a0..a3 are the
  four corners, a4/a5 look sideways, a6/a7 look vertically.
- **`quieter`:** how many times less the corners move than before the gain came on.
- **`LOCKED`:** every corner sensor is under five times its own noise floor. Expect it
  about 8 s after starting.
- **`FAULT`:** the loop saw motion growing instead of shrinking and let go; the coils sit
  at bias. It retries once it has locked before. If it prints `LATCHED OFF`, run
  `debug.py`, then `calib`.
- **`result`:** printed on exit, with lock time, final motion, faults and how much of the
  coil range was used.

Every run writes each sample to `data/<date>_<time>_v2.csv`.

## Options

| | |
|---|---|
| `bench --kicks N` | cue N hand knocks after lock, then stop |
| `bench --every S` | seconds between cues (default 25) |
| `bench --pulse [V]` | kick with coil 0 instead of by hand (default 0.15 V for 0.5 s) |
| `bench --hours H` | stop after H hours and log at 10 Hz instead of every sample |
| `bench --stale-ok` | run on a calibration older than 7 days |
| `calib --bias V` | calibrate at a different coil bias (default 0.30 V) |
| `calib FILE` | reuse an existing coil-step record and only redo the passive part |
| `debug.py --coils 6,7` | check only those coils |
| `--port PATH` | any command; needed only if more than one serial device is attached |

## How it works

`calib` steps each coil and then listens to the free plate for five minutes. From those
two records it builds `rig.json`:

- the mode frequencies $f_m$ and shapes $\Phi$ (sensors $\times$ modes): three modes on the
  corner sensors, plus the sideways and vertical modes if they show;
- the coil matrix $A$ (modes $\times$ coils), the static response of each mode to each coil;
- each sensor's noise.

`bench` then runs, once per sensor frame (about 200 times a second):

1. A Kalman filter turns the eight sensor readings $y$ into each mode's velocity
   $\dot q_m$, using $\Phi$, $f_m$ and the coil voltages it just sent.
2. The law asks for a force against each velocity, $f_m = -g_m\,\dot q_m$.
3. The allocator finds coil voltages $u$ with $A u \approx f$ that stay inside every limit:
   each coil within its window about bias, and all eight together under the supply's
   ceiling. When the limits bite it delivers as much of $f$ as they allow and never adds
   energy: $\dot q^{\mathsf T} A u \le 0$.

The power taken out is $-\sum_m g_m \dot q_m^2$, so every mode loses energy for any positive
gain as long as $A$ has the right sign. That sign is what `calib` measures and what
`debug.py` re-checks.

## Using it as a library

```python
import osem

rig = osem.Rig.load()                  # everything calib measured (rig.json)
cfg = osem.Config()                    # every threshold and limit
loop = osem.Controller(rig, cfg)

plate = osem.Sim(rig, seconds=60)      # or osem.Board(rig, cfg) for the Arduino
osem.run(plate, loop)                  # the bench loop: read, step, write, park on exit
```

Or drive it yourself, one sample at a time:

```python
t, counts = plate.read()               # None while no complete frame has arrived
volts = loop.step(counts, t)           # eight coil voltages
plate.write(volts)
loop.state, loop.locked                # "WARMUP" / "DAMPING" / "FAULT", per-sensor lock
```

| | |
|---|---|
| `osem.Rig` | the calibration: `f_hz`, `dof`, `phi`, `a_dc`, `a_coils`, `bias`, noise; `load()` / `save()` |
| `osem.Config` | frozen dataclass of settings; change one with `dataclasses.replace(cfg, modal_scale=2.0)` |
| `osem.Controller` | `step(counts, t)` returns coil volts; holds the filter, the law, the limits and the fault logic |
| `osem.Board` | the Arduino: `start(bias)`, `read()`, `write(volts)`, `park(volts)`, `close()` |
| `osem.Sim` | a simulated plate built from a `Rig`, same interface as `Board` |
| `osem.run` | the loop `bench` uses, with logging and kick cues |
| `osem.preflight` | the checks `bench` runs before it opens the port |

Settings worth knowing in `Config`:

| | default | |
|---|---|---|
| `modal_scale` | 4.0 | overall gain |
| `mode_gain` | bounce 0.3 | per-mode factor on top of it |
| `bias_swing`, `budget_frac` | 0.3, 0.9 | each coil may move $\pm 0.27$ V about its bias |
| `sum_v_max` | 3.6 V | cap on the eight coil voltages added together |
| `warmup_s` | 1.5 s | wait before the gain comes on |
| `quiet_sigma` | 5 | `LOCKED` line, in units of each sensor's noise |

## Limits of this rig

- **Coil supply.** The coil channel draws 0.31 A at rest plus 0.43 A per volt of summed
  coil command and stops following at about 2.2 A, which is 4.4 V summed. `sum_v_max`
  keeps the loop under that.
- **Sensor range.** a0 reaches the top of the ADC (1023) on a hard knock; while it is
  clipped the loop is partly blind, and recovery after a hard knock takes about 10 s.
- **The neighbour.** A table knock leaves a vertical wobble at 6.14 Hz on a6/a7 that
  lasts tens of seconds. It comes from the other suspension on the table and the coils
  cannot reach it.
- **Opening the port resets the board**, which drops the coils to zero and back and kicks
  the plate. Every run therefore starts on a ringing plate.

## Files

| | |
|---|---|
| `2.0/run.py` | `calib` and `bench` |
| `2.0/debug.py` | port, firmware, sensors, link and every coil, without closing a loop |
| `2.0/flash.py` | compiles and uploads `firmware/` |
| `2.0/harness.py` | the loop against a simulated plate, pass/fail (`python harness.py`) |
| `2.0/rig.json` | the current calibration |
| `2.0/harness_rig.json` | a frozen calibration the harness tests against |
| `2.0/osem/rig.py` | `Config`, `Rig`, and the calibration maths |
| `2.0/osem/filters.py` | the Kalman filters and sliding statistics |
| `2.0/osem/controller.py` | the law, the coil allocator, the fault logic |
| `2.0/osem/board.py` | `Board` and `Sim` |
| `2.0/osem/session.py` | the run loop, `calib`, preflight, console output |
| `2.0/firmware/` | the Arduino sketch (version 3) |
| `data/` | every recording |
| `1.0/` | the earlier controllers, frozen |
