# 2.0

Damps the plate's three modes with all eight coils. `MIMO.md` has the method.

    python run.py measure            # sign test + passive record -> rig.json -> preflight
    python run.py bench --kicks 3    # close the loop; says "jerk it" three times after lock
    python harness.py                # the loop against a synthetic plate

`run.py --help` lists the rest (`link`, `stairs`, `flash`, `preflight`, `sim`, `replay`).

## Using it from code

```python
import osem

rig = osem.Rig.load()                       # everything measured
loop = osem.Controller(rig)                 # Modal on the resolved coils, Pid on the rest
osem.run(osem.Sim(rig, 90), loop)           # or osem.Board(rig, osem.Config())
```

A regulator turns estimated velocities into coil volts. The controller offers each
one, in order, the coils nobody has claimed yet:

```python
osem.Controller(rig, cfg, [osem.Pid(rig, cfg)])                    # PID only
osem.Controller(rig, cfg, [MyLaw(rig, cfg), osem.Pid(rig, cfg)])   # your own first

class MyLaw(osem.Regulator):
    def command(self, est, free, sense_ok, dt, on):   # est.qdot, est.vel
        self.claimed = free & self.coils
        return volts_about_bias
```

## Layout

| | |
|---|---|
| `run.py` | the command line |
| `rig.json` | every measured number; rebuilt by `measure` |
| `osem/rig.py` | `Rig`, and `identify`: two records -> frequencies, Φ, A, noise |
| `osem/regulators.py` | `Regulator`, `Modal`, `Pid` |
| `osem/controller.py` | calibrate -> damp -> fault, and the limits |
| `osem/kalman.py`, `filters.py`, `safety.py` | estimator, statistics, interlocks |
| `osem/board.py` | `Board` (the Arduino), `Sim`, `Replay` |
| `osem/session.py`, `procedures.py` | the run loop and preflight; measure, link, stairs, flash |
| `osem/config.py` | every threshold |
| `firmware/` | the Arduino sketch |
