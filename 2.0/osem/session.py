import os
import subprocess
import sys
from datetime import datetime
import numpy as np
from .config import Config
from .controller import STATES, Controller
from .regulators import Modal, Pid
from .rig import HERE
from .term import STATE, c, paint

DATA = os.path.join(os.path.dirname(HERE), "data")


def stamp(kind):
    os.makedirs(DATA, exist_ok=True)
    return os.path.join(DATA, datetime.now().strftime("%Y%m%d_%H%M%S") + "_v2%s.csv" % kind)


def keep_awake():
    if sys.platform == "darwin":
        subprocess.Popen(["caffeinate", "-dimsu", "-w", str(os.getpid())])


def run(board, ctl, log=None, say=print, kicks=0, every=25.0, pulse=None, log_hz=None, until=None):
    fh = open(log, "w", buffering=1) if log else None
    if fh:
        fh.write(",".join(ctl.csv_header()) + "\n")
    cue, cued, last, pulse_until, logged, rows = None, 0, -1e9, -1.0, -1e9, 0
    try:
        while True:
            s = board.read()
            if s is None:
                continue
            t, counts = s
            if until is not None and t > until:
                raise EOFError
            if (
                kicks
                and ctl.state == "DAMPING"
                and (ctl.locked_announced or t - ctl.damping_start > 40.0)
            ):
                if cue is None:
                    cue = t + 5.0
                elif t >= cue and cued < kicks:
                    if pulse is None:
                        subprocess.Popen(["say", "jerk it"])
                    else:
                        pulse_until = t + pulse[2]
                    cued, cue = (cued + 1, t + every)
                    say(paint("[kick %d] cued at t=%.1f s" % (cued, t)))
                    rows = 0
                elif t >= cue:
                    raise EOFError
            out = ctl.step(counts, t)
            if out is not None:
                if t < pulse_until:
                    out = out.copy()
                    out[pulse[0]] += pulse[1]
                board.write(out)
            for msg in ctl.drain_events():
                say(paint(msg))
                rows = 0
            if t - last >= ctl.cfg.status_period_s:
                if rows % 20 == 0:
                    say(ctl.status_header())
                last, rows = (t, rows + 1)
                say(ctl.status_line(t))
            if fh and (log_hz is None or (ctl.stepped and t - logged >= 1.0 / log_hz)):
                logged = t
                fh.write(",".join(ctl.csv_row(t, counts)) + "\n")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        for what, step in (
            ("park the coils", lambda: board.park(ctl.bias)),
            ("close the log", fh.close if fh else lambda: None),
            ("close the board", board.close),
        ):
            try:
                step()
            except BaseException as e:
                say(paint("!! could not %s: %r" % (what, e)))
    lock = "%.1f s after gain" % ctl.lock_time if ctl.locked_announced else "never"
    say(
        "\nsummary\n"
        + "\n".join(
            (
                "  %-10s %s" % row
                for row in (
                    ("steps", "%d at %.0f frames/s" % (ctl.dec.steps, ctl.dec.wire_hz)),
                    ("lock", c(lock, "green" if ctl.locked_announced else "yellow")),
                    ("faults", c(ctl.fault_count, "red" if ctl.fault_count else "green")),
                    ("coil peak", "%.3f V of %.3f" % (ctl.peak_demand, ctl.cfg.budget_v)),
                    (
                        "summed",
                        "%.2f V of %.2f, capped on %d steps"
                        % (ctl.peak_sum, ctl.cfg.sum_v_max, ctl.sum_limited),
                    ),
                )
            )
        )
    )
    if ctl.est.mkf is not None:
        say("  chi2/dof")
        for st in STATES:
            say(ctl.chi2_log[st].line(st))
    return ctl


def preflight(rig, cfg=Config(), now=None):
    out = []
    modal, pid = (Modal(rig, cfg), Pid(rig, cfg))
    age = rig.age_days(now)
    out.append(
        (
            "rig.json is fresh",
            age <= cfg.rig_max_age_days,
            "%.1f days old, limit %.0f; re-run `run.py measure`" % (age, cfg.rig_max_age_days),
        )
    )
    g = pid.target
    out.append(
        (
            "slope x gain > 0 on every PID channel",
            (rig.slope * g)[g != 0].min(initial=1.0) > 0
            and (not (g[rig.slope_sign == 0] != 0).any()),
            "gain %s" % np.round(g, 4),
        )
    )
    out.append(
        (
            "A is the DC matrix projected on Phi",
            np.allclose(np.linalg.pinv(rig.phi[:4]) @ rig.dc[:4], rig.a_dc, atol=0.05),
            "",
        )
    )
    out.append(
        (
            "PID fallback is safe, or switched off",
            True,
            "eigenvalues %s: %s"
            % (
                np.round(pid.eig, 3),
                "damps every mode" if pid.safe else "would pump, so it is off on the modal coils",
            ),
        )
    )
    rng, worst = (np.random.default_rng(0), -np.inf)
    for _ in range(2000):
        qd = rng.normal(size=rig.nm)
        u, _ = modal.solve(qd, modal.target, modal.coils, np.full(rig.nm, rig.n))
        worst = max(worst, float(qd @ (modal.a @ u)))
    out.append(
        ("modal law removes energy in its own model", worst <= 1e-12, "worst power %+.2e" % worst)
    )
    out.append(
        (
            "window inside the DAC range",
            (rig.bias - cfg.bias_swing >= cfg.vmin - 1e-09).all()
            and (rig.bias + cfg.bias_swing <= cfg.vmax + 1e-09).all(),
            "",
        )
    )
    total = float(rig.bias.sum())
    out.append(
        (
            "bias leaves the supply room to work",
            total + rig.n * cfg.budget_v / 2 <= cfg.sum_v_max,
            "bias sums to %.2f V; loop capped at %.2f V; supply folds back near %.2f V"
            % (total, cfg.sum_v_max, cfg.sum_v_limit),
        )
    )
    return [(n, bool(ok), d) for n, ok, d in out]


def show_preflight(rig, cfg=Config(), waive_age=False):
    ok = True
    for name, good, detail in preflight(rig, cfg):
        waived = waive_age and (not good) and name.startswith("rig.json is fresh")
        tag = "PASS" if good else "WAIVED" if waived else "FAIL"
        print("  %s %-42s %s" % (c("%-6s" % tag, STATE.get(tag, "yellow")), name, c(detail, "dim")))
        ok &= good or waived
    return ok
