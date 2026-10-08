import os
import signal
import subprocess
import sys
import time
from dataclasses import replace
from datetime import datetime
import numpy as np
from rich.console import Console
from rich.padding import Padding
from rich.table import Table
from rich.text import Text
from .board import Board
from .controller import Modal
from .rig import DATA, Config, identify

STEP, DWELL, CENSUS_S = (0.15, 12.0, 300.0)
console = Console(
    highlight=False,
    markup=False,
    emoji=False,
    soft_wrap=True,
    width=None if sys.stdout.isatty() else 200,
)


def say(msg="", style=None):
    if isinstance(msg, str):
        head = msg.lstrip()
        auto = "bold red" if head[:2] == "!!" else "bold green" if head[:6] == "LOCKED" else ""
        msg = Text(msg, style or auto)
        msg.highlight_regex(r"^\s*\[[^\]]+\]", "cyan")
    console.print(msg if isinstance(msg, Text) else Padding(msg, (0, 0, 0, 2), expand=False))


def check(ok, name, detail="", width=0):
    word, style = ("INFO", "dim") if ok is None else ("PASS", "green") if ok else ("FAIL", "red")
    say(Text.assemble("  ", (word, style), "  %-*s %s" % (width, name, detail)))


def table(*head):
    return Table(*head, box=None, show_header=bool(head), header_style="dim", pad_edge=False)


def ready(why=""):
    say(
        "\nNOT READY  " + why if why else "\nREADY  run `python run.py bench`",
        "bold red" if why else "bold green",
    )
    return not why


def guard():
    sys.stdout.reconfigure(line_buffering=True)
    if sys.platform == "darwin":
        subprocess.Popen(["caffeinate", "-dimsu", "-w", str(os.getpid())])

    def stop(*_):
        raise KeyboardInterrupt

    # a plain `kill` must take the same path as Ctrl-C, or the coils are left energised
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), stop)


def need_room(bias, cfg):
    if (bias - STEP < 0).any() or bias.sum() + STEP > cfg.sum_v_max:
        sys.exit("bias +- %.2f V does not fit under %.2f V summed" % (STEP, cfg.sum_v_max))


def stamp(kind):
    os.makedirs(DATA, exist_ok=True)
    return os.path.join(DATA, datetime.now().strftime("%Y%m%d_%H%M%S") + "_v2%s.csv" % kind)


def run(board, ctl, log=None, say=say, kicks=0, every=25.0, pulse=None, log_hz=None, until=None):
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
                        pulse_until = t + 0.5
                    cued, cue = (cued + 1, t + every)
                    say("[kick %d] cued at t=%.1f s" % (cued, t))
                elif t >= cue:
                    raise EOFError
            out = ctl.step(counts, t)
            if out is not None:
                if t < pulse_until:
                    out = out.copy()
                    out[0] += pulse
                board.write(out)
            for msg in ctl.drain_events():
                say(msg)
            if t - last >= ctl.cfg.status_period_s:
                if rows % 40 == 0:
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
                say("!! could not %s: %r" % (what, e))
    v = ctl.vote
    now, free = (ctl.motion[v].mean(), ctl.free[v].mean())
    res = table()
    res.add_row(
        "lock",
        "%.1f s after gain" % ctl.lock_time if ctl.locked_announced else Text("never", "red"),
    )
    res.add_row(
        "motion",
        "corners %.1f counts rms, %.0fx quieter than before gain (%.1f)"
        % (now, free / max(now, 1e-9), free),
    )
    res.add_row("faults", Text(str(ctl.fault_count), "red") if ctl.fault_count else "none")
    res.add_row(
        "coil drive",
        "peak %.2f V of %.2f V allowed per coil; all eight together %.2f V of %.2f V"
        % (ctl.peak_demand, ctl.cfg.budget_v, ctl.peak_sum, ctl.cfg.sum_v_max),
    )
    say("result")
    say(res)
    return ctl


def preflight(rig, cfg=Config(), now=None):
    out = []
    modal = Modal(rig, cfg)
    age = rig.age_days(now)
    out.append(
        (
            "rig.json is fresh",
            age <= cfg.rig_max_age_days,
            "%.1f days old, limit %.0f; re-run `run.py calib`" % (age, cfg.rig_max_age_days),
        )
    )
    rng, worst, inside = (np.random.default_rng(0), -np.inf, True)
    for _ in range(2000):
        qd = rng.normal(size=rig.nm) * 10.0 ** rng.uniform(-1, 2)
        u, _ = modal.solve(qd, modal.target, modal.coils, np.full(rig.nm, rig.n))
        worst = max(worst, float(qd @ (modal.a @ u)))
        inside &= bool(
            (u >= modal.lo - 1e-12).all()
            and (u <= modal.hi + 1e-12).all()
            and u.sum() <= modal.room + 1e-12
        )
    out.append(
        ("modal law removes energy in its own model", worst <= 1e-12, "worst power %+.2e" % worst)
    )
    out.append(("modal law stays inside the coil and supply limits", inside, ""))
    k = modal.coils
    out.append(
        (
            "every coil in the loop can move both ways",
            (rig.bias >= cfg.vmin).all()
            and (rig.bias <= cfg.vmax).all()
            and (modal.lo[k] < 0).all()
            and (modal.hi[k] > 0).all(),
            "bias %s V, window %.2f..%.2f V"
            % (np.round(rig.bias, 2), (rig.bias + modal.lo).min(), (rig.bias + modal.hi).max()),
        )
    )
    total = float(rig.bias.sum())
    out.append(
        (
            "bias leaves the supply room to work",
            cfg.sum_v_max <= cfg.sum_v_limit and total + 2 * cfg.budget_v <= cfg.sum_v_max,
            "bias sums to %.2f V; loop capped at %.2f V, which must leave two coils' full "
            "up-swing (%.2f V); supply folds back above %.2f V"
            % (total, cfg.sum_v_max, 2 * cfg.budget_v, cfg.sum_v_limit),
        )
    )
    return [(n, bool(ok), d) for n, ok, d in out]


def show_preflight(rig, cfg=Config(), waive_age=False):
    res = preflight(rig, cfg)
    bad = [(n, d) for n, good, d in res if not good]
    waived = waive_age and all((n.startswith("rig.json is fresh") for n, _ in bad))
    say("checks  %d of %d pass%s" % (len(res) - len(bad), len(res), " (age waived)" * waived))
    for n, d in bad:
        say("  FAIL  %s: %s" % (n, d), "yellow" if waived else "red")
    return not bad or waived


class Hold:

    def __init__(self, board, cfg, n):
        self.board, self.hz = (board, cfg.control_hz)
        self.v, self.next = (np.zeros(n), 0.0)

    def __call__(self, target, seconds, ramp_s=2.0, sink=None):
        target = np.asarray(target, float)
        v0, t0, T, C = (self.v.copy(), time.perf_counter() - self.board.t0, [], [])
        while True:
            t = time.perf_counter() - self.board.t0
            if t >= self.next:
                self.v = v0 + (target - v0) * min((t - t0) / max(ramp_s, 1e-06), 1.0)
                self.board.write(self.v)
                self.next = t + 1.0 / self.hz
            s = self.board.read()
            if s is not None:
                settled = s[0] - t0 > ramp_s + 1.0
                if sink:
                    sink(s[0], s[1], settled)
                if settled:
                    (T.append(s[0]), C.append(s[1]))
            if t - t0 > ramp_s + 1.0 + seconds:
                return (np.array(T), np.array(C))


def report(new, old, cfg):
    kind = dict(Z="in-out", T1="tilt", T2="tilt", side="sideways", bounce="up-down", roll="roll")
    was = dict(zip(old.dof, old.f_hz))
    say("\nmodes   the plate's %d ways of swinging" % new.nm)
    modes = table()
    for f, d in zip(new.f_hz, new.dof):
        f0 = was.get(d)
        moved = "new" if f0 is None else "moved from %.3f" % f0 if abs(f - f0) > 0.003 else ""
        modes.add_row("%.3f Hz" % f, kind.get(d, d), Text(moved, "yellow"))
    say(modes)
    say("\ncoils   push on own sensor, counts per volt")
    coils = table("coil", "now", "before", "in the loop", "")
    for j in range(new.n):
        a, b = (new.slope[j], old.slope[j])
        note = Text("")
        if a * b < 0:
            note = Text("direction FLIPPED", "red")
        elif abs(a - b) > max(0.25 * abs(b), 4 * new.slope_sigma[j]):
            note = Text("changed", "yellow")
        used = "yes" if j in new.a_coils else Text("no: too weak on the plate to measure", "red")
        coils.add_row("%d" % j, "%+.1f" % a, "%+.1f" % b, used, note)
    for col in coils.columns[:3]:
        col.justify = "right"
    say(coils)
    noise = np.sqrt(new.kalman_r) / cfg.volts_per_count
    say("\nsensors noise floor, counts: " + "  ".join(("%.1f" % x for x in noise)))
    say("\nloop    %d of %d coils; saved to rig.json" % (len(new.a_coils), new.n))


def measure(rig, cfg, port=None, bias=0.3, dc=None):
    n, b = (rig.n, np.full(rig.n, bias))
    need_room(b, cfg)
    head = (
        "time_s,seg,phase,coil,freq_hz,amp_v,u_v," + ",".join(("a%d" % i for i in range(n))) + "\n"
    )
    paths = [dc or stamp("_dc"), stamp("_census")]
    board = Board(rig, cfg, port)
    hold = Hold(board, cfg, n)

    def writer(fh, coil, volts):

        def sink(t, counts, settled):
            fh.write(
                "%.5f,0,%s,%d,-1,0,%.4f,%s\n"
                % (
                    t,
                    "dc" if settled and coil >= 0 else "passive",
                    coil if settled else -1,
                    volts,
                    ",".join(("%.4f" % x for x in counts)),
                )
            )

        return sink

    try:
        board.start(np.zeros(n))
        hold(b, 8.0, ramp_s=5.0)
        if dc is None:
            print("calib   stepping each coil (4 min): ", end="", flush=True)
            with open(paths[0], "w", buffering=1) as fh:
                fh.write(head)
                for j in range(n):
                    for sign in (+1, -1):
                        v = b.copy()
                        v[j] += sign * STEP
                        hold(v, DWELL, sink=writer(fh, j, v[j]))
                    hold(b, 2.0, sink=writer(fh, -1, bias))
                    print("%d " % j, end="", flush=True)
        say("\ncalib   listening to the free plate (%.0f min)" % (CENSUS_S / 60 + 0.4))
        hold(b, 20.0)
        with open(paths[1], "w", buffering=1) as fh:
            fh.write(head)
            hold(b, CENSUS_S, ramp_s=0.0, sink=writer(fh, -1, bias))
    except KeyboardInterrupt:
        # a cut-short record must not be identified and saved over rig.json
        sys.exit("\ninterrupted: coils parked, rig.json left as it was")
    finally:
        board.park(b)
        board.close()
    new = identify(paths[1], paths[0], replace(rig, bias=b), cfg, say=lambda *_: None)
    new.save()
    report(new, rig, cfg)
    return ready("" if show_preflight(new, cfg) else "see the failed check above")
