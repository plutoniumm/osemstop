#!/usr/bin/env python3
"""
Interactive test runner -- pick a version, watch it damp.
=========================================================
This does not define its own test set. It installs a reporter and a monitor on
`harness.py` and runs `harness.suite()`, so what you watch here and what
`make check` asserts headlessly are the same checks against the same plant.

What the TUI adds is that it paces the run to something a human can follow and
lets you interfere with it:

    up/down, 0-9   pick a version          x   kick the optic, random sign
    enter          run the suite           +/- slower / faster
    r              re-run                  t   turbo (no pacing)
    v              back to the picker      q   quit

Every curve is a forced damped oscillation: the plant is driven at ~1 Hz
throughout every scenario in the suite, so all four axes are always ringing and
what you are watching is whether the loop takes energy back out of them.

    make test          # this
    make check         # the same suite, headless, all versions

Requires a terminal at least 80x24.
"""

import curses
import os
import random
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import harness as H                      # noqa: E402
from sim import server as S              # noqa: E402

TRACE_N = 600
SERIES = [1, 2, 3, 4]                    # curses colour pairs, one per channel
STATE_PAIR = {"CALIBRATING": 6, "DAMPING": 5, "FAULT": 7, "IDLE": 8}


class App:
    def __init__(self):
        self.lock = threading.Lock()
        self.versions = H.versions()
        # describe() loads the module, so do it once here rather than per frame
        self.info = [H.describe(p) for p in self.versions]
        self.sel = len(self.versions) - 1
        self.screen = "pick"             # pick | run
        self.reset_run()

    def reset_run(self):
        self.trace = deque(maxlen=TRACE_N)   # (t, [bp x4], state, kick)
        self.results = []                    # (ok, name, detail)
        self.notes = []
        self.label = ""
        self.header = ""
        self.sim_t = 0.0
        self.chan = []                       # per-channel readout dicts
        self.kick_flash = 0.0
        self.speed = 10.0
        self.done = False
        self.error = None
        self.worker = None
        self.pending_mark = False
        self.cur_label = None
        self.last_sim_t = 0.0
        self.run_wall0 = 0.0
        self.run_sim0 = 0.0

    # ---------------- harness hooks (called on the worker thread) ----------
    def reporter(self, kind, **f):
        with self.lock:
            if kind == "check":
                self.results.append((f["ok"], f["name"], f.get("detail", "")))
            elif kind == "phase":
                self.header = f["label"].replace("=", "").strip().splitlines()[0]
            else:
                for line in f["text"].splitlines():
                    if line.strip():
                        self.notes.append(line.rstrip())
                del self.notes[:-6]

    def monitor(self, label, sim):
        now = time.perf_counter()
        with self.lock:
            # sim.t restarts at 0 on every scenario, so re-anchor the pacing
            # clock whenever a new one begins.
            if label != self.cur_label or sim.t < self.last_sim_t:
                self.cur_label = label
                self.label = label
                self.run_wall0 = now
                self.run_sim0 = sim.t
            self.last_sim_t = self.sim_t = sim.t
            mark = self.pending_mark
            self.pending_mark = False
            self.trace.append((sim.t,
                               [float(c.bp_out) for c in sim.ctl.channels],
                               sim.ctl.state, mark))
            self.chan = [dict(en=bool(c.enabled), ratio=sim.diag_ratio[i],
                              gain=float(c.active_gain), out=float(c.out),
                              rail=bool(c.rail_fault), locked=bool(c.locked))
                         for i, c in enumerate(sim.ctl.channels)]
            speed = self.speed
        if speed > 0:
            target = self.run_wall0 + (sim.t - self.run_sim0) / speed
            delay = target - time.perf_counter()
            if delay > 0:
                time.sleep(min(delay, 0.05))

    # ---------------- run control ------------------------------------------
    def start(self):
        self.reset_run()
        H.REPORTER = self.reporter
        H.MONITOR = self.monitor
        H._pass = H._fail = 0
        H.KICKS_APPLIED = 0
        H.PENDING_KICKS.clear()
        path = self.versions[self.sel]

        def work():
            try:
                H.suite(path)
            except Exception as exc:                      # noqa: BLE001
                with self.lock:
                    self.error = f"{type(exc).__name__}: {exc}"
            finally:
                with self.lock:
                    self.done = True

        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()
        self.screen = "run"

    def kick(self):
        """A random shove, sign and severity both drawn. Queued rather than
        applied here: the worker thread owns the plant while it is stepping, and
        it drops anything queued between scenarios. The counter shown is the
        harness's applied count, not the number of keypresses, so a kick pressed
        in the gap between scenarios is honestly not counted."""
        dv = random.uniform(2.5, 7.0) * random.choice([-1.0, 1.0])
        H.PENDING_KICKS.append(dv)
        with self.lock:
            self.kick_flash = time.perf_counter()
            self.pending_mark = True
        return dv


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------
def put(win, y, x, text, attr=0):
    """Write clipped to the window; curses raises at the last cell otherwise."""
    h, w = win.getmaxyx()
    if not (0 <= y < h) or x >= w:
        return
    text = text[:max(0, w - x - 1)]
    if text:
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass


def draw_picker(win, app):
    win.erase()
    h, w = win.getmaxyx()
    put(win, 1, 2, "OSEM damping controller -- test runner", curses.A_BOLD)
    put(win, 2, 2, "Pick a version. Every one is a complete controller that also "
                   "runs on the hardware.", curses.color_pair(9))
    put(win, 4, 2, f"{'':3s}{'version':<10}{'channels':<11}{'I / D':<9}fixes",
        curses.A_UNDERLINE)
    row = 5
    for i, d in enumerate(app.info):
        sel = i == app.sel
        attr = curses.A_REVERSE | curses.A_BOLD if sel else 0
        idl = "live" if any(d["ki"]) or any(d["kd"]) else "zeroed"
        fixes = ", ".join(d["fixes"]) or "none -- known bugs intact"
        put(win, row, 2, f"{'>' if sel else ' ':3s}{d['name']:<10}"
                         f"{d['n_enabled']}/4 enabled{'':2s}{idl:<9}{fixes}", attr)
        row += 1
    put(win, row + 1, 2, "up/down or 0-9 to choose,  enter to run,  q to quit",
        curses.color_pair(9))
    if h > row + 4:
        put(win, row + 3, 2, "The suite drives every axis at ~1 Hz throughout, so all four "
                             "curves stay", curses.color_pair(9))
        put(win, row + 4, 2, "forced. Press x during a run to kick the optic.",
            curses.color_pair(9))
    win.noutrefresh()


def draw_curves(win, app, top, height, width):
    """Four stacked bands, one per channel, on a SHARED y-scale -- an
    independently-autoscaled band would hide exactly the thing being tested."""
    with app.lock:
        data = list(app.trace)
    if len(data) < 2:
        put(win, top + height // 2, 4, "waiting for the first samples...",
            curses.color_pair(9))
        return
    per = height // 4
    if per < 3:
        return
    span = min(len(data), max(10, width - 14))
    view = data[-span:]
    peak = max(1e-6, max(max(abs(v) for v in row[1]) for row in view))
    plot_w = width - 14

    for ch in range(4):
        base = top + ch * per
        rows = per - 1
        mid = base + rows // 2
        colour = curses.color_pair(SERIES[ch])
        # zero line + the band's own frame
        put(win, mid, 12, "·" * plot_w, curses.color_pair(10))
        prev_y = None
        for x in range(min(plot_w, len(view))):
            idx = int(x * (len(view) - 1) / max(1, min(plot_w, len(view)) - 1))
            t, vals, state, mark = view[idx]
            frac = vals[ch] / peak                       # -1 .. +1
            y = mid - int(round(frac * (rows // 2)))
            y = max(base, min(base + rows - 1, y))
            if mark:
                for yy in range(base, base + rows):
                    put(win, yy, 12 + x, "┊", curses.color_pair(11))
            if prev_y is not None and abs(y - prev_y) > 1:
                step = 1 if y > prev_y else -1
                for yy in range(prev_y + step, y, step):
                    put(win, yy, 12 + x, "│", colour)
            put(win, y, 12 + x, "•", colour)
            prev_y = y
        c = app.chan[ch] if ch < len(app.chan) else None
        tag = f"ch{ch}"
        if c:
            if c["rail"]:
                st, sp = "RAIL", 7
            elif not c["en"]:
                st, sp = "off", 8
            elif c["locked"]:
                st, sp = "lock", 5
            else:
                st, sp = "damp", 6
            put(win, base, 1, f"{tag}", colour | curses.A_BOLD)
            put(win, base, 5, f"{st}", curses.color_pair(sp))
            if rows > 1:
                put(win, base + 1, 1, f"{c['ratio']:.2f}", curses.color_pair(9))
            if rows > 2:
                put(win, base + 2, 1, f"{c['gain']:+.3f}", curses.color_pair(9))
        else:
            put(win, base, 1, tag, colour | curses.A_BOLD)
    put(win, top + height - 1, 1, f"peak {peak:.3f} V, shared scale",
        curses.color_pair(10))


def draw_run(win, app):
    win.erase()
    h, w = win.getmaxyx()
    if h < 20 or w < 78:
        put(win, 0, 0, f"Need at least 78x20, have {w}x{h}.")
        win.noutrefresh()
        return
    with app.lock:
        header, label, sim_t = app.header, app.label, app.sim_t
        results = list(app.results)
        notes = list(app.notes)
        done, error = app.done, app.error
        speed, flash = app.speed, app.kick_flash
        state = app.trace[-1][2] if app.trace else "IDLE"
    kicks = H.KICKS_APPLIED     # kicks that reached the plant, not keypresses
    npass = sum(1 for ok, _, _ in results if ok)
    nfail = len(results) - npass

    put(win, 0, 1, header or "starting...", curses.A_BOLD)
    sp = curses.color_pair(STATE_PAIR.get(state, 8))
    put(win, 1, 1, f"{state:<12}", sp | curses.A_BOLD)
    put(win, 1, 14, f"t={sim_t:6.1f}s", curses.color_pair(9))
    put(win, 1, 26, f"{npass} pass", curses.color_pair(5))
    put(win, 1, 35, f"{nfail} fail",
        curses.color_pair(7) if nfail else curses.color_pair(9))
    put(win, 1, 45, f"kicks {kicks}", curses.color_pair(9))
    speed_s = "turbo" if speed <= 0 else f"{speed:.0f}x"
    put(win, 1, 57, f"speed {speed_s}", curses.color_pair(9))
    if time.perf_counter() - flash < 0.6:
        put(win, 1, 70, " KICK ", curses.color_pair(7) | curses.A_REVERSE)
    put(win, 2, 1, (label or "")[:w - 3], curses.color_pair(9))

    res_h = min(max(6, h // 3), 12)
    curve_top, curve_h = 3, h - res_h - 5
    draw_curves(win, app, curve_top, curve_h, w)

    ry = curve_top + curve_h
    put(win, ry, 1, "─" * (w - 2), curses.color_pair(10))
    ry += 1
    shown = results[-(res_h - len(notes) - 1):] if res_h > len(notes) + 1 else []
    for ok, name, detail in shown:
        if ry >= h - 2:
            break
        put(win, ry, 1, "✓" if ok else "✗",
            curses.color_pair(5 if ok else 7) | curses.A_BOLD)
        line = name + (f" — {detail}" if detail else "")
        put(win, ry, 3, line[:w - 5], 0 if ok else curses.color_pair(7))
        ry += 1
    for line in notes:
        if ry >= h - 2:
            break
        put(win, ry, 3, line.strip()[:w - 5], curses.color_pair(9))
        ry += 1

    if error:
        put(win, h - 2, 1, f"ERROR  {error}"[:w - 3], curses.color_pair(7) | curses.A_BOLD)
    elif done:
        verdict = f"done — {npass} passed, {nfail} failed"
        put(win, h - 2, 1, verdict,
            curses.color_pair(7 if nfail else 5) | curses.A_BOLD)
    put(win, h - 1, 1,
        "x kick   +/- speed   t turbo   r re-run   v versions   q quit",
        curses.color_pair(9))
    win.noutrefresh()


# --------------------------------------------------------------------------
def main(stdscr):
    curses.curs_set(0)
    curses.use_default_colors()
    for pair, fg in ((1, curses.COLOR_CYAN), (2, curses.COLOR_RED),
                     (3, curses.COLOR_GREEN), (4, curses.COLOR_YELLOW),
                     (5, curses.COLOR_GREEN), (6, curses.COLOR_YELLOW),
                     (7, curses.COLOR_RED), (8, curses.COLOR_BLUE),
                     (9, curses.COLOR_WHITE), (10, curses.COLOR_BLUE),
                     (11, curses.COLOR_MAGENTA)):
        curses.init_pair(pair, fg, -1)
    stdscr.nodelay(True)

    app = App()
    if not app.versions:
        return "No osem.v*.py found next to harness.py."

    while True:
        if app.screen == "pick":
            draw_picker(stdscr, app)
        else:
            draw_run(stdscr, app)
        curses.doupdate()

        try:
            key = stdscr.getch()
        except curses.error:
            key = -1

        if key == -1:
            time.sleep(0.04)
            continue
        if key in (ord("q"), ord("Q")):
            return None

        if app.screen == "pick":
            if key in (curses.KEY_UP, ord("k")):
                app.sel = (app.sel - 1) % len(app.versions)
            elif key in (curses.KEY_DOWN, ord("j")):
                app.sel = (app.sel + 1) % len(app.versions)
            elif ord("0") <= key <= ord("9"):
                i = key - ord("0")
                if i < len(app.versions):
                    app.sel = i
            elif key in (curses.KEY_ENTER, 10, 13, ord(" ")):
                app.start()
        else:
            if key in (ord("x"), ord("X")):
                app.kick()
            elif key in (ord("+"), ord("=")):
                with app.lock:
                    app.speed = min(60.0, (app.speed or 1.0) * 1.5)
            elif key == ord("-"):
                with app.lock:
                    app.speed = max(1.0, (app.speed or 60.0) / 1.5)
            elif key in (ord("t"), ord("T")):
                with app.lock:
                    app.speed = 0.0 if app.speed > 0 else 10.0
            elif key in (ord("r"), ord("R")):
                if app.done:
                    app.start()
            elif key in (ord("v"), ord("V")):
                if app.done:
                    app.screen = "pick"


if __name__ == "__main__":
    try:
        sys.exit(curses.wrapper(main) or 0)
    except KeyboardInterrupt:
        sys.exit(130)
