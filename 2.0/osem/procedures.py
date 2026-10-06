import os
import subprocess
import sys
import time
from dataclasses import replace
import numpy as np
from .board import Board
from .rig import HERE, identify
from .session import keep_awake, show_preflight, stamp


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


def flash(port=None):
    sketch = os.path.join(HERE, "firmware")
    port = port or Board.find_port()
    for step in (
        ["compile", "--fqbn", "arduino:avr:mega", sketch],
        ["upload", "--fqbn", "arduino:avr:mega", "-p", port, sketch],
    ):
        if subprocess.run(["arduino-cli"] + step).returncode:
            sys.exit("arduino-cli %s failed" % step[0])
    print("flashed; all coils are at 0 V.")


def measure(
    rig, cfg, port=None, bias=0.3, step=0.15, dwell=12.0, census_s=300.0, dc=None, rig_path=None
):
    keep_awake()
    n, b = (rig.n, np.full(rig.n, bias))
    if b.sum() + step > cfg.sum_v_max or bias - step < 0:
        sys.exit("bias %.2f V +- %.2f V does not fit" % (bias, step))
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
            with open(paths[0], "w", buffering=1) as fh:
                fh.write(head)
                for j in range(n):
                    for sign in (+1, -1):
                        v = b.copy()
                        v[j] += sign * step
                        hold(v, dwell, sink=writer(fh, j, v[j]))
                    hold(b, 2.0, sink=writer(fh, -1, bias))
                    print("  coil %d stepped" % j)
        print("passive record, %.0f s..." % census_s)
        hold(b, 20.0)
        with open(paths[1], "w", buffering=1) as fh:
            fh.write(head)
            hold(b, census_s, ramp_s=0.0, sink=writer(fh, -1, bias))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        board.park(b)
        board.close()
    new = identify(
        paths[1],
        paths[0],
        replace(rig, bias=b),
        volts_per_count=cfg.volts_per_count,
        control_hz=cfg.control_hz,
        t_amp_s=cfg.t_amp_s,
        t_dc_s=cfg.t_dc_s,
    )
    new.save(*([rig_path] if rig_path else []))
    return show_preflight(new, cfg)


def link(rig, cfg, port=None, seconds=30.0):
    keep_awake()
    board = Board(rig, cfg, port)
    try:
        board.start(rig.bias)
        hold = Hold(board, cfg, rig.n)
        hold.v = rig.bias.copy()
        T, C = hold(rig.bias, seconds, ramp_s=0.0)
    finally:
        board.park(rig.bias)
        board.close()
    fs = (len(T) - 1) / (T[-1] - T[0])
    Y = np.fft.rfft(C - C.mean(0), axis=0)
    Y[np.fft.rfftfreq(len(C), 1.0 / fs) < 5.0] = 0
    mid = 0.5 * (C[:-2] + C[2:])
    print("%.1f frames/s, largest gap %.1f ms" % (fs, 1000.0 * np.diff(T).max()))
    print("rest  " + " ".join(("%7.1f" % x for x in C.mean(0))))
    print(
        ">5 Hz "
        + " ".join(("%7.2f" % x for x in np.fft.irfft(Y, n=len(C), axis=0).std(0)))
        + "   counts rms"
    )
    print("%d samples jump >150 counts" % int((np.abs(C[1:-1] - mid) > 150).sum()))


def staircase(rig, cfg, port=None, coil=None, bottom=0.3, top=0.5, step=0.05, dwell=10.0, rest=0.3):
    keep_awake()
    n = rig.n
    levels = np.round(np.arange(bottom, top + 1e-09, step), 3)
    worst = top * (n if coil is None else 1) + rest * (0 if coil is None else n - 1)
    if worst > cfg.sum_v_limit:
        sys.exit(
            "that reaches %.2f V summed; the supply folds back near %.1f V."
            % (worst, cfg.sum_v_limit)
        )

    def volts(v):
        out = np.full(n, rest)
        out[slice(None) if coil is None else coil] = v
        return out

    board = Board(rig, cfg, port)
    hold = Hold(board, cfg, n)
    try:
        board.start(np.zeros(n))
        print(" volts | level a0..a7 | beyond the DC matrix's prediction")
        base = None
        for v in levels:
            T, C = hold(volts(v), dwell, ramp_s=5.0 if base is None else 2.0)
            G = np.column_stack(
                [np.ones(len(T))]
                + [f(2 * np.pi * fm * T) for fm in rig.f_hz for f in (np.cos, np.sin)]
            )
            level = np.linalg.lstsq(G, C, rcond=None)[0][0]
            base = level if base is None else base
            extra = level - base - rig.dc @ (volts(v) - volts(levels[0]))
            print(
                " %.2f  | %s | %s"
                % (
                    v,
                    " ".join(("%5.0f" % x for x in level)),
                    " ".join(("%+5.0f" % x for x in extra)),
                )
            )
            if (C.mean(0) < 20).any():
                print("!! a sensor under 20 counts: stopping")
                break
    except KeyboardInterrupt:
        pass
    finally:
        hold(np.full(n, min(rest, 0.3)), 2.0, ramp_s=3.0)
        board.close()
