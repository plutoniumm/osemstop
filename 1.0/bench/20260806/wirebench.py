#!/usr/bin/env python3
"""wirebench.py -- measure the OSEM serial data path on the real board.

Deliberately does NOT import pyDAC/pyDAC2: it must be able to measure a board
whose firmware those files do not yet know how to talk to, and it must be able
to *emulate* both transports' loop shapes rather than inherit one of them.

  rate    sustained sample rate: stream only / +8 coils fire-and-forget /
          +8 coils with the per-SET ack read (the DACController loop shape)
  noise   per-channel mean, raw std, and a high-frequency noise-floor estimate
  lat     SET round-trip latency, idle and while streaming
"""
import argparse, statistics, sys, time
import serial

SYNC0, SYNC1 = 0xA5, 0xC3
FRAME = 20          # sync0 sync1 seq + 8*uint16 LE + cksum


def flush_in(ser, limit=2.0):
    """Read-drain, never tcflush.

    reset_input_buffer() can WEDGE this FTDI port on macOS when the driver
    queue is deeply backed up: observed 2026-08-06 with in_waiting pinned at the
    1020-byte report cap -- one tcflush and then ZERO bytes delivered for the
    remaining 10 s of the run, with the board still happily streaming. Reading
    the queue empty always works.
    """
    end = time.perf_counter() + limit
    while time.perf_counter() < end:
        k = ser.in_waiting
        if k:
            ser.read(k)
            continue
        time.sleep(0.003)
        if not ser.in_waiting:
            return


def open_port(port, baud, timeout=2.0):
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(2.0)
    for _ in range(400):
        if ser.readline().decode(errors="replace").strip() == "READY":
            break
    else:
        raise RuntimeError("no READY on %s at %d" % (port, baud))
    ser.write(b"STOP\n")
    time.sleep(0.2)
    flush_in(ser)
    return ser


def cmd(ser, text, want=None, deadline=3.0):
    """Write a command and scan the RAW byte stream for `want`.

    Not readline(): in binary mode there are no lines, and in ASCII mode the
    reply arrives behind however many sample rows were already in flight.
    """
    ser.write(text.encode() + b"\n")
    if want is None:
        return None
    end = time.perf_counter() + deadline
    acc = bytearray()
    tok = want.encode()
    while time.perf_counter() < end:
        k = ser.in_waiting
        if k:
            acc += ser.read(k)
            i = acc.find(tok)
            if i >= 0:
                j = acc.find(b"\n", i)
                return acc[i:j if j > 0 else len(acc)].decode(errors="replace").strip()
            del acc[:-256]
    raise RuntimeError("no %r after %r" % (want, text))


def stop_stream(ser, tries=12, each=1.5):
    """STOP, retried. Under a heavy SET load the OLD firmware stops answering
    commands at all -- one String allocation and one 19-byte `OK` echo per SET,
    800 of them a second, against a TX ring that is already full of samples.
    Report how many attempts it took rather than aborting the run.
    """
    for k in range(tries):
        try:
            cmd(ser, "STOP", "STOPPED", deadline=each)
            return k + 1
        except RuntimeError:
            continue
    return -1


# ---------------------------------------------------------------- decoders
class AsciiDecoder:
    """Counts data rows and returns parsed samples; text replies are dropped."""
    name = "ascii"

    def __init__(self, ncols=8):
        self.buf = b""
        self.ncols = ncols
        self.rows = 0
        self.bad = 0
        self.replies = 0

    def feed(self, chunk, collect=None):
        self.buf += chunk
        while True:
            i = self.buf.find(b"\n")
            if i < 0:
                break
            line, self.buf = self.buf[:i], self.buf[i + 1:]
            line = line.strip()
            if not line:
                continue
            if not (48 <= line[0] <= 57):
                self.replies += 1
                continue
            parts = line.split(b",")
            if len(parts) < self.ncols:
                self.bad += 1
                continue
            try:
                vals = [int(p) for p in parts[:self.ncols]]
            except ValueError:
                self.bad += 1
                continue
            self.rows += 1
            if collect is not None:
                collect.append(vals)

    @property
    def dropped(self):
        return 0


class BinDecoder:
    """Fixed 20-byte framed binary. Resyncs on the sync word, verifies the
    additive checksum, and uses the sequence byte to count frames lost."""
    name = "bin"

    def __init__(self, ncols=8):
        self.buf = bytearray()
        self.ncols = ncols
        self.rows = 0
        self.bad = 0
        self.replies = 0
        self.dropped = 0
        self._seq = None
        self._text = bytearray()

    def feed(self, chunk, collect=None):
        self.buf += chunk
        b = self.buf
        i = 0
        n = len(b)
        while i < n:
            if b[i] == SYNC0 and i + 1 < n and b[i + 1] == SYNC1:
                if n - i < FRAME:
                    break
                fr = b[i:i + FRAME]
                ck = (fr[2] + sum(fr[3:19])) & 0xFF
                if ck != fr[19]:
                    self.bad += 1
                    i += 1
                    continue
                vals = [fr[3 + 2 * k] | (fr[4 + 2 * k] << 8) for k in range(8)]
                if any(v > 1023 for v in vals):
                    self.bad += 1
                    i += 1
                    continue
                seq = fr[2]
                if self._seq is not None:
                    self.dropped += (seq - self._seq - 1) & 0xFF
                self._seq = seq
                self.rows += 1
                if collect is not None:
                    collect.append(vals[:self.ncols])
                i += FRAME
                continue
            if b[i] == SYNC0 and i + 1 >= n:
                break
            if b[i] == 10:
                if self._text.strip():
                    self.replies += 1
                self._text.clear()
            elif b[i] < 0x80:
                self._text.append(b[i])
            i += 1
        del b[:i]


def make_decoder(proto):
    return AsciiDecoder() if proto == "ascii" else BinDecoder()


# ---------------------------------------------------------------- scenarios
def drain_for(ser, dec, secs, writer=None, collect=None):
    """Read for `secs`, calling writer(t) each pass. Returns rows/s."""
    t0 = time.perf_counter()
    end = t0 + secs
    loops = 0
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        k = ser.in_waiting
        if k:
            dec.feed(ser.read(k), collect)
        if writer is not None:
            writer(now)
        loops += 1
    dt = time.perf_counter() - t0
    return dec.rows / dt, dt, loops


def scen_stream_only(ser, args, dec):
    cmd(ser, "STREAM", "STREAMING")
    time.sleep(0.3)
    flush_in(ser)
    dec.__init__()
    rate, dt, _ = drain_for(ser, dec, args.secs)
    cmd(ser, "STOP", "STOPPED")
    return dict(rows_per_s=rate, secs=dt, bad=dec.bad, dropped=dec.dropped)


def scen_fast_coils(ser, args, dec):
    """v11 / FastDAC shape: 8 coils, fire-and-forget, throttled to update_hz."""
    cmd(ser, "STREAM", "STREAMING")
    time.sleep(0.3)
    flush_in(ser)
    dec.__init__()
    period = 1.0 / args.update_hz
    state = {"next": time.perf_counter(), "w": 0}

    def writer(now):
        if now >= state["next"]:
            state["next"] = now + period
            buf = bytearray()
            for c in range(8):
                buf += b"SET %d %.4f\n" % (c, 0.25 + 0.02 * ((state["w"] + c) % 5))
            ser.write(bytes(buf))
            state["w"] += 8

    rate, dt, _ = drain_for(ser, dec, args.secs, writer)
    tries = stop_stream(ser)
    return dict(rows_per_s=rate, secs=dt, writes_per_s=state["w"] / dt,
                bad=dec.bad, dropped=dec.dropped, stop_tries=tries)


def scen_ack_coils(ser, args, dec):
    """v3..v10 / DACController shape: read a sample, then 8 acked SETs.

    Reports the CONTROL LOOP rate, which is what the controllers log -- the
    acks are read with readline() and every stream row consumed on the way to
    an `OK` is thrown away, exactly as pyDAC.set_voltage does.
    """
    if dec.name != "ascii":
        return dict(skipped="ack loop is an ASCII-protocol shape only")
    cmd(ser, "STREAM", "STREAMING")
    time.sleep(0.3)
    flush_in(ser)
    t0 = time.perf_counter()
    end = t0 + args.secs
    loops = 0
    samples = 0
    eaten = 0
    while time.perf_counter() < end:
        for _ in range(200):                     # read one data row
            raw = ser.readline()
            if raw[:1].isdigit():
                samples += 1
                break
        for c in range(8):                       # 8 acked writes
            ser.write(b"SET %d %.4f\n" % (c, 0.25))
            for _ in range(50):
                r = ser.readline()
                if r.startswith(b"OK") or r.startswith(b"ERR"):
                    break
                if r[:1].isdigit():
                    eaten += 1
        loops += 1
    dt = time.perf_counter() - t0
    stop_stream(ser)
    return dict(loop_hz=loops / dt, samples_used_per_s=samples / dt,
                samples_discarded_per_s=eaten / dt, secs=dt)


def scen_noise(ser, args, dec):
    cmd(ser, "STREAM", "STREAMING")
    time.sleep(0.5)
    flush_in(ser)
    dec.__init__()
    rows = []
    rate, dt, _ = drain_for(ser, dec, args.secs, collect=rows)
    cmd(ser, "STOP", "STOPPED")
    if len(rows) < 100:
        return dict(error="only %d rows" % len(rows))
    cols = list(zip(*rows))
    out = dict(rows=len(rows), rows_per_s=rate, secs=dt, ch={})
    for i, c in enumerate(cols):
        c = list(c)
        # second-difference estimator: for white noise sigma_hat =
        # std(x[n]-2x[n-1]+x[n-2])/sqrt(6); a 1 Hz pendulum at ~1 kHz sampling
        # contributes A*(w*dt)^2 ~ 1e-4 counts, i.e. nothing.
        d2 = [c[k] - 2 * c[k - 1] + c[k - 2] for k in range(2, len(c))]
        out["ch"][i] = dict(mean=statistics.fmean(c),
                            std=statistics.pstdev(c),
                            nf=statistics.pstdev(d2) / 6 ** 0.5,
                            pkpk=max(c) - min(c))
    return out


def scen_latency(ser, args, dec):
    """Round trip of a SET: write -> `OK` back. Idle, then while streaming."""
    out = {}
    for phase in ("idle", "streaming"):
        if phase == "streaming":
            cmd(ser, "STREAM", "STREAMING")
            time.sleep(0.3)
        flush_in(ser)
        ts = []
        for k in range(args.n):
            t0 = time.perf_counter()
            ser.write(b"SET %d %.4f\n" % (k % 8, 0.25))
            for _ in range(400):
                r = ser.readline()
                if r.startswith(b"OK") or r.startswith(b"ERR"):
                    break
            ts.append((time.perf_counter() - t0) * 1e3)
        ts.sort()
        out[phase] = dict(n=len(ts), median_ms=ts[len(ts) // 2],
                          p90_ms=ts[int(len(ts) * 0.9)], max_ms=ts[-1],
                          mean_ms=statistics.fmean(ts))
        if phase == "streaming":
            cmd(ser, "STOP", "STOPPED")
    return out


SCEN = dict(stream=scen_stream_only, fast=scen_fast_coils, ack=scen_ack_coils,
            noise=scen_noise, lat=scen_latency)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenarios", nargs="+", choices=sorted(SCEN) + ["all"])
    ap.add_argument("--port", default="/dev/cu.usbserial-1120")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--update-hz", type=float, default=100.0)
    ap.add_argument("--proto", default="ascii", choices=["ascii", "bin"])
    ap.add_argument("--presc", type=int, default=None)
    ap.add_argument("--ack", type=int, default=None, choices=[0, 1])
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    names = sorted(SCEN) if "all" in args.scenarios else args.scenarios
    ser = open_port(args.port, args.baud)
    try:
        if args.presc is not None:
            print("  %s" % cmd(ser, "PRESC %d" % args.presc, "OK"))
        if args.ack is not None:
            print("  %s" % cmd(ser, "ACK %d" % args.ack, "OK"))
        if args.proto == "bin":
            print("  %s" % cmd(ser, "MODE BIN", "OK"))
        print("== %s  proto=%s presc=%s baud=%d %s" %
              (time.strftime("%H:%M:%S"), args.proto, args.presc, args.baud,
               args.label))
        for nm in names:
            dec = make_decoder(args.proto)
            r = SCEN[nm](ser, args, dec)
            print("-- %s" % nm)
            if nm == "noise" and "ch" in r:
                print("   rows=%d  %.1f Hz  %.1f s" %
                      (r["rows"], r["rows_per_s"], r["secs"]))
                print("   ch   mean      std     pk-pk   noisefloor(LSB)")
                for i, d in r["ch"].items():
                    print("   a%d  %7.2f  %7.3f  %5d   %6.3f" %
                          (i, d["mean"], d["std"], d["pkpk"], d["nf"]))
            else:
                for k, v in r.items():
                    print("   %-24s %s" % (k, ("%.3f" % v) if isinstance(v, float) else v))
            sys.stdout.flush()
    finally:
        try:
            ser.write(b"STOP\n")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()
