#!/usr/bin/env python3
"""
bench.py -- the on-hardware entry point: preflight the board, then run a controller.
====================================================================================
`harness.py` is the entry point for everything OFF the bench. This is its
opposite number: it talks to the real Arduino and then hands control to a real
`osem.vN.py`. Nothing in here simulates anything.

    python3 bench.py              # pick a version interactively, then run it
    python3 bench.py v0           # run that one
    python3 bench.py v0 --port /dev/cu.usbmodem1401
    python3 bench.py --ports      # just list candidate serial ports
    python3 bench.py --flash      # compile and upload arduino.ino, nothing else

or through make, which is the intended form:

    make run              make run V=v0              make run V=v0 PORT=COM7
    make arduino          make arduino FQBN=arduino:sam:arduino_due_x

What it does before anything reaches the coils:

  1. resolves a serial port -- explicit, else auto-detected, else asks;
  2. PREFLIGHT: opens the port through the real `DACController`, which raises
     unless the sketch answers `READY`. A failure here means the board is not
     flashed or the port is wrong, and it costs nothing to find out now rather
     than after the optic is swinging;
  3. prints what is about to be applied -- bench status, enabled channels, the
     actual gain vectors -- because the gains are per-file constants and the
     versions do NOT ship the same ones;
  4. refuses `BENCH_STATUS = "broken"` outright and asks for confirmation on
     anything that is not `validated`;
  5. sets the module's PORT and calls its `main()`.

Step 5 is why this file exists rather than a shell wrapper around
`python osem.vN.py`: PORT is a module-level constant in five separate files
(`COM7`, a Windows name), and rewriting it in each one before every run is how
you end up flashing a controller whose diff you no longer trust. Loading the
module and assigning `mod.PORT` leaves every controller byte-for-byte what was
validated. `main()` is called explicitly since `__name__` is not `"__main__"`
here; everything else about the run is identical to launching the file directly.
"""

# The REAL pyserial, imported first and deliberately. `sim/server.py` installs a
# fake `serial` module into sys.modules -- guarded by `if "serial" not in
# sys.modules`, so importing the genuine one first also immunises this process
# against it. That is also why nothing below imports harness.py or sim/: pulling
# in either would replace the transport with one whose Serial() raises on
# construction. The version discovery here duplicates harness.versions() for
# that reason, and must stay in step with it.
try:
    import serial                                # noqa: F401
    from serial.tools import list_ports
except ImportError:
    raise SystemExit(
        "  pyserial is not installed in this interpreter.\n"
        "    conda activate ligo      # has numpy + pyserial\n"
        "  A bare `python3` on this Mac has neither, and every failure that\n"
        "  causes looks like a repo bug rather than a missing dependency.")

import argparse
import glob
import importlib.util
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Arduino/clone USB-serial bridges: genuine (2341, 2A03), CH340 (1A86),
# FTDI (0403), CP210x (10C4). Ranked above other USB serial devices, which are
# in turn ranked above anything with no VID at all -- on a Mac that last group
# is Bluetooth profiles and the debug console, never a board.
KNOWN_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}

DEFAULT_FQBN = "arduino:avr:mega"       # CS on pin 53 is Mega/Due specific
FIRMWARE = "arduino.ino"                # the sketch that is on the board


# --------------------------------------------------------------------------
# versions
# --------------------------------------------------------------------------
def versions():
    """Every osem.vN.py here, ordered by N."""
    found = []
    for path in glob.glob(os.path.join(HERE, "osem.v*.py")):
        m = re.match(r"osem\.v(\d+)$", os.path.basename(path)[:-3])
        if m:
            found.append((int(m.group(1)), path))
    return [p for _, p in sorted(found)]


def declared(path, name, default="unknown"):
    """Read a string constant out of a controller WITHOUT importing it.

    Building the menu must not execute five modules -- each one imports numpy
    and pyDAC, and a broken one must not execute. A regex is enough for the two
    string constants the menu needs.
    """
    with open(path) as f:
        m = re.search(r'^%s\s*=\s*["\']([^"\']*)["\']' % name, f.read(), re.M)
    return m.group(1) if m else default


def resolve(name):
    """'v2', 'osem.v2', 'osem.v2.py' or a path -> an absolute path."""
    all_versions = versions()
    if not all_versions:
        sys.exit("No osem.v*.py found next to bench.py.")
    if not name:
        return None
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = name if name.startswith("osem.") else "osem." + name
    cand = os.path.join(HERE, stem if stem.endswith(".py") else stem + ".py")
    if os.path.isfile(cand):
        return cand
    sys.exit("No such version: %s. Have: %s" % (
        name, ", ".join(os.path.basename(p)[5:-3] for p in all_versions)))


STATUS_NOTE = {
    "validated": "confirmed on hardware with a scope",
    "reported":  "provenance.md documents the behaviour, but THIS FILE has never run",
    "untested":  "never been on hardware",
    "broken":    "does not damp -- DO NOT FLASH",
}


def pick_version():
    """Numbered prompt rather than a curses picker: the controller prints to
    this same terminal the moment it starts, and its own `Press Enter` prompt
    has to work afterwards."""
    paths = versions()
    print("\n  which controller?\n")
    for i, p in enumerate(paths):
        tag = os.path.basename(p)[:-3]
        st = declared(p, "BENCH_STATUS")
        print("   %d) %-10s %-10s %s" % (i, tag, st, STATUS_NOTE.get(st, "")))
    print()
    while True:
        try:
            raw = input("  number (or vN), blank to cancel: ").strip()
        except (EOFError, KeyboardInterrupt):
            sys.exit("\nCancelled.")
        if not raw:
            sys.exit("Cancelled.")
        if raw.isdigit() and int(raw) < len(paths):
            return paths[int(raw)]
        got = resolve(raw)
        if got:
            return got


# --------------------------------------------------------------------------
# serial port
# --------------------------------------------------------------------------
def rank(port):
    if port.vid in KNOWN_VIDS:
        return 0
    if port.vid is not None:
        return 1
    return 2


def candidates():
    return sorted(list_ports.comports(), key=rank)


def show_ports():
    found = candidates()
    if not found:
        print("  no serial ports at all -- is the board plugged in?")
        return
    for p in found:
        mark = {0: "<- looks like a board", 1: "", 2: "(no VID: not a board)"}[rank(p)]
        print("  %-28s %-28s %s" % (p.device, p.description or "", mark))


def choose_port(explicit):
    if explicit:
        return explicit
    found = [p for p in candidates() if rank(p) < 2]
    if len(found) == 1:
        print("  port: %s (%s)" % (found[0].device, found[0].description or "?"))
        return found[0].device
    if not found:
        print("\n  No USB serial device found. Ports visible:")
        show_ports()
        sys.exit("\n  Plug the board in, or pass PORT=... explicitly.")
    print("\n  more than one candidate:\n")
    for i, p in enumerate(found):
        print("   %d) %-24s %s" % (i, p.device, p.description or ""))
    while True:
        raw = input("\n  number: ").strip()
        if raw.isdigit() and int(raw) < len(found):
            return found[int(raw)].device


def preflight(port):
    """Open the port through the shipping transport and confirm the sketch is
    alive. DACController.__init__ raises unless `READY` arrives, which is the
    one check that distinguishes 'wrong port' from 'board not flashed'."""
    from pyDAC import DACController        # late: needs the real `serial`
    print("\n  preflight: opening %s (the board resets on open, ~2s)..." % port)
    try:
        dac = DACController(port=port)
    except serial.SerialException as e:
        sys.exit("  FAILED to open %s: %s" % (port, e))
    except RuntimeError as e:
        sys.exit("  FAILED: %s\n  The sketch is not running -- try `make arduino`." % e)
    dac.close()
    print("  preflight OK -- sketch answered READY.")


# --------------------------------------------------------------------------
# the run
# --------------------------------------------------------------------------
def load(path):
    """Controller filenames contain a dot, so they load by path, never by import."""
    name = os.path.basename(path)[:-3]
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)            # main() is behind __main__, so this is safe
    return mod


def gains(mod):
    def row(label, arr):
        return "  %-9s %s" % (label, "  ".join("%+.4f" % float(v) for v in arr))
    lines = [row("steady", mod.STEADY_GAIN), row("capture", mod.CAPTURE_GAIN)]
    if any(float(v) for v in mod.KI_GAIN) or any(float(v) for v in mod.KD_GAIN):
        lines += [row("ki", mod.KI_GAIN), row("kd", mod.KD_GAIN)]
    else:
        lines.append("  ki, kd    all zero -- the law is pure P")
    return "\n".join(lines)


def confirm(question):
    try:
        return input(question).strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        return False


def run(path, port, force):
    tag = os.path.basename(path)[:-3]
    status = declared(path, "BENCH_STATUS")

    # The gate comes before the port is touched: there is no reason to reset the
    # board for a version that is not going to be allowed to drive it.
    if status == "broken" and not force:
        sys.exit("\n  %s is BENCH_STATUS = 'broken' -- %s\n"
                 "  Refusing. Its system identification does not converge, so it\n"
                 "  drives the coils from a wrong actuation matrix.\n"
                 "  Override with `make run V=%s FORCE=1` if you really mean it."
                 % (tag, STATUS_NOTE["broken"], tag[5:]))

    port = choose_port(port)
    preflight(port)

    mod = load(path)
    fixes = ", ".join(getattr(mod, "FIXES", ())) or "none"
    enabled = [i for i, v in enumerate(mod.ENABLE_CHANNEL) if v]
    print("\n  %s   bench status: %s -- %s" % (tag, status, STATUS_NOTE.get(status, "")))
    print("  channels driving: %s of 4   fixes claimed: %s   calibration: %.0fs"
          % (",".join("ch%d" % i for i in enabled) or "none", fixes, mod.CALIBRATION_S))
    print(gains(mod))
    print("  logging to ./data/ (cwd is %s)" % os.getcwd())

    if status != "validated" and not confirm(
            "\n  %s has never been confirmed on hardware. Continue? [y/N] " % tag):
        sys.exit("  Cancelled.")

    mod.PORT = port                     # the whole reason this file loads the module
    print()
    mod.main()


# --------------------------------------------------------------------------
# flashing
# --------------------------------------------------------------------------
def flash(port, fqbn):
    """The firmware is an Arduino sketch, not compilable C: arduino-cli will not
    look at it until it is named <dir>/<dir>.ino.

    FIRMWARE is the source of truth -- it is what is actually on the board.
    Only A0..A3 carry OSEMs, but the sketch streams all eight columns; read_sample
    takes the leading four.
    """
    if not shutil.which("arduino-cli"):
        sys.exit("  arduino-cli not on PATH. brew install arduino-cli, then\n"
                 "  arduino-cli core install arduino:avr")
    src = os.path.join(HERE, FIRMWARE)
    if not os.path.exists(src):
        sys.exit("  no firmware at %s" % src)
    sketch = os.path.join(HERE, "build", "osem_stream")
    os.makedirs(sketch, exist_ok=True)
    shutil.copyfile(src, os.path.join(sketch, "osem_stream.ino"))
    print("  sketch: %s (%s copied as osem_stream.ino)" % (sketch, FIRMWARE), flush=True)

    if subprocess.run(["arduino-cli", "compile", "--fqbn", fqbn, sketch]).returncode:
        sys.exit("  compile failed. Wrong FQBN? Board is a Mega or a Due:\n"
                 "  make arduino FQBN=arduino:sam:arduino_due_x")
    port = choose_port(port)
    if subprocess.run(["arduino-cli", "upload", "--fqbn", fqbn,
                       "-p", port, sketch]).returncode:
        sys.exit("  upload failed on %s." % port)
    print("\n  flashed. setup() has zeroed all eight DAC channels and enabled the\n"
          "  internal 2.5V reference. `make run` next.")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Preflight the board and run a controller on the hardware.")
    ap.add_argument("version", nargs="?", default=None, help="v0, v1, ... (default: ask)")
    ap.add_argument("--port", default=os.environ.get("PORT") or None,
                    help="serial port (default: auto-detect, else ask)")
    ap.add_argument("--flash", action="store_true", help="compile and upload arduino.ino, then exit")
    ap.add_argument("--fqbn", default=DEFAULT_FQBN)
    ap.add_argument("--ports", action="store_true", help="list candidate ports and exit")
    ap.add_argument("--force", action="store_true", help="allow a 'broken' version")
    args = ap.parse_args()

    if args.ports:
        show_ports()
        return 0
    if args.flash:
        flash(args.port, args.fqbn)
        return 0

    path = resolve(args.version) or pick_version()
    run(path, args.port, args.force)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("\nCancelled.")
