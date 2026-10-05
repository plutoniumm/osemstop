"""Run a controller on the hardware and call for kicks out loud.

    python3 campaign.py v4 180 55,95,135          # all four channels
    python3 campaign.py v4 180 55,95,135 ch0      # ch0 only

Says "jerk it" through the speaker at each scheduled time and stamps the same
moment into stdout, so the log can be correlated with what the optic actually
did. A watchdog raises KeyboardInterrupt once in the main thread at the end, so
main()'s finally clause runs to completion and returns every coil to bias.

Do NOT stop this with pkill -- the pattern matches the timeout wrapper too, and
the second SIGINT lands inside the finally, aborting the bias restore.
"""
import _thread
import importlib.util
import os
import subprocess
import sys
import threading
import time

ROOT = "/Users/gojira/Documents/GitHub.nosync/ligo"
VERSION = sys.argv[1]
SECONDS = float(sys.argv[2])
KICKS = [float(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 and sys.argv[3] else []
CH0_ONLY = len(sys.argv) > 4 and sys.argv[4] == "ch0"
PORT = os.environ.get("BENCHPORT", "/dev/cu.usbserial-1120")

sys.path.insert(0, ROOT)
os.chdir(ROOT)

path = os.path.join(ROOT, "osem.%s.py" % VERSION)
spec = importlib.util.spec_from_file_location("osem_" + VERSION.replace(".", "_"), path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

if getattr(mod, "BENCH_STATUS", "") == "broken":
    raise SystemExit("  %s is BENCH_STATUS='broken' -- refusing." % VERSION)

mod.PORT = PORT
if CH0_ONLY:
    mod.ENABLE_CHANNEL = [True] + [False] * (len(mod.ENABLE_CHANNEL) - 1)

t0 = time.time()


def call_for_kick(n, total):
    stamp = time.time() - t0
    print("\n>>> KICK %d/%d REQUESTED at t=%.1fs <<<\n" % (n, total, stamp), flush=True)
    try:
        subprocess.run(["say", "jerk it"], timeout=10)
    except Exception as e:
        print("  (say failed: %s)" % e, flush=True)


n_ch = sum(1 for x in mod.ENABLE_CHANNEL if x)
print("=" * 74)
print("  version   : %s (%s)" % (VERSION, mod.BENCH_STATUS))
print("  fixes     : %s" % (", ".join(getattr(mod, "FIXES", ())) or "none"))
print("  channels  : %d on of %d -- %s" % (n_ch, len(mod.ENABLE_CHANNEL), mod.ENABLE_CHANNEL))
print("  coil map  : %s" % (mod.DAC_CHANNELS,))
print("  Kp        : steady %s" % (list(mod.STEADY_GAIN[:len(mod.ENABLE_CHANNEL)]),))
print("  Ki / Kd   : %s / %s" % (mod.KI_GAIN[0], mod.KD_GAIN[0]))
print("  calib     : %.0f s     run length: %.0f s" % (mod.CALIBRATION_S, SECONDS))
print("  kicks at  : %s" % (KICKS or "none"))
print("=" * 74, flush=True)

for i, k in enumerate(KICKS, 1):
    threading.Timer(k, call_for_kick, args=(i, len(KICKS))).start()
threading.Timer(SECONDS, lambda: _thread.interrupt_main()).start()

mod.main()
