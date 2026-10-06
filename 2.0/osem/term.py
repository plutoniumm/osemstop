import os
import re
import sys

CODES = dict(red=31, green=32, yellow=33, cyan=36, dim=2, bold=1)
STATE = dict(DAMPING="green", CALIBRATING="yellow", FAULT="red", PASS="green", FAIL="red")


def c(s, *names):
    if not names or not sys.stdout.isatty() or "NO_COLOR" in os.environ:
        return str(s)
    return "\033[%sm%s\033[0m" % (";".join(str(CODES[n]) for n in names), s)


def paint(msg):
    head = msg.lstrip()
    if head.startswith("!!"):
        return c(msg, "red", "bold")
    if head.startswith("LOCKED"):
        return c(msg, "green", "bold")
    return re.sub(r"^(\s*)(\[[^\]]+\])", lambda m: m[1] + c(m[2], "cyan"), msg)
