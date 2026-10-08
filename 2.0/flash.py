#!/usr/bin/env python3
import os
import subprocess
import sys
import osem

sketch = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")
port = sys.argv[1] if len(sys.argv) > 1 else osem.Board.find_port()
for step in (["compile"], ["upload", "-p", port]):
    if subprocess.run(["arduino-cli"] + step + ["--fqbn", "arduino:avr:mega", sketch]).returncode:
        sys.exit("arduino-cli %s failed" % step[0])
print("flashed %s; all coils are at 0 V." % port)
