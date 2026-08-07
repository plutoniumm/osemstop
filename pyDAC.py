"""
pyDAC -- the shipping transport for osem.v3 .. osem.v10.
=======================================================
Acked writes, ASCII stream. Byte-for-byte the same conversation those
controllers were validated against; the only thing that changed on 2026-08-06
is the baud rate and how the input queue is emptied, both of which are fixes
for defects rather than behaviour changes.

BAUD -- 500000, and NOT a free choice
-------------------------------------
The AVR makes its UART clock by integer division of a 16 MHz crystal, so only
some rates actually exist:

        want      UBRR   actual      error
        115200     16    117647     +2.12%
        230400      8    222222     -3.55%    <-- broken
        250000      7    250000     +0.00%
        500000      3    500000     +0.00%

230400 was set here and in `arduino.ino` earlier on 2026-08-06 and it does not
work. Measured on the bench the same day: a host at 230400 reads a clean
`READY` -- the FTDI receiver tolerates the board's 222222 baud transmission --
and then EVERY command comes back `ERR unknown command`, because a -3.55%
mismatch is outside the AVR receiver's own tolerance. That failure is nasty
precisely because it passes `bench.py`'s preflight, which only checks for
READY, and the controller then waits forever for a stream that never starts.

500000 is exact on both ends and was measured clean over 8 s blocks with zero
checksum failures. 1000000 is also exact and works, but buys only ~6% because
the loop is no longer wire-limited; 2000000 corrupts frames. It is a DEFAULT,
not a constant, so a board still carrying an old sketch is reachable with
DACController(port, baud=115200) -- but the two must agree or `READY` never
arrives and __init__ raises, which is the failure you want rather than silent
garbage.
"""

import time

import serial

BAUD = 500000


def drain_input(ser, limit=2.0):
    """Empty the input queue by READING it, never with reset_input_buffer().

    tcflush (which is what reset_input_buffer() is) can wedge this FTDI port on
    macOS when the driver queue is deeply backed up. Observed 2026-08-06: with
    `in_waiting` pinned at the driver's 1020-byte report cap, one tcflush and
    then ZERO bytes were delivered for the remaining ten seconds of the run --
    while the board was still streaming perfectly happily. It is not
    reproducible on a shallow queue, which is exactly what makes it dangerous:
    it fires under load, in the middle of a run, and looks like a dead board.
    """
    end = time.perf_counter() + limit
    while time.perf_counter() < end:
        if ser.in_waiting:
            ser.read(ser.in_waiting)
            continue
        time.sleep(0.003)
        if not ser.in_waiting:
            return


class DACController:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)

        # Scan for READY rather than demanding it on the first line. Opening the
        # port toggles DTR and resets the board, but data the board sent BEFORE
        # that reset is still buffered by the OS and arrives first -- so if the
        # previous session left it streaming, line 1 is a stale half-sample and
        # a healthy board looks dead. READY is emitted after the reset, so it is
        # always at the end of that backlog. The sketch also prints one INFO
        # line just before READY; that is inside this scan by design.
        for _ in range(200):
            if self.ser.readline().decode(errors="replace").strip() == "READY":
                break
        else:
            raise RuntimeError(
                f"Arduino never sent READY on {port} (scanned 200 lines). "
                f"Wrong port, board not flashed, or wrong baud -- the sketch "
                f"and this file must both be at {BAUD}. Reflash with "
                f"`make arduino`.")

        # Land in a known state: the reset should have cleared `streaming`, but
        # say so explicitly and drop anything still in flight.
        self.ser.write(b"STOP\n")
        time.sleep(0.2)
        drain_input(self.ser)

        # v3..v10 read the `OK` echo for every SET, so the echo must be on. The
        # board resets on open and boots with ACK on, so this is belt and
        # braces against a session that left it off -- but it costs one command
        # and removes a whole class of "why is set_voltage timing out".
        self.ser.write(b"ACK 1\n")
        time.sleep(0.05)
        drain_input(self.ser, 0.3)
        print(f"Connected on {port} at {baud} baud")

    def set_voltage(self, channel: int, voltage: float) -> str:
        if not (0 <= channel <= 7):
            raise ValueError("Channel must be 0–7")
        if not (0.0 <= voltage <= 2.5):
            raise ValueError("Voltage must be 0.0–2.5 V")
        cmd = f"SET {channel} {voltage:.4f}\n"
        self.ser.write(cmd.encode())

        # Skip any stray streaming data lines until we see the real reply.
        # Every line skipped here is a sample thrown away -- measured on the
        # bench at 289 discarded samples/s against a 23 Hz loop with eight
        # coils. That is what pyDAC2.FastDAC exists to avoid; it is left alone
        # here because v3..v10 were validated with exactly this behaviour.
        for _ in range(50):  # bounded, so we can't hang forever
            response = self.ser.readline().decode(errors="replace").strip()
            if response.startswith("OK") or response.startswith("ERR"):
                break
        else:
            raise RuntimeError(f"No OK/ERR reply seen for SET {channel} {voltage:.4f} "
                            f"(serial may be desynced)")

        if response.startswith("ERR"):
            print(f"    !! DAC REJECTED: ch={channel} v={voltage:.4f} -> {response}")
        else:
            print(f"    {response}")
        return response

    def read_sample(self, ncols: int = 8):
        """One row of `ncols` ints, or None.

        v3..v10 each carry their own copy of this and call it on `dac.ser`
        directly, so this is not on their path -- it is here so that anything
        new has one correct implementation to use, and so the first-character
        filter is not re-derived a seventh time. Returns None rather than
        raising for a reply echo, a partial line or a short row.
        """
        if not self.ser.in_waiting:
            return None
        raw = self.ser.readline().decode(errors="replace").strip()
        if not raw or raw[0] not in "0123456789-":
            return None                  # OK / ERR / INFO / STREAMING / junk
        parts = raw.split(",")
        if len(parts) < ncols:
            return None
        try:
            return [int(p) for p in parts[:ncols]]
        except ValueError:
            return None

    def _await(self, want, deadline=5.0):
        """Look past whatever samples were already in flight for a reply.

        Bounded by WALL CLOCK, not by a line count: a line count multiplies by
        the read timeout when the board is silent, so `for _ in range(400)`
        against a dead board is a 13-minute hang, not an error.
        """
        end = time.perf_counter() + deadline
        while time.perf_counter() < end:
            line = self.ser.readline().decode(errors="replace").strip()
            if line == want:
                return line
        raise RuntimeError(
            "no %s seen in %.1fs -- board silent, or host and sketch disagree "
            "on baud (this file: %d)" % (want, deadline, self.ser.baudrate))

    def start_stream(self) -> str:
        drain_input(self.ser)
        self.ser.write(b"STREAM\n")
        response = self._await("STREAMING")
        print(f"    {response}")
        return response

    def stop_stream(self) -> str:
        self.ser.write(b"STOP\n")
        response = self._await("STOPPED")
        print(f"    {response}")
        return response

    def close(self):
        self.ser.close()
        print("Connection closed.")
