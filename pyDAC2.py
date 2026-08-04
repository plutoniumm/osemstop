"""
pyDAC2 -- a fire-and-forget variant of pyDAC, for system identification.
========================================================================
Same board, same wire protocol, same 0..2.5 V limits. One difference, and it is
the whole reason this file exists:

    pyDAC.set_voltage()  writes SET, then READS UP TO 50 LINES waiting for the
                         `OK` ack -- and every one of those lines is a stream
                         sample that is thrown away.

Measured on the 2026-08-03 bench logs: the median inter-sample time across a
whole run is 2.80 ms (348 Hz, the 115200-baud wire limit for eight columns), but
across consecutive DAMPING rows it is 41 ms. The loop runs at ~24 Hz whenever it
is driving coils, because each write eats samples.

For a damping controller that is bad. For system identification it is fatal:
every method here correlates a known excitation against the response, so
throwing away 14 samples out of 15 destroys both the SNR and the phase.

    FastDAC.set_voltage()  writes and returns. No readline, no print.

WHAT YOU GIVE UP. The `OK`/`ERR` acknowledgement. That is an acceptable trade
and not a silent one:

  * `ERR` is only returned for a malformed command or a channel > 7, both of
    which are argument errors this class already rejects locally before writing;
  * the firmware clamps voltage to 0..2.5 V itself, so an out-of-range value
    cannot damage anything even if it got through;
  * `pyDAC`'s own caller already treats the ack as optional -- the shipping
    `RateLimitedActuator.send()` catches the RuntimeError raised when no ack
    arrives and moves on, because "the next sample resends".

WHAT YOU MUST HANDLE. The board still SENDS `OK ch=.. v=..` for every write, and
those lines are now interleaved with the sample stream instead of being consumed.
`read_sample()` below filters them. Anything else reading this stream must too.

BANDWIDTH. `SET` shares one wire with the stream, and at 115200 baud with eight
columns the stream alone is already at capacity (33 bytes x 10 bits / 115200 =
2.86 ms against 2.88 ms measured). Writes therefore cost stream samples no matter
how they are sent, which is why the continuous drivers update the coils at
UPDATE_HZ rather than once per sample. Raising the baud rate is the real fix and
needs `arduino.ino` changed to match.
"""

import time

import serial


class FastDAC:
    """Transport for the identification runs. Not for closed-loop damping --
    the controllers ship against `pyDAC.DACController` and are validated with
    it."""

    VMIN_HW, VMAX_HW = 0.0, 2.5          # what the firmware itself clamps to

    def __init__(self, port, baud=115200, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)                    # the board resets when the port opens
        ready = self.ser.readline().decode(errors="replace").strip()
        if ready != "READY":
            raise RuntimeError("Arduino did not send READY, got: %r" % ready)
        self.writes = 0
        print("Connected on %s at %d baud (fire-and-forget)" % (port, baud))

    # ---- output ----------------------------------------------------------
    def set_voltage(self, channel, voltage):
        """Write and return. Validated locally, since nothing checks for us."""
        if not (0 <= int(channel) <= 7):
            raise ValueError("Channel must be 0-7")
        v = float(voltage)
        if not (self.VMIN_HW <= v <= self.VMAX_HW):
            raise ValueError("Voltage must be 0.0-2.5 V, got %.4f" % v)
        self.ser.write(b"SET %d %.4f\n" % (int(channel), v))
        self.writes += 1

    def set_many(self, channels, volts):
        """All channels in ONE write() call. Same bytes on the wire, but one
        syscall and no chance of the stream interleaving mid-command."""
        buf = bytearray()
        for c, v in zip(channels, volts):
            if not (0 <= int(c) <= 7):
                raise ValueError("Channel must be 0-7")
            v = float(v)
            if not (self.VMIN_HW <= v <= self.VMAX_HW):
                raise ValueError("Voltage must be 0.0-2.5 V, got %.4f" % v)
            buf += b"SET %d %.4f\n" % (int(c), v)
        self.ser.write(bytes(buf))
        self.writes += len(channels)

    # ---- input -----------------------------------------------------------
    def start_stream(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"STREAM\n")
        return self._await("STREAMING")

    def stop_stream(self):
        self.ser.write(b"STOP\n")
        return self._await("STOPPED")

    def _await(self, want, limit=200):
        """Stream lines may already be in flight, so look past them."""
        for _ in range(limit):
            line = self.ser.readline().decode(errors="replace").strip()
            if line == want:
                return line
        raise RuntimeError("no %s seen" % want)

    def read_sample(self, ncols):
        """One row of `ncols` ints, or None.

        Returns None rather than raising for an ack echo, a partial line or a
        short row -- the caller is in a tight acquisition loop and a dropped row
        is normal. `OK`/`ERR` lines are the acks nobody is reading any more.
        """
        if not self.ser.in_waiting:
            return None
        raw = self.ser.readline().decode(errors="replace").strip()
        if not raw or raw[0] not in "0123456789-":
            return None                  # OK / ERR / STREAMING / junk
        parts = raw.split(",")
        if len(parts) < ncols:
            return None
        try:
            return [int(p) for p in parts[:ncols]]
        except ValueError:
            return None

    def drain(self):
        """Throw away whatever is buffered. Use after changing the excitation,
        so the first sample of a new step is not one from the previous step."""
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()
        print("Connection closed. %d DAC writes." % self.writes)
