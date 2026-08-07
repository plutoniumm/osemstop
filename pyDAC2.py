"""
pyDAC2 -- a fire-and-forget variant of pyDAC, for system identification.
========================================================================
Same board, same wire, same 0..2.5 V limits. One difference, and it is the whole
reason this file exists:

    pyDAC.set_voltage()  writes SET, then READS UP TO 50 LINES waiting for the
                         `OK` ack -- and every one of those lines is a stream
                         sample that is thrown away.

Measured on the bench, 2026-08-06, eight coils at 100 Hz on the same board and
the same firmware:

    stream alone, no coils driven .................. 904 rows/s
    eight coils, fire-and-forget (this class) ...... 862 rows/s
    eight coils, one acked SET each (pyDAC) ........  23 Hz loop,
                                                     289 samples/s discarded

For a damping controller that is bad. For system identification it is fatal:
every method here correlates a known excitation against the response, so
throwing away 37 samples out of 38 destroys both the SNR and the phase.

    FastDAC.set_voltage()  writes and returns. No readline, no print.

WHAT YOU GIVE UP. The `OK`/`ERR` acknowledgement -- and as of 2026-08-06 this
class tells the FIRMWARE to stop sending it at all (`ACK 0`), which is worth
more than not reading it. The echo is `OK ch=N v=D.DDDD\r\n`, 19 bytes, and it
travels on the SAME wire as the sample stream. Eight coils at 100 Hz is 800
echoes/s = 15.2 kB/s, against a 50 kB/s wire that the stream already wants
30 kB/s of. Not reading them never stopped them being sent. `ERR` is still
emitted for a malformed command, so genuine faults are not silenced.

That trade is acceptable and not a silent one:

  * `ERR` was only returned for a malformed command or a channel > 7, both of
    which are argument errors this class already rejects locally before writing;
  * the firmware clamps voltage to 0..2.5 V itself, so an out-of-range value
    cannot damage anything even if it got through;
  * `pyDAC`'s own caller already treats the ack as optional -- the shipping
    `RateLimitedActuator.send()` catches the RuntimeError raised when no ack
    arrives and moves on, because "the next sample resends".

BANDWIDTH. `SET` shares one wire with the stream -- but only in the sense that
the BOARD'S REPLIES do. The UART is full duplex: host->board bytes cost the
board CPU time to parse, they do not take bandwidth from the sample stream.
That is why turning the echo off is the change that matters and why compressing
the command itself is not. See `bench/20260806/firmware_timing.md`.
"""

import time

import serial

BAUD = 500000                            # see pyDAC.BAUD for why it is not 230400

SYNC0, SYNC1 = 0xA5, 0xC3                # both have bit 7 set: never in ASCII
FRAME_LEN = 20                           # sync sync seq + 8*uint16 LE + cksum


def drain_input(ser, limit=2.0):
    """Empty the input queue by READING it, never with reset_input_buffer().

    tcflush can wedge this FTDI port on macOS when the driver queue is deeply
    backed up -- observed 2026-08-06 with `in_waiting` pinned at the driver's
    1020-byte report cap: one tcflush, then zero bytes delivered for the rest of
    the run while the board streamed on happily. It only bites under load,
    which is exactly when `drain()` gets called.
    """
    end = time.perf_counter() + limit
    while time.perf_counter() < end:
        if ser.in_waiting:
            ser.read(ser.in_waiting)
            continue
        time.sleep(0.003)
        if not ser.in_waiting:
            return


class FastDAC:
    """Transport for the identification runs, and from v11 for the eight-channel
    controller as well.

    v3..v10.5 ship against `pyDAC.DACController` and are validated with it, and
    that stays true -- nothing here changes them. What changed on 2026-08-06 is
    that the ack stopped being an inefficiency and became a stability problem:
    the measured loop rate falls with the number of coils driven -- 73 Hz on one,
    23.5 Hz on four, 12.5 Hz on eight (`analysis/out/loop_rate.csv`) -- and at
    12.5 Hz the phase margin at 3.75 Hz is 45.5 deg against 73.5 deg at four
    coils (`analysis/out/phase_budget.csv`). An eight-channel loop cannot afford
    it. See osem.v11.py."""

    VMIN_HW, VMAX_HW = 0.0, 2.5          # what the firmware itself clamps to

    def __init__(self, port, baud=BAUD, timeout=2.0, binary=False, ack=False):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.binary = False              # set for real below, after the handshake
        time.sleep(2)                    # the board resets when the port opens
        # Scan for READY rather than demanding it on line 1 -- the same defect,
        # and the same fix, that `DACController` already carries. Opening the
        # port toggles DTR and resets the board, but whatever the board sent
        # BEFORE that reset is still sitting in the OS buffer and arrives first,
        # so a board left streaming makes a healthy rig look dead. READY is
        # emitted after the reset and is therefore always at the END of that
        # backlog.
        for _ in range(200):
            if self.ser.readline().decode(errors="replace").strip() == "READY":
                break
        else:
            raise RuntimeError(
                "Arduino never sent READY on %s at %d baud (scanned 200 lines). "
                "Wrong port, board not flashed, or the sketch is at a different "
                "baud -- reflash with `make arduino`. Note that 230400 does NOT "
                "work on a 16 MHz AVR; see pyDAC.BAUD." % (port, baud))
        # Land in a known state, as DACController does.
        self.ser.write(b"STOP\n")
        time.sleep(0.2)
        drain_input(self.ser)

        # Stop the board sending an echo nobody reads. 15.2 kB/s of wire back.
        if not ack:
            self.ser.write(b"ACK 0\n")
            time.sleep(0.05)
            drain_input(self.ser, 0.3)

        if binary:
            self.ser.write(b"MODE BIN\n")
            time.sleep(0.05)
            drain_input(self.ser, 0.3)
        self.binary = bool(binary)

        self._buf = bytearray()          # binary reassembly
        self._seq = None
        self.dropped = 0                 # frames the board sent that we lost
        self.badframes = 0
        self.writes = 0
        print("Connected on %s at %d baud (fire-and-forget, %s%s)"
              % (port, baud, "binary" if self.binary else "ascii",
                 "" if ack else ", ack off"))

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
        drain_input(self.ser)
        self.ser.write(b"STREAM\n")
        return self._await(b"STREAMING")

    def stop_stream(self):
        self.ser.write(b"STOP\n")
        return self._await(b"STOPPED")

    def _await(self, want, deadline=5.0):
        """Scan the RAW byte stream for a reply.

        Raw bytes, not readline(): in binary mode there are no lines to read,
        and even in ASCII mode the reply arrives behind however many sample rows
        were already in flight. Bounded by wall clock rather than a line count,
        because a line count multiplied by the read timeout is a 13-minute hang
        when the board is silent.
        """
        if isinstance(want, str):
            want = want.encode()
        end = time.perf_counter() + deadline
        acc = bytearray()
        while time.perf_counter() < end:
            k = self.ser.in_waiting
            if k:
                acc += self.ser.read(k)
                if want in acc:
                    return want.decode()
                del acc[:-512]
            else:
                time.sleep(0.001)
        raise RuntimeError(
            "no %s seen in %.1fs -- board silent, or host and sketch disagree "
            "on baud (this file: %d)" % (want.decode(), deadline,
                                         self.ser.baudrate))

    def read_sample(self, ncols):
        """One row of `ncols` ints, or None.

        Returns None rather than raising for an ack echo, a partial line or a
        short row -- the caller is in a tight acquisition loop and a dropped row
        is normal. `OK`/`ERR` lines are the acks nobody is reading any more
        (and, since 2026-08-06, the acks the board is no longer even sending).
        """
        if self.binary:
            return self._read_binary(ncols)
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

    # -- binary framing ----------------------------------------------------
    # A5 C3 | seq | a0.lo a0.hi .. a7.lo a7.hi | cksum        -- 20 bytes
    #
    # Both sync bytes have bit 7 set, so the sync word can never occur inside
    # the ASCII replies, which are still ASCII in binary mode. `seq` wraps at
    # 256 and lets us COUNT losses instead of guessing at them. cksum is
    # (seq + sum of the 16 payload bytes) & 0xFF; combined with the structural
    # check that every sample is <= 1023 a false resync is not a practical
    # worry. Fixed length + sync + checksum is what makes this recoverable:
    # lose a byte and the reader rescans and is back in step within one frame.
    def _read_binary(self, ncols):
        k = self.ser.in_waiting
        if k:
            self._buf += self.ser.read(k)
        b = self._buf
        while True:
            i = b.find(bytes((SYNC0, SYNC1)))
            if i < 0:
                # Keep one trailing byte: the sync word may straddle two reads.
                if len(b) > 1:
                    del b[:len(b) - 1]
                return None
            if len(b) - i < FRAME_LEN:
                del b[:i]
                return None
            fr = bytes(b[i:i + FRAME_LEN])
            if (fr[2] + sum(fr[3:19])) & 0xFF != fr[19]:
                self.badframes += 1
                del b[:i + 1]            # false sync: step past and rescan
                continue
            vals = [fr[3 + 2 * j] | (fr[4 + 2 * j] << 8) for j in range(8)]
            if any(v > 1023 for v in vals):
                self.badframes += 1
                del b[:i + 1]
                continue
            if self._seq is not None:
                self.dropped += (fr[2] - self._seq - 1) & 0xFF
            self._seq = fr[2]
            del b[:i + FRAME_LEN]
            return vals[:ncols]

    def drain(self):
        """Throw away whatever is buffered. Use after changing the excitation,
        so the first sample of a new step is not one from the previous step."""
        drain_input(self.ser, 0.5)
        self._buf.clear()
        self._seq = None

    def close(self):
        self.ser.close()
        print("Connection closed. %d DAC writes, %d frames dropped, %d bad."
              % (self.writes, self.dropped, self.badframes))
