"""
pyDAC2 -- the fire-and-forget serial transport for this rig.

`set_voltage` writes and returns: no readline, no ack. Its acked predecessor read
up to 50 lines waiting for `OK`, and every one of those lines was a stream sample
thrown away. Measured 2026-08-06, eight coils at 100 Hz, same board and firmware:

    stream alone, no coils driven .................. 904 rows/s
    eight coils, fire-and-forget (this class) ...... 862 rows/s
    eight coils, one acked SET each ................  23 Hz loop,
                                                     289 samples/s discarded

For a damping controller that is bad; for system identification it is fatal --
every method here correlates a known excitation against the response, and
throwing away 37 samples of 38 destroys both the SNR and the phase.

WHAT YOU GIVE UP is the `OK`/`ERR` echo, and since 2026-08-06 this class tells
the FIRMWARE to stop sending it (`ACK 0`), which is worth more than not reading
it. That is safe because `ERR` was only ever returned for a malformed command or
a channel out of range -- both rejected locally before the write -- and the
firmware clamps voltage to 0..2.5 V itself. A malformed command still gets `ERR`.

BANDWIDTH -- CORRECTED 2026-08-20. `SET` does NOT take bandwidth from the stream:
the UART is FULL DUPLEX, so host->board bytes cost the board CPU time to parse
and nothing else. That is why turning the echo off is the change that matters and
compressing the command is not. What binds is the DOWNSTREAM stream. At 115200
8N1 the wire carries 11.52 kB/s each way, and an ASCII row measured off it,
`563,632,914,668,670,534,588,586\r\n`, is exactly 32 bytes -- so 352 Hz is
11.3 kB/s, about 98 % of the link on its own, before a single coil writes. The
20-byte binary frame below is 7.0 kB/s at the same rate, 61 %.

Earlier revisions of this header costed the echo against "a 50 kB/s wire"; that
was the 500000-baud board and is wrong here. The conclusion is unchanged.

Achieved rates, 8 channels at 115200: ASCII 418-435 Hz (2026-08-17), 350-352 Hz
undriven and 226.5 Hz the instant four coils start writing (2026-08-20,
data/20260820_171242_fast_lock.csv); binary 581 Hz held through damping
(data/20260820_180344). See `bench/20260806/firmware_timing.md`.
"""

import time

import serial

BAUD = 115200                            # MEASURED on the attached board, not chosen.
# 2026-08-17, official Mega 2560 R3 at /dev/cu.usbmodem11101 (VID:PID 2341:0042):
# 115200 returns a clean b'READY\r\n'; 500000 returns framing garbage
# (b'\x80\x80xx\x00x\x00x\x00x\x80xx\x00\x80x'). This default was 500000 until
# 2026-08-20, and arduino.ino's BAUD_HZ moved with it. That cost most of an
# afternoon: at the wrong rate both transports scan 200 lines at a 2 s timeout --
# up to 400 s of total silence with nothing printed, indistinguishable from a hung
# program. `probe_baud` is the authority; this is only what is tried first.
# NOTE: 230400 is a TRAP on a 16 MHz AVR -- -3.55% error, outside the receiver's
# tolerance. arduino.ino has the divisor table and the 2026-08-06 bench evidence.

# MOST LIKELY FIRST, because the order is what bounds the worst-case silence.
# 500000 stays on the list because a DIFFERENT board ran it: 1024-1113 Hz measured
# 2026-08-06 (versions.md) on the CH340/FTDI-bridge board at
# /dev/cu.usbserial-1120. THE RATE IS NOT A PROPERTY OF THIS REPO -- arduino.ino
# boots at BAUD_HZ but also carries a `BAUD` command, so the live rate is a
# property of what was last done to that board.
#
# This list and `probe_baud` live HERE, in the transport that owns BAUD, and not
# in each tool. bench.py and harness.py once carried a version-discovery regex
# each, the copies drifted, and `make run` silently could not see osem.v5.5.py --
# `ladder.py` exists because of it. One owner.
BAUD_CANDIDATES = (115200, 500000, 230400, 57600, 9600)


def probe_baud(port, candidates=BAUD_CANDIDATES, seconds=2.0, reset_s=2.2,
               log=print):
    """The baud this board is actually talking at, or None.

    WHY THIS IS NOT OPTIONAL. `FastDAC.__init__` scans up to 200 lines for READY
    at a 2 s serial timeout, so at the wrong rate it blocks for up to 400 SECONDS
    printing nothing -- indistinguishable from a hung program, and that cost most
    of a bench session on 2026-08-17. Bounded, and it says what it heard.
    """
    for baud in candidates:
        try:
            ser = serial.Serial(port, baud, timeout=0.3)
        except Exception as e:
            log("    %7d  cannot open: %s" % (baud, e))
            continue
        try:
            time.sleep(reset_s)          # opening the port toggles DTR: board resets
            t0, blob = time.time(), bytearray()
            while time.time() - t0 < seconds:
                blob += ser.read(4096)
        finally:
            ser.close()
        if b"READY" in blob:
            log("    %7d  READY  <-- using this" % baud)
            return baud
        log("    %7d  no READY in %d bytes  %s"
            % (baud, len(blob), repr(bytes(blob[:24])) if blob else "(silence)"))
    return None


def resolve_baud(port, log=print):
    """Probe, then make BAUD match the board so every later open just works.

    Sets the module global, which is what `FastDAC(port=...)` and every
    controller in the ladder default to. Returns the rate. Exits with the
    reflash instruction if the board says nothing at any rate.
    """
    global BAUD
    log("  baud:")
    found = probe_baud(port, log=log)
    if found is None:
        raise SystemExit(
            "  The board never said READY at any of %s.\n"
            "  It is powered and enumerating, so this is the sketch: not\n"
            "  flashed, not arduino.ino, or at a rate not in the list.\n"
            "  Try `make arduino`."
            % ", ".join("%d" % b for b in BAUD_CANDIDATES))
    if found != BAUD:
        log("  NOTE: the sketch is at %d, not the %d this tree declares -- using"
            % (found, BAUD))
        log("  %d. `make arduino` reflashes it to %d." % (found, BAUD))
    BAUD = found
    return found

NCH = 8                                  # arduino.ino's NCH; the board sends all
SYNC0, SYNC1 = 0xA5, 0xC3                # both have bit 7 set: never in ASCII
FRAME_LEN = 3 + 2 * NCH + 1              # sync sync seq + NCH*uint16 LE + cksum


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
    """Transport for the identification runs and for the eight-channel controller.

    The ack is not merely an inefficiency, it is a stability problem: measured
    2026-08-06 the loop rate falls with the number of coils driven -- 73 Hz on
    one, 23.5 Hz on four, 12.5 Hz on eight (`analysis/out/loop_rate.csv`) -- and
    at 12.5 Hz the phase margin at 3.75 Hz is 45.5 deg against 73.5 deg at four
    coils (`analysis/out/phase_budget.csv`)."""

    VMIN_HW, VMAX_HW = 0.0, 2.5          # what the firmware itself clamps to

    def __init__(self, port, baud=None, timeout=2.0, binary=False, ack=False):
        # baud=None, NOT baud=BAUD: a default argument is evaluated at DEFINITION,
        # so `baud=BAUD` would freeze the import-time value and `resolve_baud`
        # setting the module global afterwards would silently do nothing.
        baud = BAUD if baud is None else baud
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)                    # the board resets when the port opens
        # SCAN for READY rather than demanding it on line 1. Opening the port
        # toggles DTR and resets the board, but whatever the board sent BEFORE
        # that reset is still in the OS buffer and arrives first, so a board left
        # streaming makes a healthy rig look dead. READY is emitted after the
        # reset, i.e. always at the END of that backlog.
        for _ in range(200):
            if self.ser.readline().decode(errors="replace").strip() == "READY":
                break
        else:
            raise RuntimeError(
                "Arduino never sent READY on %s at %d baud (scanned 200 lines). "
                "Wrong port, board not flashed, or the sketch is at a different "
                "baud -- reflash with `make arduino`. Note that 230400 does NOT "
                "work on a 16 MHz AVR; see arduino.ino's divisor table." % (port, baud))
        # Land in a known state.
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
    def _cmd(self, channel, voltage):
        """One SET line, validated locally: with the ack off nothing checks for us."""
        c = int(channel)
        if not 0 <= c < NCH:
            raise ValueError("Channel must be 0-%d" % (NCH - 1))
        v = float(voltage)
        if not (self.VMIN_HW <= v <= self.VMAX_HW):
            raise ValueError("Voltage must be %.1f-%.1f V, got %.4f"
                             % (self.VMIN_HW, self.VMAX_HW, v))
        return b"SET %d %.4f\n" % (c, v)

    def set_voltage(self, channel, voltage):
        self.ser.write(self._cmd(channel, voltage))
        self.writes += 1

    def set_many(self, channels, volts):
        """All channels in ONE write() call. Same bytes on the wire, but one
        syscall and no chance of the stream interleaving mid-command."""
        buf = bytearray()
        for c, v in zip(channels, volts):
            buf += self._cmd(c, v)
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

        None rather than an exception for an ack echo, a partial line or a short
        row: the caller is in a tight acquisition loop and a dropped row is normal.
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
    # A5 C3 | seq | a0.lo a0.hi .. a7.lo a7.hi | cksum, matching arduino.ino.
    # Both sync bytes have bit 7 set, so the sync word can never occur inside the
    # replies, which stay ASCII in binary mode. `seq` wraps at 256 and lets us
    # COUNT losses rather than guess at them. cksum is (seq + sum of the payload
    # bytes) & 0xFF; with the structural check that every sample is <= 1023 a
    # false resync is not a practical worry, and fixed length + sync + checksum
    # is what makes it recoverable inside one frame.
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
            if (fr[2] + sum(fr[3:FRAME_LEN - 1])) & 0xFF != fr[FRAME_LEN - 1]:
                self.badframes += 1
                del b[:i + 1]            # false sync: step past and rescan
                continue
            vals = [fr[3 + 2 * j] | (fr[4 + 2 * j] << 8) for j in range(NCH)]
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
