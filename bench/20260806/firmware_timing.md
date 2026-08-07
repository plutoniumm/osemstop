# Firmware data-path timing — 2026-08-06

Board on `/dev/cu.usbserial-1120`, Arduino Mega (16 MHz AVR), FTDI bridge, optic
hanging and free. Every number below is **measured on that board**, not derived.
Tooling is next to this file: `wirebench.py` (rates, latency), `presc_sweep.py`
(prescaler sweep), `presc_ab.py` (the 64-vs-32 decision).

Files changed: `arduino.ino`, `pyDAC.py`, `pyDAC2.py`. Nothing else.

---

## 0. The thing that was actually broken: 230400 baud does not exist

Found before any optimisation, while taking the baseline. **The rig was dead and
would have looked like a hardware fault.**

The AVR makes its UART clock by integer division of the 16 MHz crystal, so only
some rates are reachable. Arduino's `HardwareSerial` rounds `UBRR = F_CPU/(8*baud) - 1`:

| requested | UBRR | actual | error |
|---|---|---|---|
| 115200 | 16 | 117647 | +2.12% |
| **230400** | **8** | **222222** | **−3.55%** |
| 250000 | 7 | 250000 | 0.00% |
| 500000 | 3 | 500000 | 0.00% |
| 1000000 | 1 | 1000000 | 0.00% |

230400 had been set in `arduino.ino`, `pyDAC.py` and `pyDAC2.py` earlier the same
day. The board therefore ran at 222222 baud. Measured symptom, host at 230400:

```
boot: b'READY\r\n'                     <- looks perfectly healthy
b'STOP\n'        -> b'ERR unknown command\r\n'
b'STREAM\n'      -> b'ERR unknown command\r\n'
b'SET 0 0.2500\n'-> b'ERR unknown command\r\n'
```

Board→host survives because the FTDI receiver tolerates −3.55%; host→board does
not, because by the stop bit the AVR's sampling point has walked ~34% of a bit
period off centre. Confirmed by moving the *host* to 222222, whereupon every
command worked immediately.

This is the worst shape a bug can have here: `bench.py`'s preflight only checks
for `READY`, so **preflight passes** and the controller then waits forever for a
stream that never starts.

Fixed by moving all three files to **500000**, which is exact on both ends.
1000000 is also exact and works, but buys only ~6% because the loop is no longer
wire-limited. 2000000 was measured and rejected — 29 checksum failures and 29
dropped frames in one 8 s block.

## 0b. `reset_input_buffer()` can wedge this port

Also found while baselining, and also capable of masquerading as a dead board.
With the FTDI driver queue deeply backed up (`in_waiting` pinned at its 1020-byte
report cap), a single `reset_input_buffer()` — i.e. `tcflush` — was followed by
**zero bytes delivered for the remaining 10 s of the run**, while the board
streamed on happily. It does not reproduce on a shallow queue, which is exactly
what makes it dangerous: it fires under load, mid-run.

`pyDAC.drain_input()` / `pyDAC2.drain_input()` now empty the queue by *reading*
it. `FastDAC.drain()` — called by `sysid.py` after every excitation change,
which is precisely the deep-queue case — now uses it.

---

## 1. Sustained sample rate

10 s blocks. "8 coils" = `SET` on all eight channels at 100 Hz = 800 writes/s,
the worst case in `CLAUDE.md` §2c.

| configuration | no coils | 8 coils driven |
|---|---|---|
| **before** (as found, presc 128, 222222 baud) | **424.6 rows/s** | **269.3 rows/s** |
| after, ASCII, ack on — the v3..v10 wire | 1112.2 | 642.2 |
| after, ASCII, ack off — the v11 wire | 1112.6 | 869.6 |
| after, binary, ack on | 1941.6 | 1104.9 |
| after, binary, ack off | **1941.7** | **1530.4** |

Speed-up over the measured baseline: **2.62×** ASCII / **4.57×** binary with no
coils; **3.23×** ASCII / **5.68×** binary with eight coils driven.

Binary streamed 19417 frames in 10 s with **0 bad checksums and 0 dropped
frames**. Under the 800 writes/s load, 3 bad / 10 dropped out of ~15300 (0.065%),
all at block start where the reader resyncs after a mid-frame drain.

### Control loop rate, `DACController` shape

The loop that actually collapsed in the v10 bench run — read a sample, then
eight acked `SET`s, discarding every stream row encountered while hunting each
`OK`:

| | loop rate | samples discarded/s |
|---|---|---|
| before | **23.0 Hz** | 288.6 |
| after | **51.5 Hz** | 773.4 |

**2.24×**, with no change to any controller. The discard count rising is the
point: the ack scan still eats everything in flight, and there is simply more in
flight now. This loop shape is the ceiling; `FastDAC` (v11) avoids it entirely.

### Where the wins come from

- The old firmware was **not** wire-limited as `CLAUDE.md` assumed: at 222222
  baud it emitted 15394 B/s against a 22 kB/s wire. It was ADC- and
  formatting-limited.
- **The UART is full duplex.** Host→board bytes cost the board CPU to parse but
  take *no* bandwidth from the sample stream. The repo's 12 kB/s-vs-11.5 kB/s
  budget conflated the two directions. What genuinely shares the stream's wire is
  the board's `OK ch=N v=D.DDDD` echo — 19 bytes, 800/s, **15.2 kB/s** — for a
  reply `FastDAC` explicitly never reads. `ACK 0` is worth **+35%** on its own
  (642.2 → 869.6 ASCII, 1104.9 → 1530.4 binary).

---

## 2. ADC prescaler — the decision, and the numbers behind it

ADC clock = 16 MHz / prescaler. The datasheet specifies 50–200 kHz for full
10-bit accuracy; 64 is 250 kHz and 32 is 500 kHz, so this had to be measured.

**Method.** Binary framing, so every sample is checksummed and sequence-numbered.
Prescalers visited **round-robin**, not one long block each, so drift in the
optic cannot be mistaken for a prescaler effect. The noise figure is the
second-difference estimator, `sigma = std(x[n] − 2x[n−1] + x[n−2]) / sqrt(6)`,
which is unbiased for white noise and suppresses the pendulum almost completely
(a 1 Hz, 100-count sinusoid contributes ~1e-4 counts at these rates). Plain std
is dominated by real optic motion and is **not** a noise floor.

### Per-sample noise floor, LSB (3 cycles × 8 s; 64/32 from 6 cycles)

| presc | ADC clk | rate | a0 | a1 | a2 | a3 | a4 | a5 | a6 | a7 |
|---|---|---|---|---|---|---|---|---|---|---|
| 128 | 125 kHz | 851 Hz | 2.256 | 0.934 | 0.763 | 0.834 | 1.170 | **1.388** | 1.166 | 1.070 |
| 64 | 250 kHz | 1383 Hz | 1.950 | 0.869 | 0.776 | 0.845 | 1.144 | **1.213** | 1.162 | 1.052 |
| 32 | 500 kHz | 1946 Hz | 1.995 | 0.884 | 0.762 | 0.845 | 1.193 | **1.226** | 1.207 | 1.079 |
| 16 | 1 MHz | 2392 Hz | 1.890 | 0.854 | 0.824 | 0.831 | 1.175 | 1.157 | 1.246 | 1.302 |

### 64 vs 32 head-to-head, 6 interleaved cycles each

| ch | presc 64 | presc 32 | delta | significance |
|---|---|---|---|---|
| a0 | 1.9501 ± 0.0105 | 1.9949 ± 0.0133 | +2.3% | 2.6σ |
| a1 | 0.8691 ± 0.0072 | 0.8840 ± 0.0045 | +1.7% | 1.7σ |
| a2 | 0.7758 ± 0.0093 | 0.7622 ± 0.0081 | −1.7% | 1.1σ |
| a3 | 0.8452 ± 0.0075 | 0.8453 ± 0.0054 | +0.0% | 0.0σ |
| a4 | 1.1440 ± 0.0229 | 1.1933 ± 0.0080 | +4.3% | 2.0σ |
| **a5** | **1.2130 ± 0.0117** | **1.2264 ± 0.0105** | **+1.1%** | **0.9σ** |
| a6 | 1.1618 ± 0.0166 | 1.2074 ± 0.0119 | +3.9% | 2.2σ |
| a7 | 1.0524 ± 0.0130 | 1.0792 ± 0.0136 | +2.5% | 1.4σ |

**a5 — the channel the brief nominated as decisive — is not degraded**: +1.1% at
0.9σ, inside the run-to-run scatter. Other channels are 2–4% worse and that *is*
resolved.

### Why 2–4% worse per sample is still the right trade

Per-sample variance is the wrong figure of merit for a band-limited loop. What
reaches the control law is the noise **density**, `sigma / sqrt(rate)` — white
noise spread over 40% more samples. On that measure every channel improves:

| presc | rate | a0 | a1 | a2 | a3 | a4 | a5 | a6 | a7 |
|---|---|---|---|---|---|---|---|---|---|
| 128 | 851 | 0.0773 | 0.0320 | 0.0262 | 0.0286 | 0.0401 | 0.0476 | 0.0400 | 0.0367 |
| 64 | 1383 | 0.0524 | 0.0234 | 0.0209 | 0.0227 | 0.0308 | 0.0326 | 0.0312 | 0.0283 |
| **32** | **1946** | **0.0452** | **0.0200** | **0.0173** | **0.0192** | **0.0271** | **0.0278** | **0.0274** | **0.0245** |
| 16 | 2392 | 0.0386 | 0.0175 | 0.0168 | 0.0170 | 0.0240 | 0.0237 | 0.0255 | 0.0266 |

LSB/√Hz. From the old default (128) to the shipped setting (32): **−31% to −42%
on every channel, and −41.6% on a5.**

### Why not 16 — rejected on settling, not on noise

Its noise floor looks fine (a5 is the *best* of the four). The failure is a
signal error, visible in two places and only at 16:

- **Quiet channels that follow a loud one in mux order gain std**: a7
  1.26→1.29→1.29→**1.64** (128/64/32/16), a6 1.84→1.92→1.91→**2.35**, a4
  2.66→2.68→2.67→**2.95**. a4 follows a3 (std 40), a6 follows a5 (std 20). That
  is charge from the previous mux step not settling out of the sample-and-hold —
  i.e. crosstalk.
- **DC walk on quiet channels**: a6 mean 852.8 → 852.5 → 851.4 → **844.3**. The
  knee is between 32 and 16, not gradual.
- Plus the only frame losses in the sweep (1 bad, 6 dropped).

At 32 these same diagnostics are flat. **Shipped: `ADC_PRESCALER = 32`**, a named
constant, runtime-overridable with `PRESC 64` if anything later disagrees.

---

## 3. `SET` round-trip latency

Write `SET ch v`, read until `OK`. n=100 each.

| | median | p90 | max |
|---|---|---|---|
| before, idle | 4.434 ms | 4.601 | 5.383 |
| after, idle | **3.238 ms** | 3.284 | 3.336 |
| before, while streaming | 4.382 ms | 5.300 | 6.546 |
| after, while streaming | **2.577 ms** | 2.639 | 2.709 |

−27% idle, −41% streaming, and the spread collapses — max while streaming goes
6.55 → 2.71 ms. The floor is USB/FTDI turnaround, not firmware; what the firmware
removed is the jitter. Two causes:

- `Serial.readStringUntil('\n')` blocked up to its **1 s** timeout once
  `Serial.available()` was true after a single byte, so a command split across
  two USB packets stalled the whole stream for the gap. Replaced with a
  non-blocking accumulator into a fixed `char[40]` — which also removes one heap
  `String` allocation per command on a part with 8 kB of RAM.
- The row write now waits for TX ring space in a loop that **polls for commands
  while it waits**, instead of blocking inside `Serial.write()`.

---

## 4. What else changed

- **One buffered write per row.** Was 16 `Serial.print()` calls per sample, each
  with its own division-based int-to-ASCII, pushing bytes individually. Now
  formatted into a `char[48]` and issued as one `Serial.write(buf, n)`.
- **Integer `setChannel`.** `(voltage / 2.5) * 4095.0` was software float on a
  chip with no FPU, at up to 800 writes/s. Volts are now carried as an integer
  count of 100 µV units — exactly the `%.4f` resolution the host already sends —
  and the code is one 32-bit multiply and one divide. Verified **bit-identical to
  the old float path across all 25001 possible 0.1 mV steps, zero mismatches**,
  so `SET <ch> <volts>` is unchanged for every existing caller. `SETC <ch> <code>`
  takes a raw DAC code for callers that want no conversion at all.
- Sketch size fell 8544 → 5922 bytes of flash.

## 5. Binary framing

Opt-in, `MODE BIN`. Fixed 20-byte frame:

```
A5 C3 | seq | a0.lo a0.hi ... a7.lo a7.hi | cksum
 2       1              16                    1
```

- Both sync bytes have **bit 7 set**, so the sync word can never occur inside the
  ASCII replies — which stay ASCII in binary mode. A reader splits the two
  streams on bit 7, so `READY` / `STREAMING` / `STOPPED` / `OK` / `ERR` all keep
  working and `FastDAC`'s first-character filter is still correct.
- `seq` wraps at 256, so losses are **counted**, not suspected. This link has
  dropped the board off USB three times in this project.
- `cksum = (seq + sum of the 16 payload bytes) & 0xFF`, plus the structural check
  that every sample is ≤ 1023 (the top 6 bits of each u16 are always zero).
- Fixed length + sync + checksum is what makes it recoverable: lose a byte, the
  reader rescans for `A5 C3` and is back in step within one frame.

**ASCII remains the default.** `osem.v3/v7/v9/v10` decode with `.decode("utf-8")`
and no `errors=` argument, so binary bytes would raise `UnicodeDecodeError`,
which is not caught by their `except (ValueError, IndexError)`. A host must ask
for binary.

## 6. New commands

`PRESC <n>`, `ACK <0|1>`, `MODE ASCII|BIN`, `SETC <ch> <code>`, `BAUD <n>`,
`INFO`. `BAUD` exists so the ladder can be re-measured without eight reflashes;
a wrong guess self-heals because the board resets when the host opens the port.
`INFO` is printed once at boot immediately **before** `READY` — never after,
because `pyDAC`/`pyDAC2` stop scanning at `READY` and `dcmatrix.py` only scans 30
lines.

## 7. Known-stale, not touched

`bench/20260804/dcmatrix.py:28` and `bench/20260804/probe8.py:17` open the port
at a hard-coded 115200. They were already broken by the morning's baud change and
are dated one-off scripts; they need `500000` if they are ever re-run.
