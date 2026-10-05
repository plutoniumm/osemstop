// arduino.ino -- OSEM eight-channel ADC stream + eight-channel SPI DAC.
// ============================================================================
// The board reads A0..A7 and streams them to the host; the host writes coil
// voltages back with SET. One UART carries both conversations.
//
// THE UART IS FULL DUPLEX. Host->board bytes cost the board CPU time to parse
// but they do NOT take bandwidth away from the sample stream -- TX and RX are
// independent shift registers. What DOES share the stream's wire is every byte
// the board SENDS, and until 2026-08-06 that included an `OK ch=N v=D.DDDD`
// echo for every SET. Eight coils at 100 Hz is 800 echoes/s of 19 bytes =
// 15.2 kB/s, which is most of the wire, spent on a reply that pyDAC2.FastDAC
// throws away unread. `ACK 0` turns those echoes off. See handleCommand().
//
// ---------------------------------------------------------------------------
// BAUD -- read this before changing it
// ---------------------------------------------------------------------------
// The AVR derives the UART clock by integer division of F_CPU, so only some
// rates exist. With U2X the divisor is UBRR = F_CPU/(8*baud) - 1, and Arduino's
// HardwareSerial rounds it. At 16 MHz:
//
//      want      UBRR   actual      error
//      115200     16    117647     +2.12%
//      230400      8    222222     -3.55%     <-- DOES NOT WORK
//      250000      7    250000     +0.00%
//      500000      3    500000     +0.00%
//      1000000     1   1000000     +0.00%
//
// 230400 was set here on 2026-08-06 and is a trap. The board actually ran at
// 222222 baud, and a -3.55% mismatch is outside the AVR receiver's tolerance:
// by the stop bit the sampling point has walked ~34% of a bit period off
// centre. Measured on the bench the same day -- a host at 230400 got a clean
// `READY` (the FTDI receiver is tolerant enough to decode the board's TX) and
// then EVERY command came back `ERR unknown command`. That failure is nasty
// because bench.py's preflight only checks for READY, so it passes, and the
// controller then sits waiting for a stream that never starts.
//
// THE DIVISOR TABLE IS NOT THE WHOLE STORY, AND THE BENCH OVERRULED IT.
// The rule used to read "use only rates where the error is 0.00%", and BAUD_HZ
// was 500000 on that argument until 2026-08-20. Measured 2026-08-17 on the
// official Mega 2560 R3 at /dev/cu.usbmodem11101 (VID:PID 2341:0042):
//
//      115200   ->  clean `READY\r\n`          (+2.12% by the table above)
//      500000   ->  framing garbage            (0.00% by the table above)
//                   b'\x80\x80xx\x00x\x00x\x00x\x80xx\x00\x80x'
//
// +2.12% is inside the receiver's tolerance -- by the stop bit the sampling
// point has walked ~21% of a bit period, against the ~34% that made 230400
// fail -- so 115200 working is consistent with the table. 500000 failing is
// NOT, and the cause is not established here: it is upstream of the USART,
// in the ATmega16U2 bridge or the host driver, neither of which is in this
// repo. A DIFFERENT board did run it -- 1024-1113 Hz at 500000 on 2026-08-06,
// on the CH340/FTDI-bridge board at /dev/cu.usbserial-1120 (versions.md).
//
// Cost of leaving BAUD_HZ at a rate this board cannot use: `make arduino` would
// flash a sketch nobody can then talk to at the probed rate. Cost of 115200:
// 418-435 Hz with 8 ASCII channels against 1024-1113 at 500000 -- still 4.2x
// the 100 Hz control clock and inside the range `decimate` was validated at
// (347 Hz), so the throughput is not binding.
//
// The `BAUD` command exists so the ladder can be re-measured without a reflash
// (the board resets when the host opens the port, so a wrong guess self-heals).
//
// THE HOST MUST MATCH -- pyDAC2.py carries the same default and probes anyway.
// Reflash with `make arduino`.
//
// ---------------------------------------------------------------------------
// FRAMING -- ASCII (default) or binary, switched at runtime with MODE
// ---------------------------------------------------------------------------
// ASCII, the default, is byte-for-byte what every osem.vN.py already parses:
//
//      648,672,633,675,559,591,851,724\r\n          up to 41 bytes
//
// Binary is a fixed 20-byte frame, for hosts that ask for it with `MODE BIN`:
//
//      A5 C3 | seq | a0.lo a0.hi ... a7.lo a7.hi | cksum
//        2      1              16                    1
//
//   * the sync word is 0xA5 0xC3 -- BOTH bytes have bit 7 set, so it can never
//     occur inside the ASCII replies (READY/OK/ERR/STREAMING/STOPPED), which
//     are still ASCII in binary mode. A reader splits the two streams on bit 7.
//   * `seq` increments per frame and wraps, so the host can COUNT dropped
//     frames rather than merely suspect them. This link has dropped the board
//     off USB three times in this project.
//   * cksum = (seq + sum of the 16 payload bytes) & 0xFF. With the structural
//     check that every sample is <= 1023 (the top 6 bits of each u16 are always
//     zero) a false resync is not a practical concern.
//   * fixed length + sync + checksum is what makes it RESYNCHRONISABLE: lose a
//     byte and the reader rescans for A5 C3 and is back in step within a frame.
//
// ASCII stays the default deliberately. osem.v3/v7/v9/v10 decode with
// `.decode("utf-8")` and NO errors= argument, so binary bytes raise
// UnicodeDecodeError, which is not in their `except (ValueError, IndexError)`.
// A host that wants binary must ask for it.
// ============================================================================

#include <SPI.h>

#define CS 53
#define NCH 8

// ---- boot defaults ---------------------------------------------------------
static const uint32_t BAUD_HZ = 115200UL;   // +2.12% at 16 MHz, and MEASURED to work;
                                           // 500000 is 0.00% and measured NOT to. See above.

// ADC clock = F_CPU / prescaler. The datasheet specifies 50-200 kHz for full
// 10-bit accuracy, and Arduino's default of 128 (125 kHz) sits inside that.
// Going faster trades effective bits for time, so this is set from a MEASURED
// per-channel noise floor, not from the datasheet -- bench/20260806/
// firmware_timing.md has the run. Summary, six interleaved 8 s blocks each:
//
//   presc  ADC clk   rate     a5 noise floor      a5 noise DENSITY
//     64   250 kHz  1383 Hz   1.2130 +/- 0.0117   0.0326 LSB/sqrt(Hz)
//     32   500 kHz  1946 Hz   1.2264 +/- 0.0105   0.0278 LSB/sqrt(Hz)
//
// a5 is the channel that decides it -- a working sensor at ~1/12 of a0's
// counts-per-metre, whose signal at the 1.046 Hz mode is only ~13 counts. Its
// per-sample noise is 1.1% worse at 32, which is 0.9 sigma, i.e. not resolved.
// Other channels are 2-4% worse and that IS resolved (a0 2.6 sigma) -- but
// per-sample variance is the wrong figure of merit for a band-limited loop.
// White noise divided over 40% more samples is 15% LESS noise density, and
// every channel improves on that measure. See the doc.
//
// 16 was measured and REJECTED, on settling rather than noise: the quiet
// channels that follow a loud one in the mux order gain 25-30% of std (a7
// 1.29 -> 1.64, a6 1.91 -> 2.35) and a6's mean walks 7 counts. That is the
// sample-and-hold not settling between mux steps, and it is a signal error,
// not a noise error. At 32 the same channels are flat.
//
// Runtime-settable with `PRESC`, so re-measuring costs no reflashes and
// backing out to 64 costs one command.
static const uint8_t ADC_PRESCALER = 32;    // 500 kHz, ~26 us/read, ~210 us/8

static const uint8_t SYNC0 = 0xA5, SYNC1 = 0xC3;
static const uint8_t FRAME_LEN = 3 + 2 * NCH + 1;      // 20

// ---- firmware 2 (2026-10-05): everything below is ADDED; ASCII rows, MODE BIN
// and the ASCII `SET` are untouched, so every 1.0 tool still works. ------------
//
// OVERSAMPLING, `OS n`. The ADC does ~3450 eight-channel scans a second (measured
// 2026-10-05) and the link carries a few hundred frames, so each frame can be the
// SUM of n scans.
// That is where the extra bits are: the reference is AVcc and the sensors sit at
// 2.3-4.9 V, so no on-chip reference (1.1 / 2.56 V) can rescale them, but noise
// of several counts dithers the 10-bit steps and a sum of 16 carries ~2 more
// bits in the same two bytes. It also averages interference above the frame
// rate BEFORE it can alias.
//
// MODE BIN2 sample frame, 23 bytes:
//   A5 C3 | seq | 8 x uint16 LE SUM of `os` scans | os | rxok | rxbad | cksum
// rxok / rxbad count binary command frames accepted / rejected (mod 256), so the
// host can SEE whether its writes land. cksum = sum of bytes 2..21, low 8 bits.
//
// Binary COMMAND frame, host -> board, 20 bytes, accepted in every mode:
//   A5 3C | 8 x uint16 LE, DAC channel 0..7, units of 100 uV (0xFFFF = leave)
//         | sum of the 16 payload bytes | xor of them
// One frame sets all eight coils. Eight ASCII `SET` lines are 104 bytes with NO
// check, and a line torn by a receive overrun is still parsed and applied --
// possibly as another channel's voltage. A frame that fails either check is
// dropped whole and counted. Neither sync byte can occur in an ASCII command.
static const uint8_t CMD0 = 0xA5, CMD1 = 0x3C;
static const uint8_t CMD_LEN    = 2 + 2 * NCH + 2;     // 20
static const uint8_t FRAME2_LEN = 3 + 2 * NCH + 3 + 1; // 23
static bool     bin2Mode = false;
static uint8_t  osN = 1, osCount = 0;
static uint16_t osAcc[NCH];
static uint8_t  rxOk = 0, rxBad = 0;
static uint16_t lastUnits[NCH];      // what each DAC channel last got; 0xFFFF = unknown

static bool    streaming = false;
static bool    binMode   = false;
static bool    ackOn     = true;      // DACController needs it; FastDAC clears it
static uint8_t seq       = 0;
static uint8_t prescNow  = ADC_PRESCALER;

// ---------------------------------------------------------------------------
// ADC
// ---------------------------------------------------------------------------
static void adcSetPrescaler(uint8_t p) {
  uint8_t bits;
  switch (p) {
    case 2:   bits = 1; break;
    case 4:   bits = 2; break;
    case 8:   bits = 3; break;
    case 16:  bits = 4; break;
    case 32:  bits = 5; break;
    case 64:  bits = 6; break;
    default:  bits = 7; p = 128; break;
  }
  ADCSRA = (ADCSRA & (uint8_t)~0x07) | bits;
  prescNow = p;
}

// ---------------------------------------------------------------------------
// DAC -- integer only. This runs at up to 800 writes/s with eight coils on a
// chip with no FPU, where the old `(voltage / 2.5) * 4095.0` cost a software
// float parse, divide and multiply every time. Volts are carried as an integer
// number of 100 uV units (0..25000), which is exactly the %.4f resolution the
// host already sends, and the DAC code is one 32-bit multiply and one divide:
// 25000 * 4095 = 102,375,000, comfortably inside uint32.
// ---------------------------------------------------------------------------
static void writeCode(uint8_t channel, uint16_t code) {
  if (code > 4095)  code = 4095;
  if (channel > 7)  channel = 7;

  byte b2 = (byte)(((channel & 0x07) << 4) | ((code >> 8) & 0x0F));
  byte b3 = (byte)(code & 0xFF);

  digitalWrite(CS, LOW);
  SPI.transfer(0x03);                 // command: write & update
  SPI.transfer(b2);                   // channel[2:0] + code[11:8]
  SPI.transfer(b3);                   // code[7:0]
  SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
}

static uint16_t unitsToCode(uint16_t units) {          // units of 100 uV
  if (units > 25000) units = 25000;
  return (uint16_t)(((uint32_t)units * 4095UL + 12500UL) / 25000UL);
}

static void setChannelUnits(uint8_t channel, uint16_t units) {
  if (channel < NCH) lastUnits[channel] = units;
  writeCode(channel, unitsToCode(units));
}

// ---------------------------------------------------------------------------
// small integer formatting -- no printf, no float
// ---------------------------------------------------------------------------
static uint8_t putU16(char *p, uint16_t v) {
  if (v >= 10000) { *p++ = '0' + v / 10000; v %= 10000;
                    *p++ = '0' + v /  1000; v %=  1000;
                    *p++ = '0' + v /   100; v %=   100;
                    *p++ = '0' + v /    10; *p = '0' + v % 10; return 5; }
  if (v >=  1000) { *p++ = '0' + v /  1000; v %=  1000;
                    *p++ = '0' + v /   100; v %=   100;
                    *p++ = '0' + v /    10; *p = '0' + v % 10; return 4; }
  if (v >=   100) { *p++ = '0' + v /   100; v %=   100;
                    *p++ = '0' + v /    10; *p = '0' + v % 10; return 3; }
  if (v >=    10) { *p++ = '0' + v /    10; *p = '0' + v % 10; return 2; }
  *p = '0' + v; return 1;
}

// "0.2500" from 2500, four fixed decimals -- the exact shape of the old
// Serial.print(vol, 4) so the OK line is byte-identical to what pyDAC parses.
static uint8_t putVolts(char *p, uint16_t units) {
  char *q = p;
  q += putU16(q, units / 10000);
  *q++ = '.';
  uint16_t f = units % 10000;
  *q++ = '0' + f / 1000; f %= 1000;
  *q++ = '0' + f /  100; f %=  100;
  *q++ = '0' + f /   10;
  *q++ = '0' + f %   10;
  return (uint8_t)(q - p);
}

// ---------------------------------------------------------------------------
// command parsing -- non-blocking accumulator, no String
// ---------------------------------------------------------------------------
// The old code was `if (Serial.available()) Serial.readStringUntil('\n')`.
// available() is true after ONE byte, and readStringUntil then BLOCKS for up to
// its 1000 ms timeout waiting for the newline -- so a command split across two
// USB packets stalled the entire stream for the gap between them. It also built
// a heap-allocated String per command, on a part with 8 kB of RAM and no
// compaction. This version consumes whatever is available, keeps the partial
// line in a fixed buffer, and returns.
static char    cmdbuf[40];
static uint8_t cmdlen  = 0;
static bool    cmdOver = false;       // line too long: drop it, resync on \n

static void handleCommand(char *s);

static uint8_t binbuf[CMD_LEN];
static uint8_t binlen = 0;            // > 0 while inside a binary command frame

static void binaryCommand() {
  uint8_t sum = 0, x = 0;
  for (uint8_t i = 2; i < 2 + 2 * NCH; i++) { sum += binbuf[i]; x ^= binbuf[i]; }
  if (sum != binbuf[CMD_LEN - 2] || x != binbuf[CMD_LEN - 1]) { rxBad++; return; }
  for (uint8_t c = 0; c < NCH; c++) {
    uint16_t u = (uint16_t)binbuf[2 + 2 * c] | ((uint16_t)binbuf[3 + 2 * c] << 8);
    if (u == 0xFFFF || u == lastUnits[c]) continue;   // unchanged: do not strobe
    if (u > 25000) u = 25000;
    setChannelUnits(c, u);
  }
  rxOk++;
}

static void pollSerial() {
  uint8_t budget = 96;                // bounded, so streaming cannot be starved
  while (budget-- && Serial.available()) {
    char c = (char)Serial.read();
    if (binlen) {                     // inside a binary frame: every byte is data
      if (binlen == 1 && (uint8_t)c != CMD1) { binlen = 0; rxBad++; continue; }
      binbuf[binlen++] = (uint8_t)c;
      if (binlen == CMD_LEN) { binlen = 0; binaryCommand(); }
      continue;
    }
    if ((uint8_t)c == CMD0) { binbuf[0] = CMD0; binlen = 1; cmdlen = 0; cmdOver = false; continue; }
    if (c == '\n' || c == '\r') {
      if (cmdlen && !cmdOver) { cmdbuf[cmdlen] = '\0'; handleCommand(cmdbuf); }
      cmdlen = 0;
      cmdOver = false;
    } else if (cmdlen < sizeof(cmdbuf) - 1) {
      cmdbuf[cmdlen++] = c;
    } else {
      cmdOver = true;                 // swallow the rest of an overlong line
    }
  }
}

static bool eat(char **p, const char *lit) {
  char *s = *p;
  while (*lit) { if (*s != *lit) return false; s++; lit++; }
  *p = s;
  return true;
}

static uint32_t eatUInt(char **p) {
  char *s = *p;
  uint32_t v = 0;
  while (*s == ' ') s++;
  while (*s >= '0' && *s <= '9') { v = v * 10 + (uint32_t)(*s++ - '0'); }
  *p = s;
  return v;
}

// "0.2500" / ".25" / "2" -> integer 100 uV units, clamped to 0..25000.
static uint16_t eatUnits(char **p) {
  char *s = *p;
  while (*s == ' ') s++;
  if (*s == '-') { *p = s + 1; return 0; }          // firmware clamps at 0
  if (*s == '+') s++;
  uint32_t whole = 0;
  while (*s >= '0' && *s <= '9') { whole = whole * 10 + (uint32_t)(*s++ - '0');
                                   if (whole > 9) whole = 9; }
  uint16_t frac = 0, nd = 0;
  if (*s == '.') {
    s++;
    while (*s >= '0' && *s <= '9') { if (nd < 4) { frac = frac * 10 + (uint16_t)(*s - '0'); nd++; } s++; }
    while (nd < 4) { frac *= 10; nd++; }
  }
  *p = s;
  uint32_t units = whole * 10000UL + frac;
  return (uint16_t)(units > 25000UL ? 25000UL : units);
}

static void okLine(const char *tag, uint32_t v) {
  char b[32];
  uint8_t n = 0;
  b[n++] = 'O'; b[n++] = 'K'; b[n++] = ' ';
  while (*tag) b[n++] = *tag++;
  n += putU16(b + n, (uint16_t)v);
  b[n++] = '\r'; b[n++] = '\n';
  Serial.write((const uint8_t *)b, n);
}

static void handleCommand(char *s) {
  while (*s == ' ') s++;

  if (eat(&s, "SET ")) {                       // SET <ch> <volts>
    uint8_t ch = (uint8_t)eatUInt(&s);
    if (*s != ' ') { Serial.print(F("ERR bad format\r\n")); return; }
    uint16_t units = eatUnits(&s);
    setChannelUnits(ch, units);
    if (ackOn) {
      char b[32];
      uint8_t n = 0;
      b[n++]='O'; b[n++]='K'; b[n++]=' '; b[n++]='c'; b[n++]='h'; b[n++]='=';
      n += putU16(b + n, ch);
      b[n++]=' '; b[n++]='v'; b[n++]='=';
      n += putVolts(b + n, units);
      b[n++]='\r'; b[n++]='\n';
      Serial.write((const uint8_t *)b, n);
    }
    return;
  }

  if (eat(&s, "SETC ")) {                      // SETC <ch> <code 0..4095>
    uint8_t  ch   = (uint8_t)eatUInt(&s);
    uint16_t code = (uint16_t)eatUInt(&s);
    writeCode(ch, code);
    if (ackOn) okLine("code=", code);
    return;
  }

  if (eat(&s, "STREAM"))  { streaming = true;  seq = 0; Serial.print(F("STREAMING\r\n")); return; }
  if (eat(&s, "STOP"))    { streaming = false;          Serial.print(F("STOPPED\r\n"));   return; }

  if (eat(&s, "PRESC ")) { adcSetPrescaler((uint8_t)eatUInt(&s)); okLine("presc=", prescNow); return; }
  if (eat(&s, "ACK "))   { ackOn = (eatUInt(&s) != 0);            okLine("ack=", ackOn ? 1 : 0); return; }

  if (eat(&s, "OS ")) {                        // scans summed per frame, 1..32
    uint32_t n = eatUInt(&s);
    osN = (uint8_t)(n < 1 ? 1 : n > 32 ? 32 : n);
    osCount = 0;
    for (uint8_t i = 0; i < NCH; i++) osAcc[i] = 0;
    okLine("os=", osN);
    return;
  }

  if (eat(&s, "MODE ")) {
    if (eat(&s, "BIN2"))       { bin2Mode = true; binMode = true; Serial.print(F("OK mode=bin2\r\n")); }
    else if (eat(&s, "BIN"))   { bin2Mode = false; binMode = true;  Serial.print(F("OK mode=bin\r\n")); }
    else if (eat(&s, "ASCII")) { bin2Mode = false; binMode = false; Serial.print(F("OK mode=ascii\r\n")); }
    else                       { Serial.print(F("ERR bad mode\r\n")); }
    return;
  }

  if (eat(&s, "BAUD ")) {                      // re-measure the ladder, no reflash
    uint32_t b = eatUInt(&s);
    if (b < 9600UL || b > 2000000UL) { Serial.print(F("ERR bad baud\r\n")); return; }
    Serial.print(F("OK baud="));
    Serial.print(b);
    Serial.print(F("\r\n"));
    Serial.flush();                            // finish the reply at the OLD rate
    Serial.begin(b);
    return;
  }

  if (eat(&s, "INFO")) {
    Serial.print(F("INFO nch=8 mode="));
    Serial.print(binMode ? F("bin") : F("ascii"));
    Serial.print(F(" presc="));  Serial.print(prescNow);
    Serial.print(F(" ack="));    Serial.print(ackOn ? 1 : 0);
    Serial.print(F(" frame="));  Serial.print(FRAME_LEN);
    Serial.print(F(" fw=2 os=")); Serial.print(osN);
    Serial.print(F(" frame2=")); Serial.print(FRAME2_LEN);
    Serial.print(F("\r\n"));
    return;
  }

  Serial.print(F("ERR unknown command\r\n"));
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_HZ);
  adcSetPrescaler(ADC_PRESCALER);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Enable internal reference (same as the original working code)
  digitalWrite(CS, LOW);
  SPI.transfer(0x08);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x01);
  digitalWrite(CS, HIGH);

  for (uint8_t c = 0; c < 8; c++) { writeCode(c, 0); lastUnits[c] = 0; }   // zero every coil first

  delay(10);
  // INFO before READY, never after: pyDAC and pyDAC2 scan for READY and stop
  // there, and dcmatrix.py scans only 30 lines. READY must stay the last line
  // of boot.
  handleCommand((char *)"INFO");
  Serial.print(F("READY\r\n"));
}

void loop() {
  pollSerial();                       // always listen, so STOP works mid-stream

  if (!streaming) return;

  uint16_t v[NCH];
  v[0] = analogRead(A0);  v[1] = analogRead(A1);
  v[2] = analogRead(A2);  v[3] = analogRead(A3);
  v[4] = analogRead(A4);  v[5] = analogRead(A5);
  v[6] = analogRead(A6);  v[7] = analogRead(A7);

  pollSerial();                       // and again, after the ~0.45 ms of ADC

  // Sum `osN` scans into one frame. With osN = 1 this is a no-op and the frame
  // below is byte-identical to firmware 1's.
  for (uint8_t i = 0; i < NCH; i++) osAcc[i] += v[i];
  if (++osCount < osN) return;
  osCount = 0;

  uint8_t buf[48];
  uint8_t n;

  if (bin2Mode) {
    buf[0] = SYNC0; buf[1] = SYNC1; buf[2] = seq;
    for (uint8_t i = 0; i < NCH; i++) {
      buf[3 + 2 * i] = (uint8_t)(osAcc[i] & 0xFF);
      buf[4 + 2 * i] = (uint8_t)(osAcc[i] >> 8);
    }
    buf[19] = osN; buf[20] = rxOk; buf[21] = rxBad;
    uint8_t ck = 0;
    for (uint8_t i = 2; i < FRAME2_LEN - 1; i++) ck += buf[i];
    buf[FRAME2_LEN - 1] = ck;
    n = FRAME2_LEN;
    seq++;
    for (uint8_t i = 0; i < NCH; i++) osAcc[i] = 0;
    while ((uint8_t)Serial.availableForWrite() < n) pollSerial();
    Serial.write(buf, n);
    return;
  }

  // Legacy modes carry the rounded MEAN, so the range stays 0..1023.
  for (uint8_t i = 0; i < NCH; i++) { v[i] = (osAcc[i] + osN / 2) / osN; osAcc[i] = 0; }

  if (binMode) {
    buf[0] = SYNC0; buf[1] = SYNC1; buf[2] = seq;
    uint8_t ck = seq;
    for (uint8_t i = 0; i < NCH; i++) {
      uint8_t lo = (uint8_t)(v[i] & 0xFF), hi = (uint8_t)(v[i] >> 8);
      buf[3 + 2 * i] = lo;  buf[4 + 2 * i] = hi;
      ck += lo;             ck += hi;
    }
    buf[FRAME_LEN - 1] = ck;
    n = FRAME_LEN;
    seq++;
  } else {
    n = 0;
    for (uint8_t i = 0; i < NCH; i++) {
      n += putU16((char *)buf + n, v[i]);
      buf[n++] = (i == NCH - 1) ? '\r' : ',';
    }
    buf[n++] = '\n';
  }

  // ONE write per row. The old code made 16 Serial.print() calls, each doing
  // its own division-based int-to-ASCII and pushing bytes into the TX ring one
  // at a time. Waiting for TX room HERE rather than inside write() is what lets
  // commands be serviced while the wire drains -- that is the SET latency.
  while ((uint8_t)Serial.availableForWrite() < n) pollSerial();
  Serial.write(buf, n);
}
