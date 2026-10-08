// Arduino Mega: 8 ADC inputs in, 8-channel SPI DAC out. Protocol: see osem/board.py.
#include <SPI.h>

#define FW 3
#define CS 53
#define NCH 8

static const uint32_t BAUD_HZ = 115200UL;

static const uint8_t ADC_PRESCALER = 32;

static const uint8_t SYNC0 = 0xA5, SYNC1 = 0xC3;
static const uint8_t FRAME_LEN = 3 + 2 * NCH + 1;

static const uint8_t CMD0 = 0xA5, CMD1 = 0x3C;
static const uint8_t CMD_LEN    = 2 + 2 * NCH + 2;
static const uint8_t FRAME2_LEN = 3 + 2 * NCH + 3 + 1;
static bool     bin2Mode = false;
static uint8_t  osN = 1, osCount = 0;
static uint8_t  crN = 1;
static uint16_t osAcc[NCH];
static uint8_t  rxOk = 0, rxBad = 0;
static uint16_t lastUnits[NCH];

static bool    streaming = false;
static bool    binMode   = false;
static bool    ackOn     = true;
static uint8_t seq       = 0;
static uint8_t prescNow  = ADC_PRESCALER;

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

static void writeCode(uint8_t channel, uint16_t code) {
  if (code > 4095)  code = 4095;
  if (channel > 7)  channel = 7;

  byte b2 = (byte)(((channel & 0x07) << 4) | ((code >> 8) & 0x0F));
  byte b3 = (byte)(code & 0xFF);

  digitalWrite(CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer(b2);
  SPI.transfer(b3);
  SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
}

static uint16_t unitsToCode(uint16_t units) {
  if (units > 25000) units = 25000;
  return (uint16_t)(((uint32_t)units * 4095UL + 12500UL) / 25000UL);
}

static void setChannelUnits(uint8_t channel, uint16_t units) {
  if (channel < NCH) lastUnits[channel] = units;
  writeCode(channel, unitsToCode(units));
}

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

static char    cmdbuf[40];
static uint8_t cmdlen  = 0;
static bool    cmdOver = false;

static void handleCommand(char *s);

static uint8_t binbuf[CMD_LEN];
static uint8_t binlen = 0;

static void binaryCommand() {
  uint8_t sum = 0, x = 0;
  for (uint8_t i = 2; i < 2 + 2 * NCH; i++) { sum += binbuf[i]; x ^= binbuf[i]; }
  if (sum != binbuf[CMD_LEN - 2] || x != binbuf[CMD_LEN - 1]) { rxBad++; return; }
  for (uint8_t c = 0; c < NCH; c++) {
    uint16_t u = (uint16_t)binbuf[2 + 2 * c] | ((uint16_t)binbuf[3 + 2 * c] << 8);
    if (u == 0xFFFF || u == lastUnits[c]) continue;
    if (u > 25000) u = 25000;
    setChannelUnits(c, u);
  }
  rxOk++;
}

static void pollSerial() {
  uint8_t budget = 96;
  while (budget-- && Serial.available()) {
    char c = (char)Serial.read();
    if (binlen) {
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
      cmdOver = true;
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

static uint16_t eatUnits(char **p) {
  char *s = *p;
  while (*s == ' ') s++;
  if (*s == '-') { *p = s + 1; return 0; }
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

  if (eat(&s, "SET ")) {
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

  if (eat(&s, "SETC ")) {
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

  if (eat(&s, "OS ")) {
    uint32_t n = eatUInt(&s);
    osN = (uint8_t)(n < 1 ? 1 : n > 32 ? 32 : n);
    if ((uint16_t)osN * crN > 64) osN = 64 / crN;
    osCount = 0;
    for (uint8_t i = 0; i < NCH; i++) osAcc[i] = 0;
    okLine("os=", osN);
    return;
  }

  if (eat(&s, "CR ")) {
    uint32_t n = eatUInt(&s);
    crN = (uint8_t)(n < 1 ? 1 : n > 4 ? 4 : n);
    if ((uint16_t)osN * crN > 64) osN = 64 / crN;
    osCount = 0;
    for (uint8_t i = 0; i < NCH; i++) osAcc[i] = 0;
    okLine("cr=", crN);
    return;
  }

  if (eat(&s, "MODE ")) {
    if (eat(&s, "BIN2"))       { bin2Mode = true; binMode = true; Serial.print(F("OK mode=bin2\r\n")); }
    else if (eat(&s, "BIN"))   { bin2Mode = false; binMode = true;  Serial.print(F("OK mode=bin\r\n")); }
    else if (eat(&s, "ASCII")) { bin2Mode = false; binMode = false; Serial.print(F("OK mode=ascii\r\n")); }
    else                       { Serial.print(F("ERR bad mode\r\n")); }
    return;
  }

  if (eat(&s, "BAUD ")) {
    uint32_t b = eatUInt(&s);
    if (b < 9600UL || b > 2000000UL) { Serial.print(F("ERR bad baud\r\n")); return; }
    Serial.print(F("OK baud="));
    Serial.print(b);
    Serial.print(F("\r\n"));
    Serial.flush();
    Serial.begin(b);
    return;
  }

  if (eat(&s, "INFO")) {
    Serial.print(F("INFO nch=8 mode="));
    Serial.print(binMode ? F("bin") : F("ascii"));
    Serial.print(F(" presc="));  Serial.print(prescNow);
    Serial.print(F(" ack="));    Serial.print(ackOn ? 1 : 0);
    Serial.print(F(" frame="));  Serial.print(FRAME_LEN);
    Serial.print(F(" fw=")); Serial.print(FW); Serial.print(F(" os=")); Serial.print(osN);
    Serial.print(F(" cr=")); Serial.print(crN);
    Serial.print(F(" frame2=")); Serial.print(FRAME2_LEN);
    Serial.print(F("\r\n"));
    return;
  }

  Serial.print(F("ERR unknown command\r\n"));
}

void setup() {
  Serial.begin(BAUD_HZ);
  adcSetPrescaler(ADC_PRESCALER);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  digitalWrite(CS, LOW);
  SPI.transfer(0x08);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x01);
  digitalWrite(CS, HIGH);

  for (uint8_t c = 0; c < 8; c++) { writeCode(c, 0); lastUnits[c] = 0; }

  delay(10);

  handleCommand((char *)"INFO");
  Serial.print(F("READY\r\n"));
}

void loop() {
  pollSerial();

  if (!streaming) return;

  uint16_t v[NCH];
  v[0] = v[1] = v[2] = v[3] = 0;
  for (uint8_t r = 0; r < crN; r++) {
    v[0] += analogRead(A0);  v[1] += analogRead(A1);
    v[2] += analogRead(A2);  v[3] += analogRead(A3);
  }
  v[4] = analogRead(A4);  v[5] = analogRead(A5);
  v[6] = analogRead(A6);  v[7] = analogRead(A7);

  pollSerial();

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

  for (uint8_t i = 0; i < NCH; i++) {
    uint16_t k = (uint16_t)osN * (i < 4 ? crN : 1);
    v[i] = (osAcc[i] + k / 2) / k; osAcc[i] = 0;
  }

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

  while ((uint8_t)Serial.availableForWrite() < n) pollSerial();
  Serial.write(buf, n);
}
