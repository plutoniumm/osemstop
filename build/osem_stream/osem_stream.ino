#include <SPI.h>
#define CS 53

bool streaming = false;

void setChannel(uint8_t channel, float voltage) {
  if (voltage < 0.0) voltage = 0.0;
  if (voltage > 2.5) voltage = 2.5;
  if (channel > 7)   channel = 7;

  uint16_t code = (uint16_t)((voltage / 2.5) * 4095.0 + 0.5);

  byte b1 = 0x03;                                          // command: write & update
  byte b2 = ((channel & 0x07) << 4) | ((code >> 8) & 0x0F); // channel[2:0] + code[11:8]
  byte b3 = code & 0xFF;                                   // code[7:0]
  byte b4 = 0x00;

  digitalWrite(CS, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  SPI.transfer(b3);
  SPI.transfer(b4);
  digitalWrite(CS, HIGH);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("SET ")) {
    int spaceIdx = line.indexOf(' ', 4);
    if (spaceIdx > 0) {
      uint8_t ch  = line.substring(4, spaceIdx).toInt();
      float   vol = line.substring(spaceIdx + 1).toFloat();
      setChannel(ch, vol);
      Serial.print("OK ch=");
      Serial.print(ch);
      Serial.print(" v=");
      Serial.println(vol, 4);
    } else {
      Serial.println("ERR bad format");
    }
  } else if (line == "STREAM") {
    streaming = true;
    Serial.println("STREAMING");
  } else if (line == "STOP") {
    streaming = false;
    Serial.println("STOPPED");
  } else {
    Serial.println("ERR unknown command");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Enable internal reference (same as original working code)
  digitalWrite(CS, LOW);
  SPI.transfer(0x08);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x01);
  digitalWrite(CS, HIGH);

  // Explicitly zero every channel used by the OSEMs before anything else runs
  setChannel(0, 0.0);
  setChannel(2, 0.0);
  setChannel(4, 0.0);
  setChannel(6, 0.0);

  delay(10);
  Serial.println("READY");
}

void loop() {
  // Always listen for commands, even while streaming (so STOP works mid-stream)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if (streaming) {
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);
    int a2 = analogRead(A2);
    int a3 = analogRead(A3);
    


    Serial.print(a0); Serial.print(",");
    Serial.print(a1); Serial.print(",");
    Serial.print(a2); Serial.print(",");
    Serial.println(a3);
  }
}
