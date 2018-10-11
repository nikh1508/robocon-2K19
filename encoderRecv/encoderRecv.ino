#include<Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

constexpr int ENC[] = {0x07, 0x08};

long encoderCount[2] = {0}, lastEncoderCount[2] = {0};

void readEncoder() {
  byte recv;
  for (int i = 0 ; i < 2; i++) {
    Wire.requestFrom(ENC[i], 4);
    encoderCount[i] = 0;
    for (int j = 0; j < 4; j++) {
      recv = Wire.read();
      encoderCount[i] = encoderCount[i] << 8 | recv;
    }
  }
}

void resetEncoder() {
  for (int i = 0 ; i < 2; i++) {
    Wire.beginTransmission(ENC[i]);
    Wire.write('r');
    Wire.endTransmission();
  }
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    resetEncoder();
  }
  readEncoder();
  if (encoderCount[0] != lastEncoderCount[0] || encoderCount[1] != lastEncoderCount[1]) {
    Serial.print(encoderCount[0]);
    Serial.print("\t");
    Serial.println(encoderCount[1]);
    lastEncoderCount[0] = encoderCount[0];
    lastEncoderCount[1] = encoderCount[1];
  }
  delay(10);
}
