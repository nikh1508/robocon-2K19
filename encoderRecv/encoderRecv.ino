#include<Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

constexpr int ENC[] = {0x07, 0x08};

int encoderCount[2] = {0};

void readEncoder() {
  byte lsb, msb;
  for (int i = 0 ; i < 2; i++) {
    Wire.requestFrom(ENC[i], 2);
    msb = Wire.read();
    lsb = Wire.read();
    encoderCount[i] = msb << 8 | lsb;
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
  Serial.print(encoderCount[0]);
  Serial.print("\t");
  Serial.println(encoderCount[1]);
  delay(10);
}
