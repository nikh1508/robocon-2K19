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
