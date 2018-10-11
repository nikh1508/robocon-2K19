#include<Wire.h>
#include<EEPROM.h>

void setup() {
  Serial.begin(115200);
  byte ADR = EEPROM.read(0);
  Wire.begin(ADR);
//  Serial.println("Started I2C AT : "+ String(ADR));
  attachInterrupt(digitalPinToInterrupt(2), enc_isr, CHANGE);
  Wire.onRequest(send_enc);
  Wire.onReceive(reset);
//  Serial.println("Started");
}

int count = 0, count_last = 0;

void loop() {
  if (count != count_last) {
    count_last = count;
    Serial.println(count);
  }
}

void enc_isr() {
  if (digitalRead(2) == digitalRead(3))
    count++;
  else count--;
}

byte data[2];

void send_enc() {
  data[0] = count >> 8 ;
  data[1] = count;
  Wire.write(data, 2);
  //    Serial.print(count);
  //    Serial.print("\t");
  //    Serial.print(data[0]);
  //    Serial.print("\t");
  //    Serial.println(data[1]);
}
void reset() {
  //  Serial.println("here");
  char ch = Wire.read();
  //  Serial.print(String(ch) + " RECVD");
  count = 0;
}
