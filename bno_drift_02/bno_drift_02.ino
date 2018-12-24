#include<Wire.h>
#define adr 0x29
//
byte lsb_qw, msb_qw;
byte lsb_qx, msb_qx;
byte lsb_qy, msb_qy;
byte lsb_qz, msb_qz;
//
double qw, qx, qy, qz, heading, heading_offset = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //
  Wire.beginTransmission(adr);
  Wire.write(0x3D);
  Wire.write(0x0C);
  Wire.endTransmission();
  //
  delay(50);
  //
  bno_initialize();
}
void bno_initialize() {
  int i;
  for (int i = 0; i < 50; i++) {
    //
    Wire.beginTransmission(adr);
    Wire.write(0x20);
    Wire.endTransmission();
    Wire.requestFrom(adr, 8);
    //
    lsb_qw = Wire.read();
    msb_qw = Wire.read();
    lsb_qx = Wire.read();
    msb_qx = Wire.read();
    lsb_qy = Wire.read();
    msb_qy = Wire.read();
    lsb_qz = Wire.read();
    msb_qz = Wire.read();
    qw = (((((int)msb_qw) << 8) | lsb_qw));
    qx = (((((int)msb_qx) << 8) | lsb_qx));
    qy = (((((int)msb_qy) << 8) | lsb_qy));
    qz = (((((int)msb_qz) << 8) | lsb_qz));
    qw /= 16384;
    qx /= 16384;
    qy /= 16384;
    qz /= 16384;
    //normalize
    double temp = 0.0;
    temp = (qw * qw) + (qx * qx) + (qy * qy) + (qz * qz);
    temp = sqrt(temp);
    qw /= temp;
    qx /= temp;
    qy /= temp;
    qz /= temp;
    double si, co;
    si = 2.0 * ((qw * qz) + (qx * qy));
    co = 1.0 - 2.0 * ((qy * qy) + (qz * qz));
    heading = atan2(si, co);
    heading *= 57.2958;
    //
    heading_offset += heading;
  }
  heading_offset /= 50.0;
}


void get_q() {
  Wire.beginTransmission(adr);
  Wire.write(0x20);
  Wire.endTransmission();
  Wire.requestFrom(adr, 8);
  //
  lsb_qw = Wire.read();
  msb_qw = Wire.read();
  lsb_qx = Wire.read();
  msb_qx = Wire.read();
  lsb_qy = Wire.read();
  msb_qy = Wire.read();
  lsb_qz = Wire.read();
  msb_qz = Wire.read();
  qw = (((((int)msb_qw) << 8) | lsb_qw));
  qx = (((((int)msb_qx) << 8) | lsb_qx));
  qy = (((((int)msb_qy) << 8) | lsb_qy));
  qz = (((((int)msb_qz) << 8) | lsb_qz));
  qw /= 16384;
  qx /= 16384;
  qy /= 16384;
  qz /= 16384;
  //normalize
  double temp = 0.0;
  temp = (qw * qw) + (qx * qx) + (qy * qy) + (qz * qz);
  temp = sqrt(temp);
  qw /= temp;
  qx /= temp;
  qy /= temp;
  qz /= temp;
  double si, co;
  si = 2.0 * ((qw * qz) + (qx * qy));
  co = 1.0 - 2.0 * ((qy * qy) + (qz * qz));
  heading = atan2(si, co);
  heading *= 57.2958;
  heading -= heading_offset;
  if (heading < 0.0)
    heading += 360.0;
}
void loop() {
  get_q();
  Serial.print("Heading "); Serial.println(heading);
  delay(20);
}
