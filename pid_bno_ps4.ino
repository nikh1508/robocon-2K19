
#include <limits.h>
#include <Arduino.h>
#include<PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualShock4_lib.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define debug false

double setpoint, input, output;
int setvalue = 30;
float i,Kp = 0, Ki = 0, Kd = 0;
int speed_f, speed_l = 0, speed_r = 0;
int pwm_l = 7, pwm_r = 6, pwm_f = 9;
int dir1_l = 48, dir1_r = 12, dir1_f = 10;
int dir2_l = 46, dir2_r = 13, dir2_f = 11;
DualShock4 DS4(Serial2);
int pins[] = {9, 10, 11, 12, 13, 6, 7, 48, 46};
int data[16];
int flag=0,flag1=0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

double get_angle(char axis) {
  sensors_event_t event;
  bno.getEvent(&event);
  double x = event.orientation.x; // taking X readings

  if (x > 180)                  //to get negative angles in CCW direction
    x = (360 - x) * -1;
  switch (axis) {
    case 'x':
      return x;
      break;

    default:
      return 0;
  }
}

void setup() {
  Serial.begin(9600);
 
 
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
  }
  Serial.println("Starting..!!");
  if (!bno.begin())
  {
    Serial.println("hi");
    Serial.print("BNO not detected. Check connections or I2C ADDR!(Run I2C Scanner in Debug Mode.)");
    while (1);
  }
  
  bno.setExtCrystalUse(true);
  
  //bno.begin();
  setpoint = get_angle('x');

  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
}
