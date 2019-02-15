/*Changes done in order to try and make the code compactible with ESP-32:
    *portMux handles the shared memory in every Interrupt Variable.
    *---DRAM_ATTR used for every INterrupt Variable (as suggested in the solution : https://github.com/espressif/arduino-esp32/issues/1907) 
*/
#ifndef Leg_H
#define Leg_H

#define debug true

#include <Arduino.h>
#include <PID_v1.h>

#ifdef ESP_H
#include <ESP32_Servo.h>
#else
#include <Servo.h>
#endif

struct MotorWithEncoder
{
  int dataPin; //This code was originally designed to drive the motors using Cytron SmartDriveDuo-30 in RC Mode
  int enc_1;
  int enc_2;
  constexpr MotorWithEncoder(int x, int y, int z) : dataPin(x), enc_1(y), enc_2(z)
  {
  }
};
enum
{
  HIP = 0,
  KNEE
};

class Leg
{
  MotorWithEncoder hip;
  MotorWithEncoder knee;
  int hipSwitch;
  int kneeSwitch;
  int pawSwitch;
  int MAX_POWER;

  portMUX_TYPE mux;
  volatile int hipCounter;
  volatile int kneeCounter;
  volatile bool hipPressed;
  volatile bool kneePressed;
  volatile bool pawPressed;

  ////  PID Variables and Objects  ////
  double input[2];
  double setPoint[2];
  double output[2];
  double Kp[2];
  double Ki[2];
  double Kd[2];
  PID hipPID; //Empty constructor calls are allowed only after modifying the original PID Library
  PID kneePID;
  ////

  Servo hipMotor;
  Servo kneeMotor;

  // void writeMotor(int motor, int value);
  void initialize(int instance);

  // Glue Routines to handle External Interrupts | This has to be declared for every Class Instance |For more info : http://www.gammon.com.au/forum/?id=12983
  static Leg *instance[2];
  // Currently the static functions are declared just for one instance(instance_0).
  static void isrHipExt0()
  {
    if (Leg::instance[0] != NULL)
      Leg::instance[0]->isrHip();
  }
  static void isrKneeExt0()
  {
    if (Leg::instance[0] != NULL)
      Leg::instance[0]->isrKnee();
  }
  static void isrHipSwitchExt0()
  {
    if (Leg::instance[0] != NULL)
      Leg::instance[0]->isrHipSwitch();
  }
  static void isrKneeSwitchExt0()
  {
    if (Leg::instance[0] != NULL)
      Leg::instance[0]->isrKneeSwitch();
  }
  static void isrPawSwitchExt0()
  {
    if (Leg::instance[0] != NULL)
      Leg::instance[0]->isrPawSwitch();
  }
  // End of Glue Routines

public:
  Leg(int instance, MotorWithEncoder _hip, MotorWithEncoder _knee, int _hipSwitch, int _kneeSwitch, int _pawSwitch, int max, const double *_hipCons, const double *_kneeCons) : hip(_hip), knee(_knee), hipSwitch(_hipSwitch), kneeSwitch(_kneeSwitch), pawSwitch(_pawSwitch), MAX_POWER(constrain(max, -100, 100)), Kp{_hipCons[0], _kneeCons[0]}, Ki{_hipCons[1], _kneeCons[1]}, Kd{_hipCons[2], _kneeCons[2]}
  {
    Serial.begin(115200);
    debug_msg("In Constructor");
    mux = portMUX_INITIALIZER_UNLOCKED;
    hipCounter = kneeCounter = 0;
    setPoint[0] = setPoint[1] = 0.0;
    PID _hipPID(&input[HIP], &output[HIP], &setPoint[HIP], 0.0, 0.0, 0.0, P_ON_M, DIRECT); //Destructor will be called for both of these PID objects once this Constructor Call is over
    PID _kneePID(&input[KNEE], &output[KNEE], &setPoint[KNEE], 0.0, 0.0, 0.0, P_ON_M, DIRECT);
    hipPID = _hipPID;
    kneePID = _kneePID;
    initialize(instance);
  }
  void writeMotor(int motor, int value);
  void debug_msg(String msg);
  void stopAll();
  void autoHome();
  int getEncoder(int enc);
  void debugComp(bool switches, bool encoders);

#ifdef ESP_H
  void IRAM_ATTR isrHip();
  void IRAM_ATTR isrKnee();
  void IRAM_ATTR isrHipSwitch();
  void IRAM_ATTR isrKneeSwitch();
  void IRAM_ATTR isrPawSwitch();
#else
  void isrHip();
  void isrKnee();
  void isrHipSwitch();
  void isrKneeSwitch();
  void isrPawSwitch();
#endif
};

Leg *Leg::instance[2] = {NULL, NULL};
/////////////////////////////////////////////////////////Func Definitions/////////////////////////////////////////////////////////

void Leg::writeMotor(int motor, int value)
{
  switch (motor)
  {
  case HIP:
    hipMotor.writeMicroseconds(map(constrain(value, -MAX_POWER, MAX_POWER), -100, 100, 1000, 2000));
    break;
  case KNEE:
    kneeMotor.writeMicroseconds(map(constrain(value, -MAX_POWER, MAX_POWER), -100, 100, 1000, 2000));
    break;
  }
}

void Leg::initialize(int instance)
{
  debug_msg("Instance Initialization Started.");
  MotorWithEncoder motors[] = {hip,
                               knee};
  for (MotorWithEncoder i : motors)
  {
    pinMode(i.dataPin, OUTPUT);
    pinMode(i.enc_1, INPUT_PULLDOWN);
    pinMode(i.enc_2, INPUT_PULLDOWN);
  }
  pinMode(hipSwitch, INPUT_PULLDOWN);
  pinMode(kneeSwitch, INPUT_PULLDOWN);
  pinMode(pawSwitch, INPUT_PULLDOWN);
  hipMotor.attach(hip.dataPin);
  kneeMotor.attach(knee.dataPin);
  stopAll();
  if (instance == 0)
  {
    attachInterrupt(digitalPinToInterrupt(hip.enc_1), isrHipExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hip.enc_2), isrHipExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(knee.enc_1), isrKneeExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(knee.enc_2), isrKneeExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hipSwitch), isrHipSwitchExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kneeSwitch), isrKneeSwitchExt0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pawSwitch), isrPawSwitchExt0, CHANGE);
    this->instance[0] = this;
  }
  hipPID.SetSampleTime(50);
  kneePID.SetSampleTime(50);
  hipPID.SetTunings(Kp[HIP], Ki[HIP], Kd[HIP]);
  kneePID.SetTunings(Kp[KNEE], Ki[KNEE], Kd[KNEE]);
  hipPID.SetOutputLimits(-100, 100);
  kneePID.SetOutputLimits(-100, 100);
  hipPID.SetMode(AUTOMATIC);
  kneePID.SetMode(AUTOMATIC);
  debug_msg("Initialization Complete.");
}

void Leg::debug_msg(String msg)
{
  if (debug)
    Serial.println("DEBUG ::\t" + msg + "\n");
}

void Leg::stopAll()
{
  hipMotor.writeMicroseconds(1500);
  kneeMotor.writeMicroseconds(1500);
}

void Leg::autoHome()
{
  portENTER_CRITICAL(&mux);
  bool kneeCurrent = kneePressed;
  portEXIT_CRITICAL(&mux);

  while (!kneeCurrent)
  {
    portENTER_CRITICAL(&mux);
    kneeCurrent = kneePressed;
    portEXIT_CRITICAL(&mux);
    writeMotor(KNEE, 10);
  }
  writeMotor(KNEE, 0);
  delay(500);

  portENTER_CRITICAL(&mux);
  bool hipCurrent = hipPressed;
  portEXIT_CRITICAL(&mux);

  while (!hipCurrent)
  {
    portENTER_CRITICAL(&mux);
    bool hipCurrent = hipPressed;
    portEXIT_CRITICAL(&mux);
    writeMotor(HIP, 10);
  }
  writeMotor(HIP, 0);
}

int Leg::getEncoder(int enc)
{
  int toReturn;
  switch (enc)
  {
  case HIP:
    portENTER_CRITICAL(&mux);
    toReturn = hipCounter;
    portEXIT_CRITICAL(&mux);
    break;

  case KNEE:
    portENTER_CRITICAL(&mux);
    toReturn = kneeCounter;
    portEXIT_CRITICAL(&mux);
    break;

  default:
    return -1;
    break;
  }
  return toReturn;
}

void Leg::debugComp(bool switches, bool encoders)
{
  String msg = "";
  if (switches)
  {
    portENTER_CRITICAL(&mux);
    msg += String(hipPressed) + "\t" + String(kneePressed) + "\t" + String(pawPressed) + "\t";
    portEXIT_CRITICAL(&mux);
  }
  if (encoders)
    msg += String(getEncoder(HIP)) + "\t" + String(getEncoder(KNEE));
  debug_msg(msg);
}

////  Interrupt Subroutines   ////
void IRAM_ATTR Leg::isrHip()
{
  static uint8_t lastEncoded = 0;
  uint8_t MSB = digitalRead(hip.enc_1);
  uint8_t LSB = digitalRead(hip.enc_2);
  uint8_t encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  {
    portENTER_CRITICAL(&mux);
    hipCounter++;
    portEXIT_CRITICAL(&mux);
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    portENTER_CRITICAL(&mux);
    hipCounter--;
    portEXIT_CRITICAL(&mux);
  }
  lastEncoded = encoded;
}

void IRAM_ATTR Leg::isrKnee()
{
  static uint8_t lastEncoded = 0;
  uint8_t MSB = digitalRead(knee.enc_1);
  uint8_t LSB = digitalRead(knee.enc_2);
  uint8_t encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  {
    portENTER_CRITICAL(&mux);
    kneeCounter++;
    portEXIT_CRITICAL(&mux);
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    portENTER_CRITICAL(&mux);
    kneeCounter--;
    portEXIT_CRITICAL(&mux);
  }
  lastEncoded = encoded;
}

void IRAM_ATTR Leg::isrHipSwitch()
{
  portENTER_CRITICAL(&mux);
  hipPressed = digitalRead(hipSwitch);
  if (hipPressed)
    hipCounter = 0;
  portEXIT_CRITICAL(&mux);
}

void IRAM_ATTR Leg::isrKneeSwitch()
{
  portENTER_CRITICAL(&mux);
  kneePressed = digitalRead(kneeSwitch);
  if (kneePressed)
    kneeCounter = 0;
  portEXIT_CRITICAL(&mux);
}

void IRAM_ATTR Leg::isrPawSwitch()
{
  portENTER_CRITICAL(&mux);
  pawPressed = digitalRead(pawSwitch);
  portEXIT_CRITICAL(&mux);
}
////  End of Interrupt Subroutines   ////
#endif