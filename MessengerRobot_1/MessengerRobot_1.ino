/*
*External Library Requiremens:
    *DualShock4_lib     ->      https://github.com/nikh1508/DualShock4_lib
    *PID_v1             ->      https://github.com/br3ttb/Arduino-PID-Library
*Rest of the libraries are included in /src folder
*/

#include <Arduino.h>
#include <Wire.h>
#include <DualShock4_lib.h>
#include "src/Encoder/Encoder.h"
#include "src/IMU/IMU.h"
#include "src/HolonomicDrive/HolonomicDrive.h"

constexpr int MAX_SPEED = 200;
constexpr int AXIS_DEAD_ZONE = 1500;
//  Function Prototypes:
void setup();
void loop();
bool checkForRoutines();
void forestRoutine();
void decodeData(byte toDecode[], byte totalRecvd, byte &decodedByteCount, byte decodedBytes[], byte specialByte);
bool ReadBytes(byte data[], byte toRead, char toSend);
void convert(byte toConvert[], unsigned int converted[], long &res);
bool funcData(double values[]);
void Serial3Flush();

constexpr motor front(5, 6, 4);  //DO | D1 | PWM        //CHANGE-HERE
constexpr motor left(10, 9, 8); //CHANGE-HERE
constexpr motor right(13, 12, 11);   //CHANGE-HERE

//  PID Constants
constexpr double linearConst[] = {0.0, 0.0, 0.0};          //Kp | Ki | Kd
constexpr double rotationalConst[] = {0.016, 0.004, 0.01}; //{0.03, 0.135, 0.011} //avg:{0.03, 0.117, 0.018}

HolonomicDrive bot(front, left, right, linearConst, rotationalConst, MAX_SPEED); //Front | Left | Right | Linear Constant | Rotational Constant | Max Speed
DualShock4 ds4(Serial3);

bool powerOn = false;
constexpr int powerLed = 53;

int Lx, Ly, Rx, Ry;