/*
*External Library Requiremens:
    *DualShock4_lib     ->      https://github.com/nikh1508/DualShock4_lib
    *PID_v1             ->      https://github.com/br3ttb/Arduino-PID-Library
*Rest of the libraries are included in /src folder
*/

#include <Arduino.h>
#include <Wire.h>
#include "src/Encoder/Encoder.h"
#include "src/IMU/IMU.h"
#include "src/HolonomicDrive/HolonomicDrive.h"

//  Function Prototypes:
void setup();
void loop();

constexpr motor front(1, 2, 3);
constexpr motor left(1, 2, 3);
constexpr motor right(1, 2, 3);

constexpr double linearConst[] = {0.0, 0.0, 0.0};
constexpr double rotationalConst[] = {0.0, 0.0, 0.0};

HolonomicDrive bot(front, left, right, linearConst, rotationalConst, 180);