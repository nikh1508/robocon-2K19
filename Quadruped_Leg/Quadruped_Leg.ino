/*
Developed @ SRM Team Robocon Lab,
            SRM University, Kattankulathur
    
External Libraries Required:
    *PID_v1             ->      https://github.com/br3ttb/Arduino-PID-Library
*/
#include "lib/Leg/Leg.h"

constexpr MotorWithEncoder leg0_hip{PB9, PB13, PB14}; //dataPin | enc_1 | enc_2
constexpr MotorWithEncoder leg0_knee{PB8, PA12, PA15};

constexpr double hipCons[] = {0.0, 0.0, 0.0};
constexpr double kneeCons[] = {0.5, 0.0, 0.0};

Leg leg0(0, leg0_hip, leg0_knee, PB6, PB7, PA2, 50, hipCons, kneeCons); // HIP_MOTOR | KNEE MOTOR | HIP_SWITCH | KNEE SWITCH | MAX_POWER | Hip PID constants | Knee PID Constants