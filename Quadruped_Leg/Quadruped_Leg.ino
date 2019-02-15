/*
Developed @ SRM Team Robocon Lab,
            SRM University, Kattankulathur
    
External Libraries Required:
    *PID_v1             ->      https://github.com/br3ttb/Arduino-PID-Library
*/
#include "lib/Leg/Leg.h"

constexpr MotorWithEncoder leg0_hip{23, 32, 35}; //dataPin | enc_1 | enc_2
constexpr MotorWithEncoder leg0_knee{22, 39, 34};

constexpr double hipCons[] = {0.0, 0.0, 0.0};
constexpr double kneeCons[] = {0.0, 0.0, 0.0};

Leg leg0(0, leg0_hip, leg0_knee, 26, 27, 28, 40, hipCons, kneeCons); // HIP_MOTOR | KNEE MOTOR | HIP_SWITCH | KNEE SWITCH | MAX_POWER | Hip PID constants | Knee PID Constants