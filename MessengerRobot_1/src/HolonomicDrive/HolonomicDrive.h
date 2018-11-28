#ifndef HolonomicDrive_H
#define HolonomicDrive_H
#include <Arduino.h>
#include "../IMU/IMU.h"
#include "../Encoder/Encoder.h"
#include "../Misc/Misc.h"
#include <PID_v1.h>

struct motor
{
    int D0;
    int D1;
    int PWM_PIN;
    constexpr motor(int x, int y, int z) : D0(x), D1(y), PWM_PIN(z)
    {
    }
};

struct comp
{
    double Vx;
    double Vy;
    double W;
};

enum PID_OBJ
{
    LIN = 0,
    ROT
}; //LIN AND ROT PID OBJECTS

static double input[2];
static double output[2];
static double setpoint[2];
static bool prevStopped = true;

PID linear(&input[LIN], &output[LIN], &setpoint[LIN], 0.0, 0.0, 0.0, DIRECT);
PID rotational(&input[ROT], &output[ROT], &setpoint[ROT], 0.0, 0.0, 0.0, DIRECT);

class HolonomicDrive
{
    motor front;
    motor left;
    motor right;

    comp comp;
    double Kp[2];
    double Ki[2];
    double Kd[2];

    int frontPWM;
    int leftPWM;
    int rightPWM;

    int MAX_PWM;

  public:
    constexpr HolonomicDrive(motor f, motor l, motor r, double *linCons, double *rotCons, int max = 150) : front(f), left(l), right(r),
                                                                                                           Kp{linCons[0], rotCons[0]}, Ki{linCons[1], rotCons[1]}, Kd{linCons[2], rotCons[2]},
                                                                                                           frontPWM(0), leftPWM(0), rightPWM(0),
                                                                                                           comp{0.0, 0.0, 0.0}, MAX_PWM(max)
    {
    }
    void initialize();
    void writeMotor(int, int, int);
    void stopAll(bool HARD = true);
    void move(int, double, double);
};

void HolonomicDrive::initialize()
{
    linear.SetSampleTime(50);
    rotational.SetSampleTime(50);
    linear.SetTunings(Kp[LIN], Ki[LIN], Kd[LIN]);
    rotational.SetTunings(Kp[ROT], Ki[ROT], Kd[ROT]);
    setpoint[0] = setpoint[1] = 0.0;
}

void HolonomicDrive::writeMotor(const int f, const int l, const int r)
{
    if (f != -1)
    {
        digitalWrite(front.D0, f > 0);
        digitalWrite(front.D1, f < 0);
        analogWrite(front.PWM_PIN, abs(f));
    }
    if (l != -1)
    {
        digitalWrite(left.D0, l > 0);
        digitalWrite(left.D1, l < 0);
        analogWrite(left.PWM_PIN, abs(l));
    }
    if (r != -1)
    {
        digitalWrite(right.D0, r > 0);
        digitalWrite(right.D1, r < 0);
        analogWrite(right.PWM_PIN, abs(r));
    }
    String msg = ("F : " + String(f) + "\tL : " + String(l), "\tR : " + String(r));
    debug_msg(msg);
}

void HolonomicDrive::stopAll(bool HARD = true)
{
    digitalWrite(front.D0, HARD);
    digitalWrite(front.D1, HARD);
    digitalWrite(front.PWM_PIN, HARD);

    digitalWrite(left.D0, HARD);
    digitalWrite(left.D1, HARD);
    digitalWrite(left.PWM_PIN, HARD);

    digitalWrite(right.D0, HARD);
    digitalWrite(right.D1, HARD);
    digitalWrite(right.PWM_PIN, HARD);

    prevStopped = true;
    debug_msg("Stopped All Motors");
}

void HolonomicDrive::move(int R, double theta, double w = 0)
{
    static long startTime = millis();
    R = map(constrain(R, 0, 100), 0, 100, 0, MAX_PWM);
    theta = toRadian(theta);
    comp.Vx = cos(theta);
    comp.Vy = sin(theta);
    comp.W = w;
    double Vf = comp.Vx + w;
    double Vl = -0.5 * comp.Vx + 0.867 * comp.Vy + w;
    double Vr = -0.5 * comp.Vx - 0.867 * comp.Vy + w;
    double maxComp = maxm(abs(Vf), abs(Vl), abs(Vr));

    if (maxComp == abs(Vf))
    {
        frontPWM = R * (Vf / abs(Vf));
        leftPWM = (double)R * (Vl / abs(Vf));
        rightPWM = (double)R * (Vr / abs(Vf));
    }
    else if (maxComp == abs(Vl))
    {
        frontPWM = (double)R * (Vf / abs(Vl));
        leftPWM = R * (Vl / abs(Vl));
        rightPWM = (double)R * (Vr / abs(Vl));
    }
    else if (maxComp == abs(Vr))
    {
        frontPWM = (double)R * (Vf / abs(Vr));
        leftPWM = (double)R * (Vl / abs(Vr));
        rightPWM = R * (Vl / abs(Vr));
    }

    readEncoder();

    if (!prevStopped)
    {
        if ((millis() - startTime) > THETA_SAMPLE_RATE)
        {
            if (newTheta)
            {
                newTheta = false;
            }
        }
    }
    else
    {
        prevStopped = false;
        startTime = millis();
    }
    writeMotor(frontPWM, leftPWM, rightPWM);
}

#endif