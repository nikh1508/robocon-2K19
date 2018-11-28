#ifndef Encoder_H
#define Encoder_H
#include <Arduino.h>
#include <Wire.h>
#include "../Misc/Misc.h"

constexpr int ENC[] = {0x07, 0x08};
constexpr int SAMPLE_RATE = 10;
constexpr int THETA_SAMPLE_RATE = 50;
bool newTheta = false;
double calculatedTheta = 0.0;

long encoderCount[2] = {0}, lastEncoderCount[2] = {0};

bool readEncoder();

struct Coordinate
{
    float x;
    float y;
    Coordinate()
    {
        x = y = 0;
    }
};

void getCoordinates(Coordinate &var, float referenceAngle = 0.0)
{
    readEncoder();
    var.x = encoderCount[0] * cos(toRadian(45.0 - referenceAngle)) - encoderCount[1] * cos(toRadian(45.0 + referenceAngle));
    var.y = encoderCount[0] * sin(toRadian(45.0 - referenceAngle)) + encoderCount[1] * sin(toRadian(45.0 + referenceAngle));
}

void calcTheta(float referenceAngle = 0.0)
{
    static Coordinate lastCoordinate;
    Coordinate now;
    getCoordinates(now, referenceAngle);
    float dx = now.x - lastCoordinate.x;
    float dy = now.y = lastCoordinate.y;
    if (dx > -0.01 && dx < 0.01)
        calculatedTheta = 90.0;
    else
        calculatedTheta = toDegree(atan(dy / dx));
    lastCoordinate = now;
}

bool readEncoder()
{
    static long lastEncoderRead = millis() - SAMPLE_RATE;
    static long lastThetaCalc = millis() - SAMPLE_RATE;
    long currentTime = millis();
    if ((currentTime - lastEncoderRead) < SAMPLE_RATE)
        return false;
    lastEncoderRead = currentTime;
    byte recv;
    for (int i = 0; i < 2; i++)
    {
        Wire.requestFrom(ENC[i], 4);
        encoderCount[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            recv = Wire.read();
            encoderCount[i] = encoderCount[i] << 8 | recv;
        }
    }
    currentTime = millis();
    if ((currentTime - lastThetaCalc) > THETA_SAMPLE_RATE)
    {
        newTheta = true;
        calcTheta();
        lastThetaCalc = currentTime;
    }
    return true;
}

void resetEncoder()
{
    for (int i = 0; i < 2; i++)
    {
        Wire.beginTransmission(ENC[i]);
        Wire.write('r');
        Wire.endTransmission();
    }
}

void encoderDebug()
{
    if (Serial.available())
    {
        char ch = Serial.read();
        resetEncoder();
    }
    readEncoder();
    if (encoderCount[0] != lastEncoderCount[0] || encoderCount[1] != lastEncoderCount[1])
    {
        Serial.print(encoderCount[0]);
        Serial.print("\t");
        Serial.println(encoderCount[1]);
        lastEncoderCount[0] = encoderCount[0];
        lastEncoderCount[1] = encoderCount[1];
    }
    delay(10);
}

#endif