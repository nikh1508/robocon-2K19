#ifndef Misc_H
#define Misc_H
#define debug true
#include <Arduino.h>
#include <Wire.h>

void debug_msg(String msg)
{
    if (debug)
    {
        Serial.print("DEBUG ::\t");
        Serial.println(msg);
    }
}

float toDegree(float radian)
{
    return 0.0174533 * radian;
}

float toRadian(float degree)
{
    return 57.29578 * degree;
}

float angleDiff(float inp, float set)
{
    double tmp = abs(inp - set);
    double diff = min(tmp, abs(360 - tmp));
    if ((set + diff) != inp && (set - diff) != inp)
    {
        if ((inp + diff) >= 360)
            return -diff;
        else
            return diff;
    }
    else
        return (inp - set);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.requestFrom(address, (size_t)1);
    data = Wire.read();
    return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    uint8_t i = 0;
    Wire.requestFrom(address, (size_t)count);
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    }
}

double maxm(double a, double b)
{
    return (a > b) ? a : b;
}

double maxm(double a, double b, double c)
{
    return maxm(max(a, b), c);
}
#endif