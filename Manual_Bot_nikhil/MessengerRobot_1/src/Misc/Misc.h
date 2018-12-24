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
    return 57.29578 * radian;
}

float toRadian(float degree)
{
    return 0.017453 * degree;
}

float angleDiff(float inp, float set)
{
    float tmp = abs(inp - set);
    float diff = min(tmp, abs(360 - tmp));
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

float maxm(float a, float b)
{
    return (a > b) ? a : b;
}

float maxm(float a, float b, float c)
{
    return maxm(max(a, b), c);
}

float map2(float val, float low_in, float high_in, float low_op, float high_op)
{
    return (((val - low_in) / (high_in - low_in)) * (high_op - low_op)) + low_op;
}

#endif