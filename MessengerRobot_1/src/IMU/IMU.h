#ifndef IMU_H
#define IMU_H
#include "../Misc/Misc.h"
constexpr int BNO_ADDR = 0x29;
constexpr int BNO_ID = 0xA0;
constexpr int BNO_ID_READ = 0x00;
constexpr int BNO_OPR_MODE = 0x3D;
constexpr int BNO_CONFIG = 0x00;
constexpr int BNO_NDOF = 0x0C;
constexpr int BNO_EUL_HEADING_LSB = 0x1A;
constexpr int BNO_SYS_TRIGGER = 0x3F;
constexpr int SAMPLE_TIME = 10; //Minimum Time interval between each Reading

static double yawOffset = 0.0;
static byte data[2];
static bool crystal = false;
static float yaw = 0.0;

void setModeBNO(byte mode)
{
    writeByte(BNO_ADDR, BNO_OPR_MODE, mode);
    delay(30);
}

void setCrystalUseBNO(bool cond)
{
    crystal = cond;
    setModeBNO(BNO_CONFIG);
    if (cond)
        writeByte(BNO_ADDR, BNO_SYS_TRIGGER, 0x80);
    else
        writeByte(BNO_ADDR, BNO_SYS_TRIGGER, 0);
    delay(10);
    setModeBNO(BNO_NDOF);
}

void initializeBNO()
{
    debug_msg("Initializing BNO055");
    setModeBNO(BNO_NDOF);
    for (int i = 0; i < 50; i++)
    {
        readBytes(BNO_ADDR, BNO_EUL_HEADING_LSB, 2, data);
        yawOffset += ((uint16_t)data[1] << 8) | data[0];
    }
    yawOffset /= 800.0;
    debug_msg("Initializing Complete.");
    debug_msg("Yaw Offset = " + String(yawOffset));
}

float getYaw()
{
    static long lastRead = millis() - SAMPLE_TIME;
    long currentTime = millis();
    if ((currentTime - lastRead) < SAMPLE_TIME)
        return yaw;
    lastRead = currentTime;
    readBytes(BNO_ADDR, BNO_EUL_HEADING_LSB, 2, data);
    yaw = ((uint16_t)data[1] << 8) | data[0];
    yaw /= 16.0;
    yaw -= yawOffset;
    if (yaw < 0)
    {
        yaw += 360.0;
    }
    return yaw;
}

void resetBNO()
{
    debug_msg("Resetting BNO055");
    setModeBNO(BNO_CONFIG);
    writeByte(BNO_ADDR, BNO_SYS_TRIGGER, 0x20);
    while (readByte(BNO_ADDR, BNO_ID_READ) != BNO_ID)
    {
        delay(10);
    }
    delay(50);
    writeByte(BNO_ADDR, BNO_SYS_TRIGGER, 0x0);
    delay(10);
    initializeBNO();
}

#endif