bool mright = false;
bool lastPower = false;
uint8_t mode = -1;
void loop()
{
    if (ds4.readGamepad())
    {
        if (ds4.buttonPressed(PS))
        {
            powerOn = !powerOn;
            digitalWrite(powerLed, powerOn);
            debug_msg("POWER : " + String((powerOn) ? "ON" : "OFF"));
        }
        if (powerOn)
        {
            Lx = map(ds4.axis(LX), 0, 65535, -32768, 32767);
            Ly = map(ds4.axis(LY), 0, 65535, -32768, 32767);
            Rx = map(ds4.axis(RX), 0, 65535, -32768, 32767);
            Ry = map(ds4.axis(RY), 0, 65535, -32768, 32767);
            if (checkForRoutines())
                return;
            else if (!(Rx > -AXIS_DEAD_ZONE && Rx < AXIS_DEAD_ZONE) || !(Ry > -AXIS_DEAD_ZONE && Ry < AXIS_DEAD_ZONE))
            {
                double theta = toDegree(atan((double)Ry / (double)Rx));
                Rx = map(Rx, -32768, 32767, -128, 127);
                Ry = map(Ry, -32768, 32767, -128, 127);
                unsigned int R = constrain(map2((double)((Rx * Rx) + (Ry * Ry)), 0.0, 16384.0, 0.0, 100.0), 0, 100);
                if (Ry >= 0)
                {
                    if (theta <= 0)
                        theta += 180.0;
                }
                else
                {
                    if (theta >= 0)
                        theta += 180.0;
                    else
                        theta += 360.0;
                }

                // debug_msg("Rx : " + String(Rx) + "\tRy : " + String(Ry) + "\tR : " + String(R) + "\tTheta : " + String(theta));
                if (R <= 25)
                {
                    rotational.SetTunings(0.03, 0.1, 0.025);
                    if (mode != 0)
                    {
                        rotational.resetIntegral();
                        mode = 0;
                    }
                }
                else //if (R <= 60)
                {
                    rotational.SetTunings(0.01325, 0.002, 0.01);
                    if (mode != 1)
                    {
                        rotational.resetIntegral();
                        mode = 1;
                    }
                }
                // else
                // {
                //     rotational.SetTunings(0.03, 0.135, 0.011);
                //     if (mode != 2)
                //     {
                //         rotational.resetIntegral();
                //         mode = 2;
                //     }
                // }
                bot.move(R, theta);
            }
            else if (ds4.button(TRIANGLE))
            {
                bot.move(30, 90);
            }
            else if (ds4.button(CROSS))
            {
                bot.move(30, 270);
            }
            else
                bot.stopAll();
        }
        else
            bot.stopAll();
    }
    // double data[7];
    // if (funcData(data))
    // {
    //     powerOn = data[0];
    //     if (lastPower == 0 && powerOn == 1)
    //         mright = !mright;
    //     digitalWrite(powerLed, powerOn);
    //     // linear.SetTunings(data[1], data[2], data[3]);
    //     rotational.SetTunings(data[4], data[5], data[6]);
    //     // Serial.println("Kp:" + String(data[4]) + "\tKi:" + String(data[5]) + "\tKd" + String(data[6]));
    //     if (powerOn)
    //         (mright) ? bot.move(70, 0) : bot.move(70, 180);
    //     else
    //         bot.stopAll(false);
    //     lastPower = powerOn;
    // }
}