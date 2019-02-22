bool autoHome = false;

void loop()
{
    // leg0.debugComp(true, true);
    // leg0.writeMotor(HIP, -10);
    // leg0.writeMotor(KNEE, -10);
    if (autoHome)
        leg0.run();
    if (Serial.available())
    {
        char ch = Serial.read();
        if (ch == 'a')
        {
            leg0.autoHome();
            Serial.println("////////////////Auto Home Complete//////////////////");
            autoHome = true;
        }
        else if (ch == 'h')
        {
            int value = Serial.parseInt();
            value = constrain(value, -150, 0);
            Serial.println("HIP::" + String(value));
            leg0.set(HIP, value);
        }
        else if (ch == 'k')
        {
            int value = Serial.parseInt();
            value = constrain(value, -200, 0);
            Serial.println("KNEE::" + String(value));
            leg0.set(KNEE, value);
        }
    }
}