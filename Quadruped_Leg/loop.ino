void loop()
{
    leg0.debugComp(true, true);
    leg0.writeMotor(HIP, -20);

    // leg0.autoHome();
    // if (Serial.available())
    // {
    //     char ch = Serial.read();
    //     leg0.autoHome();
    // }
}