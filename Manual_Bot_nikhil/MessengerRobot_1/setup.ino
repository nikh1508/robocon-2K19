void setup()
{
    Serial.begin(115200);
    Serial3.begin(115200);
    Wire.begin();

    bot.initialize();
    initializeBNO();
    resetEncoder();

    pinMode(powerLed, OUTPUT);

    debug_msg("Connecting to Controller");
    while (!ds4.readGamepad())
        ;
    debug_msg("Controller Connected");
}