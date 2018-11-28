void setup()
{
    Serial.begin(115200);
    Wire.begin();
    initializeBNO();
}