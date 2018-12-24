void Serial3Flush()
{
    while (Serial3.available() > 0)
        char ch = Serial3.read();
}

void decodeData(byte toDecode[], byte totalRecvd, byte &decodedByteCount, byte decodedBytes[], byte specialByte)
{
    for (int i = 1; i < (totalRecvd - 1); i++)
    {
        byte x = toDecode[i];
        if (x == specialByte)
            x += toDecode[++i];
        decodedBytes[decodedByteCount++] = x;
    }
}

bool ReadBytes(byte data[], byte toRead, char toSend)
{
    static bool firstCall = true;
    static bool inProgress = false;
    static byte bytesRecvd = 0;
    static byte tempBuffer[34];
    static long long lastRead;
    byte specialByte = 253;
    byte startMarker = 254;
    byte endMarker = 255;

    if (firstCall)
    {
        firstCall = false;
        Serial3Flush();
        Serial3.print(toSend);
        lastRead = millis();
    }

    while (Serial3.available())
    {
        byte x = Serial3.read();
        //Serial.println(x);
        lastRead = millis();
        if (x == startMarker)
        {
            //Serial.println("SM Recvd");
            inProgress = true;
            bytesRecvd = 0;
        }
        if (inProgress)
            tempBuffer[bytesRecvd++] = x;
        if (x == endMarker)
        {
            //Serial.println("EM Recvd");
            inProgress = false;
            firstCall = true;
            byte decodedByteCount = 0;
            decodeData(tempBuffer, bytesRecvd, decodedByteCount, data, specialByte);
            if (decodedByteCount == toRead)
                return true;
            else
                Serial.println("Wrong Bytes Recvd" + String(decodedByteCount));
        }
    }
    if ((millis() - lastRead) > 50)
    {
        firstCall = true;
        Serial.println("TOO LATE");
    }
    return false;
}

void convert(byte toConvert[], unsigned int converted[], long &res)
{
    converted[0] = int(toConvert[0]);
    for (int i = 0; i < 6; i++)
    {
        converted[i + 1] = (int)toConvert[2 * i + 1] << 8 | ((int)toConvert[2 * i + 2]);
    }
    res = (long)toConvert[13] << 16 | (long)toConvert[14] << 8 | (long)toConvert[15];
}

bool funcData(double values[])
{
    double power, LKp, LKi, LKd, AKp, AKi, AKd;
    unsigned int data[7] = {0};
    long resolution = 0;
    byte recv[16];

    if (ReadBytes(recv, 16, 'x'))
    {
        convert(recv, data, resolution);
        power = values[0] = (double)data[0];
        LKp = values[1] = (double)data[1] / (double)resolution;
        LKi = values[2] = (double)data[2] / (double)resolution;
        LKd = values[3] = (double)data[3] / (double)resolution;
        AKp = values[4] = (double)data[4] / (double)resolution;
        AKi = values[5] = (double)data[5] / (double)resolution;
        AKd = values[6] = (double)data[6] / (double)resolution;

        // Serial.println("LKp: " + (String)LKp + "\tLKi: " + (String)LKi + "\tLKd: " + (String)LKd + "\tAKp: " + (String)AKp + "\tAKi: " + (String)AKi + "\tAKd: " + (String)AKd);
        return true;
    }
    return false;
}
