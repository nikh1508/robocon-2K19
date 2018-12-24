uint8_t routineCounter = 0;
bool checkForRoutines()
{
    if (ds4.buttonPressed(UP) && routineCounter == 0)
    {
        forestRoutine();
        routineCounter++;
        return true;
    }
    return false;
}

void forestRoutine() {}