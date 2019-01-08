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

void forestRoutine() {};

void testRoutine () 
{
	static Coordinate pos, next;
	const int VEL = 50;
	const float maxX = 35000.0f;
  const float change=14800.0f;
	float _tan = 0.0f;
	int _vel = 0;
	float A = 7300.0; 	
	float B = 0.00038;	//0.000275
	float shift = 0.85f;

	getCoordinates(pos);


	if(next.x < maxX) 
	{
		_tan = toDegree(atan(A*B*cos(B*next.x))) + 90;
		if(pos.x >= next.x)
		{
			_vel = VEL;
			next.x += 100;
			next.y = A*sin(B*(next.x+shift));
		}
		else
		{
			 _vel = VEL;
		}
	}
 else if(next.x<maxX)
 {
  A=7600.0;
  B=0.00038;
  shift=0.85f;
  _tan = toDegree(atan(A*B*cos(B*next.x))) + 90;
    if(pos.x >= next.x)
    {
      _vel = VEL;
      next.x += 100;
      next.y = A*sin(B*(next.x+shift));
    }
    else
    {
       _vel = VEL;
    }
 }
	else
	{
		_vel = 0;
	}

	Serial.println("N.X: " + String(next.x) + "N.Y: " + String(next.y) + "\tVEL: " + String(_vel) + "\tTAN: " + String(_tan) + "\tX: " + String(pos.x) + "\tY: " + String(pos.y));
	bot.move(_vel, _tan);
}
