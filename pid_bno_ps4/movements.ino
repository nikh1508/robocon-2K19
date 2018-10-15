void front()
{
   setpoint=get_angle('x');
  Serial.println("here");
  while (speed_l != setvalue && speed_r != setvalue)
  {
    input = get_angle('x');
    myPID.Compute();
    speed_l = int(i);
    speed_r = int (i -output);
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, speed_l, speed_r);
    i = i + 0.5;

  }
  do
  {
    // update_data();
    if (DS4.readGamepad())
    { if (DS4.buttonPressed(R2))
        flag1 = 1;
    }
    input = get_angle('x');
    myPID.Compute();
    speed_r = setvalue - output;
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, speed_l, speed_r);
    Serial.print("\tAngle::" + String(get_angle('x')));
    Serial.print("\tSet::" + String(setpoint));
    Serial.println("\tOutput::" + String(output));
  } while (flag1 != 1);
}

void back()
{
   setpoint=get_angle('x');
  while (speed_l != setvalue && speed_r != setvalue)
  {
    input = get_angle('x');
    myPID.Compute();
    speed_l = int(i);
    speed_r = int (i + output);
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, -speed_l, -speed_r);
    i = i + 0.5;

  }
  do
  {
    // update_data();
    if (DS4.readGamepad())
   
 { if (DS4.buttonPressed(R2))
        flag1 = 1;
    }
    input = get_angle('x');
    myPID.Compute();
    speed_r = setvalue + output;
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, -speed_l, -speed_r);
     Serial.print("\tAngle::" + String(get_angle('x')));
    Serial.print("\tSet::" + String(setpoint));
    Serial.println("\tOutput::" + String(output));
  } while (flag1 != 1);
}


void left()
{
   setpoint=get_angle('x');
  speed_l = 15;
  speed_r = 15;
  speed_f = 30;
  do
  {
    //update_data();
    if (DS4.readGamepad())
       { if (DS4.buttonPressed(R2))
        flag1 = 1;
    }
    input = get_angle('x');
    myPID.Compute();
    speed_f = setvalue - output;
    speed_f = constrain(speed_f, 0, 255);

    write_motor(speed_f, speed_l, -speed_r);
    Serial.print("\tAngle::" + String(get_angle('x')));
    Serial.print("\tSet::" + String(setpoint));
    Serial.println("\tOutput::" + String(output));
  } while (flag1 != 1);
}

void right()
{
   setpoint=get_angle('x');
  speed_l = 15;
  speed_r = 15;
  speed_f = 30;
  do
  {
    // update_data();
    if (DS4.readGamepad())
      { if (DS4.buttonPressed(R2))
        flag1 = 1;
    }
    input = get_angle('x');
    myPID.Compute();
    speed_f = setvalue + output;
    speed_f = constrain(speed_f, 0, 255);

    write_motor(speed_f, speed_l, -speed_r);
          Serial.print("\tAngle::" + String(get_angle('x')));
          Serial.print("\tSet::" + String(setpoint));
          Serial.println("\tOutput::" + String(output));
  } while (flag1 != 1);
}

void encoder_routine()
{
}
