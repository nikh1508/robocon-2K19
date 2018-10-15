void loop()
{
  //Serial.print("flag:" +String(flag));
  //Serial.print("\tflag1:" +String(flag1));
  if (debug)
  {
   if(DS4.readGamepad())
      print_data();

  }
  else if(DS4.readGamepad())
  {
   if (DS4.buttonPressed(UP))
  {
    Serial.println("Moving Forward");
    front();
    
    flag = 1;
  }
  else if (DS4.buttonPressed(DOWN))
  {
    Serial.println("Moving Back");
    back();
    flag = 2;
  }
  else if (DS4.buttonPressed(HAT_LEFT))
  {
    Serial.println("Moving Left");
    left();
    flag = 3;
  }
  else if (DS4.buttonPressed(HAT_RIGHT))
  {
    Serial.println("Moving Right");
    right();
    flag = 4;
  }
  else if(DS4.button(SQUARE) && DS4.buttonPressed(R1))
  {Kp=Kp+0.25;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("KP increased by one " + String(Kp));}
  
  else if(DS4.button(TRIANGLE) && DS4.buttonPressed(R1))
  {Ki=Ki+0.05;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("Ki increased by .2 " + String(Ki));}
  
  else if(DS4.button(CIRCLE) && DS4.buttonPressed(R1))
  {Kd=Kd+0.05;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("Kd increased by .2 " + String(Kd));}
   else if(DS4.button(SQUARE) && DS4.buttonPressed(L1))
  {Kp=Kp-0.25;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("KP decreased by one " + String(Kp));}
  
  else if(DS4.button(TRIANGLE) && DS4.buttonPressed(L1))
  {Ki=Ki-0.05;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("Ki decreased by .2 " + String(Ki));}
  
  else if(DS4.button(CIRCLE) && DS4.buttonPressed(L1))
  {Kd=Kd-0.05;
  myPID.SetTunings(Kp,Ki,Kd);
  Serial.println("Kd decreased by .2 " + String(Kd));}
  }
  else if(DS4.buttonPressed(SQUARE))
  {
    encoder_routine();
  }

  

  switch (flag)
  {
    case 1:
      stop_front();
      flag=0;
      flag1=0;
      break;

    case 2:
      stop_back();
      flag=0;
      flag1=0;
      break;

    case 3:
      stop_left();
      flag=0;
      flag1=0;
      break;

    case 4:
        stop_right();
        flag=0;
      flag1=0;
      break;

    default:
      stop_all();
      break;


  }



}
