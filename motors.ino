void write_motor(int f, int l, int r)
{
  //r=r+accel*(setvalue-r);
  //l=l+accel*(setvalue-l);
  Serial.print("L::" + String(l));
  Serial.print("\tR::" + String(r));
  digitalWrite(dir1_f, (f>0)?1:0);
  digitalWrite(dir2_f, (f>0)?0:1);

  digitalWrite(dir2_r,  (r>0)?0:1 );
  digitalWrite(dir1_r,  (r>0)?1:0);


  digitalWrite(dir1_l,  (l>0)?1:0);
  digitalWrite(dir2_l, (l>0)?0:1);

  analogWrite(pwm_f, abs(f));
  analogWrite(pwm_l, abs(l));
  analogWrite(pwm_r, abs(r));

}

void stop_all()
{
  digitalWrite(dir1_f, 1);
  digitalWrite(dir2_f, 1);

  digitalWrite(dir1_r, 1);
  digitalWrite(dir2_r, 1);


  digitalWrite(dir1_l, 1);
  digitalWrite(dir2_l, 1);

  analogWrite(pwm_f, 0);
  analogWrite(pwm_l, 0);
  analogWrite(pwm_r, 0);

}

void stop_back()
{
  Serial.println("Stopping");
  
  i = speed_r;
  while ((speed_l != 0 && speed_r != 0))
{
  input = get_angle('x');
    myPID.Compute();
    speed_l = int(i);
    speed_r = int (i - output);
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, -speed_l, -speed_r);
    i = i - 0.5;

  }
  stop_all();
 
  speed_l=0;
  speed_r=0;
}

void stop_front()
{

Serial.println("Stopping");
  i = speed_r;
  while ((speed_l != 0 && speed_r != 0))
{
  input = get_angle('x');
    myPID.Compute();
    speed_l = int(i);
    speed_r = int (i + output);
    speed_r = constrain(speed_r, 0, 255);

    write_motor(0, speed_l, speed_r);
    i = i - 0.5;

  }
  stop_all();
  //setpoint=get_angle('x');
  speed_l=0;
  speed_r=0;
}

void stop_left()
{
  Serial.println("Stopping");
  stop_all();
  speed_l=0;
  speed_r=0;
}
void stop_right()
{
  Serial.println("Stopping");
  stop_all();
  speed_l=0;
  speed_r=0;
}
