int dir_1=4;
int dir_2=5;
int pwm=6;
int enc_a=2;
int enc_b=3;
int speed=10;
long count=0;
void setup() {
  // put your setup code here, to run once:
pinMode(dir_1,OUTPUT);
pinMode(dir_2,OUTPUT);
pinMode(pwm,OUTPUT);
pinMode(enc_a,INPUT);
pinMode(enc_b,INPUT);
pinMode(10,INPUT);
attachInterrupt(digitalPinToInterrupt(2),isr,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

    while(millis()<300000)
    {
      if(millis()%30000==0)
      {
        speed+=50;
      }
      analogWrite(6,speed);
      Serial.print("PWM::\t" + String(speed) + "\t");
      Serial.print("PHOTO::\t" + String(digitalRead(10)) + "\n") ;
    }
    
}

void isr()
{
   if(digitalRead(2)==digitalRead(3))
  count++;
  else 
  count--;

  Serial.print("\tENC::\t" +String(count));
}
