void update_data() {

  
} 

void print_data()
{
  Serial.print("T:"+String(DS4.buttonPressed(TRIANGLE)));
  Serial.print("\t");
  Serial.print("S:"+String(DS4.buttonPressed(SQUARE)));
  Serial.print("\t");
  Serial.print("CR:"+String(DS4.buttonPressed(CROSS)));
  Serial.print("\t");
  Serial.println("C:"+String(DS4.buttonPressed(CIRCLE)));
}

