void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
}

  int max = 0;
  int min = 1000;
  unsigned long time = millis();
void loop() {
  // send the value of analog input 0:
  int reading = analogRead(A0);
  //Serial.println(reading);
  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(2);
  Serial.println(reading);
  /*
  //store max for 1 second
  if (reading > max)
  {
    //Serial.println(reading);
    max = reading;
  }
    if (reading < min)
  {
    //Serial.println(reading);
    min = reading;
  }
  //reset values for max and min
  if (millis() > (time + 1000))
  {
    Serial.println(max);
    Serial.println(min);
    max = 0;
    min = 1000;
    time = millis();
  }
*/
}
