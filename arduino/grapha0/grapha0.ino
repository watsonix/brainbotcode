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
  Serial.println(reading);
  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(2);
}
