void setup() {
  pinMode(8, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // Read voltage from resistor
  // float voltage = analogRead(A0) * (5.0 / 1023.0);
  int state = digitalRead(8);
  // If estop is closed -> send stop cmd
  Serial.println(state);
  delay(500);
}
