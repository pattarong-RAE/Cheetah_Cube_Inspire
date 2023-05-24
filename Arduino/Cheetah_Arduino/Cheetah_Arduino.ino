const int LED_PIN = 13; // Pin for the LED

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    Serial.print("Received data: ");
    Serial.println(data);
    digitalWrite(18, HIGH); // Turn on the LED
    delay(1000); // Wait for one second
    Serial.println(digitalRead(19)); // Turn off the LED
  }
}