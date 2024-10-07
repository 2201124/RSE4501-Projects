const int testPin = 14;  // Pin sending signal to ESP8266

void setup() {
  pinMode(testPin, OUTPUT);
}

void loop() {
  Serial.println("Checking if anything works");
  digitalWrite(testPin, HIGH);   // Send HIGH signal to ESP8266
  delay(1000);               
  digitalWrite(testPin, LOW);    // Send LOW signal to ESP8266
  delay(1000);                   
}
