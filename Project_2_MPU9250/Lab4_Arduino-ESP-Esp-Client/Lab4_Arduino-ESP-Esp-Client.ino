const int incomingPin = D5; 

void setup() {
  Serial.begin(115200);     
  pinMode(receivePin, INPUT);
}

void loop() {
  int signalState = digitalRead(incomingPin); 
  if (signalState == HIGH) {
    Serial.println("AMBATU");
  } else {
    Serial.println("bus");
  }
  delay(500);
}
