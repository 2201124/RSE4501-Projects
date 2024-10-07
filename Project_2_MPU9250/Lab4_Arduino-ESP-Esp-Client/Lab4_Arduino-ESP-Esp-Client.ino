const int incomingPin = D5; 
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int lastSignalState = LOW;
int signalState = LOW;

void setup() {
  Serial.begin(115200);
  pinMode(incomingPin, INPUT);
}

void loop() {
  int reading = digitalRead(incomingPin);

  if (reading != lastSignalState) {
    lastDebounceTime = millis();  // Reset the debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != signalState) {
      signalState = reading;

      if (signalState == HIGH) {
        Serial.println("Received HIGH signal from Arduino!");
      } else {
        Serial.println("Received LOW signal from Arduino.");
      }
    }
  }

  lastSignalState = reading;
}
