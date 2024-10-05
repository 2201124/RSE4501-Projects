#define NUM_SENSORS 2
#define ultra_short_delay 30
#define short_delay 100
#define medium_delay 500
#define long_delay 1000

int trig_pins[NUM_SENSORS] = {20, 19};  // Trig pins for the sensors
int echo_pins[NUM_SENSORS] = {21, 18};  // Echo pins for the sensors
int led_pins[NUM_SENSORS] = {10, 9};    // LED pins for each sensor

float distances[NUM_SENSORS];
unsigned long previousMillis[NUM_SENSORS] = {0, 0};  // Last LED update time
int ledInterval[NUM_SENSORS] = {0, 0};                // LED delay times
bool ledState[NUM_SENSORS] = {LOW, LOW};              // LED states

// Add calibration and error variables
float calibration = 1 / 0.972154;   // Example calibration factor
float error = 0.312267;             // Example error correction value

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trig_pins[i], OUTPUT);
    pinMode(echo_pins[i], INPUT);
    pinMode(led_pins[i], OUTPUT);
  }
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    measureDistance(i);  // Measure distance
    Serial.print(i == 0 ? "Right" : "Left ");
    Serial.print(" Sensor Distance: ");
    Serial.print(distances[i]);
    Serial.println(" cm");
    handleLED(i);        // Control LEDs based on distance
  }
  delay(100);  // Small delay for serial monitor
}

void measureDistance(int i) {
  digitalWrite(trig_pins[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pins[i], LOW);

  long duration = pulseIn(echo_pins[i], HIGH);

  // Do calibration and error compensation
  distances[i] = ((duration * 0.0343) * calibration - error) / 2;
}

void handleLED(int i) {
  // Set LED interval based on distance
  if (distances[i] > 100) {
    ledInterval[i] = 0;  // Turn off LED if over 100 cm
  } else if (distances[i] > 50) {
    ledInterval[i] = long_delay;
  } else if (distances[i] > 5) {
    ledInterval[i] = medium_delay;
  } else {
    ledInterval[i] = ultra_short_delay;
  }

  // Toggle LED state based on timing
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis[i] >= ledInterval[i]) {
    previousMillis[i] = currentMillis;
    ledState[i] = ledInterval[i] > 0 ? !ledState[i] : LOW;  // Toggle state if interval > 0
    digitalWrite(led_pins[i], ledState[i]);  // Update LED state
  }
}
