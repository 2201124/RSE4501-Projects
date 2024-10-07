#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <UrlEncode.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


int lastSignalState = LOW;
int signalState = LOW;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 28800, 60000); // 0 timezone offset, sync every 60 seconds


void setup() {
  Serial.begin(115200);
  pinMode(incomingPin, INPUT);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  timeClient.update(); // Update the time client to get the current time
}


void loop() {
  int reading = digitalRead(incomingPin);
  timeClient.update();

  static bool fallDetected = false;
  static unsigned long lastCheckTime = 0;

  if (reading != lastSignalState) {
    signalState = reading;

    if (signalState == HIGH) {
      String timestamp = timeClient.getFormattedTime();
      Serial.println("Fall Detected at " + timestamp);
      // sendMessage("Fall Detected at " + timestamp);
      fallDetected = true;
      lastCheckTime = millis();  // Start tracking time
    } else {
      fallDetected = false;  // Reset if LOW signal is detected
    }
  }

  // If a fall has been detected, check every 15 seconds if the person is still on the floor
  if (fallDetected && (millis() - lastCheckTime >= 15000)) {
    String timestamp = timeClient.getFormattedTime();
    if (digitalRead(incomingPin) == HIGH) {
      Serial.println("Person is still on the floor at: " + timestamp);
      // sendMessage("Person is still on the floor at: " + timestamp);
      // Update the last check time for the next 5-second interval
      lastCheckTime = millis();
    } else {
      // If the signal goes LOW, reset fall detection
      fallDetected = false;
    }
  }

  lastSignalState = reading;
}




void sendMessage(String message){

  // Data to send with HTTP POST
  String url = "http://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);
  WiFiClient client;    
  HTTPClient http;
  http.begin(client, url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200){
    Serial.print("Message sent successfully");
  }
  else{
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}