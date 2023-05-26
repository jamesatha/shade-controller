# define BLUE_BUILT_IN_LED 2  
# define EN_BUTTON_PIN 3
# define BOOT_BUTTON_PIN 0

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo360.h>

// This file specifies wifi creds. Ex:
// #define WIFI_SSID "REPLACE WITH WIFI NAME"
// #define WIFI_PASSWORD "REPLACE WITH WIFI PASSWORD"
#include "./wifi-info.h"

// Servo pin
const int servoPin = 14; // Use a PWM-capable pin on your ESP32 (move to header config?)

// Create a servo object
ESP32Servo360 topMotor;

// Create an instance of the server
AsyncWebServer server(80);

void prepareTopMotor() {
  topMotor.attach(32, 33);
  //topMotor.calibrate();
}

void setup() {
  Serial.begin(9600);
  delay(20);
  pinMode(BLUE_BUILT_IN_LED, OUTPUT);  // Setting the blue LED to be an output that we can set
  digitalWrite(BLUE_BUILT_IN_LED, LOW); // Turn off the blue light so we know we are not ready

  // Still not really being used
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // Set the BOOT button pin as input with pullup resistor
  delay(1000);

  // Connect to Wi-Fi (attempt to wait 20 seconds for wifi to come up)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    delay(2000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //server = AsyncWebServer(80);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "YES!"); // check type
  });
  server.begin();
  
  prepareTopMotor();
}

unsigned long previousWifiCheckMillis = millis();
unsigned long wifiCheckInterval = 30 * 1000; // 30 seconds
bool wifiOkStatus = false;

void checkWifiStatus() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((currentMillis - previousWifiCheckMillis >= wifiCheckInterval) && (WiFi.status() != WL_CONNECTED)) {
    Serial.print(currentMillis);
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousWifiCheckMillis = currentMillis;
  }
  wifiOkStatus = (WiFi.status() == WL_CONNECTED);
}

unsigned long ntpCheckInterval = 60*60*1000; // 1 hour
unsigned long previousNtpCheckMillis = millis() - ntpCheckInterval;
bool ntpOkStatus = false;

void checkNtpStatus() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousNtpCheckMillis >= ntpCheckInterval) {
    ntpOkStatus = false;
    configTime(-8 * 60 * 60 /*PST*/, 60 * 60/*1 hour for PDT*/, "time.google.com", "time.nist.gov", "pool.ntp.org"); // MOVE TO CONFIG
    
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    } else {
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      ntpOkStatus = true;
    }
    previousNtpCheckMillis = currentMillis;
  }
}
void loop() {
  checkWifiStatus();
  checkNtpStatus();
  //int enButtonState = digitalRead(EN_BUTTON_PIN);  // Read the state of the button
  //Serial.println(buttonState);
  //if (enButtonState == LOW) {  // If the button is pressed (logic is reversed due to pullup resistor)
    // Do something when the button is pressed
  //  Serial.println("Button pressed!");
  //  
  //}

  int bootButtonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the BOOT button
  if (bootButtonState == LOW) {  // If the BOOT button is pressed (logic is reversed due to pullup resistor)
    // Do something when the BOOT button is pressed
    Serial.println("BOOT button pressed!");
  }

  if (wifiOkStatus && ntpOkStatus) {
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  } else {
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  }
  delay(1000); // Power saving mechanism to slow down the loop to about once a second
}
