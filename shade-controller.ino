# define BLUE_BUILT_IN_LED 2  
# define EN_BUTTON_PIN 3
# define BOOT_BUTTON_PIN 0

#define STEPPER_STEPS 200 // Change this according to your stepper motor specs (usually 200 steps for NEMA 17)

// unsure if that needs to be on a PULL_DOWN (2)
#define DIR_PIN 13
#define STEP_PIN 12

#include <mutex>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
//#include <Stepper.h>
//Stepper stepper(200, STEP_PIN, DIR_PIN);

// This file specifies wifi creds. Ex:
// #define WIFI_SSID "REPLACE WITH WIFI NAME"
// #define WIFI_PASSWORD "REPLACE WITH WIFI PASSWORD"
#include "./wifi-info.h"

std::mutex change_request_mutex; 

typedef enum {
    MOTOR_UNKNOWN,
    MOTOR_MOVING,
    MOTOR_AT_TOP,
    MOTOR_AT_BOTTOM
} MotorState;

MotorState topMotorStatus = MOTOR_UNKNOWN;

// Other status stuff
bool wifiOkStatus = false;
bool ntpOkStatus = false;



// Create an instance of the server
AsyncWebServer server(80);


void prepareTopMotor() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  //stepper.setSpeed(10);
}

void prepareStatusLED() {
  pinMode(BLUE_BUILT_IN_LED, OUTPUT);  // Setting the blue LED to be an output that we can set
  digitalWrite(BLUE_BUILT_IN_LED, LOW); // Turn off the blue light so we know we are not ready
}

void updateStatusLED() {
  if (wifiOkStatus && ntpOkStatus) {
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  } else {
    Serial.println("Something is wrong");
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  }
}

void setup() {
  prepareTopMotor(); // DO THIS FIRST TO STOP MOTOR FROM SPINNING ON BOOT
  prepareStatusLED();

  Serial.begin(9600);
  delay(10);

  // Still not really being used
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // Set the BOOT button pin as input with pullup resistor

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
    std::lock_guard<std::mutex> lck(change_request_mutex);
    if (topMotorStatus != MOTOR_MOVING) {
      if (request->hasParam("speed")) {
        int speed = request->getParam("speed")->value().toInt();
        if (speed > 240) {
          speed = 240;
        }
        Serial.print("Setting speed to ");
        Serial.println(speed);
      }
      if (request->hasParam("rotations")) {
        int rotations = request->getParam("rotations")->value().toInt();
        Serial.print("Spinning ");
        Serial.print(rotations);
        Serial.println(" degrees");
        
      }
      
      request->send(200, "text/plain", "YES!"); // check type
    } else {
      request->send(413, "text/plain", "WAIT!");
    }
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(1024);

    doc["wifi"] = wifiOkStatus;
    doc["ntp"] = ntpOkStatus;
    switch (topMotorStatus) {
      case MOTOR_UNKNOWN:
        doc["motor"] = "unknown";
        break;
      case MOTOR_AT_BOTTOM:
        doc["motor"] = "bottom";
        break;
      case MOTOR_AT_TOP:
        doc["motor"] = "top";
        break;
      case MOTOR_MOVING:
        doc["motor"] = "moving";
        break;
      default:
        doc["motor"] = "compiler error";
    }
    
    String jsonString;
    serializeJsonPretty(doc, jsonString); // switch method to serializeJson eventually
    
    request->send(200, "application/json", jsonString);
  });

  server.begin();
  
}

unsigned long previousWifiCheckMillis = millis();
unsigned long wifiCheckInterval = 30 * 1000; // 30 seconds

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


void manualRotate() {
  digitalWrite(DIR_PIN, HIGH); // Set the direction
  for(int i = 0; i < STEPPER_STEPS*16; i++) {
    digitalWrite(STEP_PIN, HIGH); // Make one step
    delayMicroseconds(100); // Change this delay as needed
    digitalWrite(STEP_PIN, LOW); // Reset step pin
    delayMicroseconds(100); // Change this delay as needed
  }
}

int prevBootButtonState = HIGH;
void loop() {
  checkWifiStatus();
  checkNtpStatus();

  int bootButtonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the BOOT button
  if (prevBootButtonState != bootButtonState) {
    // Boot button change
    if (bootButtonState == LOW) {  // If the BOOT button is pressed for first time (logic is reversed due to pullup resistor)
      Serial.println("Starting");
      manualRotate();
      
      
      
      Serial.println("Done");
    } else {
      
    }
    prevBootButtonState = bootButtonState;
  }
  

  updateStatusLED();
  
  delay(100); // Power saving mechanism to slow down the main loop
}
