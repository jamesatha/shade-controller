//# define BLUE_BUILT_IN_LED 2  
# define EN_BUTTON_PIN 3
# define BOOT_BUTTON_PIN 0
# define TOP_MOTOR_CONTROL_PIN_MUST_BE_PULL_DOWN 2
#include <mutex>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo360.h>
#include <ArduinoJson.h>

// This file specifies wifi creds. Ex:
// #define WIFI_SSID "REPLACE WITH WIFI NAME"
// #define WIFI_PASSWORD "REPLACE WITH WIFI PASSWORD"
#include "./wifi-info.h"

std::mutex change_request_mutex; 

typedef enum {
    MOTOR_OK,
    MOTOR_UNKNOWN,
    MOTOR_STALLED
} MotorState;

// Create a servo object
ESP32Servo360 topMotor;
MotorState topMotorStatus = MOTOR_UNKNOWN;

// Other status stuff
bool wifiOkStatus = false;
bool ntpOkStatus = false;



// Create an instance of the server
AsyncWebServer server(80);

void prepareTopMotor() {
  topMotor.attach(TOP_MOTOR_CONTROL_PIN_MUST_BE_PULL_DOWN, 33);
  topMotor.stop();
  //topMotor.calibrate(); // Minimal PWM: 36, maximal PWM: 1075
  topMotor.setMinimalForce(12);
  topMotor.setSpeed(10);
  
  //topMotor.setAdditionalTorque(200); // unsure about this
  topMotor.setOffset(topMotor.getAngle());
}

int topStepsInRecentCommand = 0;
int topDeltaDegrees = 0;
float topTarget = 0;
unsigned long topMotorWait = 1000; // 
unsigned long prevTopMotorCommand = millis() - topMotorWait;
void setup() {
  prepareTopMotor(); // DO THIS FIRST TO STOP MOTOR FROM SPINNING ON BOOT
  Serial.begin(9600);
  delay(10);
  //pinMode(BLUE_BUILT_IN_LED, OUTPUT);  // Setting the blue LED to be an output that we can set
  //digitalWrite(BLUE_BUILT_IN_LED, LOW); // Turn off the blue light so we know we are not ready

  // Still not really being used
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // Set the BOOT button pin as input with pullup resistor
  delay(10);

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
    if (topDeltaDegrees == 0 && millis() - prevTopMotorCommand >= topMotorWait) {
      if (request->hasParam("speed")) {
        int speed = request->getParam("speed")->value().toInt();
        if (speed > 90) {
          speed = 90;
        }
        Serial.print("Setting speed to ");
        Serial.println(speed);
        topMotor.setSpeed(speed);
      }
      if (request->hasParam("rotations")) {
        int rotations = request->getParam("rotations")->value().toInt();
        Serial.print("Spinning ");
        Serial.print(rotations);
        Serial.println(" degrees");
        topDeltaDegrees = rotations;
        topTarget = topMotor.getAngle() + topDeltaDegrees;
        topStepsInRecentCommand = 0;
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
      case MOTOR_OK:
        doc["motor"] = "ok";
        break;
      case MOTOR_UNKNOWN:
        doc["motor"] = "unknown";
        break;
      case MOTOR_STALLED:
        doc["motor"] = "stalled";
        break;
      default:
        doc["motor"] = "compiler error";
    }
    
    doc["angle"] = topMotor.getAngle();
    doc["motor_busy"] = topDeltaDegrees != 0 || millis() - prevTopMotorCommand < topMotorWait;

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


# define BUFFER 5
# define INCREMENTAL_ANGLE 180
# define EXPECTED_DELTA_PER_STEP 100 // ok with 70
int prevRemaining = 0;
void continueTopMotorRotation() {
  if (topDeltaDegrees != 0 && millis() - prevTopMotorCommand >= topMotorWait && topMotorStatus != MOTOR_STALLED) {
    printTopMotorStuff();
    float curAngle = topMotor.getAngle();
    float remaining = topTarget - curAngle;
    if (topDeltaDegrees > 0 && curAngle < topTarget - BUFFER) {
      topStepsInRecentCommand++;
      if (topStepsInRecentCommand > 1) {
        if (prevRemaining < EXPECTED_DELTA_PER_STEP + remaining) {
          topMotorStatus = MOTOR_STALLED;
          Serial.println("Stalled on the way up");
          topMotor.stop();
        } else {
          topMotorStatus = MOTOR_OK;
        }
      }
      Serial.print("Diff to target: ");
      Serial.println(topTarget - curAngle);
      //Serial.println(curAngle + INCREMENTAL_ANGLE > topTarget ? topTarget : curAngle + INCREMENTAL_ANGLE);
      topMotor.easeRotateTo(curAngle + INCREMENTAL_ANGLE > topTarget ? topTarget : curAngle + INCREMENTAL_ANGLE);
      
    } else if (topDeltaDegrees < 0 && curAngle > topTarget + BUFFER) {
      topStepsInRecentCommand++;

      if (topStepsInRecentCommand > 1) {
        if (prevRemaining + EXPECTED_DELTA_PER_STEP > remaining) {
          topMotorStatus = MOTOR_STALLED;
          Serial.println("Stalled on the way down");
          topMotor.stop();
        } else {
          topMotorStatus = MOTOR_OK;
        }
      }

      Serial.print("Easing down to: ");
      Serial.println(curAngle - INCREMENTAL_ANGLE < topTarget ? topTarget : curAngle - INCREMENTAL_ANGLE);
      topMotor.easeRotateTo(curAngle - INCREMENTAL_ANGLE < topTarget ? topTarget : curAngle - INCREMENTAL_ANGLE);
    } else {
      topDeltaDegrees = 0;
      Serial.println("Ending rotation");
    }
    prevRemaining = remaining;
    prevTopMotorCommand = millis();
  }
}

void printTopMotorStuff() {
    Serial.print("Angle: ");
    Serial.print(topMotor.getAngle());

    Serial.print(" - Orientation: ");
    Serial.print(topMotor.getOrientation());

    Serial.print(" - Busy: ");
    Serial.print(topMotor.busy());

    Serial.print(" - Speed: ");
    Serial.print(topMotor.getSpeed());

    Serial.print(" - Turns: ");
    Serial.println(topMotor.getTurns());
}

int prevBootButtonState = HIGH;
void loop() {
  checkWifiStatus();
  checkNtpStatus();
  continueTopMotorRotation();

  int bootButtonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the BOOT button
  if (prevBootButtonState != bootButtonState) {
    // Boot button change
    if (bootButtonState == LOW) {  // If the BOOT button is pressed for first time (logic is reversed due to pullup resistor)
      // Do something when the BOOT button is pressed
      //topMotor.rotate(180);
      printTopMotorStuff();
      //checkTopMotor();
      
    } else {
      
    }
    prevBootButtonState = bootButtonState;
  }
  
/*
  if (wifiOkStatus && ntpOkStatus && topMotorStatus != MOTOR_STALLED) {
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  } else {
    Serial.println("Something is wrong");
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  }
  */
  delay(100); // Power saving mechanism to slow down the main loop
}
