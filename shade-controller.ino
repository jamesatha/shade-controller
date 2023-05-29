# define BLUE_BUILT_IN_LED 2  
# define EN_BUTTON_PIN 3
# define BOOT_BUTTON_PIN 0
# define TOP_MOTOR_CONTROL_PIN_MUST_BE_PULL_DOWN 2
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo360.h>
#include <ArduinoJson.h>

// This file specifies wifi creds. Ex:
// #define WIFI_SSID "REPLACE WITH WIFI NAME"
// #define WIFI_PASSWORD "REPLACE WITH WIFI PASSWORD"
#include "./wifi-info.h"

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

int topRotations = 0;

void setup() {
  prepareTopMotor(); // DO THIS FIRST TO STOP MOTOR FROM SPINNING ON BOOT
  Serial.begin(9600);
  delay(10);
  pinMode(BLUE_BUILT_IN_LED, OUTPUT);  // Setting the blue LED to be an output that we can set
  digitalWrite(BLUE_BUILT_IN_LED, LOW); // Turn off the blue light so we know we are not ready

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
    if (request->hasParam("speed")) {
      int speed = request->getParam("speed")->value().toInt();
      if (speed > 40) {
        speed = 40;
      }
      Serial.print("Setting speed to ");
      Serial.println(speed);
      topMotor.setSpeed(speed);
    }
    if (request->hasParam("rotations")) {
      int rotations = request->getParam("rotations")->value().toInt();
      Serial.print("Spinning ");
      Serial.print(rotations);
      Serial.println(" times");
      topRotations = rotations;
    } else {
      Serial.println("Resetting motor to 0");
      topMotor.rotate(0);
    }
    
    request->send(200, "text/plain", "YES!"); // check type
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
    doc["motor_busy"] = topMotor.busy();

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

const float angleDeltaPerStep = 10;
const float START_VALUE = -999999999.99999;
float prevAngle = START_VALUE;
bool checkTopMotor() {
  return true; //  remove
  float curAngle = topMotor.getAngle();
  Serial.print("Prev: ");
  Serial.print(prevAngle);
  Serial.print(" - cur: ");
  Serial.println(curAngle);
  if (topMotorStatus == MOTOR_STALLED) {
    Serial.println("Stalled so not moving any more");
    topRotations = 0; // Stop future spinning
    return false; // We need to wait for a reset
  } else if (topMotorStatus == MOTOR_UNKNOWN || prevAngle == START_VALUE) {
    // nothing
    topMotorStatus = MOTOR_OK;
  } else if (topMotorStatus == MOTOR_OK) {
    if (topRotations > 0) {
      Serial.print("delta: ");
      Serial.println(curAngle - prevAngle);
      if (curAngle > prevAngle + angleDeltaPerStep*0.75) {
        // we made 75% progress so probably ok
        topMotorStatus = MOTOR_OK;
      } else {
        topMotorStatus = MOTOR_STALLED;
        Serial.println("STALLED FOR check 1");
        topRotations = 0; // Stop future spinning
        return false;
      }
    } else if (topRotations < 0) {
      if (curAngle < prevAngle - angleDeltaPerStep*0.75) {
        // we made 75% progress so probably ok
        topMotorStatus = MOTOR_OK;
      } else {
        topMotorStatus = MOTOR_STALLED;
        Serial.println("STALLED FOR check 2");
        topRotations = 0; // Stop future spinning
        return false;
      }
    }
  } else {
    Serial.println("uh oh");
    return 1 / 0 == 1;
  }
  prevAngle = curAngle;
  //topMotor.setOffset(curAngle);
}

unsigned long topMotorWait = 400; // 
unsigned long prevTopMotorCommand = millis() - topMotorWait;
void continueTopMotorRotation() {
  if (topRotations != 0 && millis() - prevTopMotorCommand >= topMotorWait && checkTopMotor()) {
    printTopMotorStuff();
    if (topRotations > 0) {
      //Serial.print("positive ");
      //Serial.println(topRotations);
      topMotor.easeRotateTo(180);
      topRotations--;
    } else {
      //Serial.print("negative ");
      //Serial.println(topRotations);
      topMotor.easeRotateTo(-180);
      topRotations++;
    }
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

  if (topRotations != 0) {
    //printTopMotorStuff();
  }

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
  

  if (wifiOkStatus && ntpOkStatus && topMotorStatus != MOTOR_STALLED) {
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  } else {
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  }
  delay(100); // Power saving mechanism to slow down the main loop
}
