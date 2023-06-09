#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "./stupid-synchronization-helper.h"


# define BLUE_BUILT_IN_LED 2  
# define EN_BUTTON_PIN 3
# define BOOT_BUTTON_PIN 0
# define MAX_STEPS_AT_A_TIME 40000

// unsure if that needs to be on a PULL_DOWN (2)
#include  "./StepperMotor.h"
#define STEP_PIN 12
#define DIR_PIN 13
#define ENABLE_PIN 14
StepperMotor topMotor(ENABLE_PIN, STEP_PIN, DIR_PIN); // this stepper has 200 steps per rotation but not sure we need that

TaskHandle_t taskHandle;
void SpinTask(void * parameters) {
  memory_barrier();

  do {
    topMotor.continueDrive();
  } while (topMotor.isMoving());
  
  vTaskDelete(taskHandle);
}

// This file specifies wifi creds. Ex:
// #define WIFI_SSID "REPLACE WITH WIFI NAME"
// #define WIFI_PASSWORD "REPLACE WITH WIFI PASSWORD"
#include "./wifi-info.h"

typedef struct {
  bool upIsClockwise;
  int steps;
} Configuration;

Configuration* configuration = NULL;

long startTime = millis();

// Other status stuff
bool wifiOkStatus = false;
bool ntpOkStatus = false;


// Create an instance of the server
AsyncWebServer server(80);

void prepareStatusLED() {
  pinMode(BLUE_BUILT_IN_LED, OUTPUT);  // Setting the blue LED to be an output that we can set
  digitalWrite(BLUE_BUILT_IN_LED, HIGH); // Turn off the blue light so we know we are not ready
}

void updateStatusLED() {
  if (wifiOkStatus && ntpOkStatus) {
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  } else {
    Serial.println("Something is wrong");
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  }
}

void getHumanTime(long milliseconds, char* buffer) {
  long long seconds = milliseconds / 1000;
  long long minutes = seconds / 60;
  long long hours = minutes / 60;
  long long days = hours / 24;

  // Calculate remaining hours, minutes and seconds
  hours = hours % 24;
  minutes = minutes % 60;
  seconds = seconds % 60;

  sprintf(buffer, "%lld days, %lld hours, %lld minutes, %lld seconds", days, hours, minutes, seconds);
}

void startTopMotorSpinTask() {
  xTaskCreatePinnedToCore(
    SpinTask,      /* Task function. */
    "SpinTask",    /* name of task. */
    100000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    1,             /* priority of the task */
    &taskHandle,   /* Task handle to keep track of created task */
    (xPortGetCoreID()+1) %2);         /* pin task to not used core */
}

void setup() {
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

  if (WiFi.localIP().toString().compareTo("192.168.68.65") == 0) {
    Serial.println("Found IP address: 192.168.68.65");
    configuration = (Configuration *)malloc(sizeof(Configuration));
    configuration->upIsClockwise = true;
    configuration->steps = 16000;
    topMotor.setStatus(MOTOR_AT_CLOCKWISE_MAX, true);
  } else {
    // Nothing
  }
  server.on("/top/disable", HTTP_POST, [](AsyncWebServerRequest *request) {
    topMotor.disableMotor();
    delay(1);
    topMotor.setStatus(MOTOR_UNKNOWN, true);
  });

  server.on("/top/stop", HTTP_POST, [](AsyncWebServerRequest *request) {
    topMotor.stopMotor();
    delay(1);
    topMotor.setStatus(MOTOR_UNKNOWN, true);
  });
  server.on("/top/down", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (configuration == NULL) {
      request->send(400, "text/plain", "Configuration Missing");
    } else {
      MotorStatus endState = configuration->upIsClockwise ? MOTOR_AT_COUNTER_MAX : MOTOR_AT_CLOCKWISE_MAX;
      bool turnOff = request->hasParam("turnOff") && (
          request->getParam("turnOff")->value().compareTo("true") == 0 ||
          request->getParam("turnOff")->value().compareTo("True") == 0 ||
          request->getParam("turnOff")->value().compareTo("TRUE") == 0
        );
      if (topMotor.startDrive(!configuration->upIsClockwise, configuration->steps, endState, !turnOff)) {
        startTopMotorSpinTask();
        request->send(200, "text/plain", "Moving down"); // check type
      } else {
        request->send(413, "text/plain", "WAIT!");
      }
    }
  });

  server.on("/top/up", HTTP_POST, [](AsyncWebServerRequest *request) {
    MotorStatus endState = configuration->upIsClockwise ? MOTOR_AT_CLOCKWISE_MAX : MOTOR_AT_COUNTER_MAX;
    if (configuration == NULL) {
      request->send(400, "text/plain", "Configuration Missing");
    } else {
      bool turnOff = request->hasParam("turnOff") && (
          request->getParam("turnOff")->value().compareTo("true") == 0 ||
          request->getParam("turnOff")->value().compareTo("True") == 0 ||
          request->getParam("turnOff")->value().compareTo("TRUE") == 0
        );
      if (topMotor.startDrive(configuration->upIsClockwise, configuration->steps, endState, !turnOff)) {
        startTopMotorSpinTask();
        request->send(200, "text/plain", "Moving up"); // check type
      } else {
        request->send(413, "text/plain", "WAIT!");
      }
    }
  });

  server.on("/top/move", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("wait")) {
      int wait = request->getParam("wait")->value().toInt();
      if (wait <= 1) {
        wait = 1;
      }
      Serial.print("Setting wait to ");
      Serial.println(wait);
      topMotor.setStepWait(wait);
    }

    if (request->hasParam("steps") && request->hasParam("direction")) {
      int steps = request->getParam("steps")->value().toInt();
      Serial.print("Spinning ");
      Serial.print(steps);
      Serial.println(" steps");
      if (steps < 0) {
        steps = 0;
      } else if (steps > MAX_STEPS_AT_A_TIME) {
        steps = MAX_STEPS_AT_A_TIME;
      }

      if (steps == 0) {
        request->send(200, "text/plain", "Done already!"); // check type
      } else {
        bool turnOff = request->hasParam("turnOff") && (
          request->getParam("turnOff")->value().compareTo("true") == 0 ||
          request->getParam("turnOff")->value().compareTo("True") == 0 ||
          request->getParam("turnOff")->value().compareTo("TRUE") == 0
        );
        if (request->getParam("direction")->value().compareTo("clockwise") == 0) {
          if (topMotor.startDrive(true, steps, MOTOR_UNKNOWN, !turnOff)) {
            startTopMotorSpinTask();
            request->send(200, "text/plain", "Moving clockwise"); // check type
          } else {
            request->send(413, "text/plain", "WAIT!");
          }
        } else {
          if (topMotor.startDrive(false, steps, MOTOR_UNKNOWN, !turnOff)) {
            startTopMotorSpinTask();
            request->send(200, "text/plain", "Moving counter"); // check type
          } else {
            request->send(413, "text/plain", "WAIT!");
          }
        }
      }
      
    } else {
      request->send(400, "text/plain", "Missing parameters"); // check type
    }
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(1024);
    char buffer[150];
    getHumanTime(millis() - startTime, buffer);
    doc["uptime"] = buffer;
    doc["wifi"] = wifiOkStatus;
    doc["ntp"] = ntpOkStatus;
    doc["motor_enabled"] = topMotor.isEnabled();
    switch (topMotor.getStatus()) {
      case MOTOR_UNKNOWN:
        doc["motor"] = "unknown";
        break;
      case MOTOR_AT_CLOCKWISE_MAX:
        doc["motor"] = "clockwise max";
        break;
      case MOTOR_AT_COUNTER_MAX:
        doc["motor"] = "counter max";
        break;
      case MOTOR_MOVING_CLOCKWISE:
        doc["motor"] = "moving clockwise";
        break;
      case MOTOR_MOVING_COUNTER:
        doc["motor"] = "moving counter";
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

int prevBootButtonState = HIGH;
void loop() {
  int bootButtonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the BOOT button
  if (prevBootButtonState != bootButtonState) {
    // Boot button change
    if (bootButtonState == LOW) {  // If the BOOT button is pressed for first time (logic is reversed due to pullup resistor)
      Serial.println("Starting");
      
      digitalWrite(DIR_PIN, HIGH); // Set the direction
      for(int i = 0; i < 200*16; i++) {
        digitalWrite(STEP_PIN, HIGH); // Make one step
        delayMicroseconds(100); // Change this delay as needed
        digitalWrite(STEP_PIN, LOW); // Reset step pin
        delayMicroseconds(100); // Change this delay as needed
      }

      Serial.println("Done");
    } else {
      
    }
    prevBootButtonState = bootButtonState;
  }

  checkWifiStatus();
  checkNtpStatus();

  updateStatusLED();
  delay(100); // Power saving mechanism to slow down the main loop  
}
