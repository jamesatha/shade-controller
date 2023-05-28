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
  //topMotor.calibrate(); // Minimal PWM: 36, maximal PWM: 1075
  topMotor.setMinimalForce(12);
  topMotor.setSpeed(10);
  
  //topMotor.setAdditionalTorque(200); // unsure about this
  topMotor.setOffset(topMotor.getAngle());
}

int topRotations = 0;

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
    if (request->hasParam("speed")) {
      int speed = request->getParam("speed")->value().toInt();
      if (speed > 20) {
        speed = 20;
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
      /*bool clockwise = rotations > 0;
      if (!clockwise) {
        rotations *= -1;
      }
      for (int i = 0; i < rotations*2; i++) {
        if (clockwise) {
          topMotor.rotate(180);
        } else {
          topMotor.rotate(-180);
        }
        delay(7000);
      }
      */
    } else {
      Serial.println("Resetting motor to 0");
      topMotor.rotate(0);
    }
    
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

unsigned long topMotorWait = 400; // 
unsigned long prevTopMotorCommand = millis() - topMotorWait;
void continueTopMotorRotation() {
  if (topRotations != 0 && millis() - prevTopMotorCommand >= topMotorWait) {
    if (topRotations > 0) {
      //Serial.print("positive ");
      //Serial.println(topRotations);
      topMotor.easeRotate(10);
      topRotations--;
    } else {
      //Serial.print("negative ");
      //Serial.println(topRotations);
      topMotor.easeRotate(-10);
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
    printTopMotorStuff();
  }
  //int enButtonState = digitalRead(EN_BUTTON_PIN);  // Read the state of the button
  //Serial.println(buttonState);
  //if (enButtonState == LOW) {  // If the button is pressed (logic is reversed due to pullup resistor)
    // Do something when the button is pressed
  //  Serial.println("Button pressed!");
  //  
  //}

  int bootButtonState = digitalRead(BOOT_BUTTON_PIN);  // Read the state of the BOOT button
  if (prevBootButtonState != bootButtonState) {
    // Boot button change
    if (bootButtonState == LOW) {  // If the BOOT button is pressed for first time (logic is reversed due to pullup resistor)
      // Do something when the BOOT button is pressed
      //topMotor.rotate(180);
      printTopMotorStuff();
    } else {
      
    }
    prevBootButtonState = bootButtonState;
  }
  

  if (wifiOkStatus && ntpOkStatus) {
    digitalWrite(BLUE_BUILT_IN_LED, HIGH);
  } else {
    digitalWrite(BLUE_BUILT_IN_LED, LOW);
  }
  delay(100); // Power saving mechanism to slow down the loop to about once a second
}
