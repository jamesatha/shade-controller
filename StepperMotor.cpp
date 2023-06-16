#include  "./StepperMotor.h"

StepperMotor::StepperMotor(
          unsigned int enablePin,
          unsigned int stepPin, 
          unsigned int dirPin, 
          unsigned int waitTimeMicroseconds,
          MotorStatus status): 
    enablePin(enablePin), stepPin(stepPin), dirPin(dirPin), waitTimeMicroseconds(waitTimeMicroseconds), status(status) {
  this->movingEndState = MOTOR_UNKNOWN; 
  this->stepsLeft = 0;
  this->statesToDisableMotor = std::set<MotorStatus>();
  
  pinMode(this->enablePin, OUTPUT);
  pinMode(this->stepPin, OUTPUT);
  pinMode(this->dirPin, OUTPUT);
  this->disableMotor();
}

void StepperMotor::setStepWait(unsigned int waitTimeMicroseconds) {
  this->waitTimeMicroseconds = waitTimeMicroseconds;
}

[[nodiscard]] bool StepperMotor::setStatus(MotorStatus desiredStatus, bool force) {
  std::lock_guard<std::mutex> lck(statusMutex);
  if (force) {
    this->status = desiredStatus;
    return true;
  }
  if (desiredStatus == MOTOR_MOVING_CLOCKWISE && (this->status == MOTOR_UNKNOWN || this->status == MOTOR_AT_COUNTER_MAX)) {
    this->status = desiredStatus;
    return true;
  }

  if (desiredStatus == MOTOR_MOVING_COUNTER && (this->status == MOTOR_UNKNOWN || this->status == MOTOR_AT_CLOCKWISE_MAX)) {
    this->status = desiredStatus;
    return true;
  }
  

  Serial.println("Unallowed state transition");
  return false;
}

void StepperMotor::enableMotor() {
  digitalWrite(this->enablePin, LOW);
  delay(1); // allow time for motor to get to a good state
  this->motorEnabled = true;
}

void StepperMotor::stopMotor() {
  this->movingEndState = MOTOR_UNKNOWN;
  this->stepsLeft = -1;
}

void StepperMotor::disableMotor() {
  digitalWrite(this->enablePin, HIGH);
  this->stopMotor();
  this->motorEnabled = false;
}

bool StepperMotor::isEnabled() {
  return this->motorEnabled;
}

MotorStatus StepperMotor::getStatus() {
  return this->status;
}

bool StepperMotor::isMoving() {
  return this->status == MOTOR_MOVING_CLOCKWISE || this->status == MOTOR_MOVING_COUNTER;
}

bool StepperMotor::startDrive(bool clockwise, unsigned int steps, MotorStatus desiredEndState) {
  if (steps == 0) {
    return false;
  }
  
  if (desiredEndState == MOTOR_UNKNOWN || 
      (desiredEndState == MOTOR_AT_COUNTER_MAX && this->status == MOTOR_AT_CLOCKWISE_MAX) ||
      (desiredEndState == MOTOR_AT_CLOCKWISE_MAX && this->status == MOTOR_AT_COUNTER_MAX)) {
    this->movingEndState = desiredEndState;
  } else {
    Serial.println("Error in end state change");
    return false;
  }
  
  if (clockwise) {
    if (!this->setStatus(MOTOR_MOVING_CLOCKWISE)) {
      return false;
    }
  } else {
    if (!this->setStatus(MOTOR_MOVING_COUNTER)) {
      return false;
    }
  }

  // Make sure the motor is enabled
  this->enableMotor();

  this->stepsLeft = steps;

  return true;
}

void StepperMotor::executeSteps() {
  while (this->stepsLeft > 0) {
    digitalWrite(this->stepPin, HIGH); // Make one step
    delayMicroseconds(this->waitTimeMicroseconds); // Change this delay as needed
    digitalWrite(this->stepPin, LOW); // Reset step pin
    delayMicroseconds(this->waitTimeMicroseconds); // Change this delay as needed
    this->stepsLeft--;
  }

  Serial.println("ENDED!");
  this->status = this->movingEndState;

  if (this->statesToDisableMotor.find(this->status) != this->statesToDisableMotor.end()) {
    // We will disable the motor
    this->disableMotor(); // THIS ISN'T HAPPENING YET!
  }
}

void StepperMotor::continueDrive() {
  Serial.println("Waiting on lock...");
  std::lock_guard<std::mutex> lck(statusMutex);
  Serial.println("   ... got lock");
  if (this->status == MOTOR_MOVING_CLOCKWISE) {
    digitalWrite(this->dirPin, HIGH);
    this->executeSteps();
  } else if (this->status == MOTOR_MOVING_COUNTER) {
    digitalWrite(this->dirPin, LOW);
    this->executeSteps();
  }
}