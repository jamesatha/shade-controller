#include  "./StepperMotor.h"
#include "./stupid-synchronization-helper.h"


StepperMotor::StepperMotor(
          unsigned int enablePin,
          unsigned int stepPin, 
          unsigned int dirPin, 
          MotorStatus status): 
    enablePin(enablePin), stepPin(stepPin), dirPin(dirPin), status(status) {

  this->movingInfo.keepEnabledWhenCompleted = true;
  this->movingInfo.movingEndState = MOTOR_UNKNOWN;
  this->movingInfo.stepsLeft = 0;
  
  pinMode(this->enablePin, OUTPUT);
  pinMode(this->stepPin, OUTPUT);
  pinMode(this->dirPin, OUTPUT);
  this->disableMotor();
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
  delay(10); // allow time for motor to get to a good state
  this->motorEnabled = true;
}

void StepperMotor::stopMotor() {
  this->movingInfo.movingEndState = MOTOR_UNKNOWN;
  this->movingInfo.stepsLeft = -1;
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
  memory_barrier();
  Serial.println(this->status == MOTOR_MOVING_CLOCKWISE || this->status == MOTOR_MOVING_COUNTER);
  return this->status == MOTOR_MOVING_CLOCKWISE || this->status == MOTOR_MOVING_COUNTER;
}

bool StepperMotor::startDrive(bool clockwise, MotorStatus desiredEndState, bool keepEnabledWhenCompleted,
                              unsigned int steps, unsigned int waitTimeMicroseconds) {
  if (steps == 0) {
    return false;
  }
  
  if (desiredEndState == MOTOR_UNKNOWN || 
      (desiredEndState == MOTOR_AT_COUNTER_MAX && this->status == MOTOR_AT_CLOCKWISE_MAX) ||
      (desiredEndState == MOTOR_AT_CLOCKWISE_MAX && this->status == MOTOR_AT_COUNTER_MAX) ||
      steps == 1 /* Allow for 1 step changes that keep the state the same */) {
    this->movingInfo.movingEndState = desiredEndState;
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

  this->movingInfo.stepsLeft = steps;
  this->movingInfo.keepEnabledWhenCompleted = keepEnabledWhenCompleted;
  this->movingInfo.waitTimeMicroseconds = waitTimeMicroseconds;
  memory_barrier();

  return true;
}

void StepperMotor::executeSteps() {
  memory_barrier();
  Serial.printf("executeSteps: %d\n", this->movingInfo.stepsLeft);
  while (this->movingInfo.stepsLeft > 0) {
    Serial.print("   ");
    Serial.println(this->movingInfo.stepsLeft);
    digitalWrite(this->stepPin, HIGH); // Make one step
    delayMicroseconds(10); // This isn't necessary
    digitalWrite(this->stepPin, LOW); // Reset step pin
    delayMicroseconds(this->movingInfo.waitTimeMicroseconds); 
    this->movingInfo.stepsLeft--;
  }

  Serial.println("ENDED!");
  this->status = this->movingInfo.movingEndState;

  if (!this->movingInfo.keepEnabledWhenCompleted) {
    // We will disable the motor
    this->disableMotor();
  }
}

void StepperMotor::continueDrive() {
  memory_barrier();
  Serial.println("Waiting on lock...");
  std::lock_guard<std::mutex> lck(statusMutex);
  Serial.println("   ... got lock");
  Serial.println(this->movingInfo.stepsLeft);
  if (this->status == MOTOR_MOVING_CLOCKWISE) {
    digitalWrite(this->dirPin, HIGH);
    this->executeSteps();
  } else if (this->status == MOTOR_MOVING_COUNTER) {
    digitalWrite(this->dirPin, LOW);
    this->executeSteps();
  }
}