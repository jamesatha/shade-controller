#include  "./StepperMotor.h"

StepperMotor::StepperMotor(unsigned int stepPin, 
          unsigned int dirPin, 
          unsigned int waitTimeMicroseconds,
          MotorStatus status): 
    stepPin(stepPin), dirPin(dirPin), waitTimeMicroseconds(waitTimeMicroseconds), status(status) {
  this->movingEndState = MOTOR_UNKNOWN; 
  this->stepsLeft = 0;
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void StepperMotor::setStepWait(unsigned int waitTimeMicroseconds) {
  this->waitTimeMicroseconds = waitTimeMicroseconds;
}

[[nodiscard]] bool StepperMotor::setStatus(MotorStatus desiredStatus) {
  std::lock_guard<std::mutex> lck(status_mutex);
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
  
  if (clockwise) {
    if (!this->setStatus(MOTOR_MOVING_CLOCKWISE)) {
      return false;
    }
  } else {
    if (!this->setStatus(MOTOR_MOVING_COUNTER)) {
      return false;
    }
  }

  // TODO: set desiredEndState
  this->stepsLeft = steps;
  this->continueDrive();

  return true;
}

void StepperMotor::executeSteps() {
  int maxSteps = 1000 / this->waitTimeMicroseconds;
  int curStepCount = 0;
    while (curStepCount < maxSteps && this->stepsLeft > 0) {
      Serial.println("step");
      digitalWrite(this->stepPin, HIGH); // Make one step
      delayMicroseconds(this->waitTimeMicroseconds); // Change this delay as needed
      digitalWrite(this->stepPin, LOW); // Reset step pin
      delayMicroseconds(this->waitTimeMicroseconds); // Change this delay as needed
      curStepCount++;
      this->stepsLeft--;
    }

    if (this->stepsLeft == 0) {
      Serial.println("ENDED!");
      this->status = this->movingEndState;
    }
}

void StepperMotor::continueDrive() {
  Serial.println("Waiting on lock...");
  std::lock_guard<std::mutex> lck(status_mutex);
  Serial.println("   ... got lock");
  if (this->status == MOTOR_MOVING_CLOCKWISE) {
    digitalWrite(this->dirPin, HIGH);
    this->executeSteps();
  } else if (this->status == MOTOR_MOVING_COUNTER) {
    digitalWrite(this->dirPin, LOW);
    this->executeSteps();
  }
}