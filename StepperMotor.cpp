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
  this->movingInfo.totalStepsLeft = 0;

  this->movingInfo.steps1 = 0;
  this->movingInfo.steps2 = 0;
  this->movingInfo.steps3 = 0;
  this->movingInfo.steps4 = 0;

  this->movingInfo.waitTimeMicroseconds1 = LARGE_DEFAULT_MOTOR_WAIT_MICROSECONDS; // Setting this to something large
  this->movingInfo.waitTimeMicroseconds2 = LARGE_DEFAULT_MOTOR_WAIT_MICROSECONDS; // Setting this to something large
  this->movingInfo.waitTimeMicroseconds3 = LARGE_DEFAULT_MOTOR_WAIT_MICROSECONDS; // Setting this to something large
  this->movingInfo.waitTimeMicroseconds4 = LARGE_DEFAULT_MOTOR_WAIT_MICROSECONDS; // Setting this to something large
  
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
  this->movingInfo.totalStepsLeft = -1;

  // Reset the others?
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
                              unsigned int steps1, unsigned int waitTimeMicroseconds1,
                              unsigned int steps2, unsigned int waitTimeMicroseconds2,
                              unsigned int steps3, unsigned int waitTimeMicroseconds3,
                              unsigned int steps4, unsigned int waitTimeMicroseconds4) {
  unsigned int totalSteps = steps1 + steps2 + steps3 + steps4;
  if (totalSteps == 0) {
    return false;
  }

  if (steps2 == 0 && steps3 + steps4 > 0) {
    return false;
  }

  if (steps3 == 0 && steps4 > 0) {
    return false;
  }
  
  if (desiredEndState == MOTOR_UNKNOWN || 
      (desiredEndState == MOTOR_AT_COUNTER_MAX && this->status == MOTOR_AT_CLOCKWISE_MAX) ||
      (desiredEndState == MOTOR_AT_CLOCKWISE_MAX && this->status == MOTOR_AT_COUNTER_MAX) ||
      totalSteps == 1 /* Allow for 1 step changes that keep the state the same */) {
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

  this->movingInfo.steps1 = steps1;
  this->movingInfo.steps2 = steps2;
  this->movingInfo.steps3 = steps3;
  this->movingInfo.steps4 = steps4;

  this->movingInfo.waitTimeMicroseconds1 = waitTimeMicroseconds1; 
  this->movingInfo.waitTimeMicroseconds2 = waitTimeMicroseconds2;
  this->movingInfo.waitTimeMicroseconds3 = waitTimeMicroseconds3; 
  this->movingInfo.waitTimeMicroseconds4 = waitTimeMicroseconds4; 

  this->movingInfo.totalStepsLeft = totalSteps;
  this->movingInfo.keepEnabledWhenCompleted = keepEnabledWhenCompleted;

  memory_barrier();

  return true;
}

void StepperMotor::executeSteps() {
  memory_barrier();
  Serial.printf("executeSteps: %d\n", this->movingInfo.totalStepsLeft);
  while (this->movingInfo.totalStepsLeft > 0) {
    Serial.print("   ");
    Serial.println(this->movingInfo.totalStepsLeft);
    digitalWrite(this->stepPin, HIGH); // Make one step
    delayMicroseconds(2); // This isn't necessary
    digitalWrite(this->stepPin, LOW); // Reset step pin

    if (this->movingInfo.totalStepsLeft > this->movingInfo.steps4 + this->movingInfo.steps3 + this->movingInfo.steps2) {
      // we are in the first segment
      delayMicroseconds(this->movingInfo.waitTimeMicroseconds1); 
    } else if (this->movingInfo.totalStepsLeft > this->movingInfo.steps4 + this->movingInfo.steps3) {
      // we are in the second segment
      delayMicroseconds(this->movingInfo.waitTimeMicroseconds2); 
    } else if (this->movingInfo.totalStepsLeft > this->movingInfo.steps4) {
      // we are in the third segment
      delayMicroseconds(this->movingInfo.waitTimeMicroseconds3); 
    } else {
      // we are in the fourth segement
      delayMicroseconds(this->movingInfo.waitTimeMicroseconds4);
    }
    
    this->movingInfo.totalStepsLeft--;
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
  Serial.println(this->movingInfo.totalStepsLeft);
  if (this->status == MOTOR_MOVING_CLOCKWISE) {
    digitalWrite(this->dirPin, HIGH);
    this->executeSteps();
  } else if (this->status == MOTOR_MOVING_COUNTER) {
    digitalWrite(this->dirPin, LOW);
    this->executeSteps();
  }
}