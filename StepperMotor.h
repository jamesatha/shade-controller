#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H
#include <Arduino.h>
#include <mutex>
#include <set>

typedef enum MotorStatus {
    MOTOR_UNKNOWN,
    MOTOR_MOVING_CLOCKWISE,
    MOTOR_MOVING_COUNTER,
    MOTOR_AT_CLOCKWISE_MAX,
    MOTOR_AT_COUNTER_MAX
};

typedef struct {
  volatile MotorStatus movingEndState;
  volatile unsigned int stepsLeft;
  volatile bool keepEnabledWhenCompleted;
} MovingInfo;

class StepperMotor {
    public:
        StepperMotor(
          unsigned int enablePin,
          unsigned int stepPin, 
          unsigned int dirPin, 
          unsigned int waitTimeMicroseconds = 2,
          MotorStatus status = MOTOR_UNKNOWN);

        void setStepWait(unsigned int waitTimeMicroseconds);

        [[nodiscard]] bool setStatus(MotorStatus status, bool force=false);
        MotorStatus getStatus();
        bool isEnabled();
        bool isMoving();


        [[nodiscard]] bool startDrive(bool clockwise, unsigned int steps, MotorStatus desiredEndState, bool keepEnabledWhenCompleted);
        void continueDrive();
        void stopMotor();
        void disableMotor();
    private:
        unsigned int enablePin, dirPin, stepPin, waitTimeMicroseconds;
        volatile MotorStatus status;
        bool motorEnabled;
        std::mutex statusMutex;

        volatile MovingInfo movingInfo;

        void executeSteps();
        void enableMotor();
};
#endif
