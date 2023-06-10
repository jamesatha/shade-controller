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


        [[nodiscard]] bool startDrive(bool clockwise, unsigned int steps, MotorStatus desiredEndState);
        void continueDrive();
    private:
        unsigned int enablePin, dirPin, stepPin, waitTimeMicroseconds;
        MotorStatus status;
        bool motorEnabled;
        std::mutex statusMutex;
        std::set<MotorStatus> statesToDisableMotor;

        MotorStatus movingEndState;
        unsigned int stepsLeft;

        void executeSteps();
        void enableMotor();
        void disableMotor();
};
#endif
