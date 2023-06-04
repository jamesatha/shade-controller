#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H
#include <Arduino.h>
#include <mutex>

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
          unsigned int stepPin, 
          unsigned int dirPin, 
          unsigned int waitTimeMicroseconds = 10000,
          MotorStatus status = MOTOR_UNKNOWN);

        void setStepWait(unsigned int waitTimeMicroseconds);

        [[nodiscard]] bool setStatus(MotorStatus status);
        MotorStatus getStatus();
        bool isMoving();


        [[nodiscard]] bool startDrive(bool clockwise, unsigned int steps, MotorStatus desiredEndState = MOTOR_UNKNOWN);
        void continueDrive();
    private:
        unsigned int dirPin, stepPin, waitTimeMicroseconds;
        MotorStatus status;
        std::mutex status_mutex;

        MotorStatus movingEndState;
        unsigned int stepsLeft;

        void executeSteps();
};
#endif
