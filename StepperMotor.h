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
  volatile int stepsLeft; // in case there is ever a negative value, this shouldn't wrap so leaving as an int
  volatile unsigned int waitTimeMicroseconds;
  volatile bool keepEnabledWhenCompleted;
} MovingInfo;

class StepperMotor {
    public:
        StepperMotor(
          unsigned int enablePin,
          unsigned int stepPin, 
          unsigned int dirPin, 
          MotorStatus status = MOTOR_UNKNOWN);

        [[nodiscard]] bool setStatus(MotorStatus status, bool force=false);
        MotorStatus getStatus();
        bool isEnabled();
        bool isMoving();


        [[nodiscard]] bool startDrive(bool clockwise, MotorStatus desiredEndState, bool keepEnabledWhenCompleted,
                                      unsigned int steps, unsigned int waitTimeMicroseconds);
        [[nodiscard]] bool startDrive(bool clockwise, MotorStatus desiredEndState, bool keepEnabledWhenCompleted, 
                                      unsigned int steps1, unsigned int waitTimeMicroseconds1,
                                      unsigned int steps2, unsigned int waitTimeMicroseconds2,
                                      unsigned int steps3, unsigned int waitTimeMicroseconds3,
                                      unsigned int steps4, unsigned int waitTimeMicroseconds4);

        void continueDrive();
        void stopMotor();
        void disableMotor();
    private:
        unsigned int enablePin, dirPin, stepPin;
        volatile MotorStatus status;
        bool motorEnabled;
        std::mutex statusMutex;

        volatile MovingInfo movingInfo;

        void executeSteps();
        void enableMotor();
};
#endif
