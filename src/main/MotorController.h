#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h> 
#include <AccelStepper.h>

class MotorController {
    public:
        MotorController(int stepPin, int dirPin, int stepsPerRevolution);
        void begin();
        void moveToPosition(long position);
        void setMaxSpeed(float speed);
        void jog(float speed);
        void setAcceleration(float acceleration);
        void update();
        long currentPosition();
        void setMinPulseWidth(unsigned int minWidth);
    
    private:
        AccelStepper stepper;
        int stepsPerRevolution;
};

#endif // MOTORCONTROLLER_H
