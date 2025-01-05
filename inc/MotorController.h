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
        long currentPosition();
        void setMinPulseWidth(unsigned int minWidth);
        void run();
        void move(long dist);
        void setEnablePin(uint8_t enablePin);
        void disableOutputs();
        void enableOutputs();
        void setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert);

        long getTargetPosition();
        long getPosition();
        
    
    private:
        AccelStepper stepper;
        int stepsPerRevolution;
};

#endif // MOTORCONTROLLER_H
