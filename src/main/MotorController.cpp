#include "MotorController.h"

MotorController::MotorController(int stepPin, int dirPin, int stepsPerRevolution)
    : stepper(1, stepPin, dirPin), stepsPerRevolution(stepsPerRevolution) {}

void MotorController::begin(){
    stepper.setMaxSpeed(1000); //default 6RPM
    stepper.setAcceleration(500);
}

void MotorController::moveToPosition(long position){
    stepper.moveTo(position);
}

void MotorController::setMaxSpeed(float speed){
    stepper.setMaxSpeed(speed);
}

void MotorController::setAcceleration(float acceleration){
    stepper.setAcceleration(acceleration);
}

void MotorController::update(){
    stepper.runSpeed();
}

long MotorController::currentPosition(){
    return stepper.currentPosition();
}

void MotorController::jog(float speed){
    stepper.setSpeed(speed);
}

void MotorController::setMinPulseWidth(unsigned int minWidth){
    stepper.setMinPulseWidth(minWidth);
}

long MotorController::distToGo(){
    return stepper.distanceToGo();
}

void MotorController::run(){
    stepper.run();
}

void MotorController::move(long dist){
    stepper.move(dist);
}
