#include "MotorController.h"

MotorController::MotorController(int stepPin, int dirPin, int stepsPerRevolution)
    : stepper(1, stepPin, dirPin), stepsPerRevolution(stepsPerRevolution) {}

void MotorController::begin(){
    stepper.setMaxSpeed(2000); //default 6RPM
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
long MotorController::currentPosition(){
    return stepper.currentPosition();
}

void MotorController::jog(float speed){
    stepper.setSpeed(speed);
}

void MotorController::setMinPulseWidth(unsigned int minWidth){
    stepper.setMinPulseWidth(minWidth);
}
void MotorController::run(){
    stepper.run();
}

void MotorController::move(long dist){
    stepper.move(dist);
    run();
}

void MotorController::setEnablePin(uint8_t enablePin){
  stepper.setEnablePin(enablePin);
}

void MotorController::disableOutputs(){
  stepper.disableOutputs(); 
}

void MotorController::enableOutputs(){
  stepper.enableOutputs();
}

void MotorController::setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false){
  stepper.setPinsInverted(directionInvert, stepInvert, enableInvert);
}

long MotorController::getTargetPosition(){
  return stepper.targetPosition();
}

long MotorController::getPosition(){
  return stepper.currentPosition();
}
