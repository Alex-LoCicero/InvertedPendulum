/*This class is mostly comprised of wrapper functions for arduino PID library.*/

#include "PIDController.h"

PIDController::PIDController(double* input, double* output, double* setpoint, double kp, double ki, double kd) 
    : pid( input,  output,  setpoint, kp, ki, kd, DIRECT) {
    pid.SetMode(AUTOMATIC);
}

// Compute controller output 
void PIDController::compute(){
    pid.Compute();
} 

// Set control effort limits
void PIDController::setOutputLimits(float min, float max){
    pid.SetOutputLimits(min, max); //default [0, 255]
}

// Set PID gains 
void PIDController::setTunings(float kp, float ki, float kd){
    pid.SetTunings(kp, ki, kd);
}

// Set controller sample time 
void PIDController::setSampleTime(float ts){
    pid.SetSampleTime(ts); // default is 200ms (5hz)
}
