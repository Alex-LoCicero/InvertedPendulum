#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "MotorController.h"
#include "PIDController.h"
#include "Timer.h"
#include "StateMachine.h"
#include "DataRecorder.h"
#include "InterruptHandler.h"

// Define pins
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 8;
const int phaseA = 4;
const int phaseB = 5;
const int resetPin = 9;
const int stepsPerRevolution = 200;

// PID controller settings
double Input, Output, Setpoint = 300; 
double kp = 1.0, ki = 0.0, kd = 0.0; 

// Initialize motor and controller
MotorController motor(stepPin, dirPin, stepsPerRevolution); 
PIDController pid(&Input, &Output, &Setpoint, kp, ki, kd);
DataRecorder recorder(210, TIMESTAMP | MOTOR_SP | MOTOR_POS);
StateMachine stateMachine(motor, pid, recorder);

void setup() {
    // Serial communication
    Serial.begin(115200);

    // Declare pin modes
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(phaseA, INPUT_PULLUP);
    pinMode(phaseB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(phaseA), InterruptHandler::ai0, RISING);
    attachInterrupt(digitalPinToInterrupt(phaseB), InterruptHandler::ai1, RISING);

    pinMode(resetPin, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(resetPin), InterruptHandler::control_enable, CHANGE);

    // Turn on pullup resistors
    digitalWrite(phaseA, HIGH);
    digitalWrite(phaseB, HIGH);

    // Motor settings
    motor.begin();
    motor.setMinPulseWidth(30);
    motor.setAcceleration(15000);
    motor.setMaxSpeed(15000);
    motor.setEnablePin(enablePin);
    motor.setPinsInverted(false, false, true); // Enable pin is active low
}

void loop() {
    stateMachine.update();

    // // Example condition to change state
    // if (some_condition_to_enable) {
    //     stateMachine.setState(ENABLED);
    // } else if (some_condition_to_save) {
    //     stateMachine.setState(SAVING);
    // } else {
    //     stateMachine.setState(DISABLED);
    // }
}