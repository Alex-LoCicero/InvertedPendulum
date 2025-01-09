#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "MotorController.h"
#include "PIDController.h"
#include "Timer.h"
#include "StateMachine.h"
#include "DataRecorder.h"
#include "InterruptHandler.h"

// Define pins
const int stepPin = 4;
const int dirPin = 5;
const int enablePin = 8;
const int phaseA = 2;
const int phaseB = 3;
const int resetPin = 9;
const int stepsPerRevolution = 200;

// PID controller settings
double Input, Output, Setpoint = 300; 
double kp = 1.0, ki = 0.0, kd = 0.0; 

// Initialize motor and controller
MotorController motor(stepPin, dirPin, stepsPerRevolution); 
PIDController pid(&Input, &Output, &Setpoint, kp, ki, kd);
DataRecorder recorder(210, TIMESTAMP | MOTOR_SP | MOTOR_POS | JOINT_POS);
StateMachine stateMachine(motor, pid, recorder);
Timer Ctimer;

void setup() {
    // Serial communication
    Serial.begin(9600);

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
    
    Ctimer.start();
}

void loop() {
    Ctimer.reset();

    if (InterruptHandler::isEnable() && stateMachine.getCurrentState() == DISABLED) {
        stateMachine.setState(ENABLED);
    } else if (!InterruptHandler::isEnable() && stateMachine.getCurrentState() == ENABLED) {
        stateMachine.setState(DISABLED);
    }

    stateMachine.update();
    // Serial.println(Ctimer.elapsed(), 8);
    // Serial.println(InterruptHandler::angle);

    // Print a UTF-8 encoded string with special characters
    // const char* utf8String = u8"Temperature: 23.5°C";

    // Serial.print("DATA, timestamp: ");
    // Serial.print(Ctimer.elapsed(), 8);
    // Serial.println(", motor_sp:100, motor_pos:50, joint_pos:30");

    // Serial.print("DATA, timestamp: ");
    // Serial.print(Ctimer.elapsed(), 8);
    // Serial.println(", motor_sp:110, motor_pos:60, joint_pos:40");

    // Serial.print("DATA, timestamp: ");
    // Serial.print(Ctimer.elapsed(), 8);
    // Serial.println(", motor_sp:120, motor_pos:70, joint_pos:50");

    // Serial.print("DATA, timestamp: ");
    // Serial.print(Ctimer.elapsed(), 8);
    // Serial.println(", motor_sp:130, motor_pos:80, joint_pos:60");

    // Serial.print("DATA, timestamp: ");
    // Serial.print(Ctimer.elapsed(), 8);
    // Serial.println(", motor_sp:140, motor_pos:90, joint_pos:70");

    // delay(1000);
    delay(10);
    
}