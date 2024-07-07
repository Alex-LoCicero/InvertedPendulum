#include "MotorController.h"
#include "PIDController.h"
#include "PinChangeInterrupt.h"

// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 4;
const int stepsPerRevolution = 800;
const int phaseA = 2;
const int phaseB = 3;
const int resetPin = 6;

volatile long angle = 0; // rotary encoder position 
volatile bool enable = false; // trigger for encoder reset  
const unsigned long debounceDelay = 200; // debounce delay in ms
volatile unsigned long lastDebounceTime = 0;

double Input, Output, Setpoint = 610; 

double kp = 1.0, ki = 0.0, kd = 0.0; 

// initialize stepper
MotorController motor(stepPin, dirPin, stepsPerRevolution); 
PIDController pid(&Input, &Output, &Setpoint, kp, ki, kd);

void move_stepper(int value){
  if (motor.distToGo() == 0)
    {
      motor.move(value % stepsPerRevolution);
    }
    motor.run();
}

void setup()
{
  // Serial communication 
  Serial.begin(9600);
  
  // Declare pins modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(phaseA, INPUT);
  pinMode(phaseB, INPUT);
  pinMode(resetPin, INPUT_PULLUP);

  // turn on pullup resistors
  digitalWrite(phaseA, HIGH);       
  digitalWrite(phaseB, HIGH);      
  // create interrupts

  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(phaseA), ai0, RISING);
  
  // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(phaseB), ai1, RISING);

  // Attach the new PinChangeInterrupt and event fuction 
  attachPCINT(digitalPinToPCINT(resetPin), control_enable, CHANGE);

  // Jog motor
  motor.begin();
  motor.setMinPulseWidth(100);	

  // encoder initialization


}
void loop()
{
    // print encoder feedback 
//    Serial.println(angle);
    Serial.print("Input:");
    Serial.println(Input);
    Serial.print("Output:");
    Serial.println(Output);

    if (enable) { // enable motor control
      
      // position = motor.currentPosition();
      // Serial.println(position);
      
      // motor commands
      // motor.jog(600);
      // motor.update();
      Input = angle;
      pid.compute();
      move_stepper(Output);
    }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    angle++;
  }else{
    angle--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    angle--;
  }else{
    angle++;
  }
}

void control_enable() {
  unsigned long currentMillis = millis();
  // activate interrupt if DigitalPin nr is going from HIGH to LOW
  if (currentMillis - lastDebounceTime >= debounceDelay) {
    enable = !enable;
    lastDebounceTime = currentMillis;
  }
}
