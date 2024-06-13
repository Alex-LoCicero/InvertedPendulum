#include "MotorController.h"
// #include "AccelStepper.h"

// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 4;
const int stepsPerRevolution = 200;
const int phaseA = 2;
const int phaseB = 3;

volatile long counter = 0; // rotary encoder position 
volatile float position = 0; // stepper motor position

// use OOP structure 
MotorController motor(stepPin, dirPin, stepsPerRevolution); 

// AccelStepper stepper(1, stepPin, dirPin); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{
  // Serial communication 
  Serial.begin(9600);
  // while(!Serial);

  // Declare pins modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(phaseA, INPUT);
  pinMode(phaseB, INPUT);

  // turn on pullup resistors
  digitalWrite(phaseA, HIGH);       
  digitalWrite(phaseB, HIGH);      
  // create interrupts

  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(phaseA), ai0, RISING);
  
  // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(phaseB), ai1, RISING);

  // // Jog motor
  motor.begin();
  motor.jog(600);
  motor.setMinPulseWidth(100);	

}
void loop()
{
    // encoder feedback 
    // Serial.println(counter);
    // motor feedback 
    // position = motor.currentPosition();
    // Serial.println(position);
    

    motor.update();
    // delayMicroseconds(2000);
    // delay(10);

	// // Set motor direction clockwise
	// digitalWrite(dirPin, LOW);

	// // Spin motor slowly
	// for(int x = 0; x < stepsPerRevolution; x++)
	// {
	// 	digitalWrite(stepPin, HIGH);
	// 	delayMicroseconds(2000);
	// 	digitalWrite(stepPin, LOW);
	// 	delayMicroseconds(2000);
	// }
	// delay(1000); // Wait a second
	
  	// // // // Set motor direction clockwise
	// digitalWrite(dirPin, HIGH);

	// // Spin motor slowly
	// for(int x = 0; x < stepsPerRevolution; x++)
	// {
	// 	digitalWrite(stepPin, HIGH);
	// 	delayMicroseconds(2000);
	// 	digitalWrite(stepPin, LOW);
	// 	delayMicroseconds(2000);
	// }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}


