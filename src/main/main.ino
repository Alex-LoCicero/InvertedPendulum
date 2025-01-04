#include "MotorController.h"
#include "PIDController.h"
#include "PinChangeInterrupt.h"

// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 4;
const int stepsPerRevolution = 800;
const int phaseA = 2;
const int phaseB = 3;
const int resetPin = 9;
const uint8_t enablePin = 8;

// feedback
volatile long angle = 0.0; // rotary encoder position 
long motor_sp = 0;
long motor_pos = 0;
long joint_pos = 0;

// enable
volatile bool enable = false; // trigger for encoder reset  

// timers
const unsigned long debounceDelay = 200; // debounce delay in ms
volatile unsigned long lastDebounceTime = 0;
static unsigned long lastTime = 0;
volatile unsigned long stopwatch = 0; 
static unsigned long startTime = 0;
static unsigned long timestamp = 0; 

// data collection and plotting
const int bufferSize = 210;
int dataBuffer[bufferSize];
int bufferIdx = 0;
bool enable_flag = false;
const int printInterval = 25;
bool save_flag = false;

double Input, Output, Setpoint = 613.0;//613.0; 
double kp = 0.1, ki = 0.0, kd = 0.5; 

// initialize motor and controller 
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
  Serial.begin(115200);
  
  // Declare pins modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(phaseA, INPUT_PULLUP);
  pinMode(phaseB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(phaseA), ai0, RISING);  // Attach interrupt for Phase A
  attachInterrupt(digitalPinToInterrupt(phaseB), ai1, RISING);  // Attach interrupt for Phase B
  
  pinMode(resetPin, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(resetPin), control_enable, CHANGE);

  // turn on pullup resistors
  digitalWrite(phaseA, HIGH);       
  digitalWrite(phaseB, HIGH);      

  // motor settings
  motor.begin();
  motor.setMinPulseWidth(30);	
//  motor.jog(2000);
  motor.setAcceleration(15000);
  motor.setMaxSpeed(15000);
  motor.setEnablePin(enablePin);
  motor.setPinsInverted(false, false, true); // enable pin is active low

  // controller settings
  pid.setOutputLimits(-8*stepsPerRevolution,8*stepsPerRevolution);

}
void loop()
{

    if (enable && bufferIdx < bufferSize)
    {
      dataBuffer[bufferIdx++] = motor_sp;
      dataBuffer[bufferIdx++] = motor_pos;
      dataBuffer[bufferIdx++] = joint_pos;
    }
    if (~enable && ~save_flag)
    {
      save_flag = true;
      Serial.println("motor_sp, motor_pos, joint_pos");
      for (int i = 0; i < bufferIdx; i += 3) {
        if ((i+2) >= bufferIdx){
          break;
        }
        Serial.print(dataBuffer[i]);
        Serial.print(", ");
        Serial.print(dataBuffer[i + 1]);
        Serial.print(", ");
        Serial.println(dataBuffer[i + 2]);
      }
      bufferIdx = 0;
      for (int i = 0; i < bufferSize; i++) {
        dataBuffer[i] = 0;
      }
    }
    
    // if (stopwatch - lastTime > 1000){
    //   Serial.println(enable);
    //   lastTime = stopwatch;
    // }
    // motor_sp = Output;
    // motor_pos = motor.currentPosition();
    // joint_pos = angle;

    ///////////////////////////////////////////// test encoder
  // stopwatch = millis();
  // if (stopwatch - lastTime > 100){
  //   lastTime = stopwatch;
  //   Serial.println(angle);
  // }
    ///////////////////////////////////////////////
        ///////////////////////////////////////////// auto start
    // if (stopwatch - startTime > 5000 && !enable){
    //   enable = true;
    // }
    ///////////////////////////////////////////////
    
    
    // if (enable && (bufferIdx < (bufferSize-2)) && (stopwatch - lastTime > printInterval)) { // Collect data
    //       // dataBuffer[bufferIdx++] = motor_sp;
    //       // dataBuffer[bufferIdx++] = motor_pos;
    //       // dataBuffer[bufferIdx++] = joint_pos;
    //       timestamp = millis();
    //       Serial.print(timestamp);
    //       Serial.print(", ");
    //       Serial.println(angle);
    //       lastTime = stopwatch;
    // } else if (!enable) {
    //   // Serial.println(enable);
    // }

    // Input = angle; // joint encoder feedback (controller input)
    
    // stopwatch = millis();
    // switch (enable){
    //   case true:
    //     if (!enable_flag){
    //       startTime = stopwatch;
    //       motor.enableOutputs();
    //     }
    //     if (stopwatch - startTime > 5000){
    //       pid.compute(); // compute PID controller output
    //       move_stepper(Output);
    //       motor.update();
    //     }
    //     enable_flag = true;
    //     break;
    //   case false:
    //     if (enable_flag) { // disable motor and trasnmit buffer
    //       enable_flag = false;
    //       motor.disableOutputs();
    //       for (int i = 0; i < bufferSize; i++) {
    //         Serial.println(dataBuffer[i]);
    //         dataBuffer[i] = 0;
    //       } 
    //     } else { // disabled state
    //       enable_flag = false;
    //       motor.disableOutputs();
    //       angle = 0; // reset joint position
    //       // TODO: reset controller to prevent integral windup
    //     }
    //     break;
    //   default:
    //     Serial.println("unknown control state");
    //     break;
    // }  
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    angle++;
  } else {
    angle--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    angle--;
  } else {
    angle++;
  }
}

void control_enable() {
  unsigned long currentMillis = millis();
  // activate interrupt if DigitalPin nr is going from HIGH to LOW
  if (currentMillis - lastDebounceTime >= debounceDelay) {
    enable = true;
    lastDebounceTime = currentMillis;
  }
}
