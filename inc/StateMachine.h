#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "MotorController.h"
#include "PIDController.h"
#include "Timer.h"
#include "DataRecorder.h"
#include "InterruptHandler.h"

enum State {
    DISABLED,
    ENABLED,
    SAVING
};

class StateMachine {
public:
    StateMachine(MotorController& motor, PIDController& pid, DataRecorder& recorder);
    Timer timer;
    void update();
    void setState(State newState);
    State getCurrentState();

private:
    State currentState;
    State previousState;
    MotorController& motor;
    PIDController& pid;
    DataRecorder& recorder;

    void onEnable();
    void onSave();
    void onDisable();
    void running();
};

#endif // STATEMACHINE_H