#include "StateMachine.h"

StateMachine::StateMachine(MotorController& motor, PIDController& pid, DataRecorder& recorder)
    : motor(motor), pid(pid), recorder(recorder), currentState(DISABLED), previousState(DISABLED) {}

void StateMachine::update() {
    if (currentState != previousState) {
        switch (currentState) {
            case ENABLED:
                onEnable();
                break;
            case SAVING:
                onSave();
                break;
            case DISABLED:
                onDisable();
                break;
        }
        previousState = currentState;
    } else {
        switch (currentState) {
            case ENABLED:
                running();
                break;
            case SAVING:
                // Perform actions when saving
                break;
            case DISABLED:
                // Perform actions when disabled
                break;
        }
    }
}

void StateMachine::setState(State newState) {
    currentState = newState;
}

void StateMachine::onEnable() {
    // Actions to perform when enabling
    Serial.println("Enabling...");
    motor.enableOutputs();
    timer.start();
}

void StateMachine::onSave() {
    // Actions to perform when saving
    Serial.println("Saving data...");
    recorder.saveData();
    recorder.resetBuffer();
}

void StateMachine::onDisable() {
    motor.disableOutputs();
    Serial.println("Disabled...");
}

void StateMachine::running() {
    motor.run();
    recorder.recordData(timer.elapsed(), motor.getTargetPosition(), motor.getPosition(),
                 InterruptHandler::angle);

}