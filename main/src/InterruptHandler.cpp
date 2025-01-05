#include "InterruptHandler.h"

volatile long InterruptHandler::angle = 0;
volatile bool InterruptHandler::enable = false;
volatile unsigned long InterruptHandler::lastDebounceTime = 0;

void InterruptHandler::ai0() {
    if (digitalRead(3) == LOW) {
        angle++;
    } else {
        angle--;
    }
}

void InterruptHandler::ai1() {
    if (digitalRead(2) == LOW) {
        angle--;
    } else {
        angle++;
    }
}

void InterruptHandler::control_enable() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastDebounceTime >= debounceDelay) {
        enable = true;
        lastDebounceTime = currentMillis;
    }
}

bool InterruptHandler::isEnable() {
    return enable;
}