#include "InterruptHandler.h"

namespace InterruptHandler {
    volatile long angle = 0;
    volatile bool enable = false;
    volatile unsigned long lastDebounceTime = 0;

    void ai0() {
        if (digitalRead(3) == LOW) {
            angle++;
        } else {
            angle--;
        }
    }

    void ai1() {
        if (digitalRead(2) == LOW) {
            angle--;
        } else {
            angle++;
        }
    }

    void control_enable() {
        unsigned long currentMillis = millis();
        if (currentMillis - lastDebounceTime >= debounceDelay) {
            enable = !enable;
            lastDebounceTime = currentMillis;
        }
    }

    bool isEnable() {
        return enable;
    }
}