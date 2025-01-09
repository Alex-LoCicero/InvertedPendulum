#ifndef INTERRUPTHANDLER_H
#define INTERRUPTHANDLER_H

#include <Arduino.h>

namespace InterruptHandler {

    void ai0();
    void ai1();
    void control_enable();

    extern volatile long angle;
    extern volatile bool enable;
    extern volatile unsigned long lastDebounceTime;
    constexpr unsigned long debounceDelay = 200;

    bool isEnable();
};

#endif // INTERRUPTHANDLER_H