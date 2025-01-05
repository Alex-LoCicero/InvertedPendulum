#ifndef INTERRUPTHANDLER_H
#define INTERRUPTHANDLER_H

#include <Arduino.h>

class InterruptHandler {
public:
    static void ai0();
    static void ai1();
    static void control_enable();

    static volatile long angle;
    static volatile bool enable;
    static volatile unsigned long lastDebounceTime;
    static const unsigned long debounceDelay = 200;

    static bool isEnable();
};

#endif // INTERRUPTHANDLER_H