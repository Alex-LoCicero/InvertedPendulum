#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
public:
    Timer();
    void start();
    void stop();
    void reset();
    unsigned long elapsed();
    bool isRunning();

private:
    unsigned long startTime;
    unsigned long stopTime;
    bool running;
};

#endif // TIMER_H