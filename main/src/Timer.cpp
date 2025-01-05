#include "Timer.h"

Timer::Timer() : startTime(0), stopTime(0), running(false) {}

void Timer::start() {
    if (!running) {
        startTime = millis();
        running = true;
    }
}

void Timer::stop() {
    if (running) {
        stopTime = millis();
        running = false;
    }
}

void Timer::reset() {
    startTime = millis();
    if (running) {
        stopTime = startTime;
    }
}

unsigned long Timer::elapsed() {
    if (running) {
        return millis() - startTime;
    } else {
        return stopTime - startTime;
    }
}

bool Timer::isRunning() {
    return running;
}