cmake_minimum_required(VERSION 3.0)

project(InvertedPendulum)

# src files 
set(SOURCES
    src/MotorController.cpp
    src/PIDController.cpp
    main/main.ino
)

# header files 
set(HEADERS
    inc/MotorController.h
    inc/PIDController.h
)

# Arduino core library 
set(ARDUINO_CORE_PATH "../libraries")
include_directories(${ARDUINO_CORE_PATH})
include_directories(inc)