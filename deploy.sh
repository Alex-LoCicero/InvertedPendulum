#!/bin/bash

# Compile the Arduino sketch
arduino-cli compile --fqbn arduino:avr:uno --build-path build --library inc/ main/
if [ $? -ne 0 ]; then
    echo "Compilation failed"
    exit 1
fi

# Upload the compiled sketch to the Arduino board
arduino-cli upload --fqbn arduino:avr:uno --port COM3 --input-dir build main
if [ $? -ne 0 ]; then
    echo "Upload failed"
    exit 1
fi

# Open the serial monitor
arduino-cli monitor -p COM3 -b arduino:avr:uno