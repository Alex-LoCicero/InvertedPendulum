#include "DataRecorder.h"

DataRecorder::DataRecorder(int bufferSize, uint8_t dataFieldsToRecord)
    : bufferSize(bufferSize), bufferIdx(0), dataFieldsToRecord(dataFieldsToRecord) {
    dataBuffer = new long[bufferSize];
}

void DataRecorder::recordData(long timestamp, long motor_sp, long motor_pos, long joint_pos) {
    int fieldsToRecord = __builtin_popcount(dataFieldsToRecord);
    if (bufferIdx + fieldsToRecord <= bufferSize) {
        if (dataFieldsToRecord & TIMESTAMP) dataBuffer[bufferIdx++] = timestamp;
        if (dataFieldsToRecord & MOTOR_SP) dataBuffer[bufferIdx++] = motor_sp;
        if (dataFieldsToRecord & MOTOR_POS) dataBuffer[bufferIdx++] = motor_pos;
        if (dataFieldsToRecord & JOINT_POS) dataBuffer[bufferIdx++] = joint_pos;
    }
}

void DataRecorder::saveData() {
    for (int i = 0; i < bufferIdx; i += __builtin_popcount(dataFieldsToRecord)) {
        for (int j = 0; j < 4; j++) {
            if (dataFieldsToRecord & (1 << j)) {
                Serial.print(dataFieldNames[j]);
                Serial.print(": ");
                Serial.print(dataBuffer[i + j]);
                Serial.print(", ");
            }
        }
        Serial.println();
        delay(100);
    }
    resetBuffer();
}

void DataRecorder::resetBuffer() {
    bufferIdx = 0;
    for (int i = 0; i < bufferSize; i++) {
        dataBuffer[i] = 0;
    }
}