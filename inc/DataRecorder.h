#ifndef DATARECORDER_H
#define DATARECORDER_H

#include <Arduino.h>

enum DataField {
    TIMESTAMP = 1 << 0,
    MOTOR_SP = 1 << 1,
    MOTOR_POS = 1 << 2,
    JOINT_POS = 1 << 3
};

class DataRecorder {
public:
    DataRecorder(int bufferSize, uint8_t dataFieldsToRecord);
    void recordData(long timestamp, long motor_sp, long motor_pos, long joint_pos);
    void saveData();
    void resetBuffer();

private:
    int bufferSize;
    int bufferIdx;
    long* dataBuffer;
    uint8_t dataFieldsToRecord;
    const char* dataFieldNames[4] = {"timestamp", "motor_sp", "motor_pos", "joint_pos"};
};

#endif // DATARECORDER_H