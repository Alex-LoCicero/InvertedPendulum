#ifdef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "PID_v1.h"

class PIDController {  
    public:
        // Constructor
        PIDController(float* input, float* output, float*setpoint, float kp, float ki, float kd);

        // public methods 
        void compute(int encoderValue); // updates output
        void setOutputLimits(float min, float max);
        void setTunings(float kp, float ki, float kd);
        void setSampleTime(float ts); 
    
    private:
        PID pid;
};

#endif // PIDCONTROLLER_H