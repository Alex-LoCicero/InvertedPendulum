#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <PID_v1_bc.h>

class PIDController {  
    public:
        // Constructor
        PIDController(double* input, double* output, double* setpoint, double kp, double ki, double kd);

        // public methods 
        void compute(); // updates output
        void setOutputLimits(float min, float max);
        void setTunings(float kp, float ki, float kd);
        void setSampleTime(float ts); 
    
    private:
        PID pid;
};

#endif // PIDCONTROLLER_H
