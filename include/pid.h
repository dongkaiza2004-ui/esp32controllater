#ifndef __PID_H
#define __PID_H


class pid
{
private:
    float integral = 0.0;
    float previous_error = 0.0;
    float kp = 0.1;                     // Proportional gain
    float ki = 0.05;                    // Integral gain
    float kd = 0.01;                    // Derivative gain
    float output_max = 255;
    float output_min = -255;
    int previous_time = 0;
    int curent_time = 0;
public:
    pid(float kp, float ki, float kd);
    float calculate(float setpoint, float input);
    bool is_settled(float setpoint, float input, float threshold = 0.5);
    void reset();
    ~pid();
};

#endif
