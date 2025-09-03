#include "pid.h"
#include <Arduino.h>
#include "pinout.h"


float pid::calculate(float setpoint, float input){
    // curent_time = millis();
    float error = setpoint - input;
    integral += error;
    float derivative = (error - previous_error);
    float output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    // previous_time = curent_time;
    // Serial.printf("Output:=%.2f\n", output);
    if(output < output_min)
        output = output_min;//-255
  
    if(output > output_max)
        output = output_max;
    return output;
}

void pid::reset(){
    integral = 0;
    //previous_error = 0;
}
bool pid::is_settled(float setpoint, float input, float threshold) {
    return fabs(setpoint - input) < threshold;
}
pid::pid(float kp_, float ki_, float kd_)
    : kp(kp_), ki(ki_), kd(kd_) {}

pid::~pid()
{
}
