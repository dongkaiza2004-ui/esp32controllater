#ifndef __MATHS_H
#define __MATHS_H
#include "pid.h"
#define pi 3.1415926f
extern const int pulses_per_rev;
extern float R; 
extern float L;
extern float W;
extern float vx;
extern float vy;
extern float w;
extern float x;
extern float y;
extern float k;
extern float base_freq;
extern float target_freq_l;
extern float target_freq_r;
extern float rpm_r;
extern float rpm_l;
extern int pwm_r;
extern int pwm_l;
extern int pwm_x;
extern pid pid_r;
extern pid pid_l;
extern float freq_r;
extern float freq_l;
extern volatile bool control_flag;
extern int dir_r;
extern int dir_l;
float RPS_to_speed(float RPS);
float speed_to_RPS(float v);
void speed_R_L(float vx, float vy, float w, float &freq1_l, float &freq1_r);
void freq_to_speed(float freq_l, float freq_r); 
#endif