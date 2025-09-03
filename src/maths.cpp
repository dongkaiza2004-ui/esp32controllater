#include "maths.h"
#include <Arduino.h>
#include "motor.h"
float vx = 0.163;
float vy = 0;
float w = 0;
float L = 0.28; 
float W = 0.26;
float R = 0.045;
float x =0;
float y =0;
float k =0;
float target_freq_l = 0;
float target_freq_r = 0;

float freq_r = 0;
float freq_l = 0;

float rpm_r;
float rpm_l;

int pwm_r = 0;
int pwm_l = 0;
int pwm_x = 0;

pid pid_r(0.2, 0.01, 0.005);
pid pid_l(0.2, 0.01, 0.005);

int dir_r = 0;
int dir_l = 0;

volatile bool control_flag = false;

const int pulses_per_rev = 770;
// float RPS_to_speed(float RPS){
//     return (2*pi*RPS*R);
// }
float speed_to_RPS(float v){
    return v/(R*2*pi);
}
void speed_R_L(float vx, float vy, float w, float &target_freq_r, float &target_freq_l) {
    float k = (L + W)/2;
    float v_fl = vx - vy - k * w;
    float v_fr = vx + vy + k * w;
    float v_rl = vx + vy - k * w;
    float v_rr = vx - vy + k * w;

    float RPS_r = speed_to_RPS(v_rr);
    float RPS_l = speed_to_RPS(v_rl);
    target_freq_r = RPS_r*pulses_per_rev;
    target_freq_l = RPS_l*pulses_per_rev;

    // Serial.printf("RPS_r = %.2f, RPS_l = %.2f\n", RPS_r,RPS_l);
    // Serial.printf("Freq_r1 = %.2f, Freq_l1 = %.2f\n", RPS_r*pulses_per_rev,target_freq_l);

}
// void freq_to_speed(float freq1_l, float freq1_r) {
//     float RPS_l = freq1_l / pulses_per_rev;
//     float RPS_r = freq1_r / pulses_per_rev;

//     float v_l = RPS_to_speed(RPS_l);
//     float v_r = RPS_to_speed(RPS_r);

//     float v = (v_l + v_r) / 2.0f;
//     float w = (v_r - v_l) / L;
//     Serial.printf("v = %.2f -------- w = %.2f \n", v,w);
// }