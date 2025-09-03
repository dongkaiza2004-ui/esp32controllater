#ifndef _MOTOR_H
#define _MOTOR_H

#include <Arduino.h>

#define FORWARD   0
#define BACKWARD  1

#define R_MOTOR   0
#define L_MOTOR   1
#define X_MOTOR   2
#define PWM_MIN   70
#define PWM_MAX   255

#define SERVO_RESOLUTION 16
const int motor_x_pwmChannel = 2;
const int motor_s_pwmChannel = 7;
void Motor_Init(void);
void control_x(void);
void Control_Task(void);
void Motor_Control(uint8_t motor, uint8_t speed, bool dir);
uint32_t angleToDuty(int angle);
// void servo_enable(int angle) ;
// void servo_disable(int angle);
// void Motor_Backward(uint8_t motor, uint8_t speed, bool dir);
uint16_t Motor_Encoder_Left_A(void);
uint16_t Motor_Encoder_Left_B(void);
uint16_t Motor_Encoder_Right_A(void);
uint16_t Motor_Encoder_Right_B(void);
void computeRPM(int a, float b, float base_rpm, float &rpm_l, float &rpm_r);
#endif