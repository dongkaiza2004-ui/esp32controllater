#include "pinout.h"
#include "motor.h"
#include "pid.h"
#include "maths.h"
const int freq = 50;
const int motor_r_pwmChannel = 0;
const int motor_l_pwmChannel = 1;
// const int motor_x_pwmChannel = 2;
// const int motor_s_pwmChannel = 3;
const int resolution = 8;

volatile uint16_t encoder_freq_r_a = 0; 
volatile uint16_t encoder_freq_r_b = 0;
volatile uint16_t encoder_freq_l_a = 0;
volatile uint16_t encoder_freq_l_b = 0;

volatile unsigned long lastTime_r_a = 0;
volatile unsigned long lastTime_r_b = 0;
volatile unsigned long lastTime_l_a = 0;
volatile unsigned long lastTime_l_b = 0;

bool flag_motor_r_a = false;
bool flag_motor_l_a = false;
bool flag_motor_r_b = false;
bool flag_motor_l_b = false;

volatile int32_t encoder_count_r = 0;
volatile int32_t encoder_count_l = 0;
void control_x(){
    pwm_x = constrain(pwm_x, 0, 255);
    // Serial.printf("pwm_x = %d\n", pwm_x);
    Motor_Control(X_MOTOR, pwm_x, FORWARD); 
}
uint32_t angleToDuty(int angle) {
  return map(angle, 0, 180, 1638, 8192); // 0.5ms–2.5ms duty
}
// void servo_enable(int angle) {
//     ledcAttachPin(SERVO_PIN, motor_s_pwmChannel);
//     ledcWrite(motor_s_pwmChannel, angleToDuty(angle));
// }

// void servo_disable(int angle) {
//     ledcWrite(motor_s_pwmChannel, angleToDuty(angle));
//     ledcDetachPin(SERVO_PIN);
// }
void Control_Task() {
    freq_r = Motor_Encoder_Right_A();
    freq_l = Motor_Encoder_Left_A();
    rpm_r = (freq_r / pulses_per_rev) * 60.0;
    rpm_l = (freq_l / pulses_per_rev) * 60.0;
    static int32_t last_encoder_count_r = 0;
    static int32_t last_encoder_count_l = 0;

    int32_t delta_r = encoder_count_r - last_encoder_count_r;
    int32_t delta_l = encoder_count_l - last_encoder_count_l;

    last_encoder_count_r = encoder_count_r;
    last_encoder_count_l = encoder_count_l;

    dir_r = (delta_r > 0) ? 1 : ((delta_r < 0) ? -1 : 0);
    dir_l = (delta_l > 0) ? 1 : ((delta_l < 0) ? -1 : 0);
    if (dir_r == -1) freq_r = -freq_r;
    if (dir_l == -1) freq_l = -freq_l;

    // if (abs(target_freq_r) < 1 && abs(freq_r) < 100 ){
    //     Motor_Control(R_MOTOR, 0, FORWARD); 
    //     target_freq_r = 0;
    //     freq_r = 0;
    //     return;
    // }
    // if (abs(target_freq_l) < 1 && abs(freq_l) < 100 ){
    //     Motor_Control(L_MOTOR, 0, FORWARD);
    //     target_freq_l = 0;
    //     freq_l = 0;
    //     return;
    // }
    bool stop_condition = abs(target_freq_r) < 1 && abs(target_freq_l) < 1;
    bool speed_low = abs(freq_r) < 100 && abs(freq_l) < 100;

    if (stop_condition && speed_low) {
        Motor_Control(R_MOTOR, 0, FORWARD); 
        Motor_Control(L_MOTOR, 0, FORWARD);
        return;
    }

    pwm_r = pid_r.calculate(target_freq_r, freq_r);
    pwm_l = pid_l.calculate(target_freq_l, freq_l);

    Motor_Control(R_MOTOR, abs(pwm_r), pwm_r >= 0 ? FORWARD : BACKWARD);
    Motor_Control(L_MOTOR, abs(pwm_l), pwm_l >= 0 ? FORWARD : BACKWARD);
}
void IRAM_ATTR Encoder_Right_A() {

    flag_motor_r_a = true;
    unsigned long currentTime = micros();
    uint16_t encoder_cycle_r = currentTime - lastTime_r_a;

    if(encoder_cycle_r > 0)
        encoder_freq_r_a = 1000000 / encoder_cycle_r;     // Encoder frequency

    lastTime_r_a = currentTime;
}

void IRAM_ATTR Encoder_Right_B(){
    bool a = digitalRead(R_MOTOR_PWM_A);
    bool b = digitalRead(R_MOTOR_PWM_B);

    if (a != b)
        encoder_count_r++;  
    else
        encoder_count_r--;  


}

void IRAM_ATTR Encoder_Left_A() {
    flag_motor_l_a = true;
    unsigned long currentTime = micros();
    uint16_t encoder_cycle_l = currentTime - lastTime_l_a;

    if(encoder_cycle_l > 0)
        encoder_freq_l_a = 1000000 / encoder_cycle_l;      // Encoder frequency

    lastTime_l_a = currentTime;
}

void IRAM_ATTR Encoder_Left_B() {
    bool a = digitalRead(L_MOTOR_PWM_A);
    bool b = digitalRead(L_MOTOR_PWM_B);
    if (a == b)
        encoder_count_l++;  
    else
        encoder_count_l--;  
}

void Motor_Init(void){
    // Serial.println("Motor Init!");
    /*
        Motor Right Configuration
    */
    pinMode(R_MOTOR_EN, OUTPUT);
    pinMode(R_MOTOR_INT_A, OUTPUT);
    pinMode(R_MOTOR_INT_B, OUTPUT);    
    ledcSetup(motor_r_pwmChannel, freq, resolution);
    ledcAttachPin(R_MOTOR_EN, motor_r_pwmChannel);
    ledcWrite(motor_r_pwmChannel, 0);

    /*
        Motor Left Configuration
    */
    pinMode(L_MOTOR_EN, OUTPUT);
    pinMode(L_MOTOR_INT_A, OUTPUT);
    pinMode(L_MOTOR_INT_B, OUTPUT);    
    ledcSetup(motor_l_pwmChannel, freq, resolution);
    ledcAttachPin(L_MOTOR_EN, motor_l_pwmChannel);
    ledcWrite(motor_l_pwmChannel, 0);

    pinMode(X_MOTOR_EN, OUTPUT);
    pinMode(X_MOTOR_INT_A, OUTPUT);
    pinMode(X_MOTOR_INT_B, OUTPUT);    
    ledcSetup(motor_x_pwmChannel, freq, resolution);
    // ledcAttachPin(X_MOTOR_EN, motor_x_pwmChannel);
    // ledcWrite(motor_x_pwmChannel, 0);
    
    pinMode(R_MOTOR_PWM_A, INPUT_PULLUP);
    pinMode(R_MOTOR_PWM_B, INPUT_PULLUP);
    pinMode(L_MOTOR_PWM_A, INPUT_PULLUP);
    pinMode(L_MOTOR_PWM_B, INPUT_PULLUP);

    ledcSetup(motor_s_pwmChannel, freq, SERVO_RESOLUTION);
    // ledcAttachPin(SERVO_PIN, motor_s_pwmChannel);
    // ledcWrite(motor_s_pwmChannel, 0);

    attachInterrupt(digitalPinToInterrupt(R_MOTOR_PWM_B), Encoder_Right_B, FALLING);
    attachInterrupt(digitalPinToInterrupt(L_MOTOR_PWM_A), Encoder_Left_A, FALLING);

    attachInterrupt(digitalPinToInterrupt(R_MOTOR_PWM_A), Encoder_Right_A, FALLING);
    attachInterrupt(digitalPinToInterrupt(L_MOTOR_PWM_B), Encoder_Left_B, FALLING);
}

void Motor_Control(uint8_t motor, uint8_t speed, bool dir)
{
    if(motor == R_MOTOR){
        if (FORWARD == dir) {
            // Serial.println("Motor R Forward");
            digitalWrite(R_MOTOR_INT_A, HIGH);
            digitalWrite(R_MOTOR_INT_B, LOW);
            ledcWrite(motor_r_pwmChannel, speed);
        }
        else if (BACKWARD == dir) {
            // Serial.println("Motor R Backward");
            digitalWrite(R_MOTOR_INT_A, LOW);
            digitalWrite(R_MOTOR_INT_B, HIGH);
            ledcWrite(motor_r_pwmChannel, speed);
        }
        
        
    }
    else if(motor == L_MOTOR){
        if (FORWARD == dir) {
            // Serial.println("Motor L Forward");
            digitalWrite(L_MOTOR_INT_A, LOW);
            digitalWrite(L_MOTOR_INT_B, HIGH);
            ledcWrite(motor_l_pwmChannel, speed);
        }
        else if (BACKWARD == dir) {
            // Serial.println("Motor L Backward");
            digitalWrite(L_MOTOR_INT_A, HIGH);
            digitalWrite(L_MOTOR_INT_B, LOW);
            ledcWrite(motor_l_pwmChannel, speed);
        }
    }
    else if(motor == X_MOTOR){
        if (FORWARD == dir) {
            // Serial.println("Motor L Forward");
            digitalWrite(X_MOTOR_INT_A, LOW);
            digitalWrite(X_MOTOR_INT_B, HIGH);
            ledcWrite(motor_x_pwmChannel, speed);
        }
        else if (BACKWARD == dir) {
            // Serial.println("Motor L Backward");
            digitalWrite(X_MOTOR_INT_A, HIGH);
            digitalWrite(X_MOTOR_INT_B, LOW);
            ledcWrite(motor_x_pwmChannel, speed);
        }
    }
}
uint16_t Motor_Encoder_Left_A(void){
    unsigned long now = micros();
    if (now - lastTime_l_a > 100000) { // 500ms không có xung
        return 0;
    }
    return encoder_freq_l_a;
}

uint16_t Motor_Encoder_Left_B(void){
    return encoder_freq_l_b;
}

uint16_t Motor_Encoder_Right_A(void){
    unsigned long now = micros();
    if (now - lastTime_r_a > 100000) { // 500ms không có xung
        return 0;
    }
    return encoder_freq_r_a;
}

uint16_t Motor_Encoder_Right_B(void){
    return encoder_freq_r_b;
}

