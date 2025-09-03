#include <Arduino.h>
#include "motor.h"
#include "pid.h"
#include "pinout.h"
#include "maths.h"
#include "ros_esp32_cmdvel.h"
#include "esp_task_wdt.h"

unsigned long last_time = 0;
const unsigned long interval = 500; 
bool servo_enabled = false; 
bool pwmx_enabled = false; 
TaskHandle_t ROSHandle;
TaskHandle_t MotorHandle;
void ROS_Task(void *parameter) {
  esp_task_wdt_add(NULL); 
  while (1) {
    ROS_Spin(); 
    esp_task_wdt_reset(); 
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}
void Motor_Task(void *parameter) {
  esp_task_wdt_add(NULL); 
  for (;;) {
    Control_Task(); 
    esp_task_wdt_reset(); 
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(115200);
  Motor_Init();
  pinMode(R_MOTOR_PWM_A, INPUT_PULLUP);
  pinMode(L_MOTOR_PWM_A, INPUT_PULLUP);
  pid_r.reset();
  pid_l.reset();
  last_time = millis();
  ROS_Init();
  esp_task_wdt_init(2, true); 
  xTaskCreatePinnedToCore(ROS_Task, "ROS_Task",8192, NULL, 1, &ROSHandle, 0);
  xTaskCreatePinnedToCore(Motor_Task, "Motor_Task",4096, NULL, 2, &MotorHandle, 1);


}

void loop() {
  // unsigned long now = millis();
  // // speed_R_L(v, w, target_freq_l, target_freq_r);

  // if (now - last_time >= interval) {
  //   // Serial.printf("L: RPM=%.2f PWM=%d\tR: RPM=%.2f PWM=%d\n\n", rpm_l, pwm_l, rpm_r, pwm_r);
  //   // Serial.printf("Freq_r_setpoint = %.2f, Freq_l_setpoint = %.2f\n\n",target_freq_r,target_freq_l);
  //   Serial.printf("Freq_r_input = %.2f,Freq_l_input =%.2f\n\n",freq_r,freq_l);
  //   // Serial.printf("dir_r =%d,dir_l=%d\n\n", dir_r,dir_l);
  //   // freq_to_speed(freq_l,freq_r);
  //   last_time = now;
  // }
  int angle = int(k);
  pwm_x = int(x);

  if (angle < 100) {
    if (servo_enabled) {
      ledcAttachPin(SERVO_PIN, motor_s_pwmChannel);
      servo_enabled = false;
      Serial.println("bat servo\n");
    }
    ledcWrite(motor_s_pwmChannel, angleToDuty(angle));
  } 
  else {
    if (!servo_enabled) {
      ledcWrite(motor_s_pwmChannel, angleToDuty(angle)); 
      delay(1000);
      ledcDetachPin(SERVO_PIN);
      servo_enabled = true;
      Serial.printf("tat servo\n");
    }
  }
  // control_x();
  if(pwm_x != 0){
    if (pwmx_enabled) {
      ledcAttachPin(X_MOTOR_EN, motor_x_pwmChannel);
      pwmx_enabled = false;
      Serial.printf("bat thu bong\n");
    }
    control_x();
  }
  else {
    if (!pwmx_enabled) {
      control_x();
      delay(1000);
      ledcDetachPin(X_MOTOR_EN);
      pwmx_enabled = true;
      Serial.printf("tat thu bong\n");
    }
  }
}
