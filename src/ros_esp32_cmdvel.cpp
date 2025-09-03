#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <ArduinoJson.h>
#include "maths.h"
#include <WiFi.h>
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String msg_pub;
std_msgs__msg__String msg_sub;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

#define LED_PIN 4
#define RCCHECK(x) if((x) != RCL_RET_OK){Serial.println("❌ RCCHECK lỗi"); error_loop();}
#define RCSOFTCHECK(x) if((x) != RCL_RET_OK){}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
// const char * ssid = "PTIT-RD-01";
// const char * password = "11246879";
// const char * ip_address = "192.168.0.131";
// const uint16_t port = 8888;
const char * ssid = "IEC lab";
const char * password = "roboticsptit";
const char * ip_address = "192.168.2.176";
const uint16_t port = 8888;
void subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, msg->data.data);
  if (err) {
    return;
  }
  vx = doc["vx"] | 0.0;
  vx = -vx;
  vy = doc["vy"] | 0.0;
  w  = doc["w"]  | 0.0;
  x = doc["x"]  | 0.0;
  y = doc["y"]  | 0.0;
  k = doc["k"]  | 0.0;
  speed_R_L(vx, vy, w, target_freq_l, target_freq_r);
}
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n WiFi connected!");
}
void ROS_Init() {
  Serial.begin(115200);
  delay(1000);
  connectToWiFi(); 
  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)ip_address, port);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_later_node", "", &support));

  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp_vel"  
  ));

  msg_sub.data.data = (char *)malloc(100);
  msg_sub.data.capacity = 100;
  msg_sub.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

}

void ROS_Spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
