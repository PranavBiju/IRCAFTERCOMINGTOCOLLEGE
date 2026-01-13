#include <micro_ros_arduino.h>

#include <Arduino.h>
#include "driver/pcnt.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>


double output1, output2, output3, output4;
unsigned long start;


#define M1_PWM 4
#define M1_DIR 15
#define ENC1_A 32
#define ENC1_B 33

#define M2_PWM 27
#define M2_DIR 14
#define ENC2_A 23
#define ENC2_B 19

#define M3_PWM 17
#define M3_DIR 16
#define ENC3_A 35
#define ENC3_B 34

#define M4_PWM 25
#define M4_DIR 28
#define ENC4_A 18
#define ENC4_B 5


const float WHEEL_RADIUS = 0.2;

const float ENC1_PPR = 1410.0;
const float ENC2_PPR = 1410.0;
const float ENC3_PPR = 1410.0;
const float ENC4_PPR = 1410.0;


float Kp = 75.0;
float Ki = 0.08;
float Kd = 0.03;


double target_v1 = 0.0;
double target_v2 = 0.0;
double target_v3 = 0.0;
double target_v4 = 0.0;


class MDD10A {
  public:
    MDD10A(int pwm, int dir) : _pwm_pin(pwm), _dir_pin(dir) {
      pinMode(_pwm_pin, OUTPUT);
      pinMode(_dir_pin, OUTPUT);
    }
    void run(int pwr) {
      if (pwr > 255) pwr = 255;
      if (pwr < -255) pwr = -255;
      if (abs(pwr) < 10) pwr = 0;

      digitalWrite(_dir_pin, pwr > 0 ? HIGH : LOW);
      analogWrite(_pwm_pin, abs(pwr));
    }
  private:
    int _pwm_pin, _dir_pin;
};

MDD10A motor1(M1_PWM, M1_DIR);
MDD10A motor2(M2_PWM, M2_DIR);
MDD10A motor3(M3_PWM, M3_DIR);
MDD10A motor4(M4_PWM, M4_DIR);


pcnt_unit_t pcnt1 = PCNT_UNIT_0;
pcnt_unit_t pcnt2 = PCNT_UNIT_1;
pcnt_unit_t pcnt3 = PCNT_UNIT_2;
pcnt_unit_t pcnt4 = PCNT_UNIT_3;


unsigned long prevT_us = 0;

double ie1 = 0, ie2 = 0, ie3 = 0, ie4 = 0;
double pe1 = 0, pe2 = 0, pe3 = 0, pe4 = 0;


rcl_node_t node;
rcl_subscription_t drive_sub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__Float64MultiArray drive_msg;
unsigned long last_cmd_time = 0;


void setupEncoder(pcnt_unit_t unit, int A, int B) {
  pcnt_config_t cfg = {
    .pulse_gpio_num = A,
    .ctrl_gpio_num  = B,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_INC,
    .neg_mode       = PCNT_COUNT_DEC,
    .counter_h_lim  = 30000,
    .counter_l_lim  = -30000,
    .unit           = unit,
    .channel        = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&cfg);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}


void driveCallback(const void * msg) {
  const std_msgs__msg__Float64MultiArray * arr =
    (const std_msgs__msg__Float64MultiArray *)msg;

  if (arr->data.size < 4) return;

  target_v1 = arr->data.data[0];
  target_v2 = arr->data.data[1];
  target_v3 = arr->data.data[2];
  target_v4 = arr->data.data[3];

  last_cmd_time = millis();
}


void setup() {
  Serial.begin(115200);

  setupEncoder(pcnt1, ENC1_A, ENC1_B);
  setupEncoder(pcnt2, ENC2_A, ENC2_B);
  setupEncoder(pcnt3, ENC3_A, ENC3_B);
  setupEncoder(pcnt4, ENC4_A, ENC4_B);

  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "pid_motor_node", "", &support);

  rclc_subscription_init_default(
    &drive_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/drive_controller/commands"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &drive_sub,
    &drive_msg,
    &driveCallback,
    ON_NEW_DATA
  );

  prevT_us = micros();
  start = millis();
}


void loop() {

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  if (millis() - last_cmd_time > 50) {
    target_v1 = 0.0;
    target_v2 = 0.0;
    target_v3 = 0.0;
    target_v4 = 0.0;
  }

  unsigned long currT_us = micros();
  float dt = (currT_us - prevT_us) * 1e-6;
  if (dt <= 0.0 || dt > 0.1) return;

  int16_t c1, c2, c3, c4;
  pcnt_get_counter_value(pcnt1, &c1);
  pcnt_get_counter_value(pcnt2, &c2);
  pcnt_get_counter_value(pcnt3, &c3);
  pcnt_get_counter_value(pcnt4, &c4);

  pcnt_counter_clear(pcnt1);
  pcnt_counter_clear(pcnt2);
  pcnt_counter_clear(pcnt3);
  pcnt_counter_clear(pcnt4);

  double v1 = (c1 / dt / ENC1_PPR) * (2 * PI * WHEEL_RADIUS);
  double v2 = (c2 / dt / ENC2_PPR) * (2 * PI * WHEEL_RADIUS);
  double v3 = (c3 / dt / ENC3_PPR) * (2 * PI * WHEEL_RADIUS);
  double v4 = (c4 / dt / ENC4_PPR) * (2 * PI * WHEEL_RADIUS);

  double e1 = target_v1 - v1;
  double e2 = target_v2 - v2;
  double e3 = target_v3 - v3;
  double e4 = target_v4 - v4;

  ie1 = constrain(ie1 + e1 * dt, -20, 20);
  ie2 = constrain(ie2 + e2 * dt, -20, 20);
  ie3 = constrain(ie3 + e3 * dt, -20, 20);
  ie4 = constrain(ie4 + e4 * dt, -20, 20);

  output1 += Kp*e1 + Ki*ie1 + Kd*((e1 - pe1)/dt);
  output2 += Kp*e2 + Ki*ie2 + Kd*((e2 - pe2)/dt);
  output3 += Kp*e3 + Ki*ie3 + Kd*((e3 - pe3)/dt);
  output4 += Kp*e4 + Ki*ie4 + Kd*((e4 - pe4)/dt);

  motor1.run((int)output1);
  motor2.run((int)output2);
  motor3.run((int)output3);
  motor4.run((int)output4);

  pe1 = e1;
  pe2 = e2;
  pe3 = e3;
  pe4 = e4;

  prevT_us = currT_us;

  delay(20);
}