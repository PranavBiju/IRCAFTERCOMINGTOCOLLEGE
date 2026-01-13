#include <micro_ros_arduino.h>
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>

// ---------------- CONFIG ----------------

#define NUM_WHEELS 4
#define LED_PIN 13
#define PI 3.1415926

// ---- YOUR PINS ----
const int PWM_PINS[NUM_WHEELS]   = { 6, 7, 8, 9 };
const int DIR_PINS[NUM_WHEELS]   = { 23, 25, 27, 29 };
const int LIMIT_PINS[NUM_WHEELS] = { 30, 32, 34, 36 };

const float DEG_PER_STEP = 22.5;   
const int PWM_VAL = 90;
const unsigned long DEBOUNCE_MS = 50;

// ---------------- STATE ----------------

volatile bool step_hit[NUM_WHEELS] = { false };

int steps_remaining[NUM_WHEELS] = { 0 };
unsigned long last_step_time[NUM_WHEELS] = { 0 };

// ---------------- ISRs ----------------

void isr0() { step_hit[0] = true; }
void isr1() { step_hit[1] = true; }
void isr2() { step_hit[2] = true; }
void isr3() { step_hit[3] = true; }

void (*isr_list[NUM_WHEELS])() = { isr0, isr1, isr2, isr3 };

// ---------------- ROS ----------------

rcl_node_t node;
rcl_subscription_t sub;
rcl_publisher_t pub;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__Float64MultiArray pivot_msg;
std_msgs__msg__Float64MultiArray debug_msg;

// ---------------- CALLBACK ----------------

void pivot_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg =
    (const std_msgs__msg__Float64MultiArray *)msgin;

  if (msg->data.size < NUM_WHEELS) return;

  for (int i = 0; i < NUM_WHEELS; i++) {
    float deg = msg->data.data[i] * (180.0 / PI);
    steps_remaining[i] = (int)round(deg / DEG_PER_STEP);
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_WHEELS; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIMIT_PINS[i]),
                    isr_list[i], FALLING);
  }

  set_microros_transports();
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pivot_controller", "", &support);

  rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/pivot_controller/commands"
  );

  rclc_publisher_init_default(
    &pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/pivot_controller/debug"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &sub, &pivot_msg, &pivot_callback, ON_NEW_DATA
  );

  // Debug buffer
  static double debug_buf[NUM_WHEELS];
  debug_msg.data.data = debug_buf;
  debug_msg.data.capacity = NUM_WHEELS;
  debug_msg.data.size = NUM_WHEELS;
}

// ---------------- LOOP ----------------

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  for (int i = 0; i < NUM_WHEELS; i++) {

    // ---------- Direction ----------
    if (steps_remaining[i] > 0) {
      digitalWrite(DIR_PINS[i], LOW);
    } else if (steps_remaining[i] < 0) {
      digitalWrite(DIR_PINS[i], HIGH);
    }

    // ---------- Motor enable ----------
    if (steps_remaining[i] != 0) {
      analogWrite(PWM_PINS[i], PWM_VAL);
    } else {
      analogWrite(PWM_PINS[i], 0);
    }

    // ---------- Step handling ----------
    if (step_hit[i]) {
      unsigned long now = millis();

      if (now - last_step_time[i] >= DEBOUNCE_MS) {
        if (steps_remaining[i] > 0) steps_remaining[i]--;
        else if (steps_remaining[i] < 0) steps_remaining[i]++;

        last_step_time[i] = now;
      }

      step_hit[i] = false;
    }

    debug_msg.data.data[i] = steps_remaining[i];
  }

  // ---------- Publish debug ----------
  rcl_publish(&pub, &debug_msg, NULL);
}