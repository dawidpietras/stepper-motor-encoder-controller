#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <micro_ros_platformio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

rcl_publisher_t my_publisher;
sensor_msgs__msg__JointState encoder_joint;

const double RADIAN_CNT = 0.00392699;

const uint8_t PIN_A = 4;
const uint8_t PIN_B = 5;

const uint8_t LED = 48;
const uint8_t NUMPIXELS = 1;


volatile long encoder_counter = 0;
long current_count = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, LED, NEO_GRB + NEO_KHZ800);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    delay(500);
  }
}

void IRAM_ATTR counter_a_interrupt(){
    if (digitalRead(PIN_A) != digitalRead(PIN_B)) {
        encoder_counter++;
    }
    else {
        encoder_counter--;
    }
}

void IRAM_ATTR counter_b_interrupt(){
  if (digitalRead(PIN_B) == (digitalRead(PIN_A))){
    encoder_counter++;
  }
  else {
    encoder_counter--;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){

  RCL_UNUSED(last_call_time);

    if (timer != NULL){

      double raw_angle = encoder_counter * RADIAN_CNT;

      double radian_bounded_angle = fmod(raw_angle, TWO_PI);

      if (radian_bounded_angle < 0){
        radian_bounded_angle += TWO_PI;
      }

      encoder_joint.position.data[0] = radian_bounded_angle;

      int64_t time_ns = rmw_uros_epoch_nanos();
      encoder_joint.header.stamp.sec = time_ns / 1000000000;
      encoder_joint.header.stamp.nanosec = time_ns % 1000000000;
    

    RCSOFTCHECK(rcl_publish(&my_publisher, &encoder_joint, NULL));
    }
}



void setup(){
    
    pixels.begin();

    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
    pixels.show();
    delay(1000);
    Serial.begin(115200);
    
    while(!Serial){
    
      delay(10);
    }
    
    
    set_microros_serial_transports(Serial);

    
    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "encoder", "", &support));

    RCCHECK(rclc_publisher_init_default(&my_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState) , "joint_states"));

    sensor_msgs__msg__JointState__init(&encoder_joint);

    rosidl_runtime_c__String__Sequence__init(&encoder_joint.name, 1);
    rosidl_runtime_c__double__Sequence__init(&encoder_joint.position, 1);

    rosidl_runtime_c__String__assign(&encoder_joint.name.data[0], "joint_1");


    const unsigned int frequency_ms = 100;

    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(frequency_ms), timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    pinMode(PIN_A, INPUT_PULLUP);
    pinMode(PIN_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PIN_A), counter_a_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), counter_b_interrupt, CHANGE);

    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
}

void loop(){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(10);
}