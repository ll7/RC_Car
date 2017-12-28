/*
 * two servos
 * subscribe to two topics
 * 
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Int16.h> // for US sensor data
#include <Servo.h>

#define PIN_SERVO_STEERING 9
#define PIN_SERVO_ESC 10

ros::NodeHandle nh;

Servo servo_steering;
Servo servo_esc;

// steering callback
void servo_steering_cb( const std_msgs::Int16& steering_angle){
  servo_steering.write(steering_angle.data);
}
ros::Subscriber<std_msgs::Int16> sub_Steer("steering_servo", &servo_steering_cb);

// throttle callback
void servo_esc_cb( const std_msgs::Int16& esc_throttle){
  servo_esc.write(esc_throttle.data);
}
ros::Subscriber<std_msgs::Int16> sub_ESC("esc_throttle", &servo_esc_cb);



void setup() {
  // put your setup code here, to run once:

  servo_steering.attach(PIN_SERVO_STEERING);
  servo_esc.attach(PIN_SERVO_ESC);
  nh.initNode();
  nh.subscribe(sub_Steer);
  nh.subscribe(sub_ESC);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
