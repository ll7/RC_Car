/*
 * A program that communicates with ROS.
 * Sends a topic with the Ultra Sonic (US) sensor data to the roscore. Float32 from 0.0 to 32... m
 * Recieves a steering angle -pi/2 (left turn) to +pi/2 (right turn)
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Int16.h> // for steering_angle
#include <std_msgs/Float32.h> // for US sensor data
#include <Servo.h>

#include <std_msgs/String.h>

#define US_FRONT_TRIG_PIN 5
#define US_FRONT_ECHO_PIN 6


#define STEERING_SERVO_PIN 9

Servo steeringServo;

ros::NodeHandle nh;

std_msgs::Float32 US_front_dist;
ros::Publisher pub_US_front("US_front", &US_front_dist);



// ROS call back (cb) for the steering servo subscription
void steering_servo_cb( const std_msgs::Int16& steering_angle){
  steeringServo.write(steering_angle.data);
}

// subscriber for steering servo / angle
ros::Subscriber<std_msgs::Int16> sub_Steer("steering_servo", &steering_servo_cb);


float USdistanceMeasurement(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);

  float distance = duration / 29 / 2 / 100.0;
  
  return distance;
}



void setup() {
  // put your setup code here, to run once:
  steeringServo.attach(STEERING_SERVO_PIN);
  nh.initNode();
  nh.subscribe(sub_Steer);
  nh.advertise(pub_US_front);

  pinMode(US_FRONT_ECHO_PIN, INPUT);
  pinMode(US_FRONT_TRIG_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  float US_front_dist_data = USdistanceMeasurement(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
  US_front_dist.data =  US_front_dist_data;// Place Holder for the distance
  pub_US_front.publish( &US_front_dist );
  nh.spinOnce();
  
}
