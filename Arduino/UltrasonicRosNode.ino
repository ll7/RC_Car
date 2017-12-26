#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>


std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);
ros::NodeHandle nh;

#define  NUM_ULTR_SEN 1

int trigPin[NUM_ULTR_SEN] = {13};
int echoPin[NUM_ULTR_SEN] = {12};

float duration, distance;

float measurement[NUM_ULTR_SEN];

long publisher_timer;



// functions

float microsecondsToCentimeters(float microseconds, int i)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  float distance_func = microseconds / 29 / 2 / 100.0;
  distance_func = distanceFilter(distance_func, i);
  return distance_func;
}


float durationMeasurement(int i)
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  
  digitalWrite(trigPin[i], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[i], LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  
  float duration_func = pulseIn(echoPin[i], HIGH);
  delay(5);
  return duration_func;
}


float distanceFilter(float distance_func, int i)
{
  if (distance_func > 30.0) {
    distance_func = 10.0;
    
  }
  return distance_func;
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_temp);
  Wire.begin();

  for (int i=0; i < NUM_ULTR_SEN; i++){
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
  
}


void loop()
{
  if (millis() > publisher_timer) {
    for (int i=0; i < NUM_ULTR_SEN; i++){
    duration = durationMeasurement(i);
  
    // convert the time into a distance
    distance = microsecondsToCentimeters(duration, i);

    temp_msg.data = distance;
    pub_temp.publish(&temp_msg);
   
    
  }
  publisher_timer = millis() + 1000;
  }
  nh.spinOnce();
}

