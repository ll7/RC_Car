// based on: http://www.robodino.de/2011/12/ultraschall-distanz-sensor-hc-sr04.html
// edited by: ll7

// number of ultrasonic sensors:
#define  NUM_ULTR_SEN 2

int trigPin[NUM_ULTR_SEN] = {13, 8};
int echoPin[NUM_ULTR_SEN] = {12, 7};

// establish variables for duration of the ping,
// and the distance result in m:
float duration, distance;

float measurement[NUM_ULTR_SEN];

String message = "Start; ";

/*
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 */
 
void setup() {
  Serial.begin(9600);
  for (int i=0; i < NUM_ULTR_SEN; i++){
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
}


void loop()
{
  message = "Start; ";

  for (int i=0; i < NUM_ULTR_SEN; i++){
    duration = durationMeasurement(i);
  
    // convert the time into a distance
    distance = microsecondsToCentimeters(duration, i);
    
    message = message + String(i+1) + ": " + String(distance) + "; ";
  }
  Serial.println(message);

}

/*
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 */

float distanceFilter(float distance_func, int i)
{
  if (distance_func > 30.0) {
    distance_func = 10.0;
    message = message + "0: Error in ultrasonic sensor " + String(i+1) + "; ";
  }
  return distance_func;
}


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




