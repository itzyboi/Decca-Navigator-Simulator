#include <math.h> 

/* Frequency Data */
#define baseFrequency 14   // Base Frequency in kHz
#define redMultiplier 4
#define greenMultiplier 5
#define purpleMultiplier 7

/* Location Data - decimal lat and long - 1 = 0.71555 */
#define stepSize 15

#define masterLat 
#define masterLong 0
#define redLat 0
#define redLong 0
#define greenLat 0
#define greenLong 0
#define purpleLat 0
#define purpleLong 0

/* Constants */
#define earthRadiusM 6371000

// Spinnaker tower: 50.79556, -1.10848
#define spinnakerLat 5079556
#define spinnakerLong -110848
// Haslar Marina: 50.79098, -1.11754
#define haslarMarinaLat 5079098
#define haslarMarinaLong -111754



int difference( int x, int y) {
  /* Returns the difference between two integers */
  return x - y;
}

int stepNumber(int start, int target) {
  /* Calculates the number of steps needed to move from start to target at step size */
  int distance = difference(start, target);
  return abs(distance / stepSize);
}

int stepDistance(int start, int target, int steps) {
  /* Takes distance and number of steps calculates a suitable step size to travel along */  
  int distance = difference(start,target);
  return distance/steps;
}

float toRadians(float x) {
  /* Provides a good estimate of degree to radian conversion */
  return (x*71)/4068;
}

int haversine(float startLat, float startLong, float targetLat, float targetLong) {
  /* Calculates the distance over the earths surface to get from start to target */
  float startLatRad = toRadians(startLat /= 100000);
  float startLongRad = toRadians(startLong /= 100000);
  float targetLatRad = toRadians(targetLat /= 100000);
  float targetLongRad = toRadians(targetLong /= 100000);

  
  
  float deltaLatRad = targetLatRad - startLatRad;
  float deltaLongRad = targetLongRad - startLongRad;

  float a = sq(sin(deltaLatRad/2)) + (cos(startLatRad)*cos(targetLatRad)*sq(sin(deltaLongRad/2)));
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  return earthRadiusM * c;
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(haversine(spinnakerLat, spinnakerLong, haslarMarinaLat, haslarMarinaLong));
}

void loop() {
  // put your main code here, to run repeatedly:

}
