#include <math.h> 

/* Frequency Data */
#define baseFrequency 14046670   // Base Frequency in mHz

/* Comparason Wavelengths */
#define lambdaMR 889
#define lambdaMG 1186
#define lambdaMP 711

/* Location Data - decimal lat and long - 1 = 0.71555 */
#define stepSize 15

#define masterLat 5023300
#define masterLong -383300
#define redLat 4925000
#define redLong -208300
#define greenLat 4993300
#define greenLong -630000
#define purpleLat 5143300
#define purpleLong -338300

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

int decca(void) {
  /*  */
  long distanceMaster = 0;
  long distanceTransmitter = 0;
  int phaseMaster = 0;
  int phaseTransmitter = 0;
  
  // Location (Place holder values
  long currentLat = 5000000;
  long currentLong = 300000;

  // Distance to master transmitter
  distanceMaster = abs(haversine(currentLat, currentLong, masterLat, masterLong));
  // Red Dial
  distanceTransmitter = abs(haversine(currentLat, currentLong, redLat, redLong));
  Serial.println(distanceMaster);
  Serial.println(distanceTransmitter);
  
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  decca();
}

void loop() {
  // put your main code here, to run repeatedly:

}
