#include <math.h> 

/* Frequency Data */
#define baseFrequency 14046670   // Base Frequency in mHz

/* Comparason Wavelengths - All lambda/2*/
#define lambdaMR 445
#define lambdaMG 593
#define lambdaMP 355

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
  long distanceRed = 0;
  long distanceGreen = 0;
  long distancePurple = 0;
  float phaseMaster = 0;
  float phaseRed = 0;
  float phaseGreen = 0;
  float phasePurple = 0;
  
  // Location (Place holder values
  long currentLat = 5000000;
  long currentLong = 300000;

  // Distance to master transmitter
  distanceMaster = abs(haversine(currentLat, currentLong, masterLat, masterLong));
  

  // Red Dial
  distanceRed = abs(haversine(currentLat, currentLong, redLat, redLong));
  phaseRed = distanceRed % lambdaMR;
  phaseMaster = distanceMaster % lambdaMR;
  // 2*pi approximation -> 355/113
  phaseRed =  (phaseRed/lambdaMR)*(355/113);
  phaseMaster = (phaseMaster/lambdaMR)*(355/113);
  //Phase difference is colour - master, This may need to be changed after testing.
  
  
  // Green Dial
  distanceGreen = abs(haversine(currentLat, currentLong, greenLat, greenLong));
  phaseGreen = distanceGreen % lambdaMG;
  phaseMaster = distanceMaster % lambdaMG;

  // Purple Dial
  distancePurple = abs(haversine(currentLat, currentLong, purpleLat, purpleLong));
  phasePurple = distancePurple % lambdaMP;
  phaseMaster = distanceMaster % lambdaMP;
  
}

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  /* Main Menu */
  char state = 'a';

  switch(state){
    case 'a':
      // Display GPS Tracker <-- Highlighted
      // Display Location
      // Display GPS Input
      // Display Settings
      // If down is pressed go to state b
      // If up is pressed go to state d
      // If select is pressed to to state e (GPS Tracker)

    case 'b':
      // Display GPS Tracker 
      // Display Location <-- Highlighted
      // Display GPS Input
      // Display Settings
      // If down is pressed go to state c
      // If up is pressed go to state a
      // If select is pressed to to state Location Menu

    case 'c':
      // Display GPS Tracker 
      // Display Location 
      // Display GPS Input <-- Highlighted
      // Display Settings
      // If down is pressed go to state d
      // If up is pressed go to state b
      // If select is pressed to to state f (GPS Input)

    case 'd':
      // Display GPS Tracker 
      // Display Location
      // Display GPS Input
      // Display Settings <-- Highlighted
      // If down is pressed go to state a
      // If up is pressed go to state c
      // If select is pressed to to state Settings

  /* GPS Tracking menu */
    case 'e':
      // Display GPS Tracker Splash
      // Start GPS tracking routine

  /* GPS input Menu */
    case 'f':
      // Display Latitude input line and wait for input
      // If select pressed input is done and move to state g
      // If back pressed go to state c

    case 'g': 
      // Display Longitude input line and take input
      // If select pressed run dial movement routine to input and then move to state f
      // If back is pressed move to state f

  /* Location Menu */
    case 'h':
      // Display Location 1 (Haslar Marina) <-- Highlighted
      // Display Location 2 (Spinnaker Tower)
      // Display Location 3 (D-Day position)
      // Display Loaction 4 (Omaha Beach)
      // If select pressed run dial routine to Location 1 
      // If back pressed go to sate b
      // If up pressed to to state <Fix when #loactions known>
      // If down pressed go to state i

    case 'i':
      // Display Location 1 
      // Display Location 2 <-- Highlighted
      // Display Location 3
      // Display Loaction 4
      // If select pressed run dial routine to Location 2 
      // If back pressed go to sate b
      // If up pressed go to state h
      // If down pressed go to state j

    case 'j':
      // Display Location 1 
      // Display Location 2 
      // Display Location 3 <-- Highlighted
      // Display Loaction 4
      // If select pressed run dial routine to Location 3 
      // If back pressed go to sate b
      // If up pressed go to state i
      // If down pressed go to state k

    case 'k':
      // Display Location 1 
      // Display Location 2 
      // Display Location 3 
      // Display Loaction 4 <-- Highlighted
      // If select pressed run dial routine to Location 4 
      // If back pressed go to sate b
      // If up pressed go to state j
      // If down pressed go to state l

    case 'l':
      // Display Location 5 (Slapton Sands) <-- Highlighted
      // Display Location 6 (Nab Tower)
      // Display Location 7 (Portland)
      // Display Loaction 8 (TBD)
      // If select pressed run dial routine to Location 5 
      // If back pressed go to sate b
      // If up pressed to to state k
      // If down pressed go to state m

    case 'm':
      // Display Location 5 (Slapton Sands) 
      // Display Location 6 (Nab Tower) <-- Highlighted
      // Display Location 7 (Portland)
      // Display Loaction 8 (TBD)
      // If select pressed run dial routine to Location 6 
      // If back pressed go to sate b
      // If up pressed to to state l
      // If down pressed go to state n

    case 'n':
      // Display Location 5 (Slapton Sands) 
      // Display Location 6 (Nab Tower) 
      // Display Location 7 (Portland) <-- Highlighted
      // Display Loaction 8 (TBD)
      // If select pressed run dial routine to Location 7 
      // If back pressed go to sate b
      // If up pressed to to state m
      // If down pressed go to state o

    case 'o':
      // Display Location 5 (Slapton Sands) 
      // Display Location 6 (Nab Tower) 
      // Display Location 7 (Portland) 
      // Display Loaction 8 (TBD) <-- Highlighted
      // If select pressed run dial routine to Location 8 
      // If back pressed go to sate b
      // If up pressed to to state n
      // If down pressed go to state h
      
    default:;
  }
}
