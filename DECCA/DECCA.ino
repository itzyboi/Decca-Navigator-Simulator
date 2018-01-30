/* Frequency Data */
#define baseFrequency 14   // Base Frequency in kHz
#define redMultiplier 4
#define greenMultiplier 5
#define purpleMultiplier 7

/* Location Data - decimal lat and long - 1 = 1.0274m */
#define stepSize 15

#define masterLat 0
#define masterLong 0
#define redLat 0
#define redLong 0
#define greenLat 0
#define greenLong 0
#define purpleLat 0
#define purpleLong 0

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





void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  int steps = stepNumber(haslarMarinaLat, spinnakerLat);
  Serial.print("Steps: ");
  Serial.println(steps);
  Serial.print("Step Distance: ");
  Serial.println(stepDistance(haslarMarinaLat, spinnakerLat, steps));
}

void loop() {
  // put your main code here, to run repeatedly:

}
