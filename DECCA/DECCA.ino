#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <Keypad.h>
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

/* Locations */
// Haslar Marina: 50.792517, -1.115500
#define haslarMarinaLat 5079251
#define haslarMarinaLong -111550
// Spinnaker tower: 50.795583, -1.108517
#define spinnakerLat 5079558
#define spinnakerLong -110851
// D-Day Location: 50.083333, -0.778833
#define DDayLat 5008333
#define DDayLong -077883
// Omaha Beach: 49.386467, -0.865167
#define omahaLat 4938646
#define omahaLong -086516
// Slapton Sands: 50.291700, -3.616967
#define slaptonLat 5029170
#define slaptonLong -361696
// Nab Tower: 50.667950, -0.952550
#define nabLat 5066795
#define nabLong -095255
// Portland: 50.570617, -2.440050
#define portlandLat 5057061
#define portlandLong -244005

/* I/O */
char state = 'a';
char key = 'a';
// Keypad
const byte rows = 4; //four rows
const byte cols = 3; //three columns
char keys[rows][cols] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[rows] = {11, 6, 7, 9}; //connect to the row pinouts of the keypad
byte colPins[cols] = {10, 12, 8}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols );

// LCD
Adafruit_LiquidCrystal lcd(0);
boolean menuTracker = 0;

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
  Serial.begin(115200);
  lcd.begin(20,4);
  
}

void loop() {
  /* Main Menu */
  key = keypad.getKey();

  switch(state){
    case 'a':
      if(menuTracker == 0) {
        lcd.clear();
        // Display GPS Tracker <-- Highlighted
        lcd.setCursor(0,0);
        lcd.print(" GPS Tracker");
        // Display Location
        lcd.setCursor(0,1);
        lcd.print("Locations");
        // Display GPS Input
        lcd.setCursor(0,2);
        lcd.print("GPS Input");
        // Display Settings
        lcd.setCursor(0,3);
        lcd.print("Settings");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down is pressed go to state b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      else if(key == '2') { // If up is pressed go to state d
        state = 'd';
        menuTracker = 0;
        Serial.println("d: Settings");
      }
      else if(key == '#') { // If select is pressed to to state e (GPS Tracker)
        Serial.println("e: GPS Tracker - Though I didn't actually go there");
        menuTracker = 0;
      }
      break;

    case 'b':
      if(menuTracker == 0) {
        lcd.clear();
        // Display GPS Tracker
        lcd.setCursor(0,0);
        lcd.print("GPS Tracker");
        // Display Location <-- Highlighted
        lcd.setCursor(0,1);
        lcd.print(" Locations");
        // Display GPS Input
        lcd.setCursor(0,2);
        lcd.print("GPS Input");
        // Display Settings
        lcd.setCursor(0,3);
        lcd.print("Settings");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down is pressed go to state c
        state = 'c';
        menuTracker = 0;
        Serial.println("c: GPS Input");
      }
      else if(key == '2') { // If up is pressed go to state a
        state = 'a';
        menuTracker = 0;
        Serial.println("a: GPS Tracker");
      }
      else if(key == '#') { // If select is pressed to to state Location Menu
        state = 'h';
        menuTracker = 0;
        Serial.println("h: Location 1");
      }
      break;

    case 'c':
      if(menuTracker == 0) {
        lcd.clear();
        // Display GPS Tracker 
        lcd.setCursor(0,0);
        lcd.print("GPS Tracker");
        // Display Location
        lcd.setCursor(0,1);
        lcd.print("Locations");
        // Display GPS Input <-- Highlighted
        lcd.setCursor(0,2);
        lcd.print(" GPS Input");
        // Display Settings
        lcd.setCursor(0,3);
        lcd.print("Settings");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down is pressed go to state d
        state = 'd';
        menuTracker = 0;
        Serial.println("d: Settings");
      }
      else if(key == '2') { // If up is pressed go to state b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      else if(key == '#') { // If select is pressed to to state f (GPS Input)
        //state = 'a';
        menuTracker = 0;
        Serial.println("f: GPS Input - Though I didn't actually go there");
      }
      break;

    case 'd':
      if(menuTracker == 0) {
        lcd.clear();
        // Display GPS Tracker 
        lcd.setCursor(0,0);
        lcd.print("GPS Tracker");
        // Display Location
        lcd.setCursor(0,1);
        lcd.print("Locations");
        // Display GPS Input 
        lcd.setCursor(0,2);
        lcd.print("GPS Input");
        // Display Settings <-- Highlighted
        lcd.setCursor(0,3);
        lcd.print(" Settings");
        menuTracker = 1;
      }
      if(key == '8') { // If down is pressed go to state a
        state = 'a';
        menuTracker = 0;
        Serial.println("a: GPS Tracker");
      }
      else if(key == '2') { // If up is pressed go to state c
        state = 'c';
        menuTracker = 0;
        Serial.println("c: GPS Input");
      }
      else if(key == '#') { // If select is pressed to to state Settings
        //state = 'a';
        Serial.println("DOES NOT EXIST YET - Though I didn't actually go there");
      }
      break;

  /* GPS Tracking menu */
    case 'e':
      // Display GPS Tracker Splash
      // Start GPS tracking routine
      break;

  /* GPS input Menu */
    case 'f':
      // Display Latitude input line and wait for input
      // If select pressed input is done and move to state g
      // If back pressed go to state c
      break;

    case 'g': 
      // Display Longitude input line and take input
      // If select pressed run dial movement routine to input and then move to state f
      // If back is pressed move to state f
      break;

  /* Location Menu */
    case 'h':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 1 (Haslar Marina) <-- Highlighted 
        lcd.setCursor(0,0);
        lcd.print(" Haslar Marina");
        // Display Location 2 (Spinnaker Tower)
        lcd.setCursor(0,1);
        lcd.print("Spinnaker Tower");
        // Display Location 3 (D-Day position)
        lcd.setCursor(0,2);
        lcd.print("D-Day position");
        // Display Loaction 4 (Omaha Beach)
        lcd.setCursor(0,3);
        lcd.print("Omaha Beach");
        menuTracker = 1;
      }

      if(key == '8') { // If down pressed go to state i
        state = 'i';
        menuTracker = 0;
        Serial.println("i: Location 2");
      }
      else if(key == '2') { // If up pressed to to state o
        state = 'o';
        menuTracker = 0;
        Serial.println("o: Loaction 8");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 1 

        Serial.println("Going to Location 1");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'i':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 1 (Haslar Marina) 
        lcd.setCursor(0,0);
        lcd.print("Haslar Marina");
        // Display Location 2 (Spinnaker Tower) <-- Highlighted 
        lcd.setCursor(0,1);
        lcd.print(" Spinnaker Tower");
        // Display Location 3 (D-Day position)
        lcd.setCursor(0,2);
        lcd.print("D-Day position");
        // Display Loaction 4 (Omaha Beach)
        lcd.setCursor(0,3);
        lcd.print("Omaha Beach");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state j
        state = 'j';
        menuTracker = 0;
        Serial.println("j: Location 3");
      }
      else if(key == '2') { // If up pressed to to state h
        state = 'h';
        menuTracker = 0;
        Serial.println("h: Loaction 1");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 2 

        Serial.println("Going to Location 2");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'j':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 1 (Haslar Marina) 
        lcd.setCursor(0,0);
        lcd.print("Haslar Marina");
        // Display Location 2 (Spinnaker Tower) 
        lcd.setCursor(0,1);
        lcd.print("Spinnaker Tower");
        // Display Location 3 (D-Day position) <-- Highlighted 
        lcd.setCursor(0,2);
        lcd.print(" D-Day position");
        // Display Loaction 4 (Omaha Beach)
        lcd.setCursor(0,3);
        lcd.print("Omaha Beach");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state k
        state = 'k';
        menuTracker = 0;
        Serial.println("k: Location 4");
      }
      else if(key == '2') { // If up pressed go to state i
        state = 'i';
        menuTracker = 0;
        Serial.println("i: Loaction 2");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 3
        
        Serial.println("Going to Location 3");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'k':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 1 (Haslar Marina) 
        lcd.setCursor(0,0);
        lcd.print("Haslar Marina");
        // Display Location 2 (Spinnaker Tower) 
        lcd.setCursor(0,1);
        lcd.print("Spinnaker Tower");
        // Display Location 3 (D-Day position)
        lcd.setCursor(0,2);
        lcd.print("D-Day position");
        // Display Loaction 4 (Omaha Beach) <-- Highlighted 
        lcd.setCursor(0,3);
        lcd.print(" Omaha Beach");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state l
        state = 'l';
        menuTracker = 0;
        Serial.println("l: Location 5");
      }
      else if(key == '2') { // If up pressed go to state j
        state = 'j';
        menuTracker = 0;
        Serial.println("j: Loaction 3");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 4 
        
        Serial.println("Going to Location 4");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'l':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 5 (Slapton Sands) <-- Highlighted 
        lcd.setCursor(0,0);
        lcd.print(" Slapton Sands");
        // Display Location 6 (Nab Tower) 
        lcd.setCursor(0,1);
        lcd.print("Nab Tower");
        // Display Location 7 (Portland)
        lcd.setCursor(0,2);
        lcd.print("Portland");
        // Display Loaction 8 (TBD)
        lcd.setCursor(0,3);
        lcd.print("TBD");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state m
        state = 'm';
        menuTracker = 0;
        Serial.println("m: Location 6");
      }
      else if(key == '2') { // If up pressed to to state k
        state = 'k';
        menuTracker = 0;
        Serial.println("k: Loaction 4");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 5 
        
        Serial.println("Going to Location 5");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'm':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 5 (Slapton Sands)  
        lcd.setCursor(0,0);
        lcd.print("Slapton Sands");
        // Display Location 6 (Nab Tower) <-- Highlighted
        lcd.setCursor(0,1);
        lcd.print(" Nab Tower");
        // Display Location 7 (Portland)
        lcd.setCursor(0,2);
        lcd.print("Portland");
        // Display Loaction 8 (TBD)
        lcd.setCursor(0,3);
        lcd.print("TBD");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state n
        state = 'n';
        menuTracker = 0;
        Serial.println("n: Location 7");
      }
      else if(key == '2') { // If up pressed to to state l
        state = 'l';
        menuTracker = 0;
        Serial.println("l: Loaction 5");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 6 
        
        Serial.println("Going to Location 6");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'n':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 5 (Slapton Sands)  
        lcd.setCursor(0,0);
        lcd.print("Slapton Sands");
        // Display Location 6 (Nab Tower) 
        lcd.setCursor(0,1);
        lcd.print("Nab Tower");
        // Display Location 7 (Portland) <-- Highlighted
        lcd.setCursor(0,2);
        lcd.print(" Portland");
        // Display Loaction 8 (TBD)
        lcd.setCursor(0,3);
        lcd.print("TBD");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state o
        state = 'o';
        menuTracker = 0;
        Serial.println("o: Location 8");
      }
      else if(key == '2') { // If up pressed to to state m
        state = 'm';
        menuTracker = 0;
        Serial.println("m: Loaction 6");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 7 
        
        Serial.println("Going to Location 7");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;

    case 'o':
      if(menuTracker == 0) {
        lcd.clear();
        // Display Location 5 (Slapton Sands)  
        lcd.setCursor(0,0);
        lcd.print("Slapton Sands");
        // Display Location 6 (Nab Tower) 
        lcd.setCursor(0,1);
        lcd.print("Nab Tower");
        // Display Location 7 (Portland) 
        lcd.setCursor(0,2);
        lcd.print("Portland");
        // Display Loaction 8 (TBD) <-- Highlighted
        lcd.setCursor(0,3);
        lcd.print(" TBD");
        menuTracker = 1;
      }
      
      if(key == '8') { // If down pressed go to state h
        state = 'h';
        menuTracker = 0;
        Serial.println("h: Location 1");
      }
      else if(key == '2') { // If up pressed to to state n
        state = 'n';
        menuTracker = 0;
        Serial.println("n: Loaction 7");
      }
      else if(key == '#') { // If select pressed run dial routine to Location 8 
        
        Serial.println("Going to Location 8");
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
        Serial.println("b: Locations");
      }
      break;
      
    default:
      break;
  }
  
}
