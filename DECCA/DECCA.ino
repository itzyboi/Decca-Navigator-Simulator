#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <Keypad.h>
#include <math.h> 

/* Frequency Data */
#define baseFrequency 14046670   // Base Frequency in mHz

/* Comparason Wavelengths - All lambda/2*/
const unsigned int lambdaMR PROGMEM = 890;
const unsigned int lambdaMG PROGMEM = 1186;
const unsigned int lambdaMP PROGMEM = 711;

/* Location Data - decimal lat and long - 1 = 0.71555 */
#define stepSize 10

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
#define DDayLong -77883
// Omaha Beach: 49.386467, -0.865167
#define omahaLat 4938646
#define omahaLong -86516
// Slapton Sands: 50.291700, -3.616967
#define slaptonLat 5029170
#define slaptonLong -361696
// Nab Tower: 50.667950, -0.952550
#define nabLat 5066795
#define nabLong -95255
// Portland: 50.570617, -2.440050
#define portlandLat 5057061
#define portlandLong -244005

/* Globals */
float currentLat = 5079251;
float currentLong = -111550;
boolean menuTracker = 0;
long GPSInputLat = 0;
long GPSInputLong = 0;
byte counter = 0;
int milliseconds = 0;

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

// GPS
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;

// PWM Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define PWMFreq 500
// Pins
#define redSin 0
#define redCos 0
#define greenSin 0
#define greenCos 0
#define purpleSin 14
#define purpleCos 15
#define PWMEnable A3


long stepNumber(long startLat, long startLong, long targetLat, long targetLong) {
  /* Calculates the number of steps needed to move from start to target at step size */
  long x = abs((startLat - targetLat)/stepSize);
  long y = abs((startLong - targetLong)/stepSize);

  if(x > y){
    return x;
  }
  else return y;
}

float stepDistance(long start, long target, long steps) {
  /* Takes distance and number of steps calculates a suitable step size to travel along */  
  return (float)(target - start)/steps;
}

float toRadians(float x) {
  /* Provides a good estimate of degree to radian conversion */
  return (x*71)/4068;
}

long haversine(float startLat, float startLong, float targetLat, float targetLong) {
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

boolean dialDrive(long targetLat, long targetLong) {
  /* Calculates and then drive dials to the input position */
    // 2*pi approximation -> 710/113
  long distanceMaster = 0;
  long distanceRed = 0;
  long distanceGreen = 0;
  long distancePurple = 0;
  float phaseMaster = 0;
  float phaseRed = 0;
  float phaseGreen = 0;
  float phasePurple = 0;

  // Distance to master transmitter
  distanceMaster = abs(haversine(targetLat, targetLong, masterLat, masterLong));


  // Red Dial
  distanceRed = abs(haversine(targetLat, targetLong, redLat, redLong));
  phaseRed = distanceRed - distanceMaster;
  phaseRed = (710*phaseRed)/(113*lambdaMR);
  pwm.setPin(redSin,(2047*sin(phaseRed) + 2048));
  pwm.setPin(redCos,(2047*cos(phaseRed) + 2048));
  
  // Green Dial
  distanceGreen = abs(haversine(targetLat, targetLong, greenLat, greenLong));
  phaseGreen = distanceGreen - distanceMaster;
  phaseGreen = (710*phaseGreen)/(113*lambdaMG);
  pwm.setPin(greenSin,(2047*sin(phaseGreen) + 2048));
  pwm.setPin(greenCos,(2047*cos(phaseGreen) + 2048));
  
  // Purple Dial
  distancePurple = abs(haversine(targetLat, targetLong, purpleLat, purpleLong));
  phasePurple = distancePurple - distanceMaster;
  phasePurple = (710*phasePurple)/(113*lambdaMP);
  pwm.setPin(purpleSin,(2047*sin(phasePurple) + 2048));
  pwm.setPin(purpleCos,(2047*cos(phasePurple) + 2048));

  return 1;
}

boolean decca(long targetLat, long targetLong) {
  long steps = stepNumber(currentLat, currentLong, targetLat, targetLong);
  float distanceLat = stepDistance(currentLat, targetLat, steps);
  float distanceLong = stepDistance(currentLong, targetLong, steps);

  for(int x = 0; x < steps; x++) {
    currentLat += distanceLat;
    currentLong += distanceLong;
    dialDrive(currentLat, currentLong);
    delay(1);
    
    Serial.print(x);
    Serial.print("/");
    Serial.println(steps - 1);
  }
  currentLat = targetLat;
  currentLong = targetLong;
  dialDrive(currentLat, currentLong);
  Serial.print("Final Lat: ");
  Serial.println(currentLat);
  Serial.print("Final Long: ");
  Serial.println(currentLong);
  return 1;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void setup() {
  // LCD + PWM Setup
  Serial.begin(115200);
  lcd.begin(20,4);
  pwm.begin();
  pwm.setPWMFreq(PWMFreq);
  pinMode(PWMEnable, OUTPUT);
  digitalWrite(PWMEnable, LOW);

  // GPS Setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  useInterrupt(true);
  
  Serial.print("Start Lat: ");
  Serial.println(currentLat);
  Serial.print("Start Long: ");
  Serial.println(currentLong);
}

void loop() {
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()));   // this also sets the newNMEAreceived() flag to false
  }
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
      }
      else if(key == '2') { // If up is pressed go to state d
        state = 'd';
        menuTracker = 0;
      }
      else if(key == '#') { // If select is pressed to to state e (GPS Tracker)
        state = 'e';
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
      }
      else if(key == '2') { // If up is pressed go to state a
        state = 'a';
        menuTracker = 0;
      }
      else if(key == '#') { // If select is pressed to to state Location Menu
        state = 'h';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up is pressed go to state b
        state = 'b';
        menuTracker = 0;
      }
      else if(key == '#') { // If select is pressed to to state f (GPS Input)
        state = 'f';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up is pressed go to state c
        state = 'c';
        menuTracker = 0;
      }
      else if(key == '#') { // If select is pressed to to state Settings
        //state = 'a';
      }
      break;

  /* GPS Tracking Splash */
    case 'e':
      // Display GPS Tracker Splash
      // Start GPS tracking routine
      if(!menuTracker) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("GPS Tracking Mode");
        lcd.setCursor(0,1);
        lcd.print("Awaiting GPS Fix");
        lcd.setCursor(0,3);
        lcd.print("Exit: Back(*)");
        menuTracker = 1;
      }
      // Get location
      if(GPS.fix) {
        menuTracker = 0;
        state = 'p';
      }
      //Drive Dial
      if(key == '*') {
        state = 'a';
        menuTracker = 0;
      }
      break;

  /* GPS input Menu */
    case 'f':
      // Display Latitude input line and take input
      // If select pressed input is done and move to state g
      // If back pressed go to state c
      if(!menuTracker) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("GPS Input Mode");
        lcd.setCursor(0,1);
        lcd.print("Enter Latitude 5D.P");
        lcd.setCursor(0,3);
        lcd.print("Exit: Back(*)");
        lcd.setCursor(0,2);
        menuTracker = 1;
      }
      if(key == '#') { 
        state = 'g';
        menuTracker = 0;
        counter = 0;
        Serial.println("g: Longitude");
       // Serial.println(GPSInputLat);
      }
      else if(key == '*') { 
        state = 'c';
        menuTracker = 0;
        GPSInputLat = 0;
        counter = 0;
        Serial.println("c: GPS Inout");
      }
      else if(key && (counter < 7)) {
        GPSInputLat = (GPSInputLat*10) + (key-'0');
        lcd.setCursor(0,2);
        lcd.print(GPSInputLat);
        counter++;
      }
      break;

    case 'g': 
      // Display Longitude input line and take input
      // If select pressed run dial movement routine to input and then move to state f
      // If back is pressed move to state f
      if(!menuTracker) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("GPS Input Mode");
        lcd.setCursor(0,1);
        lcd.print("Enter Longitude 5D.P");
        lcd.setCursor(0,3);
        lcd.print("Exit: Back(*)");
        lcd.setCursor(0,2);
        menuTracker = 1;
      }
      if(key == '#') { 
        //decca(GPSInputLat,GPSInputLong);
        state = 'c';
        menuTracker = 0;
        GPSInputLat = 0;
        GPSInputLong = 0;
        counter = 0;
      }
      else if(key == '*') { 
        state = 'f';
        menuTracker = 0;
        GPSInputLat = 0;
        GPSInputLong = 0;
        counter = 0;

      }
      else if(key && (counter < 8)) {
        GPSInputLong = (GPSInputLong*10) + (key-'0');
        lcd.setCursor(0,2);
        lcd.print(GPSInputLong);
        counter++;
      }
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
      }
      else if(key == '2') { // If up pressed to to state o
        state = 'o';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 1 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Haslar Marina");
        decca(haslarMarinaLat, haslarMarinaLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed to to state h
        state = 'h';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 2 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Spinnaker Tower");
        decca(spinnakerLat, spinnakerLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed go to state i
        state = 'i';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 3
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("D-Day Position");
        decca(DDayLat, DDayLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed go to state j
        state = 'j';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 4 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Omaha Beach");
        decca(omahaLat, omahaLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed to to state k
        state = 'k';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 5 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Slapton Sands");
        decca(slaptonLat, slaptonLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed to to state l
        state = 'l';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 6 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Nab Tower");
        decca(nabLat, nabLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed to to state m
        state = 'm';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 7 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Moving to");
        lcd.setCursor(0,1);
        lcd.print("Portland");
        decca(portlandLat, portlandLong);
        menuTracker = 0;
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
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
      }
      else if(key == '2') { // If up pressed to to state n
        state = 'n';
        menuTracker = 0;
      }
      else if(key == '#') { // If select pressed run dial routine to Location 8 
      }
      else if(key == '*') { // If back pressed go to sate b
        state = 'b';
        menuTracker = 0;
      }
      break;
    case 'p':
      if(!menuTracker) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("GPS Tracking Mode");
        lcd.setCursor(0,1);
        lcd.print("Lat:");
        lcd.setCursor(0,2);
        lcd.print("Long:");
        lcd.setCursor(0,3);
        lcd.print("Exit: Back(*)");
        menuTracker = 1;
      }
      if(!GPS.fix) {
        menuTracker = 0;
        state = 'e';
      }
      else if((millis()-milliseconds) >1000) {
        lcd.setCursor(4,1);
        lcd.print(GPS.latitudeDegrees,5);
        lcd.setCursor(5,2);
        lcd.print(GPS.longitudeDegrees,5);
        decca((GPS.latitudeDegrees*100000),(GPS.longitudeDegrees*100000));
      }
      if(key == '*') {
        state = 'a';
        menuTracker = 0;
      }
    default:
      break;
  }
  
}
