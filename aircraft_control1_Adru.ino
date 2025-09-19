/* ----------------------------  ARDUINO AIRPLANE CONTROL  ---------------------------- */
/* 
  Nethul Bodiratne 
  Last updated: 9/19/2025
*/
/* ----------------------------    ---------------------------- */

#include <Wire.h>
#include <SPI.h>
// Add other necessary libraries for the sensors (e.g., MPU6050 for gyro/accelerometer, HMC5883L for magnetometer, etc.)

// Pin Definitions for Mode Selection
#define MODE_0 2
#define MODE_1 3
#define MODE_2 4
#define MODE_3 5
#define MODE_4 6

// Error & Status LED Pins
#define ERROR_LED_PIN 13
//#define STATUS_GOOD_PIN 14

// Flight Plan Timers
unsigned long flightStartTime = 0;
unsigned long modeStartTime = 0;
unsigned long stepStartTime = 0;

// Sensor data variables
float gyroData[3];
float accelData[3];
float distanceData = 0.0;
float headingData = 0.0;

// Mode Enum for Better Readability
enum Mode {
  MODE_RESET_SENSORS = 1,
  MODE_STOPPED,
  MODE_TAXI,
  MODE_TAKEOFF_LEVEL_LAND,
  MODE_HOLDING_PATTERN
  // Add more flight modes here
};

Mode currentMode = MODE_STOPPED;

// Log file creation
File logFile;

// Constants
float TEST_ALTITUDE = 1.0;
float TEST_LEVEL_DURATION = 10000;
float TEST_HOLDING_PATTERN_DURATION = 20000;


// ----------------------------  SETUP  ---------------------------- //
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Setup pins
  pinMode(MODE_0, INPUT);
  pinMode(MODE_1, INPUT);
  pinMode(MODE_2, INPUT);
  pinMode(MODE_3, INPUT);
  pinMode(MODE_4, INPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  //pinMode(STATUS_GOOD_PIN, OUTPUT);

  // Initialize sensors here

  // Open the log file
  logFile = SD.open("flight_log.txt", FILE_WRITE);
  if (logFile) {
    logFile.println("Flight Log Start");
    logFile.close();
  }
  
  // Initialize timer for system uptime
  flightStartTime = millis();
}

// ----------------------------  LOOP  ---------------------------- //
void loop() {
  // put your main code here, to run repeatedly:

}

// ----------------------------  MODE 0 (RESET SENSORS)  ---------------------------- //
/* Flight Plan: Stops the plane then if stationary for duration reset sensors. */
void modeResetSensors() {
  logEvent("Entered MODE_RESET_SENSORS");

  unsigned long modeDuration = millis() - modeStartTime;

  stopPlane();
  
  // Check if the plane is moving or stationary
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("Error: Plane is not stationary as expected!");
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
  }
  
  // Check if stationary for x seconds
  if (modeDuration > 10000) {  // x = 10 seconds for sensor reset, can be changed later
    if (!resetSensors()) {
      digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED if reset fails
      logEvent("Error: Sensor reset failed!");
    } else {
      logEvent("Sensors reset successfully.");
    }
  }
}

// ----------------------------  MODE 1 (STOPPED)  ---------------------------- //
/* Flight Plan: Stops the plane then checks if staionary. */
void modeStopped() {
  logEvent("Entered MODE_STOPPED");
  
  unsigned long modeDuration = millis() - modeStartTime;

  stopPlane();
  
  // Check if the plane is moving or stationary
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("Error: Plane is not stationary as expected!");
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
  }
}

// ----------------------------  MODE 2 (TAXI)  ---------------------------- //
/* Flight Plan: Taxi the plane for a certain duration. */
void modeTaxi() {
  logEvent("Entered MODE_STOPPED");
  
  unsigned long modeDuration = millis() - modeStartTime;

  // Add taxi logic based on distance/time
  if (modeDuration < 5000) {  // Taxi for 5 seconds (as an example)
    // Code to control the plane's motors for taxiing
  } else {
    // Stop and transition to parked state
    stopPlane();
    if (isPlaneMoving()) {
      digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
      logEvent("Error: Plane is not stationary as expected!");
    } else {
      digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
      logEvent("Plane has completed taxi and stopped.");
    }
  }
}

// ----------------------------  MODE 3 (TAKEOFF, LEVEL, LAND)  ---------------------------- //
/* Flight Plan: Plane should takeoff and climb to a preset altitude. It should then level off and cruise for a preset duration then land and park. */
void modeTakeoffLevelLand() {
  logEvent("Entered MODE_TAKEOFF_LEVEL_LAND");
  
  unsigned long modeDuration = millis() - modeStartTime;
  
  // Step 1: Takeoff
  logEvent("Taking off...");
  while (getCurrentAltitude() < TEST_ALTITUDE) {
    takeOff(TEST_ALTITUDE);
  }
  logEvent("Reached target altitude. Transitioning to level flight.");

  // Step 2: Level Flight
  logEvent("Flying level...");
  long startTime = getMillis();
  while (getMillis() - startTime < TEST_LEVEL_DURATION) {
    levelFlight(TEST_ALTITUDE);
  }
  logEvent("Level flight duration complete. Transitioning to landing.");
 
  // Step 3: Landing
  logEvent("Landing...");
  while (getCurrentAltitude() > ground_Altitude) {
    // This loop will continue until the plane is on the ground (altitude saved from sensor reset).
    land();
  }
  logEvent("Landed successfully. Transitioning to parking.");

  // Step 4: Park
  stopPlane();
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("Error: Plane is not stationary as expected!");
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
    logEvent("Plane has landed and stopped.");
  }
}

// ----------------------------  MODE 4 (CIRCULAR HOLDING PATTERN)  ---------------------------- //
/* Flight Plan: Take off then maintain altitude while flying in a constant turn. */
void modeHoldingPattern() {
  logEvent("Entered MODE_HOLDING_PATTERN");

  unsigned long modeDuration = millis() - modeStartTime;

  // Step 1: Takeoff
  logEvent("Taking off...");
  while (getCurrentAltitude() < TEST_ALTITUDE) {
    takeOff(TEST_ALTITUDE);
  }
  logEvent("Reached target altitude. Transitioning to level flight.");

  // Step 2: Circle and hold
  logEvent("Flying in circular holding pattern...");
  long holdingPatternStartTime = getMillis();
  while (getMillis() - holdingPatternStartTime < TEST_HOLDING_PATTERN_DURATION) {
    holdingPattern();
  }
  logEvent("Holding pattern complete. Transitioning to landing.");
  
  // Step 3: Landing
  logEvent("Landing...");
  while (getCurrentAltitude() > ground_Altitude) {
    // This loop will continue until the plane is on the ground (altitude saved from sensor reset).
    land();
  }
  logEvent("Landed successfully. Transitioning to parking.");

  // Step 4: Park
  stopPlane();
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("Error: Plane is not stationary as expected!");
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
    logEvent("Plane has landed and stopped.");
  }
}

// ----------------------------  TAKEOFF SEQUENCE  ---------------------------- //
/* Autopilot for takeoff stage. */
void takeOff(targetAltitude) {
  // Code for increasing altitude until targetAltitude is reached (using distance sensor or altitude control system)
  // Code to increase speed and get the plane airborne
}

// ----------------------------  MAINTAIN LEVEL FLIGHT  ---------------------------- //
/* Autopilot for level flight. Takes a altitude target as input and attempts to maintain. */
void levelFlight(targetAltitude) {
  // Maintain altitude for the given time
  // Use the distance sensor to monitor altitude
  if (getCurrentAltitude() < targetAltitude) {
    // Adjust motor speed and/or control surfaces to maintain altitude target
  } else {
    // Adjust speed and/or control surfaces if too high/low
  }
}

// ----------------------------  LANDING SEQUENCE  ---------------------------- //
/* Autopilot for landing sequence.  */
void land() {
  // Decrease altitude and safely land the plane by reducing throttle and/or control surfaces
  // Use distance sensor to guide the plane to the ground
}

// ----------------------------  STOP PLANE  ---------------------------- //
/* Stops the plane. */
void stopPlane() {
  // to be completed.
  // Set motors to 0. set control surfaces to neutral.
}

// ----------------------------  CHECK IF MOVING  ---------------------------- //
/* Checks if the plane is moving. Returns true if moving. */
bool isPlaneMoving() {
  // A simple example; this could be expanded based on gyro/accel data
  float speed = sqrt(pow(accelData[0], 2) + pow(accelData[1], 2) + pow(accelData[2], 2));
  return speed > 0.1; // If movement detected (simple threshold)
}

// ----------------------------  CURRENT ALTITUDE  ---------------------------- //
/* Get the current altitude. */
float getCurrentAltitude() {
  // return altitude; // Returns the value from the distance sensor that is pointed at the ground to get altitude
}

// ----------------------------  RESET SENSORS  ---------------------------- //
/* Resets sensors to default values. */
bool resetSensors() {
  // Reset logic here
  // E.g., set all sensor data to known starting values
  ground_Altitude = getCurrentAltitude();
  return true;  // Return true if reset successful
}

// ----------------------------  LOG TO FILE  ---------------------------- //
/* Logs data to file on external memory. */
void logEvent(String message) {
  unsigned long elapsedTime = millis() - flightStartTime;
  logFile = SD.open("flight_log.txt", FILE_WRITE);
  if (logFile) {
    logFile.print("Time: ");
    logFile.print(elapsedTime);
    logFile.print("ms - ");
    logFile.println(message);
    logFile.close();
  }
  Serial.println(message); // Also print to serial monitor for real-time debugging
}
