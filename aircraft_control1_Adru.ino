/* ----------------------------  ARDUINO AIRPLANE CONTROL  ---------------------------- */
/* 
  Nethul Bodiratne 
  Last updated: 9/23/2025
*/
/* ------------------------------------------------------------------------------------ */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
// Add other necessary libraries for the sensors (e.g., MPU6050 for gyro/accelerometer, HMC5883L for magnetometer, etc.)

// Pin Definitions for Mode Selection
#define MODE_0 2
#define MODE_1 3
#define MODE_2 4
#define MODE_3 5
#define MODE_4 6

// Error & Status LED Pins
#define ERROR_LED_PIN 13
//#define STATUS_GOOD_PIN 12

// Flight Plan Timers
unsigned long flightStartTime = 0;
unsigned long modeStartTime = 0;
unsigned long stepStartTime = 0;

// Sensor data variables
float gyroData[3] = {0.0, 0.0, 0.0};
float accelData[3] = {0.0, 0.0, 0.0};
float distanceData = 0.0;
float headingData = 0.0;
float groundAltitude = 0.0;
float rollAngle = 0.0;

// Control Surface Variables
float currentThrottle = 0.0;
float currentElevator = 0.0;
float currentAileronLeft = 0.0;
float currentAileronRight = 0.0;
float currentRudder = 0.0;

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
Mode lastMode = MODE_STOPPED;

// Flight State Enum for complex flight modes
enum FlightState {
  STATE_START,
  STATE_TAKEOFF,
  STATE_LEVEL_FLIGHT,
  STATE_HOLDING_PATTERN,
  STATE_LANDING,
  STATE_PARKED
};

FlightState currentFlightState = STATE_START;

// Log file creation
File logFile;
bool sdCardAvailable = false;

// Constants
const float LANDING_FLARE_ALTITUDE = 0.25; // meters - The altitude at which the plane initiates the flare maneuver.
const float ROLL_TOLERANCE = 1.0; // degrees - The acceptable tolerance for the plane's roll angle.
const float TEST_ALTITUDE = 10.0; // meters - Target altitude for test flights.
const unsigned long TEST_LEVEL_DURATION = 10000; // milliseconds - Duration for maintaining level flight.
const unsigned long TEST_HOLDING_PATTERN_DURATION = 20000; // milliseconds - Duration for circular holding pattern.
const unsigned long SENSOR_RESET_STATIONARY_DURATION = 10000; // milliseconds - Duration the plane must be stationary for a sensor reset.
const unsigned long TAXI_DURATION = 5000; // milliseconds - Duration for taxiing on the runway.
const float MIN_TAKEOFF_SPEED = 15; // m/s - The minimum speed required for takeoff.
const float MIN_THROTTLE = 0.0; // minimum throttle power (0.0 to 1.0)
const float MAX_THROTTLE = 1.0; // maximum throttle power (0.0 to 1.0)
const float THROTTLE_STEP = 0.05; // step throttle power by 5%
const float LEVEL_ELEVATOR = 0.0; // degrees - elevator angle for level flight
const float MIN_ELEVATOR = -15.0; // degrees - minimum elevator angle (TE up)
const float MAX_ELEVATOR = 15.0; // degrees - maximum elevator angle (TE down)
const float ELEVATOR_STEP = 1.0; // degrees - step elevator angle by 1
const float MIN_AILERON = -15.0; // degrees - minimum aileron angle
const float MAX_AILERON = 15.0; // degrees - maximum aileron angle
const float AILERON_STEP = 1.0; // degrees - step ailerons angle by 1
const float MIN_RUDDER = -15.0; // degrees - rudder full left angle
const float MAX_RUDDER = 15.0; // degrees - rudder full right angle
const float RUDDER_STEP = 1.0; // degrees - step rudder angle by 1
const float FLARE_ELEVATOR_ANGLE = 10.0; // degrees - The elevator angle for the landing flare.
const float FLARE_AILERON_ANGLE = 5.0; // degrees - Aileron angle to maintain stability during flare.

// ----------------------------  FUNCTION PROTOTYPES  ---------------------------- //
// Define all function prototypes to avoid compilation errors
// General Utility Functions
void logEvent(String message);
Mode checkModePins();
void readSensors();
void stopPlane();
bool isPlaneMoving();
float getAltitude();
float getSpeed();
float getThrottle();
float getElevatorAngle();
float getAileronLeftAngle();
float getAileronRightAngle();
float getRudderAngle();
float getRollAngle();
bool resetSensors();

// Control surface Functions
void setThrottle(float throttleValue);
void setElevator(float elevatorAngle);
void setAileronLeft(float aileronAngle);
void setAileronRight(float aileronAngle);
void setRudder(float rudderAngle);

// Mode functions
void modeResetSensors();
void modeStopped();
void modeTaxi();
void modeTakeoffLevelLand();
void modeHoldingPattern();

// Flight Sequence functions
void takeOff(float targetAltitude);
void levelFlight(float targetAltitude);
void levelRoll();
void holdingPattern();
void land();

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

  // Initialize SD card
  if (!SD.begin(BUILTIN_CS)) {
    Serial.println("SD card initialization failed!");
    digitalWrite(ERROR_LED_PIN, HIGH);
    sdCardAvailable = false; // Flag for SD card error
} else {
  sdCardAvailable = true;
  // Open the log file ONCE in setup()
  logFile = SD.open("flight_log.txt", FILE_WRITE);
  if (logFile) {
    logFile.println("Flight Log Start");
    logFile.flush(); // Use flush() to ensure data is written to the card
  }
}

  // Initialize sensors here
  Wire.begin();
  Serial.println("Sensors Initializing...");
  // (e.g., MPU6050.begin(), HMC5883L.begin(), etc.)
  Serial.println("Sensors Initialized.");

  // Initialize timer for system uptime
  flightStartTime = millis();
  modeStartTime = millis();

  // Read initial mode from pins
  currentMode = checkModePins();
  lastMode = currentMode;
  logEvent("Initial mode set to " + String(currentMode));
}

// ----------------------------  LOOP  ---------------------------- //
void loop() {
  // Always read sensor data at the beginning of the loop
  readSensors();

  // Check the physical mode selection pins
  currentMode = checkModePins();

  // Handle mode transitions and reset the mode timer
  if (currentMode != lastMode) {
    logEvent("Transitioning from Mode " + String(lastMode) + " to Mode " + String(currentMode));
    modeStartTime = millis();
    lastMode = currentMode;
  }

  // Use a state machine pattern to run the current mode's logic
  switch (currentMode) {
    case MODE_RESET_SENSORS:
      modeResetSensors();
      break;
    case MODE_STOPPED:
      modeStopped();
      break;
    case MODE_TAXI:
      modeTaxi();
      break;
    case MODE_TAKEOFF_LEVEL_LAND:
      modeTakeoffLevelLand();
      break;
    case MODE_HOLDING_PATTERN:
      modeHoldingPattern();
      break;
    default:
      // Unknown mode, default to stopped and log an error
      stopPlane();
      digitalWrite(ERROR_LED_PIN, HIGH);
      logEvent("Error: Unknown mode selected!");
      break;
  }
}

// ----------------------------  READ SENSORS  ---------------------------- //
/* Read and store sensor data in appropriate variables. */
void readSensors() {
  // Placeholder for sensor reading logic
  // Example: MPU6050.readSensors(&accelData, &gyroData);
}

// ----------------------------  READ MODE PINS  ---------------------------- //
/* Checks the mode selection pins using the returns corresponding enum. */
Mode checkModePins() {
  // Read the state of each pin
  int pinA = digitalRead(MODE_0);
  int pinB = digitalRead(MODE_1);
  int pinC = digitalRead(MODE_2);
  int pinD = digitalRead(MODE_3);
  int pinE = digitalRead(MODE_4)
  
  // Combine pin states into a binary number
  // Using INPUT_PULLUP, LOW means the switch is closed.
  int modeValue = (!pinD << 3) | (!pinC << 2) | (!pinB << 1) | !pinA;
  
  // Map binary value to the Mode enum
  // Example: 0001 -> MODE_RESET_SENSORS
  // This is a simple example; you can define the mapping as you wish
  switch (modeValue) {
    case 1:
      return MODE_RESET_SENSORS;
    case 2:
      return MODE_STOPPED;
    case 3:
      return MODE_TAXI;
    case 4:
      return MODE_TAKEOFF_LEVEL_LAND;
    case 5:
      return MODE_HOLDING_PATTERN;
    default:
      return MODE_STOPPED; // Default to a safe stopped mode
  }
}

// ----------------------------  MODE 0 (RESET SENSORS)  ---------------------------- //
/* Flight Plan: Stops the plane then if stationary for duration reset sensors. */
void modeResetSensors() {
  logEvent("Entered MODE_RESET_SENSORS");

  stopPlane();
  
  // Check if the plane is moving or stationary
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("Error: Plane is not stationary as expected!");
  } else {
    digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
  }
  
  // Check if stationary for x seconds
  if (millis() - modeStartTime > SENSOR_RESET_STATIONARY_DURATION) {  // x = 10 seconds for sensor reset, can be changed later
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

  // Add taxi logic based on distance/time
  if (millis() - modeStartTime < TAXI_DURATION) { // Taxi for 5 seconds (as an example)
    // Code to control the plane's motors for taxiing
    setThrottle(0.2); // Example throttle for taxiing, can be changed to setThrottle(TAXI_THROTTLE)
    setRudder(0.0);
  } else { // Stop and transition to parked state
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
  
  switch (currentFlightState) {
    case STATE_START:
      logEvent("Starting takeoff sequence.");
      currentFlightState = STATE_TAKEOFF;
      stepStartTime = millis();
      break;

    // Step 1: Takeoff
    case STATE_TAKEOFF:
      if (getAltitude() < TEST_ALTITUDE) {
        takeOff(TEST_ALTITUDE);
      } else {
        logEvent("Reached target altitude. Transitioning to level flight.");
        currentFlightState = STATE_LEVEL_FLIGHT;
        stepStartTime = millis();
      }
      break;
      
    // Step 2: Level Flight
    case STATE_LEVEL_FLIGHT:
      if (millis() - stepStartTime < TEST_LEVEL_DURATION) {
        levelFlight(TEST_ALTITUDE);
      } else {
        logEvent("Level flight duration complete. Transitioning to landing.");
        currentFlightState = STATE_LANDING;
        stepStartTime = millis();
      }
      break;
    
    // Step 3: Landing
    case STATE_LANDING:
      if (getAltitude() > groundAltitude) {
        land();
      } else {
        logEvent("Landed successfully. Transitioning to parked state.");
        currentFlightState = STATE_PARKED;
        stepStartTime = millis();
      }
      break;

    // Step 4: Park
    case STATE_PARKED:
      stopPlane();
      if (isPlaneMoving()) {
        digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
        logEvent("Error: Plane is not stationary as expected!");
      } else {
        digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
        logEvent("Plane has landed and stopped.");
      }
      break;
  }
}

// ----------------------------  MODE 4 (CIRCULAR HOLDING PATTERN)  ---------------------------- //
/* Flight Plan: Take off then maintain altitude while flying in a constant turn. */
void modeHoldingPattern() {
  logEvent("Entered MODE_HOLDING_PATTERN");

  switch (currentFlightState) {
    case STATE_START:
      logEvent("Starting takeoff sequence for holding pattern.");
      currentFlightState = STATE_TAKEOFF;
      stepStartTime = millis();
      break;
    
    // Step 1: Take off
    case STATE_TAKEOFF:
      if (getAltitude() < TEST_ALTITUDE) {
        takeOff(TEST_ALTITUDE);
      } else {
        logEvent("Reached target altitude. Transitioning to holding pattern.");
        currentFlightState = STATE_HOLDING_PATTERN;
        stepStartTime = millis();
      }
      break;
    
    // Step 2: Hold Constant Turn
    case STATE_HOLDING_PATTERN:
      if (millis() - stepStartTime < TEST_HOLDING_PATTERN_DURATION) {
        holdingPattern();
      } else {
        logEvent("Holding pattern complete. Transitioning to landing.");
        currentFlightState = STATE_LANDING;
        stepStartTime = millis();
      }
      break;
      
    // Step 3: Landing
    case STATE_LANDING:
      if (getAltitude() > groundAltitude) {
        land();
      } else {
        logEvent("Landed successfully. Transitioning to parked state.");
        currentFlightState = STATE_PARKED;
        stepStartTime = millis();
      }
      break;

    // Step 4: Park
    case STATE_PARKED:
      stopPlane();
      if (isPlaneMoving()) {
        digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
        logEvent("Error: Plane is not stationary as expected!");
      } else {
        digitalWrite(ERROR_LED_PIN, LOW); // Turn off the error LED
        logEvent("Plane has landed and stopped.");
      }
      break;
  }
}

// ----------------------------  TAKEOFF SEQUENCE  ---------------------------- //
/* Autopilot for takeoff stage. */
void takeOff(float targetAltitude) {
  // Code for increasing altitude until targetAltitude is reached (using distance sensor or altitude control system)
  // Code to increase speed and get the plane airborne
  // Use a non-blocking approach with millis()
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > 100) {
    lastCheckTime = millis();
    // Step 1: Increase throttle until takeoff speed is reached
    if (getSpeed() < MIN_TAKEOFF_SPEED) {
      if (getThrottle() < MAX_THROTTLE) {
        setThrottle(getThrottle() + THROTTLE_STEP);
      }
    }
    // Step 2: Pitch airplane up to take off
    if (getAltitude() < targetAltitude) {
      if (getElevatorAngle() < MAX_ELEVATOR) {
        setElevator(getElevatorAngle() + ELEVATOR_STEP);
      }
    }
  }
}

// ----------------------------  MAINTAIN LEVEL FLIGHT  ---------------------------- //
/* Autopilot for level flight. Takes a altitude target as input and attempts to maintain. */
void levelFlight(float targetAltitude) {
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > 100) {
    lastCheckTime = millis();
    // Maintain altitude for the given time
    // Use the distance sensor to monitor altitude
    float currentAltitude = getAltitude();
    float altitudeTolerance = 0.5; // Altitude must be +-0.5 of the target

    if (currentAltitude < targetAltitude - altitudeTolerance) {
      // Too low - adjust control surfaces
      setElevator(ELEVATOR_STEP);
    } else if (currentAltitude > targetAltitude + altitudeTolerance) {
      // Too high - adjust control surfaces
      setElevator(-ELEVATOR_STEP);
    } else {
      // Altitude good
      setElevator(LEVEL_ELEVATOR);
    }
    levelRoll(); // Maintain level roll during flight
  }
}

// ----------------------------  LANDING SEQUENCE  ---------------------------- //
/* Autopilot for landing sequence.  */
void land() {
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > 100) {
    lastCheckTime = millis();
    float currentAltitude = getAltitude();
    
    if (currentAltitude > LANDING_FLARE_ALTITUDE) {
      // Decrease altitude and safely land the plane by reducing throttle and/or control surfaces
      float throttle = getThrottle();
      if (throttle > MIN_THROTTLE) {
        throttle -= THROTTLE_STEP;
        setThrottle(throttle);
      }
      // setElevator(-ELEVATOR_STEP); // Pitch down slowly
    } else {
      // Once flare height is reached, increase elevator for flare and level roll
      setThrottle(MIN_THROTTLE);
      setElevator(FLARE_ELEVATOR_ANGLE);
      setAileronLeft(FLARE_AILERON_ANGLE);
      setAileronRight(FLARE_AILERON_ANGLE);
    }
  }
}

// ---------------------------- HOLDING PATTERN ---------------------------- //
/* Controls the plane to maintain a circular holding pattern. */
void holdingPattern() {
  // Example logic:
  // Maintain altitude
  levelFlight(TEST_ALTITUDE);
  // Apply a constant roll and rudder for a circular turn
  setAileronLeft(-10.0);
  setAileronRight(10.0);
  setRudder(-5.0);
}

// ----------------------------  STOP PLANE  ---------------------------- //
/* Stops the plane. */
void stopPlane() {
  // Set motors to 0. set control surfaces to neutral.
  setThrottle(0.0);
  setElevator(0.0);
  setAileronLeft(0.0);
  setAileronRight(0.0);
  setRudder(0.0);
  // enter low-power mode on microcontroller 
}

// ----------------------------  LEVEL ROLL  ---------------------------- //
/* Pulls the plane out of a roll. */
void levelRoll() {
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > 100) {
    lastCheckTime = millis();
    float rollAngle = getRollAngle();
    
    if (abs(rollAngle) > ROLL_TOLERANCE) {
      float correction = -rollAngle * 0.5; // Proportional control (must be tuned)
      // Clamp correction angle to ensure it doesn't exceed maximum aileron angle
      if (correction > MAX_AILERON) correction = MAX_AILERON;
      if (correction < MIN_AILERON) correction = MIN_AILERON;
      // Set ailerons to counteract roll
      setAileronLeft(-correction);
      setAileronRight(correction);
    } else {
      // Finally, set ailerons to 0.0 to keep plane level
      setAileronLeft(0.0);
      setAileronRight(0.0);
    }
  }
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
float getAltitude() {
  // return altitude; // Returns the value from the distance sensor that is pointed at the ground to get altitude
}

// ----------------------------  CURRENT SPEED  ---------------------------- //
/* Get the current speed. */
float getSpeed() {
  // return speed; // Return the speed
}

// ----------------------------  CURRENT THROTTLE  ---------------------------- //
/* Gets the current throttle value. */
float getThrottle() {
  // return currentThrottle; // Return the throttle power
}

// ----------------------------  CURRENT ROLL ANGLE  ---------------------------- //
/* Get the current roll angle. */
float getRollAngle() {
  // return rollAngle; // Return the roll angle
}

// ----------------------------  CURRENT ELEVATOR ANGLE  ---------------------------- //
/* Gets the current elevator angle. */
float getElevatorAngle() {
  // return currentElevator; // Return the elevator angle
}

// ----------------------------  CURRENT LEFT AILERON  ---------------------------- //
/* Gets the current left aileron angle. */
float getAileronLeftAngle() {
  // return currentAileronLeft;
}

// ----------------------------  CURRENT RIGHT AILERON  ---------------------------- //
/* Gets the current right aileron angle. */
float getAileronRightAngle() {
  // return currentAileronRight;
}

// ----------------------------  CURRENT RUDDER  ---------------------------- //
/* Gets the current rudder angle. */
float getRudderAngle() {
  // return currentRudder;
}

// ----------------------------  RESET SENSORS  ---------------------------- //
/* Resets sensors to default values. */
bool resetSensors() {
  // Reset logic here
  // E.g., set all sensor data to known starting values
  // groundAltitude = getAltitude();
  // Example: MPU6050.calibrate();
  return true;  // Return true if reset successful
}

// ----------------------------  SET THROTTLE  ---------------------------- //
/* Sets the throttle value between 0.0 and 1.0. */
void setThrottle(float throttleValue) {
  // currentThrottle = max(MIN_THROTTLE, min(MAX_THROTTLE, throttleValue));
  // return ; // Set the throttle power
}

// ----------------------------  SET ELEVATOR  ---------------------------- //
/* Sets the elevator angle to control aircraft pitch. */
void setElevator(float elevatorAngle) {
  // currentElevator = max(MIN_ELEVATOR, min(MAX_ELEVATOR, elevatorAngle));
  // return ; // Set the elevator angle
}

// ----------------------------  SET LEFT AILERON  ---------------------------- //
/* Sets the left aileron angle to control roll. */
float setAileronLeft(float leftAileronAngle) {
  // currentAileronLeft = max(MIN_AILERON, min(MAX_AILERON, aileronAngle));
  // return ;
}

// ----------------------------  SET RIGHT AILERON  ---------------------------- //
/* Sets the right aileron angle to control roll. */
float setAileronRight(float rightAileronAngle) {
  // currentAileronRight = max(MIN_AILERON, min(MAX_AILERON, aileronAngle));
  // return ;
}

// ----------------------------  SET RUDDER  ---------------------------- //
/* Sets the rudder angle to control yaw. */
float setRudder(float rudderAngle) {
  // currentRudder = max(MIN_RUDDER, min(MAX_RUDDER, rudderAngle));
  // return ;
}

// ----------------------------  LOG TO FILE  ---------------------------- //
/* Logs data to file on external memory. */
void logEvent(String message) {
  unsigned long elapsedTime = millis() - flightStartTime;

  // Calculate time components
  unsigned long milliseconds = elapsedTime % 1000;
  unsigned long totalSeconds = elapsedTime / 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long totalMinutes = totalSeconds / 60;
  unsigned long minutes = totalMinutes % 60;

  // Print to serial monitor for real-time debugging
  Serial.print("Time: ");
  if (minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print(" MIN : ");
  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
  Serial.print(" SEC : ");
  if (milliseconds < 100) Serial.print("0");
  if (milliseconds < 10) Serial.print("0");
  Serial.print(milliseconds);
  Serial.print(" MSEC - ");
  Serial.println(message);

  if (logFile) {
    logFile.print("Time: ");    // Prints time as XX MIN: YY SEC: ZZZZ MSEC
    if (minutes < 10) logFile.print("0");
    logFile.print(minutes);
    logFile.print(" MIN : ");
    if (seconds < 10) logFile.print("0");
    logFile.print(seconds);
    logFile.print(" SEC : ");
    if (milliseconds < 100) logFile.print("0");
    if (milliseconds < 10) logFile.print("0");
    logFile.print(milliseconds);
    logFile.print(" MSEC - ");
    logFile.println(message);
    logFile.flush();
  } else {
    Serial.println("Error: Failed to open log file for writing!");
  }
}
