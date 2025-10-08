/* ----------------------------  ARDUINO AIRPLANE CONTROL  ---------------------------- */
/* 
  Nethul Bodiratne 
  Last updated: 10/7/2025
*/
/* ------------------------------------------------------------------------------------ */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
// Add other necessary libraries for the sensors (e.g., MPU6050 for gyro/accelerometer, HMC5883L for magnetometer, etc.)

// --- Helper Functions/Macros (Arduino environment) ---
// Define the constrain function since it's used in the logic. Might not be needed for Arduino implementation.
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Pin Definitions for Mode Selection
#define MODE_0 2
#define MODE_1 3
#define MODE_2 4
#define MODE_3 5
#define MODE_4 6

// Error & Status LED Pins
#define ERROR_LED_PIN 13
//#define STATUS_GOOD_PIN 12

// Battery Monitoring. Assumes a 3S LiPo (max 12.6V) monitored via a 4:1 voltage divider (e.g., 30kOhm and 10kOhm). Formula: V_in = V_out * ((R1 + R2) / R2) 
#define BATTERY_ADC_PIN A0 
const float ADC_REFERENCE_VOLTAGE = 5.0f; // V (Arduino VCC/AREF)
const float VOLTAGE_DIVIDER_RATIO = 4.0f; 
const float VOLTAGE_FULL_LEVEL = 12.6f;
const float VOLTAGE_WARNING_LEVEL = 10.5f; // ~3.5V per cell
const float VOLTAGE_CRITICAL_LEVEL = 9.9f; // ~3.3V per cell - Triggers auto-shutdown
const unsigned long BATTERY_CHECK_PERIOD = 60000; // milliseconds - Rate at which to check battery voltage
unsigned long lastBatteryCheckTime = 0;
float currentBatteryVoltage = VOLTAGE_FULL_LEVEL; // (NEW) Initial voltage

// Flight Plan Timers
unsigned long flightStartTime = 0;
unsigned long modeStartTime = 0;
unsigned long stepStartTime = 0;
unsigned long lastLoopTime = 0;

// Set a fixed loop frequency for real-time control (e.g., 100 Hz)
const unsigned long CONTROL_LOOP_PERIOD = 10; // milliseconds - Sets the rate at which main loop logic runs
unsigned long lastControlTime = 0;
const unsigned long LOG_PERIOD = 1000; // milliseconds - Rate at which to flush logs to storage
unsigned long lastLogFlushTime = 0;

// Sensor data variables
float gyroData[3] = {0.0f, 0.0f, 0.0f};    // Example: {roll, pitch, yaw} or {x, y, z}
float accelData[3] = {0.0f, 0.0f, 0.0f};   // Example: {x, y, z} acceleration
float distanceData = 0.0f;
float headingData = 0.0f;
float groundAltitude = 0.0f; 		// Altitude relative to takeoff point (calibrated)
float lastAltitude = 0.0f;
float rollAngle = 0.0f; 				// Calculated roll angle
float lastRoll = 0.0f;
float flightSpeed = 0.0f; 			// Air speed or ground speed

// Sensor offsets for error correction during sensor reset
float altitudeOffset = 0.0f;
float accelOffset[3] = {0.0f, 0.0f, 0.0f};
float gyroOffset[3] = {0.0f, 0.0f, 0.0f};
float headingOffset = 0.0f;

// Checking sensors
bool constantsOkay = true;

// Control Surface Variables
float currentThrottle = 0.0f;
float currentElevator = 0.0f;
float currentAileronLeft = 0.0f;
float currentAileronRight = 0.0f;
float currentRudder = 0.0f;

// PID variables (might have to reset these between every mode or flight step change)
// PID for Altitude Control (Elevator)
float Kp_alt = 1.5f;   // Proportional gain for altitude
float Ki_alt = 0.1f;   // Integral gain for altitude
float Kd_alt = 0.5f;   // Derivative gain for altitude
float error_alt = 0.0f;
float last_error_alt = 0.0f;
float integral_alt = 0.0f;

// PID Variables for Roll Control (Ailerons)
float Kp_roll = 1.0f;  // Proportional gain for roll
float Ki_roll = 0.01f; // Integral gain for roll
float Kd_roll = 0.2f;  // Derivative gain for roll
float error_roll = 0.0f;
float last_error_roll = 0.0f;
float integral_roll = 0.0f;

// Global fault state tracker
bool systemInFault = false;

// Mode Enum for Better Readability
enum Mode {
  MODE_RESET_SENSORS = 0,
  MODE_STOPPED,
  MODE_TAXI,
  MODE_TAKEOFF_LEVEL_LAND,
  MODE_HOLDING_PATTERN,
  MODE_FAULT  // Mode for critical, non-recoverable errors which should ground the plane
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

// Log file creation and log buffer
File logFile;
bool sdCardAvailable = false;
char logBuffer[128];  // Must be longer than 96

// Constants
const unsigned long TEST_LEVEL_DURATION = 10000; // milliseconds - Duration for maintaining level flight.
const unsigned long TEST_HOLDING_PATTERN_DURATION = 20000; // milliseconds - Duration for circular holding pattern.
const unsigned long SENSOR_RESET_STATIONARY_DURATION = 5000; // milliseconds - Duration the plane must be stationary for a sensor reset.
const unsigned long TAXI_DURATION = 5000; // milliseconds - Duration for taxiing on the runway.
const float LANDING_FLARE_ALTITUDE = 0.25f; // meters - The altitude at which the plane initiates the flare maneuver.
const float ROLL_TOLERANCE = 1.0f; // degrees - The acceptable tolerance for the plane's roll angle.
const float TEST_ALTITUDE = 1.0f; // meters - Target altitude for test flights.
const float TEST_BANK_ANGLE = 15.0f;  // degrees - Target bank angle for the circular holding pattern.
const float MIN_TAKEOFF_SPEED = 15.0f; // m/s - The minimum speed required for takeoff.
const float MIN_THROTTLE = 0.0f; // minimum throttle power (0.0 to 1.0)
const float MAX_THROTTLE = 1.0f; // maximum throttle power (0.0 to 1.0)
const float THROTTLE_STEP = 0.05f; // step throttle power by 5%
const float LEVEL_ELEVATOR = 0.0f; // degrees - elevator angle for level flight
const float MIN_ELEVATOR = -15.0f; // degrees - minimum elevator angle (TE up)
const float MAX_ELEVATOR = 15.0f; // degrees - maximum elevator angle (TE down)
const float ELEVATOR_STEP = 1.0f; // degrees - step elevator angle by 1
const float MIN_AILERON = -15.0f; // degrees - minimum aileron angle
const float MAX_AILERON = 15.0f; // degrees - maximum aileron angle
const float AILERON_STEP = 1.0f; // degrees - step ailerons angle by 1
const float MIN_RUDDER = -15.0f; // degrees - rudder full left angle
const float MAX_RUDDER = 15.0f; // degrees - rudder full right angle
const float RUDDER_STEP = 1.0f; // degrees - step rudder angle by 1
const float FLARE_ELEVATOR_ANGLE = 10.0f; // degrees - The elevator angle for the landing flare.
const float FLARE_AILERON_ANGLE = 10.0f; // degrees - Aileron angle to maintain stability during flare.
const float MIN_ACCEL_SQUARED_THRESHOLD = 0.01f;  // m/s^2 - Sensor Stationary Threshold (for isPlaneMoving optimization). Check against the squared value to avoid sqrt() and pow() in isPlaneMoving().

// ----------------------------  FUNCTION PROTOTYPES  ---------------------------- //
// Define all function prototypes to avoid compilation errors
// General Utility Functions
void logEvent(const char* message);
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
void checkConstantsOrder();
void checkSensorConnections();
void enterSleep();
float readBatteryvoltage();
void checkBatteryStatus();

// Control surface Functions
void setThrottle(float throttleValue);
void setElevator(float elevatorAngle);
void setAileronLeft(float aileronAngle);
void setAileronRight(float aileronAngle);
void setRudder(float rudderAngle);

// PID Funstions
float calcElevatorPID(float targetAltitude, float dt);
float calcAileronPID(float targetRoll, float dt);

// Mode functions
void modeResetSensors();
void modeStopped();
void modeTaxi();
void modeTakeoffLevelLand(float dt);
void modeHoldingPattern(float dt);
void modeFault();

// Flight Sequence functions
void takeOff(float targetAltitude, float dt);
void cruiseFlight(float targetAltitude, float targetRoll, float dt);
void levelRoll(float dt);
void holdingPattern(float dt);
void land(float dt);

// ----------------------------  SETUP  ---------------------------- //
void setup() {
  // Initialize serial communication
  Serial.begin(115200); // (9600 or 115200) Increasing the baud rate to 115200 may allow faster non-blocking logging

  // Setup pins
  pinMode(MODE_0, INPUT_PULLUP); // Using INPUT_PULLUP might be more stable than INPUT
  pinMode(MODE_1, INPUT_PULLUP);
  pinMode(MODE_2, INPUT_PULLUP);
  pinMode(MODE_3, INPUT_PULLUP);
  pinMode(MODE_4, INPUT_PULLUP);
  pinMode(ERROR_LED_PIN, OUTPUT);
  //pinMode(STATUS_GOOD_PIN, OUTPUT);

  // Set ERROR LED to default start state (low)
  digitalWrite(ERROR_LED_PIN, LOW);

  // Initialize SD card
  if (!SD.begin(BUILTIN_CS)) {
    Serial.println("SD card initialization failed.");
    digitalWrite(ERROR_LED_PIN, HIGH);
    sdCardAvailable = false; // Flag for SD card error
  } else {
    sdCardAvailable = true;
    // Open the log file ONCE in setup()
    logFile = SD.open("flight_log.txt", FILE_WRITE);
    if (logFile) {
      logFile.println("Flight Log Start - Initailized");
      logFile.flush(); // Use flush() to ensure data is written to the card
    } else {
      Serial.println("ERROR: Could not open flight_log.txt");
    }
  }

  // Initialize sensors here
  Wire.begin();
  Serial.println("Sensors Initializing...");
  // (e.g., MPU6050.begin(), HMC5883L.begin(), etc.)
  Serial.println("Sensors Initialized.");

  // Initialize timers for system uptime
  flightStartTime = millis();
  modeStartTime = millis();
  lastLoopTime = millis();
  lastControlTime = millis();
  lastBatteryCheckTime = millis();

  // Initail checks and reads
  checkConstantsOrder();
  checkSensorConnections();
  readSensors();
  checkBatteryStatus();

  // Read initial mode from pins
  currentMode = checkModePins();
  lastMode = currentMode;

  // --- Power Reduction Optimization ---
  // Turn off the Analog-to-Digital Converter (ADC) if not using analogRead() as its a constant power drain.
  // PRR |= (1 << PRADC); // Not turned off because ADC is used for battery monitoring.
  // Turn off Timer 2 and Timer 1 if they are not used for servo/motor PWM. If Timer1/Timer2 are being used for PWM, DO NOT disable them.
  PRR |= (1 << PRTIM2); 
  PRR |= (1 << PRTIM1);
  // TWI (Wire) and SPI (SD) must remain active as they are used.
  
  if (systemInFault) {
    currentMode = MODE_FAULT;
    logEvent("CRITICAL FAULT during setup. System locked to MODE_FAULT.");
  } else {
    snprintf(logBuffer, sizeof(logBuffer), "System Setup completed. Initial mode set to %d", currentMode);
    logEvent(logBuffer);
  }
}

// ----------------------------  LOOP  ---------------------------- //
void loop() {
  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_LOOP_PERIOD) {
    // Calculate delta time for PID calculations
    float dt = (now - lastLoopTime) / 1000.0f; // Convert to seconds
    if (dt < 0.001f) dt = 0.001f; // Prevent division by zero
    lastControlTime = now;

    // Always read sensor data at the beginning of the loop
    readSensors();

    // Check battery status periodically
    if (now - lastBatteryCheckTime >= BATTERY_CHECK_PERIOD) {
      checkBatteryStatus();
      lastBatteryCheckTime = now;
    }
    
    // Check the physical mode selection pins
    currentMode = checkModePins();

    // Handle mode transitions and reset the mode timer
    if (systemInFault) {
      currentMode = MODE_FAULT; // Move to fault mode if a fault occurs
    } else if (currentMode != lastMode) {
      snprintf(logBuffer, sizeof(logBuffer), "Transitioning from Mode %d to Mode %d", lastMode, currentMode);
      logEvent(logBuffer);
      lastMode = currentMode;
      modeStartTime = millis();
      currentFlightState = STATE_START;
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
        modeTakeoffLevelLand(dt);
        break;
      case MODE_HOLDING_PATTERN:
        modeHoldingPattern(dt);
        break;
      case MODE_FAULT:
        modeFault();
        break;
      default:
        // Unknown mode, default to fault and log an error
        systemInFault = true;
        currentMode = MODE_FAULT;
        snprintf(logBuffer, sizeof(logBuffer), "ERROR: Unknown mode selected (%d). Defaulting to FAULT Mode.", currentMode);
        logEvent(logBuffer);
        break;
    }

    // Update lastLoopTime after other logic to keep it accurate
    lastLoopTime = now;
  } else {
    // Enter Idle mode to save power. Shouldn't cause the program to miss deadlines or interfere with sensors
    snprintf(logBuffer, sizeof(logBuffer), "Entering Sleep Mode.");
    logEvent(logBuffer);
    enterSleep();
  }

  if (sdCardAvailable && now - lastLogFlushTime >= LOG_PERIOD) {
    lastLogFlushTime = now;
    if (logFile) {
      logFile.flush();  // Flush logs to file now
    }
  }
}

// ----------------------------  READ SENSORS  ---------------------------- //
/* Read and store sensor data in appropriate variables. */
void readSensors() {
  // Placeholder for sensor reading logic
  // Example: MPU6050.readSensors(&accelData, &gyroData);
  // A simple way to manage a mix of sensors that update at different rates
  // static unsigned long lastAccelTime = 0;
  // if (millis() - lastAccelTime > 50) {
  //   // Read accelerometer data here
  //   lastAccelTime = millis();
  //   // Example:
  //   mpu.getMotion6(&accelData[0], &accelData[1], &accelData[2], &gyroData[0], &gyroData[1], &gyroData[2]);
  // }
}

// ----------------------------  READ MODE PINS  ---------------------------- //
/* Checks the mode selection pins using the returns corresponding enum. */
Mode checkModePins() {
  // Read the state of each pin
  int pin0 = digitalRead(MODE_0);
	int pin1 = digitalRead(MODE_1);
	int pin2 = digitalRead(MODE_2);
	int pin3 = digitalRead(MODE_3);
	int pin4 = digitalRead(MODE_4);
  
  // Combine pin states into a binary number
  // Using INPUT_PULLUP, LOW means the switch is closed.
	int modeValue = (!pin4 << 4) | (!pin3 << 3) | (!pin2 << 2) | (!pin1 << 1) | !pin0;
  
  // Map binary value to the Mode enum
  // Example: 0001 -> MODE_RESET_SENSORS
  // This is a simple example; you can define the mapping as you wish
  switch (modeValue) {
    case 1: // 00001
			return MODE_RESET_SENSORS;
		case 2: // 00010
			return MODE_STOPPED;
		case 3: // 00011
			return MODE_TAXI;
		case 4: // 00100
			return MODE_TAKEOFF_LEVEL_LAND;
		case 5: // 00101
			return MODE_HOLDING_PATTERN;
		// case 31: // 11111 - All pins pulled LOW might be a special debug/override mode
		// 	return MODE_FAULT;
		default:
			return MODE_FAULT; // Default to a safe stopped or fault mode
  }
}

// ----------------------------  FAULT MODE  ---------------------------- //
/* Mode for landling critical fault states for non-recoverable errors. */
void modeFault() {
  stopPlane();

  // Keep the error LED on
  digitalWrite(ERROR_LED_PIN, HIGH);
}

// ----------------------------  MODE 0 (RESET SENSORS)  ---------------------------- //
/* Flight Plan: Stops the plane then if stationary for duration reset sensors. */
void modeResetSensors() {
  stopPlane();
  
  // Check if the plane is moving or stationary
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("ERROR: Plane is not stationary as expected.");
    modeStartTime = millis(); // Reset timer until stationary
		return;
  }
  
  // Check if stationary for x seconds
  if (millis() - modeStartTime > SENSOR_RESET_STATIONARY_DURATION) {  // x = 10 seconds for sensor reset, can be changed later
    if (!resetSensors()) {
      digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED if reset fails
      logEvent("ERROR: Sensor reset failed.");
      systemInFault = true;
    } else {
      logEvent("Sensors reset successfully.");
      // Transition to MODE_STOPPED automatically
      currentMode = MODE_STOPPED;
    }
  }
}

// ----------------------------  MODE 1 (STOPPED)  ---------------------------- //
/* Flight Plan: Stops the plane then checks if staionary. */
void modeStopped() {
  stopPlane();
  
  // Check if the plane is moving or stationary
  if (isPlaneMoving()) {
    digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
    logEvent("ERROR: Plane is not stationary as expected.");
  }
}

// ----------------------------  MODE 2 (TAXI)  ---------------------------- //
/* Flight Plan: Taxi the plane for a certain duration. */
void modeTaxi() {
  // Add taxi logic based on distance/time
  if (millis() - modeStartTime < TAXI_DURATION) { // Taxi for 5 seconds (as an example)
    // Code to control the plane's motors for taxiing
    setThrottle(0.2f); // Example throttle for taxiing, can be changed to setThrottle(TAXI_THROTTLE)
    setRudder(0.0f);
  } else { // Stop and transition to parked state
    stopPlane();
    if (isPlaneMoving()) {
      digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
      logEvent("ERROR: Plane is not stationary as expected.");
    } else {
      logEvent("Plane has completed taxi and stopped.");
    }
  }
}

// ----------------------------  MODE 3 (TAKEOFF, LEVEL, LAND)  ---------------------------- //
/* Flight Plan: Plane should takeoff and climb to a preset altitude. It should then level off and cruise for a preset duration then land and park. */
void modeTakeoffLevelLand(float dt) {
  switch (currentFlightState) {
    case STATE_START:
      logEvent("Starting takeoff sequence.");
      currentFlightState = STATE_TAKEOFF;
      stepStartTime = millis();
      break;

    // Step 1: Takeoff
    case STATE_TAKEOFF:
      if (getAltitude() < TEST_ALTITUDE) {
        takeOff(TEST_ALTITUDE, dt);
      } else {
        logEvent("Reached target altitude. Transitioning to level flight.");
        currentFlightState = STATE_LEVEL_FLIGHT;
        stepStartTime = millis();
      }
      break;
      
    // Step 2: Level Flight
    case STATE_LEVEL_FLIGHT:
      if (millis() - stepStartTime < TEST_LEVEL_DURATION) {
        cruiseFlight(TEST_ALTITUDE, 0.0f, dt);
      } else {
        logEvent("Level flight duration complete. Transitioning to landing.");
        currentFlightState = STATE_LANDING;
        stepStartTime = millis();
      }
      break;
    
    // Step 3: Landing
    case STATE_LANDING:
      if (getAltitude() > groundAltitude) {
        land(dt);
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
        logEvent("ERROR: Plane is not stationary as expected.");
      } else {
        logEvent("Plane has landed and stopped.");
      }
      break;
  }
}

// ----------------------------  MODE 4 (CIRCULAR HOLDING PATTERN)  ---------------------------- //
/* Flight Plan: Take off then maintain altitude while flying in a constant turn. */
void modeHoldingPattern(float dt) {
  switch (currentFlightState) {
    case STATE_START:
      logEvent("Starting takeoff sequence for holding pattern.");
      currentFlightState = STATE_TAKEOFF;
      stepStartTime = millis();
      break;
    
    // Step 1: Take off
    case STATE_TAKEOFF:
      if (getAltitude() < TEST_ALTITUDE) {
        takeOff(TEST_ALTITUDE, dt);
      } else {
        logEvent("Reached target altitude. Transitioning to holding pattern.");
        currentFlightState = STATE_HOLDING_PATTERN;
        stepStartTime = millis();
      }
      break;
    
    // Step 2: Hold Constant Turn
    case STATE_HOLDING_PATTERN:
      if (millis() - stepStartTime < TEST_HOLDING_PATTERN_DURATION) {
        holdingPattern(dt); // Might be able to be changed to cruiseFlight() but cruiseFlight() cant use the rudder yet.
      } else {
        logEvent("Holding pattern complete. Transitioning to landing.");
        currentFlightState = STATE_LANDING;
        stepStartTime = millis();
      }
      break;
      
    // Step 3: Landing
    case STATE_LANDING:
      if (getAltitude() > groundAltitude) {
        land(dt);
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
        logEvent("ERROR: Plane is not stationary as expected.");
      } else {
        logEvent("Plane has landed and stopped.");
      }
      break;
  }
}

// ----------------------------  TAKEOFF SEQUENCE  ---------------------------- //
/* Autopilot for takeoff stage. */
void takeOff(float targetAltitude, float dt) {
  // Code for increasing altitude until targetAltitude is reached (using distance sensor or altitude control system)
  // Code to increase speed and get the plane airborne
  // Step 1: Increase throttle until takeoff speed is reached
  if (getSpeed() < MIN_TAKEOFF_SPEED) {
    // Ramp up throttle gradually
		setThrottle(constrain(getThrottle() + (THROTTLE_STEP), MIN_THROTTLE, MAX_THROTTLE));
	} else {
		// Maintain a fixed high throttle for climb-out
		setThrottle(MAX_THROTTLE * 0.9f);
	}
  // Step 2: Pitch airplane up to take off and climb
  if (getAltitude() < targetAltitude) {
    float elevatorCommand = calcElevatorPID(targetAltitude, dt);
		setElevator(elevatorCommand);
	}

	// Maintain level roll during the climb-out
	levelRoll(dt);
}

// ----------------------------  MAINTAIN LEVEL FLIGHT  ---------------------------- //
/* Autopilot for level flight. Takes a altitude target as input and attempts to maintain. */
void cruiseFlight(float targetAltitude, float targetRoll, float dt) {
  // PID control for altitude
  float elevatorCommand = calcElevatorPID(targetAltitude, dt);
  setElevator(elevatorCommand);
	// Set cruise throttle (or use speed PID if implemented)
	setThrottle(MAX_THROTTLE * 0.6f); // Could use a CRUISE_THROTTLE var instead or PID
  
  // PID control for roll
  float aileronCommand = calcAileronPID(targetRoll, dt); 
  // Set ailerons differentially based on the unified command
  setAileronLeft(-aileronCommand);
  setAileronRight(aileronCommand);
  
  // Set rudder based on roll angle. The angle should be higher based on the targetRoll but start at a certain threshold (eg. targetRoll = 5 gives rudder = 0 and targetRoll = 6 gives rudder = 1 and so on).
  // if (targetRoll <= 5) {
  //   setRudder(0.0f);
  // } else {
  //   setRudder();
  // }
  // Reset rudder angle at the end
}

// ----------------------------  LANDING SEQUENCE  ---------------------------- //
/* Autopilot for landing sequence.  */
void land(float dt) {
  float currentAltitude = getAltitude();
   
  levelRoll(dt);
  
  // Decrease altitude and safely land the plane by reducing throttle and/or control surfaces
  if (currentAltitude > LANDING_FLARE_ALTITUDE) {
    // Gently reduce throttle for descent
		setThrottle(constrain(getThrottle() - (THROTTLE_STEP), MIN_THROTTLE, MAX_THROTTLE * 0.3f));
		
		// Use elevator to maintain a shallow glide slope (pitch slightly down or neutral)
		setElevator(LEVEL_ELEVATOR - ELEVATOR_STEP); 
  } else {
    // Once flare height is reached, increase elevator for flare and level roll
    logEvent("Initiating Landing Flare.");
    setThrottle(MIN_THROTTLE);
    setElevator(FLARE_ELEVATOR_ANGLE);
    setAileronLeft(FLARE_AILERON_ANGLE);
    setAileronRight(FLARE_AILERON_ANGLE);
  }
}

// ---------------------------- HOLDING PATTERN ---------------------------- //
/* Controls the plane to maintain a circular holding pattern. */
void holdingPattern(float dt) {
  // Example logic:
  // Maintain altitude
  cruiseFlight(TEST_ALTITUDE, TEST_BANK_ANGLE, dt);
  
	// Apply a constant rudder input to assist the turn (optional, depends on airframe)
	setRudder(MAX_RUDDER * 0.3f);
}

// ----------------------------  STOP PLANE  ---------------------------- //
/* Stops the plane. */
void stopPlane() {
  // Set motors to 0. set control surfaces to neutral.
  setThrottle(0.0f);
  setElevator(0.0f);
  setAileronLeft(0.0f);
  setAileronRight(0.0f);
  setRudder(0.0f);
  // Reset PID integrals
  integral_alt = 0.0f;
  integral_roll = 0.0f;
  last_error_alt = 0.0f;
  last_error_roll = 0.0f;
  enterSleep();  // Enter low-power idle on microcontroller. A deeper sleep mode can be implemented in a later version.
}

// ----------------------------  LEVEL ROLL  ---------------------------- //
/* Pulls the plane out of a roll. */
void levelRoll(float dt) {
  float aileronCommand = calcAileronPID(0.0f, dt); // Target roll is 0 degrees
  setAileronLeft(-aileronCommand); // Aileron deflection is opposite on each side
  setAileronRight(aileronCommand);
}

// ----------------------------  ELEVATOR PID  ---------------------------- //
/* PID function to calculate the elevator angle. */
float calcElevatorPID(float targetAltitude, float dt) {
  // Get current altitude from sensor (placeholder)
  float currentAltitude = getAltitude();
  
  // Proportional term
  error_alt = targetAltitude - currentAltitude;
  float p_term = Kp_alt * error_alt;

  // Integral term
  integral_alt += error_alt * dt;
  // Anti-windup: clamp the integral term to a reasonable range
  float integral_limit = 10.0f;
  integral_alt = constrain(integral_alt, -integral_limit, integral_limit);
  float i_term = Ki_alt * integral_alt;
  
  // Derivative term
  float derivative_alt = (currentAltitude - lastAltitude) / dt;
  float d_term = Kd_alt * derivative_alt;
  lastAltitude = currentAltitude;

  // Calculate the PID output and clamp to the allowed range
  float output = p_term + i_term + d_term;
  
  return constrain(output, MIN_ELEVATOR, MAX_ELEVATOR);
}

// ----------------------------  AILERON PID  ---------------------------- //
/* PID function to calculate the aileron angles. */
float calcAileronPID(float targetRoll, float dt) {
  // Get current roll angle from sensor (placeholder)
  float currentRoll = getRollAngle();
  
  // Proportional term
  error_roll = targetRoll - currentRoll;
  float p_term = Kp_roll * error_roll;

  // Integral term
  integral_roll += error_roll * dt;
  // Anti-windup: clamp the integral term to a reasonable range
  float integral_limit = 5.0f;
  integral_roll = constrain(integral_roll, -integral_limit, integral_limit);
  float i_term = Ki_roll * integral_roll;
  
  // Derivative term
  float derivative_roll = (currentRoll - lastRoll) / dt;
  float d_term = Kd_roll * derivative_roll;
  lastRoll = currentRoll;

  // Calculate the PID output and clamp to the allowed range
  float output = p_term + i_term + d_term;
  
  return constrain(output, MIN_AILERON, MAX_AILERON);
}

// ----------------------------  CHECK IF MOVING  ---------------------------- //
/* Checks if the plane is moving. Returns true if moving. */
bool isPlaneMoving() {
  // A simple example; this could be expanded based on gyro/accel data
  return (accelData[0]*accelData[0] + accelData[1]*accelData[1] + accelData[2]*accelData[2]) > (0.1f * 0.1f); // (Accel[x]^2 + Accel[y]^2 + Accel[z]^2) > (threshold^2)
}

// ----------------------------  CURRENT ALTITUDE  ---------------------------- //
/* Get the current altitude. */
float getAltitude() {
  // return altitude - altitudeOffset; // Returns the value from the distance sensor that is pointed at the ground to get altitude
}

// ----------------------------  CURRENT SPEED  ---------------------------- //
/* Get the current speed. */
float getSpeed() {
  // return flightSpeed; // Return the speed
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

// ----------------------------  SET THROTTLE  ---------------------------- //
/* Sets the throttle value between 0.0 and 1.0. */
void setThrottle(float throttleValue) {
  // currentThrottle = constrain(throttleValue, MIN_THROTTLE, MAX_THROTTLE);
  // return ; // Set the throttle power
}

// ----------------------------  SET ELEVATOR  ---------------------------- //
/* Sets the elevator angle to control aircraft pitch. */
void setElevator(float elevatorAngle) {
  // currentElevator = constrain(elevatorAngle, MIN_ELEVATOR, MAX_ELEVATOR);
  // return ; // Set the elevator angle
}

// ----------------------------  SET LEFT AILERON  ---------------------------- //
/* Sets the left aileron angle to control roll. */
void setAileronLeft(float leftAileronAngle) {
  // currentAileronLeft = constrain(leftAileronAngle, MIN_AILERON, MAX_AILERON);
  // return ;
}

// ----------------------------  SET RIGHT AILERON  ---------------------------- //
/* Sets the right aileron angle to control roll. */
void setAileronRight(float rightAileronAngle) {
  // currentAileronRight = constrain(rightAileronAngle, MIN_AILERON, MAX_AILERON);
  // return ;
}

// ----------------------------  SET RUDDER  ---------------------------- //
/* Sets the rudder angle to control yaw. */
void setRudder(float rudderAngle) {
  // currentRudder = constrain(rudderAngle, MIN_RUDDER, MAX_RUDDER);
  // return ;
}

// ----------------------------  RESET SENSORS  ---------------------------- //
/* Resets sensors to default values. */
bool resetSensors() {
  logEvent("Calibrating sensors. Please ensure plane is stationary.");

  // Reset logic here

  // Set all offsets to 0
  altitudeOffset = 0.0f;
  for (int i = 0; i < 3; i++) {
    accelOffset[i] = 0.0f;
    gyroOffset[i] = 0.0f;
  }
  headingOffset = 0.0f;

  float rawDistance = getAltitude();
  // the rest of the raw measurements go here
  // E.g., set all sensor data to known starting values
  // altitudeOffset = getAltitude();
  // Alternate option example if sensor supports it: MPU6050.calibrate();  
  // Read raw sensor data to get the offset values
  
  // Store these values as offsets, they will be used as corrections for the get functions
  // altitudeOffset = rawDistance;
  // accelOffset[0] = rawAccel[0];
  // accelOffset[1] = rawAccel[1];
  // accelOffset[2] = rawAccel[2] - 9.81f; // Account for gravity
  // gyroOffset[0] = rawGyro[0];
  // gyroOffset[1] = rawGyro[1];
  // gyroOffset[2] = rawGyro[2];
  // headingOffset = rawHeading;
  
  logEvent("Calibration complete. Stored offsets:");
  snprintf(logBuffer, sizeof(logBuffer), "Altitude Offset: %.2f m", altitudeOffset);
  logEvent(logBuffer);
  snprintf(logBuffer, sizeof(logBuffer), "Accel Offsets: {%.2f, %.2f, %.2f}", accelOffset[0], accelOffset[1], accelOffset[2]);
  logEvent(logBuffer);
  snprintf(logBuffer, sizeof(logBuffer), "Gyro Offsets: {%.2f, %.2f, %.2f}", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  logEvent(logBuffer);
  snprintf(logBuffer, sizeof(logBuffer), "Heading Offset: %.2f", headingOffset);
  logEvent(logBuffer);

  return true;  // Return true if reset successful
}

// ----------------------------  CHECK BATTERY VOLTAGE  ---------------------------- //
/* Reads the current battery voltage level and returns it in a useable format. */
float readBatteryVoltage() {
  // Read the raw 10-bit value (0-1023)
  int rawBatteryADC = analogRead(BATTERY_ADC_PIN); 

  // Conversion Formula: V_in = (Raw_ADC / 1023.0) * V_Ref * V_Divider_Ratio
  currentBatteryVoltage = ((float)rawADC / 1023.0f) * ADC_REFERENCE_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
  
  return currentBatteryVoltage;
}

// ----------------------------  CHECK BATTERY STAUS  ---------------------------- //
/* Checks the battery voltage against critical and warning thresholds. */
void checkBatteryStatus() {
  readBatteryVoltage(); // Update the global voltage variable

  if (currentBatteryVoltage <= VOLTAGE_CRITICAL_LEVEL) {
    // CRITICAL FAULT: Initiate emergency shutdown
    if (!systemInFault) {
      snprintf(logBuffer, sizeof(logBuffer), "CRITICAL: Battery voltage %.2fV. Initiating auto-shutdown and entering MODE_FAULT.", currentBatteryVoltage);
      logEvent(logBuffer);
      systemInFault = true; // This will trigger the MODE_FAULT transition in the next loop() iteration
    }
  } else if (currentBatteryVoltage < VOLTAGE_WARNING_LEVEL) {
    // WARNING: Notify of low power, but continue flight
    snprintf(logBuffer, sizeof(logBuffer), "WARNING: Low battery voltage at %.2fV. Consider landing.", currentBatteryVoltage);
    logEvent(logBuffer);
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN)); // Blink the error LED for warning
  }
}

// ----------------------------  CHECK CONSTANTS MIN & MAX  ---------------------------- //
/* Ensures that the min andd max values of the constants are correctly ordered (min < max). */
void checkConstantsOrder() {
  if (MIN_THROTTLE >= MAX_THROTTLE) {
    logEvent("ERROR: MIN_THROTTLE is not less than MAX_THROTTLE.");
    constantsOkay = false;
  }
  if (MIN_ELEVATOR >= MAX_ELEVATOR) {
    logEvent("ERROR: MIN_ELEVATOR is not less than MAX_ELEVATOR.");
    constantsOkay = false;
  }
  if (MIN_AILERON >= MAX_AILERON) {
    logEvent("ERROR: MIN_AILERON is not less than MAX_AILERON.");
    constantsOkay = false;
  }
  if (MIN_RUDDER >= MAX_RUDDER) {
    logEvent("ERROR: MIN_RUDDER is not less than MAX_RUDDER.");
    constantsOkay = false;
  }

  if (!constantsOkay) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    logEvent("FATAL ERROR: Invalid min/max constants detected.");
    systemInFault = true;
  } else {
    logEvent("All min/max constants are correctly ordered.");
  }
}

// ----------------------------  CHECK SENSOR CONNECTIONS  ---------------------------- //
/* Checks if there are sensor connection issues at startup. */
void checkSensorConnections() {
  // if (!mpu.testConnection()) {
  //   logEvent("ERROR: MPU6050 sensor failure. Possible connection issue.");
  //   digitalWrite(ERROR_LED_PIN, HIGH); // Turn on error LED
  //   systemInFault = true;
  // }
  // if (!compass.testConnection()) {
  //   logEvent("ERROR: HMC5883L sensor failure. Possible connection issue.");
  //   digitalWrite(ERROR_LED_PIN, HIGH);
  //   systemInFault = true;
  // }
  // if (distanceData < 0.1) {  // Threshold to detect if the distance sensor is malfunctioning
  //   logEvent("ERROR: Distance sensor reading invalid. Possible connection issue.");
  //   digitalWrite(ERROR_LED_PIN, HIGH);
  //   systemInFault = true;
  // }
  logEvent("Sensor connection check complete.");
}

void enterSleep() {
  // 1. Select the lightest sleep state: Idle Mode.
  // This stops the CPU clock (clkCPU) and Flash clock (clkFLASH), but leaves the I/O clock (clkI/O) running for all peripherals.
  set_sleep_mode(SLEEP_MODE_IDLE); 

  // 2. Enable the sleep mode.
  sleep_enable();

  // 3. Put the CPU to sleep.
  // The program wakes up immediately when the next interrupt occurs (e.g., Timer 0 overflow for millis()).
  sleep_cpu();

  // 4. Disable sleep mode immediately upon waking.
  sleep_disable(); 
  
  // Note: No need to disable/re-enable interrupts (cli()/sei()) here because the function is being called *after* an interrupt has already been detected (or within the main loop awaiting an event), and Idle Mode has very low latency.
}

// ----------------------------  LOG TO FILE  ---------------------------- //
/* Logs data to file on external memory. */
void logEvent(const char* message) {
  unsigned long elapsedTime = millis() - flightStartTime;

  // Calculate time components
  unsigned long milliseconds = elapsedTime % 1000;
  unsigned long totalSeconds = elapsedTime / 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long totalMinutes = totalSeconds / 60;
  unsigned long minutes = totalMinutes % 60;

  // Format time string
  char timeStr[32];
  snprintf(timeStr, sizeof(timeStr), "%02lu MIN : %02lu SEC : %03lu MSEC - ", minutes, seconds, milliseconds);
  
  // Print to serial monitor for real-time debugging (Immediate)
  Serial.print(timeStr);
  Serial.println(message);

  // Write to log file (will be flushed periodically)
  if (sdCardAvailable && logFile) {
    logFile.print(timeStr);
    logFile.println(message);
    // Do NOT call logFile.flush() here, as it's slow. It's called periodically in loop().
  } else if (sdCardAvailable) {
    // logFile is closed, but SD is available. Should not happen unless corrupted.
    Serial.println("ERROR: Failed to open log file for writing.");
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
}
