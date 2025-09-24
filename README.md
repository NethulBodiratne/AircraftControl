# AircraftControl <br>

This project has two variations. <br>
A. Arduino Flight Controller. aircraft_control1_Ardu.ino: C++ code which reads sensors to fly the plane in various modes. Designed to run on an Arduino. <br>
B. Raspberry Pi Flight Controller. aircraft_controlX_RPi.py: Python code that senses the orientation of an aircraft and adjusts accordingly. Designed to run on a SenseHat. <br>

A. Arduino Flight Controller <br>
A1. Overview <br>
This project is an autonomous flight controller for a 3D-printed aircraft I designed, written in Arduino C++. It utilizes multiple flight modes, onboard sensors, and a state-machine architecture to execute predefined flight plans, including takeoff, level flight, landing, and holding patterns. It also logs all events to an SD card for post-flight analysis. Flight modes are selected by digital pins, and each mode corresponds to a different flight plan (e.g, stopped, taxi, short flight, etc.).

A2. Sensors <br>
A list of sensors the software uses includes: <br>
1. IMU (Gyroscope & Accelerometer) for sensing the orientation of the plane.
2. Distance Sensor (Ultrasonic or ToF) mounted underneath the plane to detect altitude.
3. Magnetometer for detecting heading.

A3. Flight Plans <br>
Binary Input  |  Mode Name                 |  Description                                                          | <br>
0000          |  MODE_RESET_SENSORS        |  Resets sensors after ensuring plane is stationary                    | <br>
0001          |  MODE_STOPPED              |  Parks the plane and checks if it's stationary                        | <br>
0010          |  MODE_TAXI                 |  Taxi the plane on the runway                                         | <br>
0011          |  MODE_TAKEOFF_LEVEL_LAND   |  Takeoff --> Level flight for set duration --> Landing                | <br>
0100          |  MODE_HOLDING_PATTERN      |  Takeoff --> Holding pattern (circular) for duration --> Landing      | <br>

Additional modes can be added by expanding the Mode enum and updating the switch logic in loop().

A4. Logging <br>
Flight logs are saved to flight_log.txt on the SD card and display information on events with timestamps. Examples of the display include: <br>
Time: 00 MIN : 05 SEC : 024 MSEC - Entered MODE_TAXI
Time: 00 MIN : 10 SEC : 011 MSEC - Plane has completed taxi and stopped.

A5. Errors/Unexpected Behaviour <br>
If an error or an unexpected behaviour occurs, the system will record the time and error details in the flight log and trigger the error LED.





