''' 
Airplane Control System - Pitch and Roll Monitoring

This program reads gyroscopic data from the Raspberry Pi Sense HAT to track the pitch and roll of a model airplane in real time. 
It calculates the change in pitch and roll based on gyroscope readings and integrates these values over time to estimate the current orientation. 

Based on the pitch and roll values, it provides real-time visual feedback using the Sense HAT's LED matrix. 
Colors on the display indicate whether the orientation deviates beyond a set tolerance:
- Green/Red: Forward/Backward pitch deviation
- Orange/Purple: Left/Right roll deviation
- Black: Within acceptable limits

The brightness of the colors scales with the magnitude of deviation. Small movements under a dead zone threshold are ignored.

The program prints out the real-time change and current value of pitch and roll for debugging and monitoring purposes.

Author: Nethul Bodiratne
Date: April 23, 2025

Problems: Does not accurately display angle in terminal
'''

from sense_hat import SenseHat
import time
import math
from threading import Thread

sense = SenseHat()

# -----------------------
# CONFIGURATION
# -----------------------
TOLERANCE = 3.0                  # degrees
MAX_DEVIATION = TOLERANCE * 2.0  # max brightness when deviation > 2x tolerance

# Configure IMU to reduce drift by disabling gyro
sense.set_imu_config(compass_enabled=True, gyro_enabled=False, accel_enabled=True)

# Target pitch and roll will be relative to calibration values
pitch_offset = 0.0
roll_offset = 0.0
yaw_offset = 0.0

# Define base colors
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_ORANGE = (255, 140, 0)
COLOR_PURPLE = (255, 0, 255)
COLOR_BLACK = (0, 0, 0)

# Counter for entry number
count = 0

# Time tracking for auto-recalibration
last_recalibration = time.time()
drift_counter = 0

# -----------------------
# COLOR SCALING
# -----------------------
def scale_color(color, deviation):
    intensity = min(max(deviation / MAX_DEVIATION, 0.1), 1.0)
    return tuple(int(c * intensity) for c in color)

# -----------------------
# LED DISPLAY FUNCTION
# -----------------------
def display_pitch_roll(pitch_color, roll_color):
    pixels = [[COLOR_BLACK for _ in range(8)] for _ in range(8)]

    # Even indexed LEDs (rows 0 and 7) for pitch
    for x in range(8):
        if x % 2 == 0:
            pixels[0][x] = pitch_color
            pixels[7][x] = pitch_color

    # Odd indexed LEDs (columns 0 and 7) for roll
    for y in range(8):
        if y % 2 == 1:
            pixels[y][0] = roll_color
            pixels[y][7] = roll_color

    # Flatten and send to Sense HAT
    sense.set_pixels([pixel for row in pixels for pixel in row])

# -----------------------
# RECALIBRATION HANDLER
# -----------------------
def recalibrate():
    global pitch_offset, roll_offset, last_recalibration
    orientation = sense.get_orientation()
    pitch_offset = orientation['pitch']
    roll_offset = orientation['roll']
    yaw_offset = orientation['yaw']
    pitch = 0
    roll = 0
    yaw = 0
    last_recalibration = time.time()
    print("\n[CALIBRATION] Orientation reset!")
    print(f"New zero => Pitch: {pitch_offset:.2f}, Roll: {roll_offset:.2f}, Yaw: {yaw:.2f}\n")

# -----------------------
# BUTTON PRESS THREAD
# -----------------------
def watch_joystick():
    while True:
        for event in sense.stick.get_events():
            if event.action == "pressed":
                recalibrate()
        time.sleep(0.1)

# Start joystick thread
Thread(target=watch_joystick, daemon=True).start()

# Initial calibration
recalibrate()

# Function to get pitch and roll from the accelerometer data
def get_pitch_roll(accel_data):
    # Accelerometer data: ax, ay, az
    ax = accel_data['x']
    ay = accel_data['y']
    az = accel_data['z']
    
    # Calculate pitch and roll from accelerometer data
    pitch = math.atan2(ay, az) * 180 / math.pi  # Pitch (degrees)
    roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi  # Roll (degrees)
    
    return pitch, roll

# Initialize variables for tracking previous values
last_pitch = 0.0
last_roll = 0.0

# -----------------------
# MAIN LOOP
# -----------------------
while True:
    # Increment count
    count += 1
    
    # Read orientation and apply calibration offset
    orientation = sense.get_orientation()
    
    pitch = orientation['pitch'] - pitch_offset
    roll = orientation['roll'] - roll_offset
    yaw = orientation['yaw'] - yaw_offset
    
    # Normalize pitch and roll to [-180, 180]
    pitch = (pitch + 360) % 360
    if pitch > 180: pitch -= 360
    roll = (roll + 360) % 360
    if roll > 180: roll -= 360
    yaw = (yaw + 360) % 360
    if yaw > 180: yaw -= 360
    
    print(count, f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°")
    
    # Get accelerometer data from Sense HAT
    accel_data = sense.get_accelerometer_raw()
    
    # Get the current pitch and roll values
    pitch, roll = get_pitch_roll(accel_data)
    
    # Calculate the change in pitch and roll
    pitch_change = pitch - last_pitch
    roll_change = roll - last_roll
    
    # Determine pitch color
    if pitch < -TOLERANCE:
        deviation = abs(pitch)
        pitch_color = scale_color(COLOR_RED, deviation)
    elif pitch > TOLERANCE:
        deviation = abs(pitch)
        pitch_color = scale_color(COLOR_GREEN, deviation)
    else:
        pitch_color = COLOR_BLACK

    # Determine roll color
    if roll < -TOLERANCE:
        deviation = abs(roll)
        roll_color = scale_color(COLOR_ORANGE, deviation)
    elif roll > TOLERANCE:
        deviation = abs(roll)
        roll_color = scale_color(COLOR_PURPLE, deviation)
    else:
        roll_color = COLOR_BLACK

    # Display pitch and roll status visually
    display_pitch_roll(pitch_color, roll_color)

    # Update the last known pitch and roll values
    last_pitch = pitch
    last_roll = roll
    
    # Short delay
    #time.sleep(0.5)
