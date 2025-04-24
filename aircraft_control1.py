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

Problems: Does not accurately display angle in terminal or display orientation lights
'''

from sense_hat import SenseHat
import time
from threading import Thread

sense = SenseHat()

# -----------------------
# CONFIGURATION
# -----------------------
TOLERANCE = 1.0                  # degrees
MAX_DEVIATION = TOLERANCE * 2.0  # max brightness when deviation > 2x tolerance
ALPHA = 0.3                      # Low-pass filter coefficient
DEAD_ZONE = 0.3                  # Ignore very small movements
GYRO_SCALE = 1.0                 # Scale factor for gyroscope values if needed

# Define base colors
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_ORANGE = (255, 140, 0)
COLOR_PURPLE = (255, 0, 255)
COLOR_BLACK = (0, 0, 0)

# Initialize pitch and roll
pitch = 0.0
roll = 0.0

# Time tracking
prev_time = time.time()

# -----------------------
# COLOR SCALING FUNCTION
# -----------------------
def scale_color(color, deviation):
    # Adjusts color intensity based on deviation
    intensity = min(max(deviation / MAX_DEVIATION, 0.1), 1.0)
    return tuple(int(c * intensity) for c in color)

# -----------------------
# LED DISPLAY FUNCTION
# -----------------------
def display_pitch_roll(pitch_color, roll_color):
    # Creates an 8x8 grid with pitch and roll colors
    pixels = [[COLOR_BLACK for _ in range(8)] for _ in range(8)]

    # Display pitch on top and bottom rows
    for x in range(8):
        if x % 2 == 0:
            pixels[0][x] = pitch_color
            pixels[7][x] = pitch_color

    # Display roll on left and right columns
    for y in range(8):
        if y % 2 == 1:
            pixels[y][0] = roll_color
            pixels[y][7] = roll_color

    # Flatten the 2D array and display it
    sense.set_pixels([pixel for row in pixels for pixel in row])

# -----------------------
# MAIN LOOP
# -----------------------
while True:
    # Record current time and calculate time difference
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # Read raw gyroscope values (degrees/sec)
    gyro = sense.get_gyroscope_raw()
    gyro_pitch_rate = gyro['y'] * GYRO_SCALE  # Rotation around Y-axis affects pitch
    gyro_roll_rate = gyro['x'] * GYRO_SCALE   # Rotation around X-axis affects roll

    # Integrate to get new pitch and roll
    delta_pitch = gyro_pitch_rate * dt
    delta_roll = gyro_roll_rate * dt
    pitch += delta_pitch
    roll += delta_roll

    # Clamp small movements to zero (dead zone)
    if abs(pitch) < DEAD_ZONE:
        pitch = 0.0
    if abs(roll) < DEAD_ZONE:
        roll = 0.0

    # Print current pitch, roll, and deltas
    print(f"ΔPitch: {delta_pitch:.2f}°, ΔRoll: {delta_roll:.2f}°  |  Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")

    # Determine pitch color
    if pitch < -TOLERANCE:
        pitch_color = scale_color(COLOR_RED, abs(pitch))
    elif pitch > TOLERANCE:
        pitch_color = scale_color(COLOR_GREEN, abs(pitch))
    else:
        pitch_color = COLOR_BLACK

    # Determine roll color
    if roll < -TOLERANCE:
        roll_color = scale_color(COLOR_ORANGE, abs(roll))
    elif roll > TOLERANCE:
        roll_color = scale_color(COLOR_PURPLE, abs(roll))
    else:
        roll_color = COLOR_BLACK

    # Display pitch and roll status
    display_pitch_roll(pitch_color, roll_color)

    # Small delay to stabilize output
    time.sleep(0.5)
