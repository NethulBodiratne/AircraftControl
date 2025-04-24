''' 
Airplane Control System - Pitch and Roll Monitoring

This program reads gyroscopic data from the Raspberry Pi Sense HAT to track the pitch and roll of a model airplane in real time. 
It calculates the change in pitch and roll based on gyroscope readings and integrates these values over time to estimate the current orientation. 

Based on the pitch and roll values, it provides real-time visual feedback using the Sense HAT's LED matrix. 
Colors on the display indicate whether the orientation deviates beyond a set tolerance:
- Green/Red: Forward/Backward pitch deviation
- Orange/Purple: Left/Right roll deviation
- Black: Within acceptable limits

The brightness of the senseHat LEDs scales with the magnitude of deviation. Small movements under a tolerance threshold are ignored.
Lights up GPIO LEDs when control surfaces (aileron and elevator) would be actuated based on the current orientation.

The program prints out the real-time change and current value of pitch and roll for debugging and monitoring purposes.

Author: Nethul Bodiratne
Date: April 23, 2025

Problems: Does not accurately display angles in terminal
'''

import time
import math
from sense_hat import SenseHat
from threading import Thread
import RPi.GPIO as GPIO

# -----------------------
# CONFIGURATION
# -----------------------
TOLERANCE = 3.0                  # degrees
MAX_DEVIATION = TOLERANCE * 3.0  # max brightness when deviation > 3x tolerance

# Initialize the SenseHat
sense = SenseHat()

# Initialize variables for tracking previous values
last_pitch = 0.0
last_roll = 0.0

# Define base colors
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (255, 0, 0)
COLOR_ORANGE = (255, 140, 0)
COLOR_PURPLE = (255, 0, 255)
COLOR_BLACK = (0, 0, 0)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
LEFT_AILERON_LED = 5
RIGHT_AILERON_LED = 26
ELEVATOR_LED = 22

GPIO.setup(LEFT_AILERON_LED, GPIO.OUT)
GPIO.setup(RIGHT_AILERON_LED, GPIO.OUT)
GPIO.setup(ELEVATOR_LED, GPIO.OUT)

# Set up PWM with 100Hz frequency
left_ail_led_pwm = GPIO.PWM(LEFT_AILERON_LED, 100)
right_ail_led_pwm = GPIO.PWM(RIGHT_AILERON_LED, 100)
elevator_led_pwm = GPIO.PWM(ELEVATOR_LED, 100)

# Start PWM with 0% duty cycle (off)
left_ail_led_pwm.start(0)
right_ail_led_pwm.start(0)
elevator_led_pwm.start(0)

# Counter for entry number
count = 0

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
        pixels[0][x] = pitch_color
        pixels[7][x] = pitch_color

    # Odd indexed LEDs (columns 0 and 7) for roll
    for y in range(8):
        pixels[y][0] = roll_color
        pixels[y][7] = roll_color

    # Flatten and send to Sense HAT
    sense.set_pixels([pixel for row in pixels for pixel in row])

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

# -----------------------
# LED SCALING FUNCTION (CLAMPED TO 0-100%)
# -----------------------
def angle_to_duty_cycle(angle, max_angle=45):
    dc = min(max(abs(angle) / max_angle * 100, 0), 100)
    return dc

# Initial calibration
recalibrate()
# Continuously read the sensor and track pitch and roll
try:
    while True:
        # Get accelerometer data from Sense HAT
        accel_data = sense.get_accelerometer_raw()
        
        # Get the current pitch and roll values
        pitch, roll = get_pitch_roll(accel_data)
        
        # Calculate the change in pitch and roll
        pitch_change = pitch - last_pitch
        roll_change = roll - last_roll
        
        # Normalize pitch and roll to [-180, 180]
        pitch = (pitch + 360) % 360
        if pitch > 180: pitch -= 360
        roll = (roll + 360) % 360
        if roll > 180: roll -= 360
        
        # Print the current pitch, roll, and their changes
        print(count, f"Pitch: {pitch:.2f}° (Change: {pitch_change:.2f}°), Roll: {roll:.2f}° (Change: {roll_change:.2f}°)")
        
        # Determine pitch color and ELEVATOR LED brightness
        if pitch < -TOLERANCE:
            pitch_color = scale_color(COLOR_RED, abs(pitch))
            duty = angle_to_duty_cycle(pitch)
            elevator_led_pwm.ChangeDutyCycle(duty)  # Nose down = RED
        elif pitch > TOLERANCE:
            pitch_color = scale_color(COLOR_GREEN, abs(pitch))
            duty = angle_to_duty_cycle(pitch)
            elevator_led_pwm.ChangeDutyCycle(duty)  # Nose up = GREEN
        else:
            pitch_color = COLOR_BLACK
            elevator_led_pwm.ChangeDutyCycle(0)

        # Determine roll color and AILERON LED brightness
        if roll < -TOLERANCE:
            roll_color = scale_color(COLOR_ORANGE, abs(roll))
            duty = angle_to_duty_cycle(roll)
            left_ail_led_pwm.ChangeDutyCycle(duty)
            right_ail_led_pwm.ChangeDutyCycle(0)
        elif roll > TOLERANCE:
            roll_color = scale_color(COLOR_PURPLE, abs(roll))
            duty = angle_to_duty_cycle(roll)
            right_ail_led_pwm.ChangeDutyCycle(duty)
            left_ail_led_pwm.ChangeDutyCycle(0)
        else:
            roll_color = COLOR_BLACK
            left_ail_led_pwm.ChangeDutyCycle(0)
            right_ail_led_pwm.ChangeDutyCycle(0)

        # Display pitch and roll status visually
        display_pitch_roll(pitch_color, roll_color)
        
        # Update the last known pitch and roll values
        last_pitch = pitch
        last_roll = roll
        
        # Increment count
        count += 1
        
        # Wait before reading again
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program terminated by user.")
