#!/usr/bin/env python3

# Import necessary libraries
import time  # Used for pausing execution (sleep) and timing intervals (dt)
import math  # Used for mathematical functions (though not directly in this version)
from sense_hat import SenseHat # Library to interact with the Sense HAT sensor/LED matrix
import RPi.GPIO as GPIO # Library to control Raspberry Pi GPIO pins

# --- Configuration ---

# Control Loop Settings
LOOP_DELAY_SECONDS = 0.1  # Defines the pause duration in the main loop, effectively setting the loop frequency (1 / 0.1 = 10 Hz)

# Target Angles (degrees) - Relative to the initial orientation captured at startup
TARGET_PITCH = 0.0  # Target pitch angle relative to the starting pitch
TARGET_ROLL = 0.0   # Target roll angle relative to the starting roll
# Yaw target is ignored in this version

# --- NEW: Orientation Tolerance (Deadzone) ---
# The PID controller will only actively correct if the orientation deviates
# from the initial target by MORE than this amount (in degrees).
PITCH_TOLERANCE = 3.0 # Ignore pitch deviations smaller than +/- 1.5 degrees
ROLL_TOLERANCE = 3.0  # Ignore roll deviations smaller than +/- 1.5 degrees

# PID Controller Gains
# These constants determine how strongly the controller reacts to errors.
# --- TUNING IS CRITICAL for real-world performance ---
# Kp: Proportional gain - Reacts to the current error. Higher value = stronger reaction.
# Ki: Integral gain - Accumulates past errors to correct steady-state offsets. Set to 0 initially.
# Kd: Derivative gain - Reacts to the rate of change of the error (predicts future error). Dampens oscillations. Set to 0 initially.
PITCH_KP = 0.9  # Proportional gain for the pitch control loop
PITCH_KI = 0.0  # Integral gain for the pitch control loop
PITCH_KD = 0.0  # Derivative gain for the pitch control loop

ROLL_KP = 0.9   # Proportional gain for the roll control loop
ROLL_KI = 0.0   # Integral gain for the roll control loop
ROLL_KD = 0.0   # Derivative gain for the roll control loop

# GPIO Pin numbers (using BCM numbering scheme) for PWM output to LEDs (User specified)
ELEVATOR_LED_GPIO = 22 # GPIO pin for the LED simulating the elevator servo output (controlled by pitch)
RIGHT_AILERON_LED_GPIO = 26  # GPIO pin for the LED simulating the right aileron servo output (controlled by roll)
LEFT_AILERON_LED_GPIO = 5    # GPIO pin for the LED simulating the left aileron servo output (controlled by roll)
PWM_FREQUENCY = 100 # PWM frequency in Hz for controlling LED brightness (100 Hz is suitable for LEDs)

# Sense HAT LED Matrix Colors
BG_COLOR = [0, 0, 0]         # RGB color for the background (off LEDs) - Black
ORIENTATION_COLOR = [0, 0, 255] # RGB color for the orientation indicator LEDs - Blue

# --- Global Variables ---

# Initialize the Sense HAT object
sense = SenseHat()
# Set the Sense HAT LED matrix to low light mode to reduce brightness (User commented out)
#sense.low_light = True

# Variables to store the initial orientation offset
initial_pitch = 0.0
initial_roll = 0.0

# PID state variables - These store values between loop iterations
pitch_integral = 0.0        # Accumulator for the integral term of the pitch PID controller
pitch_previous_error = 0.0  # Stores the pitch error from the previous loop iteration for the derivative calculation
roll_integral = 0.0         # Accumulator for the integral term of the roll PID controller
roll_previous_error = 0.0   # Stores the roll error from the previous loop iteration for the derivative calculation

# --- NEW: Variables to store previous orientation for delta calculation ---
previous_raw_pitch = 0.0
previous_raw_roll = 0.0

# PWM object placeholders - these will be assigned in setup_gpio()
elevator_pwm = None
right_aileron_pwm = None
left_aileron_pwm = None

# --- Functions ---

def setup_gpio():
    """Sets up GPIO pins for PWM output to the three LEDs."""
    # Use global variables to store the PWM objects so they can be accessed elsewhere
    global elevator_pwm, right_aileron_pwm, left_aileron_pwm

    # Set the GPIO numbering mode to BCM (Broadcom SOC channel)
    GPIO.setmode(GPIO.BCM)
    # Disable GPIO warnings (e.g., about channels already being in use)
    GPIO.setwarnings(False)

    # Set up the GPIO pins connected to the LEDs as outputs
    GPIO.setup(ELEVATOR_LED_GPIO, GPIO.OUT)       # Elevator LED pin
    GPIO.setup(RIGHT_AILERON_LED_GPIO, GPIO.OUT)  # Right Aileron LED pin
    GPIO.setup(LEFT_AILERON_LED_GPIO, GPIO.OUT)   # Left Aileron LED pin

    # Create PWM instances for each LED pin with the defined frequency
    elevator_pwm = GPIO.PWM(ELEVATOR_LED_GPIO, PWM_FREQUENCY)
    right_aileron_pwm = GPIO.PWM(RIGHT_AILERON_LED_GPIO, PWM_FREQUENCY)
    left_aileron_pwm = GPIO.PWM(LEFT_AILERON_LED_GPIO, PWM_FREQUENCY)

    # Start PWM with 0% duty cycle (LEDs initially off)
    elevator_pwm.start(0)
    right_aileron_pwm.start(0)
    left_aileron_pwm.start(0)

    # Print confirmation message
    print("GPIO setup complete for 3 LEDs.")
    # Return the PWM objects (though they are global, returning can be good practice)
    return elevator_pwm, right_aileron_pwm, left_aileron_pwm

def cleanup_gpio():
    """Stops PWM and cleans up GPIO resources before exiting."""
    # Use global variables to access the PWM objects
    global elevator_pwm, right_aileron_pwm, left_aileron_pwm

    # Check if the PWM objects exist and stop them if they do
    if elevator_pwm:
        elevator_pwm.stop() # Stop the elevator PWM signal
    if right_aileron_pwm:
        right_aileron_pwm.stop() # Stop the right aileron PWM signal
    if left_aileron_pwm:
        left_aileron_pwm.stop() # Stop the left aileron PWM signal

    # Release all GPIO resources used by this script
    GPIO.cleanup()
    # Print confirmation message
    print("GPIO cleanup complete.")

def get_orientation():
    """Gets raw pitch and roll angles from the Sense HAT IMU."""
    # Retrieve orientation data from the Sense HAT sensors in degrees
    orientation = sense.get_orientation_degrees()

    # Extract the raw pitch value
    raw_pitch = orientation['pitch']
    # The Sense HAT raw pitch often ranges from 0 to 360.
    # Adjust it to a more conventional -180 to +180 range.
    # If angle is > 180, subtract 360 to wrap around (e.g., 350 becomes -10).
    if raw_pitch > 180:
        pitch = raw_pitch - 360
    else:
        pitch = raw_pitch

    # Extract the raw roll value
    raw_roll = orientation['roll']
    # Adjust the raw roll (0 to 360) to a -180 to +180 range.
    if raw_roll > 180:
        roll = raw_roll - 360
    else:
        roll = raw_roll

    # Clamp pitch to avoid extreme values near +/- 90 degrees (potential gimbal lock issues)
    pitch = max(-89.9, min(89.9, pitch))
    # Clamp roll to prevent wrap-around issues if it hits exactly +/- 180
    roll = max(-179.9, min(179.9, roll))

    # Return the adjusted pitch and roll values
    return pitch, roll

def calculate_pid(error, Kp, Ki, Kd, integral, previous_error, dt):
    """Calculates the PID control output based on the error and gains."""
    # Proportional term: Current error * proportional gain
    P = Kp * error

    # Integral term: Accumulate the error over time * integral gain
    # This helps eliminate steady-state errors where the proportional term alone isn't enough.
    integral += error * dt # Add the current error scaled by the time step (dt) to the integral sum
    I = Ki * integral
    # Note: In a real system, 'Integral Windup' can occur if the integral term grows too large.
    # Clamping the 'integral' value to a certain range might be necessary.
    # Note: We are now resetting the integral term when within the tolerance zone in the main loop.

    # Derivative term: Rate of change of error * derivative gain
    # This anticipates future error based on the current trend. Helps dampen oscillations.
    # Avoid division by zero if dt is too small or zero
    if dt > 0:
        derivative = (error - previous_error) / dt # Calculate the change in error divided by the time step
    else:
        derivative = 0 # Assign 0 if dt is not positive
    D = Kd * derivative

    # Total PID output: Sum of the P, I, and D terms
    output = P + I + D

    # Update state for the next iteration: current error becomes the previous error
    previous_error = error

    # Clamp the output to a practical range (e.g., -100% to +100%)
    # This simulates the limits of a servo or actuator.
    output = max(-100.0, min(100.0, output))

    # Return the calculated output and the updated integral and previous_error values
    return output, integral, previous_error


def update_external_leds(pitch_output, roll_output, elevator_pwm, right_aileron_pwm, left_aileron_pwm):
    """Updates brightness of the three external LEDs based on control outputs."""

    # --- Elevator LED Control (Pitch) ---
    # The brightness is proportional to the *magnitude* of the pitch correction needed.
    # Map PID output (-100 to 100) to PWM duty cycle (0 to 100).
    # abs() takes the absolute value, so positive and negative errors cause brightness.
    # If pitch_output is 0 (because we are within tolerance), duty cycle will be 0.
    elevator_duty_cycle = min(100, max(0, abs(pitch_output)))
    # Update the PWM duty cycle for the elevator LED
    elevator_pwm.ChangeDutyCycle(elevator_duty_cycle)

    # --- Aileron LED Control (Roll) ---
    # Ailerons work differentially.
    # Positive roll_output means "roll right" command -> Right aileron up (bright), Left aileron down (dim/off).
    # Negative roll_output means "roll left" command -> Left aileron up (bright), Right aileron down (dim/off).
    # If roll_output is 0 (within tolerance), both duty cycles will be 0.

    # Right Aileron: Brightness proportional to positive roll output (command to roll right).
    # max(0, roll_output) ensures only positive values (or 0) are used.
    right_aileron_duty_cycle = min(100, max(0, roll_output))
    # Update the PWM duty cycle for the right aileron LED
    right_aileron_pwm.ChangeDutyCycle(right_aileron_duty_cycle)

    # Left Aileron: Brightness proportional to negative roll output (command to roll left).
    # max(0, -roll_output) uses the negative of the output, so negative outputs become positive brightness commands.
    left_aileron_duty_cycle = min(100, max(0, -roll_output))
    # Update the PWM duty cycle for the left aileron LED
    left_aileron_pwm.ChangeDutyCycle(left_aileron_duty_cycle)


def update_sense_hat_matrix(relative_pitch, relative_roll):
    """Displays relative pitch and roll orientation graphically on the Sense HAT LED matrix.
        NOTE: Axes have been swapped to correct the 90-degree offset reported by the user.
        Pitch changes will now affect horizontal LED movement.
        Roll changes will now affect vertical LED movement.
    """
    # Clear the LED matrix before drawing the new orientation
    sense.clear(BG_COLOR) # Use background color defined earlier

    # Define the maximum angle (in degrees) that will correspond to the edge of the display
    max_display_angle = 45.0

    # --- Pitch Visualization (NOW Horizontal Axis due to 90-degree correction) ---
    # Normalize the *relative* pitch angle to a range of -1.0 to +1.0 based on max_display_angle
    # Always display the actual relative pitch, even if it's within tolerance
    pitch_normalized = max(-1.0, min(1.0, relative_pitch / max_display_angle))

    # If pitching up (positive normalized value, beyond a small threshold) -> Light up LEDs to the RIGHT
    if pitch_normalized > 0.1:
        num_leds = int(round(pitch_normalized * 3)) # Calculate number of columns (0-3)
        # Light LEDs from column 4 rightwards
        for x in range(4, 4 + num_leds + 1): # Iterate from column 4 rightwards
            if 0 <= x <= 7: # Check bounds
                for y in range(3, 5): # Middle two rows
                    sense.set_pixel(x, y, ORIENTATION_COLOR) # Set pixel
    # If pitching down (negative normalized value, beyond a small threshold) -> Light up LEDs to the LEFT
    elif pitch_normalized < -0.1:
        num_leds = int(round(abs(pitch_normalized) * 3)) # Calculate number of columns (0-3)
        # Light LEDs from column 3 leftwards
        for x in range(3 - num_leds, 4): # Iterate from column (3-num_leds) leftwards
            if 0 <= x <= 7: # Check bounds
                for y in range(3, 5): # Middle two rows
                    sense.set_pixel(x, y, ORIENTATION_COLOR) # Set pixel

    # --- Roll Visualization (NOW Vertical Axis due to 90-degree correction) ---
    # Normalize the *relative* roll angle to a range of -1.0 to +1.0 based on max_display_angle
    # Always display the actual relative roll, even if it's within tolerance
    roll_normalized = max(-1.0, min(1.0, relative_roll / max_display_angle))

    # If rolling right (positive normalized value, beyond a small threshold) -> Light up LEDs DOWNWARDS
    if roll_normalized > 0.1:
        num_leds = int(round(roll_normalized * 3)) # Calculate number of rows (0-3)
        # Light LEDs from row 4 downwards
        for y in range(4, 4 + num_leds + 1): # Iterate from row 4 downwards
            if 0 <= y <= 7: # Check bounds
                for x in range(3, 5): # Middle two columns
                    sense.set_pixel(x, y, ORIENTATION_COLOR) # Set pixel
    # If rolling left (negative normalized value, beyond a small threshold) -> Light up LEDs UPWARDS
    elif roll_normalized < -0.1:
        num_leds = int(round(abs(roll_normalized) * 3)) # Calculate number of rows (0-3)
        # Light LEDs from row 3 upwards
        for y in range(3 - num_leds, 4): # Iterate from row (3-num_leds) upwards
            if 0 <= y <= 7: # Check bounds
                for x in range(3, 5): # Middle two columns
                    sense.set_pixel(x, y, ORIENTATION_COLOR) # Set pixel

    # Optional: Light up the center 4 pixels if the orientation is close to the initial (relative zero)
    # This condition remains the same, indicating closeness to target visually
    if abs(pitch_normalized) < 0.1 and abs(roll_normalized) < 0.1:
        sense.set_pixel(3, 3, ORIENTATION_COLOR) # Top-left center pixel
        sense.set_pixel(4, 3, ORIENTATION_COLOR) # Top-right center pixel
        sense.set_pixel(3, 4, ORIENTATION_COLOR) # Bottom-left center pixel
        sense.set_pixel(4, 4, ORIENTATION_COLOR) # Bottom-right center pixel


# --- Main Execution Block ---
if __name__ == "__main__":
    # This block runs only when the script is executed directly (not imported as a module)
    try:
        # Use global to modify the initial orientation and PID state variables if needed inside loop
        #global initial_pitch, initial_roll
        #global pitch_integral, pitch_previous_error, roll_integral, roll_previous_error
        # Use global for previous raw values needed for delta calculation
        #global previous_raw_pitch, previous_raw_roll

        # Initialize GPIO and get PWM objects
        elevator_pwm, right_aileron_pwm, left_aileron_pwm = setup_gpio()

        # --- Capture Initial Orientation ---
        print("Capturing initial orientation... Keep the Pi steady.")
        # Add a small delay to allow sensors to stabilize if needed
        time.sleep(1.0)
        initial_pitch, initial_roll = get_orientation()
        print(f"Initial orientation captured: Pitch={initial_pitch:.2f}, Roll={initial_roll:.2f}")
        print(f"This will be treated as the zero reference. Tolerance: Pitch=+/-{PITCH_TOLERANCE:.2f}, Roll=+/-{ROLL_TOLERANCE:.2f}")
        # --- End Initial Orientation Capture ---

        # --- Initialize Previous Raw Values for Delta Calculation ---
        # Set the first 'previous' values to the initial reading
        previous_raw_pitch = initial_pitch
        previous_raw_roll = initial_roll
        # --- End Initialization ---

        # Record the starting time for the first dt calculation
        last_time = time.time()

        # Print a message indicating the loop is starting
        print("Starting control loop... Press Ctrl+C to exit.")
        print("-" * 60) # Separator line

        # Start the main infinite loop
        while True:
            # --- Time Calculation ---
            # Get the current time
            current_time = time.time()
            # Calculate the time elapsed since the last loop iteration (delta time)
            dt = current_time - last_time
            # If dt is 0 or negative (e.g., system clock changed), skip iteration to avoid issues
            if dt <= 0:
                # Maybe sleep a tiny bit to ensure time progresses
                time.sleep(0.01)
                # Re-read time to recalculate dt for the PID, or just continue
                current_time = time.time()
                dt = current_time - last_time
                if dt <= 0: # If still zero, skip this cycle
                    last_time = current_time # Update time anyway
                    continue
            # Update last_time for the next iteration's dt calculation
            last_time = current_time

            # --- Control Cycle ---
            # 1. Read Sensors: Get current raw pitch and roll from the Sense HAT
            current_raw_pitch, current_raw_roll = get_orientation()

            # --- NEW: Calculate Change Since Last Reading ---
            delta_pitch = current_raw_pitch - previous_raw_pitch
            delta_roll = current_raw_roll - previous_raw_roll

            # --- NEW: Print Current Orientation and Delta ---
            print(f"Pitch: {current_raw_pitch: 1.2f} (Delta: {delta_pitch: 1.2f}) | Roll: {current_raw_roll: 1.2f} (Delta: {delta_roll: 1.2f})", end="") # Print without newline initially

            # 2. Calculate Relative Orientation: Subtract initial offset
            current_relative_pitch = current_raw_pitch - initial_pitch
            current_relative_roll = current_raw_roll - initial_roll

            # 3. Calculate Errors: Find the difference between target and *relative* current angles
            # Target is 0 relative to the start position
            pitch_error = TARGET_PITCH - current_relative_pitch
            roll_error = TARGET_ROLL - current_relative_roll

            # --- NEW: Apply Tolerance (Deadzone) ---
            # If the relative orientation is within the tolerance band, treat the error as zero
            # and reset the integral and previous error to prevent PID windup/lingering response.
            pitch_active = True
            if abs(current_relative_pitch) < PITCH_TOLERANCE:
                pitch_error = 0.0
                pitch_integral = 0.0 # Reset integral when within tolerance
                pitch_previous_error = 0.0 # Reset previous error (affects D term on next cycle if error becomes non-zero)
                pitch_active = False

            roll_active = True
            if abs(current_relative_roll) < ROLL_TOLERANCE:
                roll_error = 0.0
                roll_integral = 0.0 # Reset integral when within tolerance
                roll_previous_error = 0.0 # Reset previous error
                roll_active = False

            # 4. Calculate PID Control Outputs: Determine the necessary correction effort
            # Pass current error (potentially zeroed by tolerance), gains, PID state variables, and dt
            pitch_output, pitch_integral, pitch_previous_error = calculate_pid(
                pitch_error, PITCH_KP, PITCH_KI, PITCH_KD,
                pitch_integral, pitch_previous_error, dt
            )
            roll_output, roll_integral, roll_previous_error = calculate_pid(
                roll_error, ROLL_KP, ROLL_KI, ROLL_KD,
                roll_integral, roll_previous_error, dt
            )

            # Append active status to the print line
            print(f" | P_Active: {pitch_active} | R_Active: {roll_active}") # Now print newline

            # 5. Update External LEDs (Simulated Actuators)
            # Pass the calculated outputs (which will be 0 if error was zeroed) and PWM objects
            update_external_leds(pitch_output, roll_output, elevator_pwm, right_aileron_pwm, left_aileron_pwm)

            # 6. Update Sense HAT Display: Show *relative* orientation visually
            # This still shows the actual relative position, even inside the tolerance zone
            update_sense_hat_matrix(current_relative_pitch, current_relative_roll)

            # --- NEW: Update Previous Raw Values for Next Iteration ---
            previous_raw_pitch = current_raw_pitch
            previous_raw_roll = current_raw_roll

            # Optional: Print values for debugging/monitoring (showing relative values now)
            # print(f"Rel Pitch: {current_relative_pitch:.1f} (Err:{pitch_error:.1f}, Out:{pitch_output:.1f}) | Rel Roll: {current_relative_roll:.1f} (Err:{roll_error:.1f}, Out:{roll_output:.1f})")

            # Wait for the defined delay before starting the next loop iteration
            time.sleep(LOOP_DELAY_SECONDS)

    # Handle graceful exit on keyboard interrupt (Ctrl+C)
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting gracefully.")
    # Handle any other unexpected errors
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        import traceback
        traceback.print_exc() # Print detailed traceback for debugging
    # Finally block: This code runs whether the try block completed successfully or an exception occurred
    finally:
        # Clean up resources before the script fully exits
        sense.clear()      # Clear the Sense HAT LED matrix
        cleanup_gpio()     # Stop PWM and release GPIO pins
        print("Program terminated.")

'''
**Hardware Setup Change:** (No change from your previous version)

* **Elevator LED:** Connect to **GPIO 22** (Pin 15) via a resistor to GND.
* **Right Aileron LED:** Connect to **GPIO 26** (Pin 37) via a resistor to GND.
* **Left Aileron LED:** Connect to **GPIO 5** (Pin 29) via a resistor to GND.

**Behavior Changes:**

* **Zero Reference:** Same as before - initial orientation is the target.
* **Tolerance/Deadzone:** The PID controller will *not* attempt to correct orientation (pitch/roll outputs will be 0) if the deviation from the initial orientation is less than the `PITCH_TOLERANCE` or `ROLL_TOLERANCE` values respectively. When inside the tolerance zone, the corresponding external LED(s) will be off, and the PID integral term is reset.
* **Console Output:** Each loop iteration now prints:
    * The current **raw** pitch and roll readings.
    * The **change** (delta) in raw pitch and roll since the previous loop iteration.
    * Whether the Pitch or Roll PID controllers are **active** (i.e., outside their tolerance zone).
* **Sense HAT Matrix:** Still displays the *actual* relative orientation graphically, even if it's within the tolerance zone (the graphical indicator might show a slight offset, but the corrective outputs/LEDs are inactive).
* **External LEDs:** Will only light up significantly when the orientation deviation *exceeds* the defined tolerance for that axis.
'''
