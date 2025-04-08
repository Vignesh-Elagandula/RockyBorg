import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor control
LEFT_IN1 = 17      # GPIO pin to control direction of left motor
LEFT_IN2 = 27
RIGHT_IN1 = 22     # GPIO pin to control direction of right motor
RIGHT_IN2 = 23
LEFT_PWM = 18      # PWM pin for speed control of left motor
RIGHT_PWM = 13     # PWM pin for speed control of right motor

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set all pins as output
motor_pins = [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, LEFT_PWM, RIGHT_PWM]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Setup PWM frequency at 100 Hz
left_pwm = GPIO.PWM(LEFT_PWM, 100)
right_pwm = GPIO.PWM(RIGHT_PWM, 100)

# Start PWM with 0% duty cycle (motors off)
left_pwm.start(0)
right_pwm.start(0)

# Function to set motor speed
def set_speed(left_speed, right_speed):
    left_pwm.ChangeDutyCycle(left_speed)
    right_pwm.ChangeDutyCycle(right_speed)

# Function to stop the motors
def stop():
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    set_speed(0, 0)

# Function to move forward
def move_forward(speed=60):
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    set_speed(speed, speed)

# Function to move backward
def move_backward(speed=60):
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.HIGH)
    set_speed(speed, speed)

# Function to turn left
def turn_left(speed=60):
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    set_speed(speed, speed)

# Function to turn right
def turn_right(speed=60):
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.HIGH)
    set_speed(speed, speed)
