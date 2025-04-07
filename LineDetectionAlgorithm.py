import cv2
import numpy as np
import time
from ThunderBorg import ThunderBorg

# Initialize ThunderBorg for motor control
TB = ThunderBorg.ThunderBorg()
TB.Init()

# Set up camera (use PiCamera or webcam)
cap = cv2.VideoCapture(0)  # Change to 1 or 2 if using another camera

# Motor control function
def move_forward(speed, duration):
    TB.SetMotor1(speed)  # Left motor
    TB.SetMotor2(speed)  # Right motor
    time.sleep(duration)
    TB.MotorsOff()

def turn_left(speed, duration):
    TB.SetMotor1(-speed)  # Left motor backward
    TB.SetMotor2(speed)   # Right motor forward
    time.sleep(duration)
    TB.MotorsOff()

def turn_right(speed, duration):
    TB.SetMotor1(speed)   # Left motor forward
    TB.SetMotor2(-speed)  # Right motor backward
    time.sleep(duration)
    TB.MotorsOff()

# Function to find the center of the line
def find_line_center(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply threshold to isolate the black line (adjust the threshold value as necessary)
    _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours (the black line should form a contour)
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the largest contour, which is the line
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate the moments of the contour
        M = cv2.moments(largest_contour)
        
        # Find the center of the contour
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy
    return None, None

# Main loop
while True:
    ret, frame = cap.read()  # Capture a frame from the camera
    if not ret:
        print("Failed to grab frame")
        break

    # Find the center of the black line
    cx, cy = find_line_center(frame)
    
    if cx is not None:
        # Draw a circle at the center of the line
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
        
        # Get the center of the frame (width of the image)
        frame_center = frame.shape[1] // 2
        
        # Logic to determine movement based on line position
        if cx < frame_center - 50:  # If the line is too far left
            print("Turn left")
            turn_left(0.5, 0.5)  # Adjust speed and duration
        elif cx > frame_center + 50:  # If the line is too far right
            print("Turn right")
            turn_right(0.5, 0.5)  # Adjust speed and duration
        else:
            print("Move forward")
            move_forward(0.5, 1)  # Move forward at speed 0.5 for 1 second

    # Display the resulting frame with center mark
    cv2.imshow("Line Following", frame)
    
    # Wait for a key press to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
