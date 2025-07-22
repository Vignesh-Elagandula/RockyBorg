#!/usr/bin/env python3
# RockyBorg Line Follower - CLEAN VERSION WITH CAMERA OFFSET
import cv2
import numpy as np
import time
import sys
from picamera2 import Picamera2
sys.path.insert(0, '/home/pi/RockyBorg')
RockyBorg = None
import_success = False
try:
 from RockyBorg import RockyBorg
 print("RockyBorg imported using Method 1")
 import_success = True
except ImportError:
 try:
 import RockyBorg as rb_module
 RockyBorg = rb_module.RockyBorg
 print("RockyBorg imported using Method 2")
 import_success = True
 except ImportError:
 try:
 import Rockyborg
 if hasattr(Rockyborg, 'RockyBorg'):
 RockyBorg = Rockyborg.RockyBorg
 print("RockyBorg imported using Method 3")
 import_success = True
 else:
 print("Found Rockyborg module but no RockyBorg class")
 except ImportError:
 print("All import methods failed")
if not import_success or RockyBorg is None:
 print("ERROR: Could not import RockyBorg!")
 print("Please check:")
 print("1. RockyBorg folder exists: ls /home/pi/RockyBorg")
 print("2. Python files exist: ls /home/pi/RockyBorg/*.py")
 print("3. Run from correct directory")
 sys.exit(1)
def main():
 print("Initializing RockyBorg...")
 RB = RockyBorg()
 RB.Init()
 RB.SetCommsFailsafe(False)
 RB.SetMotorsEnabled(True)

 print("Initializing PiCamera...")
 picam2 = Picamera2()

 config = picam2.create_preview_configuration(
 main={"size": (640, 480), "format": "RGB888"}
 )
 picam2.configure(config)

 picam2.set_controls({
 "AeEnable": True,
 "AwbEnable": True
 })

 picam2.start()
 time.sleep(3)

 THRESHOLD = 80
 MIN_AREA = 150
 MAX_AREA = 3000
 STRIP_WIDTH = 120 # Detection area width
 camera_offset = 80 # Offset to compensate for camera angle

 # Motor parameters
 BASE_SPEED = 0.25
 MAX_SPEED = 0.5
 TURN_GAIN = 0.6
 LEFT_INVERT = -1
 RIGHT_INVERT = 1
 RIGHT_BOOST = 4.0
 RIGHT_MIN = 0.4

 print("Line follower ready! Press Ctrl+C to stop")
 print("Controls:")
 print(" 'a'/'s' - Lower/Raise threshold")
 print(" 'w'/'d' - Move detection zone left/right")
 print(" 'q' - Quit")

 try:
 while True:
 # Capture and rotate frame
 frame = picam2.capture_array()
 frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
 frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
 gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

 h, w = gray.shape

 # Create detection zone - SHIFTED RIGHT to compensate for left-angled camera
 center_x = w // 2
 adjusted_center = center_x + camera_offset
 half_width = STRIP_WIDTH // 2

 # Define boundaries (shifted right)
 left = max(0, adjusted_center - half_width)
 right = min(w, adjusted_center + half_width)
 top = int(h * 0.25)
 bottom = int(h * 0.65)

 # Extract region of interest
 roi = gray[top:bottom, left:right]

 # Process image
 blurred = cv2.GaussianBlur(roi, (5, 5), 0)
 _, binary = cv2.threshold(blurred, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

 # Clean up
 kernel = np.ones((3, 3), np.uint8)
 binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

 # Find contours
 contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
cv2.CHAIN_APPROX_SIMPLE)

 # Detect line
 line_found = False
 error = 0

 if contours:
 for contour in contours:
 area = cv2.contourArea(contour)

 if MIN_AREA <= area <= MAX_AREA:
 # Get line center
 M = cv2.moments(contour)
 if M["m00"] != 0:
 cx = int(M["m10"] / M["m00"])
 cy = int(M["m01"] / M["m00"])

 # Convert to global coordinates
 cx_global = cx + left
 cy_global = cy + top

 # Calculate error relative to ROBOT CENTER, not camera center
 robot_center_x = w // 2
 error = ((cx_global - robot_center_x) / (w // 2))
 line_found = True

 # Draw detection
 x, y, cw, ch = cv2.boundingRect(contour)
 cv2.rectangle(frame_bgr, (x + left, y + top),
 (x + left + cw, y + top + ch), (0, 255, 0), 2)
 cv2.circle(frame_bgr, (cx_global, cy_global), 5, (0, 0, 255), -1)
 break

 # Motor control
 if line_found:
 # Calculate motor speeds
 turn = error * TURN_GAIN
 left_speed = BASE_SPEED + turn
 right_speed = (BASE_SPEED - turn) * RIGHT_BOOST

 # Apply limits
 left_speed = np.clip(left_speed, -MAX_SPEED, MAX_SPEED)
 right_speed = np.clip(right_speed, -MAX_SPEED, MAX_SPEED)

 # Ensure minimum right motor speed
 if 0 < right_speed < RIGHT_MIN:
 right_speed = RIGHT_MIN
 elif -RIGHT_MIN < right_speed < 0:
 right_speed = -RIGHT_MIN

 # Set motors
 RB.SetMotor1(left_speed * LEFT_INVERT)
 RB.SetMotor2(right_speed * RIGHT_INVERT)

 # Status
 direction = "LEFT" if error < -0.1 else "RIGHT" if error > 0.1 else "STRAIGHT"
 print(f"\r{direction} | Error={error:.2f} | L={left_speed:.2f} R={right_speed:.2f}", end="")
 else:
 RB.MotorsOff()
 print("\rNo line detected - Stopped", end="")

 # Draw detection zone and reference lines
 cv2.rectangle(frame_bgr, (left, top), (right, bottom), (255, 255, 0), 2)
 cv2.line(frame_bgr, (w//2, 0), (w//2, h), (255, 0, 0), 2) # Robot center (blue)
 cv2.line(frame_bgr, (adjusted_center, 0), (adjusted_center, h), (0, 0, 255), 1) # Detection
center (red)

 # Status text
 status = "LINE FOUND!" if line_found else "NO LINE"
 color = (0, 255, 0) if line_found else (0, 0, 255)
 cv2.putText(frame_bgr, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
 cv2.putText(frame_bgr, f"Threshold: {THRESHOLD}", (10, 60),
cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
 cv2.putText(frame_bgr, f"Area: {STRIP_WIDTH}px", (10, 80),
cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
 cv2.putText(frame_bgr, f"Offset: {camera_offset}px", (10, 100),
cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
 cv2.putText(frame_bgr, "Blue=Robot, Red=Detection", (10, 120),
cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

 # Display
 cv2.imshow("Camera View", cv2.resize(frame_bgr, (480, 360)))
 cv2.imshow("Binary", cv2.resize(binary, (240, 180)))

 # Handle keys for threshold AND offset adjustment
 key = cv2.waitKey(1) & 0xFF
 if key == ord('q'):
 break
 elif key == ord('a'):
 THRESHOLD = max(20, THRESHOLD - 5)
 print(f"\nThreshold: {THRESHOLD}")
 elif key == ord('s'):
 THRESHOLD = min(150, THRESHOLD + 5)
 print(f"\nThreshold: {THRESHOLD}")
 elif key == ord('d'): # Move detection zone right
 camera_offset = min(150, camera_offset + 10)
 print(f"\nCamera offset: {camera_offset}")
 elif key == ord('w'): # Move detection zone left
 camera_offset = max(-50, camera_offset - 10)
 print(f"\nCamera offset: {camera_offset}")

 time.sleep(0.05)

 except KeyboardInterrupt:
 print("\nStopping...")
 finally:
 RB.MotorsOff()
 picam2.stop()
 cv2.destroyAllWindows()
 print("Shutdown complete.")
if __name__ == "__main__":
 main()