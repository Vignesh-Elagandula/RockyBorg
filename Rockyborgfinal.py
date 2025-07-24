#!/usr/bin/ev python3
"""
PERFECT LINE FOLLOWING - AGGRESSIVE SEARCH ALGORITHM
Enhanced with aggressive line search capabilities:
- Immediate aggressive search when line is lost 
- Wider and faster sweep patterns
- Systematic left-right coverage
- Higher search speeds
- No gentle timeouts - hunts for the line actively
"""

import cv2
import numpy as np
import time
from collections import deque
from picamera2 import Picamera2
from libcamera import Transform
import sys

# Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬ RockyBorg Setup Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬
sys.path.insert(0, "/home/pi/RockyBorg")
import RockyBorg

# Ã¢â€¢â€Ã¢â€¢ÂÃ¢â€¢Â FIXED RockyBorg Initialization Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢â€”
print("Ã°Å¸â€Â§ Initializing RockyBorg...")

try:
    RB = RockyBorg.RockyBorg()
    print("Ã¢Å“â€¦ Using RockyBorg.RockyBorg()")
except AttributeError:
    try:
        RB = RockyBorg.RockyBorgClass()
        print("Ã¢Å“â€¦ Using RockyBorg.RockyBorgClass()")
    except AttributeError:
        print("Ã°Å¸â€œâ€¹ Available in RockyBorg module:")
        print([x for x in dir(RockyBorg) if not x.startswith('_')])
        
        available_classes = [x for x in dir(RockyBorg) if not x.startswith('_') and x[0].isupper()]
        if available_classes:
            class_name = available_classes[0]
            RB = getattr(RockyBorg, class_name)()
            print(f"Ã¢Å“â€¦ Using RockyBorg.{class_name}()")
        else:
            
            
            
            
            
            
            
            
            
            print("Ã¢ÂÅ’ No suitable class found!")
            sys.exit(1)

# Initialize the robot
RB.Init()
RB.SetServoPosition(0.09)  # Calibrated neutral trim
RB.MotorsOff()

# Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬ OPTIMIZED Parameters Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬b a
# Camera and ROI
HEIGHT = 640
WIDTH = 640
ROI_HEIGHT = 200            # Increased for better curve detection
ROI_WIDTH_CROP = 30        # Crop sides to focus on center area

# Speed Control - FAST AND DYNAMIC
BASE_SPEED = 0.15        # Fast base speed
MIN_SPEED = 0.15           # High minimum to maintain momentum
MAX_SPEED = 0.28            # High top speed for straight sections

# Steering Control - RESPONSIVE
STEERING_GAIN = 0.8         # Strong but not excessive
SMOOTHING_WINDOW = 2        # Minimal smoothing for quick response
SERVO_TRIM = 0.0           # Your calibrated neutral

# Line Detection
BINARY_THRESHOLD = 60      # Adjusted for better line detection
MIN_CONTOUR_AREA = 100      # Minimum area to consider as line

# Search and Recovery - MORE AGGRESSIVE
LINE_LOST_TIMEOUT = 0.5     # Start searching immediately (was 0.4)
SEARCH_AMPLITUDE = 0.5    # Much wider search range (was 0.35)
SEARCH_FREQUENCY = 1.2     # Faster search sweeps (was 0.6)
SEARCH_STEP_SIZE = 0.00    # How much to move servo per step
MAX_SEARCH_ANGLE = 0.10     # Maximum search angle

# Processing
FRAME_DELAY = 0.10          # Fast processing
CAMERA_ROTATE = True        # Rotate frame 90Ã‚Â° clockwise
DEBUG_VIEW = True

# Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬ Camera Setup Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬
print("Ã°Å¸â€œÂ· Initializing camera...")
picam2 = Picamera2()
picam2.preview_configuration.main.size = (WIDTH, HEIGHT)
picam2.preview_configuration.main.format = "RGB888"
# Normal transform - rotation handled in software
picam2.preview_configuration.transform = Transform(hflip=0, vflip=0, rotation=0)
picam2.configure("preview")
picam2.start()
time.sleep(1)

print("Ã°Å¸Â¤â€“ Ready. Aggressive search line following algorithm starting...")

# Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬ Enhanced State Variables Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬
centroid_history = deque(maxlen=SMOOTHING_WINDOW)
motors_on = False
last_seen_time = None
last_good_steering = 0.0
consecutive_detections = 0
search_direction = 1  # 1 or -1 for search direction

# Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬ Helper Functions Ã¢â€â‚¬Ã¢â€â‚¬Ã¢â€â‚¬
def calculate_adaptive_speed(abs_error, base_speed, min_speed, max_speed):
    """Calculate speed based on line position error"""
    normalized_error = abs_error / (WIDTH // 2)
    
    if normalized_error > 0.25:        # Sharp curve
        return min_speed
    elif normalized_error > 0.15:      # Moderate curve  
        return base_speed * 0.85
    elif normalized_error > 0.08:      # Slight curve
        return base_speed * 0.95
    else:                              # Straight line
        return min(max_speed, base_speed * 1.15)

def aggressive_search_pattern(time_lost):
    """AGGRESSIVE search pattern - sweeps wider and faster to find line"""
    if time_lost < 0.2:
        # Phase 1: Quick hold then start aggressive search
        return last_good_steering * 0.9
    else:
        # Phase 2: Aggressive sweeping search
        search_time = time_lost - 0.2
        
        # Fast sine wave with increasing amplitude
        base_sweep = SEARCH_AMPLITUDE * np.sin(2 * np.pi * SEARCH_FREQUENCY * search_time)
        
        # Add step-wise expansion for more coverage
        expansion_factor = min(2.0, 1.0 + search_time / 2.0)
        aggressive_sweep = base_sweep * expansion_factor
        
        # Clamp to maximum search angle
        return np.clip(aggressive_sweep, -MAX_SEARCH_ANGLE, MAX_SEARCH_ANGLE)

try:
    while True:
        # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â FRAME CAPTURE AND PREPROCESSING Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
        frame = picam2.capture_array()
        
        # Apply rotation like your working code
        if CAMERA_ROTATE:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        
        current_time = time.time()

        # Convert to grayscale and binary
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, binary = cv2.threshold(gray, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        
        # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â SMART ROI EXTRACTION Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
        # Position ROI at 53% from top with better parameters for higher detection
        roi_y_start = int(HEIGHT * 0.53)  # 53% from top
        roi_y_end = roi_y_start + ROI_HEIGHT
        roi_x_start = ROI_WIDTH_CROP
        roi_x_end = WIDTH - ROI_WIDTH_CROP
        
        # Ensure ROI doesn't go beyond frame boundaries
        roi_y_end = min(roi_y_end, HEIGHT)
        
        roi = binary[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        roi_display = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]  # For visualization

        # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â LINE DETECTION USING CONTOURS Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found_line = False
        
        if contours:
            # Find the largest contour (most likely the line)
            largest = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest) > MIN_CONTOUR_AREA:
                # Calculate centroid using moments
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    # Centroid coordinates (relative to ROI)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Add smoothing
                    centroid_history.append(cx)
                    smoothed_cx = int(np.mean(centroid_history))
                    
                    # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â STEERING CALCULATION Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
                    roi_center = (roi_x_end - roi_x_start) // 2
                    error = smoothed_cx - roi_center
                    normalized_error = error / roi_center
                    
                    # Apply steering gain with deadzone
                    if abs(normalized_error) < 0.03:
                        normalized_error = 0.0
                    steering = np.clip(normalized_error * STEERING_GAIN, -1.12, 0.55)
                    final_steering = np.clip(steering + SERVO_TRIM, -1.12, 0.55)  # Clip after adding trim
                    
                    # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â ADAPTIVE SPEED CONTROL Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
                    abs_error = abs(error)
                    speed = calculate_adaptive_speed(abs_error, BASE_SPEED, MIN_SPEED, MAX_SPEED)
                    
                    # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â APPLY CONTROL COMMANDS Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
                    RB.SetServoPosition(final_steering)
                    RB.SetMotor1(-speed)
                    RB.SetMotor2(speed)
                    
                    # Update state
                    motors_on = True
                    last_seen_time = current_time
                    last_good_steering = steering
                    found_line = True
                    consecutive_detections += 1
                    search_direction = 1  # Reset search direction
                    
                    status = f"Ã¢Å“â€¦ Line | Pos:{smoothed_cx} | Err:{error} | Steer:{steering:.2f} | Speed:{speed:.2f}"
                    
                    # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â VISUAL FEEDBACK Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
                    if DEBUG_VIEW:
                        # Draw on ROI display
                        cv2.drawContours(roi_display, [largest], -1, (0, 0, 255), 2)
                        cv2.circle(roi_display, (smoothed_cx, cy), 6, (0, 255, 0), -1)
                        cv2.line(roi_display, (roi_center, 0), (roi_center, roi_display.shape[0]), (255, 0, 0), 2)
                        
                        # Bounding rectangle
                        x, y, w, h = cv2.boundingRect(largest)
                        cv2.rectangle(roi_display, (x, y), (x + w, y + h), (255, 255, 0), 1)

        # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â AGGRESSIVE LINE LOST LOGIC Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
        if not found_line:
            consecutive_detections = 0
            
            if last_seen_time is None:
                last_seen_time = current_time
            
            time_since_seen = current_time - last_seen_time
            
            if time_since_seen > LINE_LOST_TIMEOUT:
                if motors_on:
                    # AGGRESSIVE search mode
                    search_steering = aggressive_search_pattern(time_since_seen)
                    
                    # Add directional bias for more systematic search
                    if time_since_seen > 1.0:
                        # After 1 second, add systematic left-right sweeping
                        systematic_search = search_direction * MAX_SEARCH_ANGLE * 0.8
                        search_steering = systematic_search
                        
                        # Reverse direction every 0.5 seconds for thorough coverage
                        if int(time_since_seen * 2) % 2 == 0:
                            search_direction *= -1
                    
                    final_search_steering = np.clip(search_steering + SERVO_TRIM, -1.0, 1.0)
                    search_speed = BASE_SPEED * 0.85  # Higher speed while searching (was 0.75)
                    
                    RB.SetServoPosition(final_search_steering)
                    RB.SetMotor1(-search_speed)
                    RB.SetMotor2(search_speed)
                    
                    status = f"Ã°Å¸â€Â AGGRESSIVE SEARCH | Angle:{search_steering:.2f} | Dir:{search_direction} | Time:{time_since_seen:.1f}s"
                    
                    # Stop completely after extended search
                    if time_since_seen > 5.0:  # Extended time for aggressive search
                        RB.MotorsOff()
                        motors_on = False
                        status = f"Ã°Å¸â€ºâ€˜ Line lost for {time_since_seen:.1f}s Ã¢â€ â€™ stopped"
            else:
                # Very brief pause before starting aggressive search
                status = f"Ã¢ÂÂ³ Line lost | AGGRESSIVE SEARCH in {LINE_LOST_TIMEOUT - time_since_seen:.2f}s"

        # Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â DEBUG VISUALIZATION Ã¢â€¢ÂÃ¢â€¢ÂÃ¢â€¢Â
        if DEBUG_VIEW:
            # Draw ROI boundaries on main frame
            cv2.rectangle(frame, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 255), 2)
            
            # Status display
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Line detection indicator
            if found_line:
                cv2.circle(frame, (30, 50), 8, (0, 255, 0), -1)
                cv2.putText(frame, f"FOUND({consecutive_detections})", (50, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            else:
                cv2.circle(frame, (30, 50), 8, (0, 0, 255), -1)
                cv2.putText(frame, "LOST", (50, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
            # Search direction indicator
            search_color = (255, 0, 255) if not found_line else (100, 100, 100)
            cv2.putText(frame, f"Search Dir:{search_direction}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, search_color, 1)
            
            # Parameter display
            cv2.putText(frame, f"Gain:{STEERING_GAIN} Speed:{BASE_SPEED} SearchAmp:{SEARCH_AMPLITUDE}", (10, 95), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Show windows
            cv2.imshow("Camera View", frame)
            cv2.imshow("Binary Threshold", binary)
            cv2.imshow("ROI Detection", roi_display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Ã°Å¸â€˜â€¹ Exiting...")
                break

        time.sleep(FRAME_DELAY)

except KeyboardInterrupt:
    print("Ã¢Å¡ Ã¯Â¸Â Interrupted by user")
except Exception as e:
    print(f"Ã¢ÂÅ’ Error: {e}")
finally:
    print("Ã°Å¸â€Â§ Cleaning up...")
    RB.MotorsOff()
    RB.SetServoPosition(SERVO_TRIM)  # Return to neutral
    picam2.close()
    cv2.destroyAllWindows()
    print("Ã¢Å“â€¦ Cleanup complete")
