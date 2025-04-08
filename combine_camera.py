import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import RockyBorg

# Initialize RockyBorg
RB = RockyBorg.RockyBorg()
RB.Init()
RB.SetCommsFailsafe(False)

# Set motor power level
POWER = 0.6

# Camera setup
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
raw_capture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)  # Camera warm-up

# Movement functions
def move_forward():
    RB.SetMotor1(POWER)
    RB.SetMotor2(POWER)

def move_backward():
    RB.SetMotor1(-POWER)
    RB.SetMotor2(-POWER)

def turn_left():
    RB.SetMotor1(-POWER)
    RB.SetMotor2(POWER)

def turn_right():
    RB.SetMotor1(POWER)
    RB.SetMotor2(-POWER)

def stop():
    RB.MotorsOff()

# Main loop
try:
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array

        # Show the camera image (for debugging or object detection)
        cv2.imshow("RockyBorg Camera", image)

        # Control with keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('w'):
            move_forward()
        elif key == ord('s'):
            move_backward()
        elif key == ord('a'):
            turn_left()
        elif key == ord('d'):
            turn_right()
        elif key == ord('x'):  # Stop
            stop()
        elif key == ord('q'):  # Quit
            break

        raw_capture.truncate(0)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    stop()
    cv2.destroyAllWindows()