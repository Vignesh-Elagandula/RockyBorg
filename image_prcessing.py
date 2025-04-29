import cv2
from picamera2 import Picamera2
import time

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888'}))
picam2.start()

# Give camera time to warm up
time.sleep(1)

print("Press 'q' to quit...")

while True:
    # Capture a frame
    frame = picam2.capture_array()

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the grayscale frame
    cv2.imshow("Grayscale Camera Fee", gray)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
