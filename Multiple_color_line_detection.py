import cv2
import numpy as np

def detect_colored_lines():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera not accessible")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Resize for performance
        frame = cv2.resize(frame, (320, 240))

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV color ranges
        color_ranges = {
            'red': [
                ((0, 120, 70), (10, 255, 255)),     # Lower red
                ((170, 120, 70), (180, 255, 255))   # Upper red
            ],
            'green': [((40, 40, 40), (80, 255, 255))],
            'blue': [((100, 150, 0), (140, 255, 255))]
        }

        # Process each color
        for color, ranges in color_ranges.items():
            mask = None
            for lower, upper in ranges:
                lower_np = np.array(lower)
                upper_np = np.array(upper)
                if mask is None:
                    mask = cv2.inRange(hsv, lower_np, upper_np)
                else:
                    mask |= cv2.inRange(hsv, lower_np, upper_np)

            # Morphological operations to reduce noise
            mask = cv2.GaussianBlur(mask, (5, 5), 0)
            edges = cv2.Canny(mask, 50, 150)

            # Hough Line Detection
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, 
                                    minLineLength=30, maxLineGap=10)

            # Set color for drawing
            bgr_color = {
                'red': (0, 0, 255),
                'green': (0, 255, 0),
                'blue': (255, 0, 0)
            }[color]

            # Draw lines
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(frame, (x1, y1), (x2, y2), bgr_color, 2)

        # Show the result
        cv2.imshow("Multi-Color Line Detection", frame)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_colored_lines()
