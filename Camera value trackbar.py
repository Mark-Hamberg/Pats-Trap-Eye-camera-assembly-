import cv2
import numpy as np
import os

def empty(a):
    pass

path = 'Image/hotcamera.jpeg'

# Read previous values if the file exists
default_values = [0, 179, 0, 255, 0, 255]  # Default HSV ranges
if os.path.exists("value.txt"):
    with open("value.txt", "r") as file:
        try:
            loaded_values = list(map(int, file.read().strip().split(",")))
            if len(loaded_values) == 6:
                default_values = loaded_values
        except:
            print("Error reading value.txt. Using default values.")

# Create a window with trackbars
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
cv2.createTrackbar("Hue Min", "TrackBars", default_values[0], 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", default_values[1], 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", default_values[2], 255, empty)
cv2.createTrackbar("Sat Max", "TrackBars", default_values[3], 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", default_values[4], 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", default_values[5], 255, empty)

previous_values = default_values.copy()

while True:
    img = cv2.imread(path)

    # Resize the image for display
    img = cv2.resize(img, (640, 480))

    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

    current_values = [h_min, h_max, s_min, s_max, v_min, v_max]

    # Save the values to the file if they have changed
    if current_values != previous_values:
        with open("value.txt", "w") as file:
            file.write(f"{h_min},{h_max},{s_min},{s_max},{v_min},{v_max}")
        previous_values = current_values

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)

    # Display the images
    cv2.imshow("HSV Image", imgHSV)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        break

cv2.destroyAllWindows()
 