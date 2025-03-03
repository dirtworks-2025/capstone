import cv2
import numpy as np

def nothing(x):
    pass

# Load the image
image = cv2.imread("inputs/younger_with_drip_tape.jpg") 

# Create a window
cv2.namedWindow("HSV Filter")

# Create trackbars
cv2.createTrackbar("Hue Min", "HSV Filter", 0, 179, nothing)
cv2.createTrackbar("Hue Max", "HSV Filter", 179, 179, nothing)
cv2.createTrackbar("Sat Min", "HSV Filter", 0, 255, nothing)
cv2.createTrackbar("Sat Max", "HSV Filter", 255, 255, nothing)
cv2.createTrackbar("Val Min", "HSV Filter", 0, 255, nothing)
cv2.createTrackbar("Val Max", "HSV Filter", 255, 255, nothing)

while True:
    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get the trackbar positions
    h_min = cv2.getTrackbarPos("Hue Min", "HSV Filter")
    h_max = cv2.getTrackbarPos("Hue Max", "HSV Filter")
    s_min = cv2.getTrackbarPos("Sat Min", "HSV Filter")
    s_max = cv2.getTrackbarPos("Sat Max", "HSV Filter")
    v_min = cv2.getTrackbarPos("Val Min", "HSV Filter")
    v_max = cv2.getTrackbarPos("Val Max", "HSV Filter")

    # Create a mask
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    # Apply the mask
    result = cv2.bitwise_and(image, image, mask=mask)

    # Show the result
    cv2.imshow("HSV Filter", mask)

    # Break the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Destroy all windows
cv2.destroyAllWindows()

# Save the mask
cv2.imwrite("outputs/mask.jpg", mask)
# Save the result
cv2.imwrite("outputs/result.jpg", result)