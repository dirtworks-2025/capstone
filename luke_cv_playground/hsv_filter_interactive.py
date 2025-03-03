import cv2
import numpy as np

def nothing(x):
    pass

image = cv2.imread("inputs/younger_with_drip_tape.jpg") 
cv2.namedWindow("HSV Filter")

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
s_mean = cv2.mean(hsv)[1]
print("Mean Saturation:", s_mean)

cv2.createTrackbar("Sat Max", "HSV Filter", int(s_mean*0.8), 255, nothing)

while True:
    s_max = cv2.getTrackbarPos("Sat Max", "HSV Filter")

    # Create a mask
    lower = np.array([0, 0, 0])
    upper = np.array([179, s_max, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # Clean the mask (reduce noise)
    kernel = np.ones((3, 3), np.uint8)  
    mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_cleaned = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel)

    # Show the result
    cv2.imshow("HSV Filter", mask_cleaned)

    # Break the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Destroy all windows
cv2.destroyAllWindows()

# Save the mask
cv2.imwrite("outputs/mask.jpg", mask)