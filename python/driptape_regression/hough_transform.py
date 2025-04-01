import cv2
import numpy as np
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
image_name = "younger_with_drip_tape.jpg"
# image_name = "older_with_drip_tape.jpg"
# image_name = "wandering_drip_tape.jpg"
# image_name = "synthetic.png"
image_path = os.path.join(base_dir, "inputs", image_name)
image = cv2.imread(image_path)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
h_channel = hsv[:, :, 0]
s_channel = hsv[:, :, 1]
v_channel = hsv[:, :, 2]
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blank = np.zeros_like(image, dtype=np.uint8)

blurred = cv2.GaussianBlur(s_channel, (5, 5), 10)
mask = cv2.inRange(blurred, 0, 50)

roi = np.zeros_like(gray, dtype=np.uint8)
height, width = gray.shape
roi[height // 2:, :] = 255
roi = cv2.bitwise_and(mask, roi)

lines = cv2.HoughLinesP(
    roi,
    rho=1,
    theta=np.pi / 180,
    threshold=100,
    minLineLength=100,
    maxLineGap=30,
)

print(f"Detected {len(lines)} lines")

lines_image = blank.copy()
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(lines_image, (x1, y1), (x2, y2), (0, 255, 0), 2)


row1 = np.hstack((
    image, 
    cv2.cvtColor(h_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(s_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(v_channel, cv2.COLOR_GRAY2BGR),
))

row2 = np.hstack((
    cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR),
    lines_image,
))

combined = np.vstack((row1, row2))
scale_factor = 0.3
combined = cv2.resize(combined, (0, 0), fx=scale_factor, fy=scale_factor)

# Display the image with detected lines
cv2.imshow("Detected Lines", combined)
cv2.waitKey(0)
cv2.destroyAllWindows()