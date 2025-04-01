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

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
enhanced = clahe.apply(s_channel)

blurred = cv2.GaussianBlur(enhanced, (5, 5), 10)
edges = cv2.Canny(blurred, 50, 150, apertureSize=3)

dilation_kernel_size = 3
dilation_kernel = np.ones((dilation_kernel_size, dilation_kernel_size), np.uint8)
dilated = cv2.dilate(edges, dilation_kernel, iterations=1)
blurred_again = cv2.GaussianBlur(dilated, (5, 5), 10)

inverted = cv2.bitwise_not(blurred_again)
thresholded = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

roi = np.zeros_like(gray, dtype=np.uint8)
height, width = gray.shape
roi[height // 2:, :] = 255
roi = cv2.bitwise_and(thresholded, roi)

opened = cv2.morphologyEx(roi, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

lines = cv2.HoughLinesP(
    edges, 
    rho=1,
    theta=np.pi / 180,
    threshold=100,
    minLineLength=100,
    maxLineGap=10,
)

image_with_lines = image.copy()
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 255, 0), 2)

placeholder = np.zeros_like(image_with_lines, dtype=np.uint8)

row1 = np.hstack((
    image, 
    cv2.cvtColor(h_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(s_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(v_channel, cv2.COLOR_GRAY2BGR),
))

row2 = np.hstack((
    cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR),
))

row3 = np.hstack((
    cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(blurred_again, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(inverted, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(opened, cv2.COLOR_GRAY2BGR),
))

combined = np.vstack((row1, row2, row3))
scale_factor = 0.3
combined = cv2.resize(combined, (0, 0), fx=scale_factor, fy=scale_factor)

# Display the image with detected lines
cv2.imshow("Detected Lines", combined)
cv2.waitKey(0)
cv2.destroyAllWindows()