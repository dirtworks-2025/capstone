import cv2
import numpy as np
from collections import deque

def nothing(x):
    pass

def get_pixel_islands(mask):
    """
    Uses OpenCV's connected components to extract pixel archipelagos efficiently.
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    pixel_archipelagos = []
    for i in range(1, num_labels):  # Ignore background (label 0)
        if stats[i, cv2.CC_STAT_AREA] >= 10:  # Filter out small components
            y, x = np.where(labels == i)
            pixels = list(zip(y, x))
            pixel_archipelagos.append(pixels)
    
    return pixel_archipelagos

def get_best_fit_line_segment(pixels):
    """
    Computes the best fit line for a given set of pixels using cv2.fitLine.
    """
    if len(pixels) < 2:
        raise ValueError("At least two points are required to fit a line.")

    points = np.array(pixels).reshape(-1, 1, 2).astype(np.float32)
    [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

    min_x, min_y = np.min(points[:, 0, :], axis=0)
    max_x, max_y = np.max(points[:, 0, :], axis=0)

    # Find intersection points
    def line_intersection_with_box(x, y, vx, vy, min_x, min_y, max_x, max_y):
        intersections = []
        for x_bound in [min_x, max_x]:
            t = (x_bound - x) / vx
            y_intersect = y + t * vy
            if min_y <= y_intersect <= max_y:
                intersections.append((x_bound, int(y_intersect)))

        for y_bound in [min_y, max_y]:
            t = (y_bound - y) / vy
            x_intersect = x + t * vx
            if min_x <= x_intersect <= max_x:
                intersections.append((int(x_intersect), y_bound))

        return intersections

    intersections = line_intersection_with_box(x, y, vx, vy, min_x, min_y, max_x, max_y)
    
    if len(intersections) != 2:
        raise RuntimeError("Failed to find two intersection points with the bounding box.")

    return intersections[0], intersections[1]

# Load and resize image
image = cv2.imread("inputs/younger_with_drip_tape.jpg") 
image = cv2.resize(image, (0, 0), fx=0.25, fy=0.25)
height, width = image.shape[:2]
image = image[int(height * 0.5):] # Crop to bottom half

cv2.namedWindow("HSV Filter")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
s_mean = cv2.mean(hsv)[1]
print("Mean Saturation:", s_mean)

cv2.createTrackbar("Sat Max", "HSV Filter", int(s_mean * 0.8), 255, nothing)
cv2.createTrackbar("Open Kernel", "HSV Filter", 4, 20, nothing)
cv2.createTrackbar("Close Kernel", "HSV Filter", 3, 20, nothing)

prev_s_max, prev_open_kernel_size, prev_close_kernel_size = -1, -1, -1

while True:
    s_max = cv2.getTrackbarPos("Sat Max", "HSV Filter")
    open_kernel_size = cv2.getTrackbarPos("Open Kernel", "HSV Filter")
    close_kernel_size = cv2.getTrackbarPos("Close Kernel", "HSV Filter")

    if (prev_s_max == s_max) and (prev_open_kernel_size == open_kernel_size) and (prev_close_kernel_size == close_kernel_size):
        cv2.waitKey(1)
        continue

    # Create a mask
    lower = np.array([0, 0, 0])
    upper = np.array([179, s_max, 255])
    sat_mask = cv2.inRange(hsv, lower, upper)

    # Morphological transformations
    if open_kernel_size > 0:
        open_kernel = np.ones((open_kernel_size, open_kernel_size), np.uint8)
        denoised_mask = cv2.morphologyEx(sat_mask, cv2.MORPH_OPEN, open_kernel)

    if close_kernel_size > 0:
        close_kernel = np.ones((close_kernel_size, close_kernel_size), np.uint8)
        denoised_mask = cv2.morphologyEx(denoised_mask, cv2.MORPH_CLOSE, close_kernel)

    # Get the pixel archipelagos using connected 
    mask_copy = denoised_mask.copy()
    pixel_archipelagos = get_pixel_islands(mask_copy)

    # Convert to color image
    mask_colored = cv2.cvtColor(mask_copy, cv2.COLOR_GRAY2BGR)

    # Draw archipelagos in random colors
    for pixels in pixel_archipelagos:
        color = np.random.randint(0, 255, 3).tolist()
        for y, x in pixels:
            mask_colored[y, x] = color

    # # Draw best fit lines (optional)
    # for pixels in pixel_archipelagos:
    #     try:
    #         p1, p2 = get_best_fit_line_segment(pixels)
    #         cv2.line(mask_colored, p1, p2, (0, 255, 0), 2)
    #     except Exception as e:
    #         print(e)

    combined = np.hstack((
        image, 
        cv2.cvtColor(sat_mask, cv2.COLOR_GRAY2BGR), 
        cv2.cvtColor(denoised_mask, cv2.COLOR_GRAY2BGR), 
        mask_colored
    ))

    cv2.imshow("HSV Filter", combined)

    # Update previous values
    prev_s_max, prev_open_kernel_size, prev_close_kernel_size = s_max, open_kernel_size, close_kernel_size

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
cv2.imwrite("outputs/mask.jpg", mask_colored)
