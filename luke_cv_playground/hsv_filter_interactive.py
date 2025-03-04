import cv2
import numpy as np

def nothing(x):
    pass

def get_pixel_archipelagos(mask):
    # iterate through every pixel in the mask (binary image)
    # if the pixel is white, then it is part of an archipelago
    # we will then find all the pixels connected to this pixel within some tolerance
    # and add them to the archipelago
    pixel_archipelagos = []

    h, w = mask.shape[:2]
    
    for y in range(h):
        for x in range(w):
            if mask[y, x] == 0:
                continue
            pixels = build_pixel_archipelago(mask, y, x)
            if not pixels or len(pixels) < 10:
                # Ignore small or null archipelagos
                continue
            pixel_archipelagos.append(pixels)

    return pixel_archipelagos
        
def build_pixel_archipelago(mask, y, x):
    pixels_to_visit = [(y, x)]
    pixels_in_archipelago = []
    while len(pixels_to_visit) > 0:
        y, x = pixels_to_visit.pop()
        if mask[y, x] == 0:
            continue
        mask[y, x] = 0 # Mark as visited by changing the pixel value to 0
        pixels_in_archipelago.append((y, x))
        maybe_nearby_pixels = get_nearby_white_pixels(mask, y, x, pixels_in_archipelago)
        if not maybe_nearby_pixels:
            continue
        pixels_to_visit.extend(maybe_nearby_pixels)
    return pixels_in_archipelago


def get_nearby_white_pixels(mask, y, x, pixels_to_exclude):
    tol = 2 # Tolerance for gaps between islands
    h, w = mask.shape[:2]

    x_min = max(x - tol, 0)
    x_max = min(x + tol + 1, w)
    y_min = max(y - tol, 0)
    y_max = min(y + tol + 1, h)

    nearby_pixels = []
    for y in range(y_min, y_max):
        for x in range(x_min, x_max):
            if mask[y, x] != 0 and (y, x) not in pixels_to_exclude:
                nearby_pixels.append((y, x))

    return nearby_pixels

def get_best_fit_line_segment(pixels):
    if len(pixels) < 2:
        raise ValueError("At least two points are required to fit a line.")

    # Convert input pixels into the required format for cv2.fitLine
    points = np.array(pixels).reshape(-1, 1, 2).astype(np.float32)

    # Fit a line using least squares
    [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

    # Compute the bounding box of the points
    min_x, min_y = np.min(points[:, 0, :], axis=0)
    max_x, max_y = np.max(points[:, 0, :], axis=0)

    # Define a function to find intersection points of the line with the bounding box
    def line_intersection_with_box(x, y, vx, vy, min_x, min_y, max_x, max_y):
        intersections = []

        # Solve for t at each boundary
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

    # Find the intersection points
    intersections = line_intersection_with_box(x, y, vx, vy, min_x, min_y, max_x, max_y)

    # Ensure exactly two points are found
    if len(intersections) != 2:
        raise RuntimeError("Failed to find two intersection points with the bounding box.")

    return intersections[0], intersections[1]


image = cv2.imread("inputs/older_with_drip_tape.jpg") 
reduction_factor = 0.25
image = cv2.resize(image, (0, 0), fx=reduction_factor, fy=reduction_factor)
cv2.namedWindow("HSV Filter")

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
s_mean = cv2.mean(hsv)[1]
print("Mean Saturation:", s_mean)

cv2.createTrackbar("Sat Max", "HSV Filter", int(s_mean*0.8), 255, nothing)
cv2.createTrackbar("Open Kernel", "HSV Filter", 0, 20, nothing)
cv2.createTrackbar("Close Kernel", "HSV Filter", 0, 20, nothing)

prev_s_max = -1
prev_open_kernel_size = -1
prev_close_kernel_size = -1

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
    mask = cv2.inRange(hsv, lower, upper)

    # Clean the mask (reduce noise)
    open_kernel = np.ones((open_kernel_size, open_kernel_size), np.uint8)
    close_kernel = np.ones((close_kernel_size, close_kernel_size), np.uint8)
    dilate_kernel = np.ones((2, 2), np.uint8)
    erode_kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    # Crop the mask
    height, width = mask.shape
    amount_of_image = 0.5
    bottom_half = mask[int(height*amount_of_image):height, 0:width]
    mask = bottom_half

    # Get the pixel archipelagos
    pixel_archipelagos = get_pixel_archipelagos(mask)

    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # Draw the pixel archipelagos in random colors
    for pixels in pixel_archipelagos:
        color = np.random.randint(0, 255, 3)
        for y, x in pixels:
            mask_colored[y, x] = color
    mask = mask_colored

    print("Number of archipelagos:", len(pixel_archipelagos))
    print("Image shape:", mask.shape)

    # # Get the best fit line segments
    # for pixels in pixel_archipelagos:
    #     try:
    #         p1, p2 = get_best_fit_line_segment(pixels)
    #         cv2.line(mask, p1, p2, (0, 255, 0), 2)
    #     except Exception as e:
    #         print(e)

    # Show the result
    cv2.imshow("HSV Filter", mask)

    # Update the previous values
    prev_s_max = s_max
    prev_open_kernel_size = open_kernel_size
    prev_close_kernel_size = close_kernel_size

    # Break the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Destroy all windows
cv2.destroyAllWindows()

# Save the mask
cv2.imwrite("outputs/mask.jpg", mask)