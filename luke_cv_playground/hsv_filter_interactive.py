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
            if mask[y, x] == 255:
                if is_already_member_of_an_archipelago(pixel_archipelagos, y, x):
                    continue
                pixels = [(y, x)]
                get_pixels_in_same_archipelago(mask, y, x, pixels)
                pixel_archipelagos.append(pixels)

    return pixel_archipelagos

def is_already_member_of_an_archipelago(pixel_archipelagos, y, x):
    for pixels in pixel_archipelagos:
        if (y, x) in pixels:
            return True

def get_pixels_in_same_archipelago(mask, y, x, pixels):
    # Recursively find all the pixels connected to the pixel at (y, x) within some tolerance

    tol = 2 # Tolerance for gaps between islands
    h, w = image.shape[:2]

    x_min = max(x - tol, 0)
    x_max = min(x + tol + 1, w)
    y_min = max(y - tol, 0)
    y_max = min(y + tol + 1, h)

    pixels_to_visit = []

    for i in range(y_min, y_max):
        for j in range(x_min, x_max):
            if mask[i, j] == 255 and (i, j) not in pixels:
                pixels.append((i, j))
                pixels_to_visit.append((i, j))

    for i, j in pixels_to_visit:
        get_pixels_in_same_archipelago(mask, i, j, pixels)

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
cv2.namedWindow("HSV Filter")

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
s_mean = cv2.mean(hsv)[1]
print("Mean Saturation:", s_mean)

cv2.createTrackbar("Sat Max", "HSV Filter", int(s_mean*0.8), 255, nothing)
cv2.createTrackbar("Open Kernel", "HSV Filter", 0, 20, nothing)
cv2.createTrackbar("Close Kernel", "HSV Filter", 0, 20, nothing)
cv2.createTrackbar("Dilate Iterations", "HSV Filter", 0, 20, nothing)
cv2.createTrackbar("Erode Iterations", "HSV Filter", 0, 20, nothing)

while True:
    s_max = cv2.getTrackbarPos("Sat Max", "HSV Filter")
    open_kernel_size = cv2.getTrackbarPos("Open Kernel", "HSV Filter")
    close_kernel_size = cv2.getTrackbarPos("Close Kernel", "HSV Filter")
    dilate_iterations = cv2.getTrackbarPos("Dilate Iterations", "HSV Filter")
    erode_iterations = cv2.getTrackbarPos("Erode Iterations", "HSV Filter")

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
    mask = cv2.dilate(mask, dilate_kernel, iterations = dilate_iterations)
    mask = cv2.erode(mask, erode_kernel, iterations = erode_iterations)

    # Crop the mask
    height, width = mask.shape
    amount_of_image = 0.5
    bottom_half = mask[int(height*amount_of_image):height, 0:width]
    mask = bottom_half

    # Get the pixel archipelagos
    pixel_archipelagos = get_pixel_archipelagos(mask)

    # Get the best fit line segments
    for pixels in pixel_archipelagos:
        try:
            p1, p2 = get_best_fit_line_segment(pixels)
            cv2.line(mask, p1, p2, (0, 255, 0), 2)
        except Exception as e:
            print(e)

    # Show the result
    cv2.imshow("HSV Filter", mask)

    # Break the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Destroy all windows
cv2.destroyAllWindows()

# Save the mask
cv2.imwrite("outputs/mask.jpg", mask)