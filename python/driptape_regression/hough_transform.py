import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

base_dir = os.path.dirname(os.path.abspath(__file__))
image_name = "younger_with_drip_tape.jpg"
# image_name = "older_with_drip_tape.jpg"
# image_name = "wandering_drip_tape.jpg"
# image_name = "synthetic.png"
image_path = os.path.join(base_dir, "inputs", image_name)
image = cv2.imread(image_path)
scale_factor = 0.5
image = cv2.resize(image, (0, 0), fx=scale_factor, fy=scale_factor)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
h_channel = hsv[:, :, 0]
s_channel = hsv[:, :, 1]
v_channel = hsv[:, :, 2]
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blank = np.zeros_like(image, dtype=np.uint8)

blurred = cv2.blur(s_channel, (3, 3))
mask = cv2.inRange(blurred, 0, 50)

roi_gray = np.zeros_like(gray, dtype=np.uint8)
roi_colors = np.zeros_like(image, dtype=np.uint8)
height, width = gray.shape
roi_gray[height // 2:, :] = 255
roi_colors[height // 2:, :] = 255, 255, 255

mask = cv2.bitwise_and(mask, roi_gray)

lines = cv2.HoughLinesP(
    mask,
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

from sklearn.cluster import KMeans

def line_to_feature(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1

    # Normal vector angle
    theta = np.arctan2(-dx, dy)

    # Midpoint of the line segment
    x0 = (x1 + x2) / 2
    y0 = (y1 + y2) / 2

    # Distance from the origin to the line
    rho = (x0 * np.cos(theta) + y0 * np.sin(theta))

    return rho, theta

def feature_to_line(rho, theta, width, height):
    # Point closest to the origin on the line
    x_c = rho * np.cos(theta)
    y_c = rho * np.sin(theta)

    # Direction vector along the line
    dx = np.sin(theta)
    dy = -np.cos(theta)

    # Extend the line across the whole image
    length = np.hypot(width, height)  # length to extend in each direction

    x0 = x_c + dx * length
    y0 = y_c + dy * length
    x1 = x_c - dx * length
    y1 = y_c - dy * length

    return x0, y0, x1, y1

features = np.array([line_to_feature(*line[0]) for line in lines])
dbscan = DBSCAN(eps=20, min_samples=5)
clusters = dbscan.fit_predict(features)

# Calculate centroids of clusters
centroids = []
for cluster_id in range(max(clusters) + 1):
    if cluster_id != -1:
        cluster_points = features[clusters == cluster_id]
        centroid = np.mean(cluster_points, axis=0)
        centroids.append(centroid)
centroids = np.array(centroids)

# Plot features
rhos = features[:, 0]
thetas = features[:, 1]

plt.figure(figsize=(10, 6))
plt.scatter(thetas, rhos, c='blue', alpha=0.7)
plt.scatter(centroids[:, 1], centroids[:, 0], c='red', marker='x', s=100, label='Centroids')
plt.legend()
plt.xlabel("Theta (radians)")
plt.ylabel("Rho (distance from origin)")
plt.title("Line Features (Rho vs Theta)")
plt.grid(True)
plt.show()

# Draw representative lines
rep_lines_image = blank.copy()
for rho, theta in centroids:
    x0, y0, x1, y1 = feature_to_line(rho, theta, width, height)
    cv2.line(rep_lines_image, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
rep_lines_image = cv2.bitwise_and(rep_lines_image, roi_colors)

row1 = np.hstack((
    image, 
    cv2.cvtColor(h_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(s_channel, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(v_channel, cv2.COLOR_GRAY2BGR),
))

row2 = np.hstack((
    cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR),
    cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
    lines_image,
    rep_lines_image,
))

combined = np.vstack((row1, row2))
scale_factor = 0.6
combined = cv2.resize(combined, (0, 0), fx=scale_factor, fy=scale_factor)

# Display the image with detected lines
cv2.imshow("Detected Lines", combined)
cv2.waitKey(0)
cv2.destroyAllWindows()