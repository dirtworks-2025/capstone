import cv2
import numpy as np

# Load the image
image = cv2.imread("inputs/synthetic.png")  # Replace with your image file

# Split channels
blue, green, red = cv2.split(image)

# Create blank channels
zero_channel = np.zeros_like(blue)

# Create images with only one color channel
red_image = cv2.merge([zero_channel, zero_channel, red])
green_image = cv2.merge([zero_channel, green, zero_channel])
blue_image = cv2.merge([blue, zero_channel, zero_channel])

# Save the images
cv2.imwrite("outputs/red_channel.jpg", red_image)
cv2.imwrite("outputs/green_channel.jpg", green_image)
cv2.imwrite("outputs/blue_channel.jpg", blue_image)

# Show images (optional)
cv2.imshow("Red Channel", red_image)
cv2.imshow("Green Channel", green_image)
cv2.imshow("Blue Channel", blue_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
