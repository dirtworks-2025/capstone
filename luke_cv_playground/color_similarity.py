import cv2
import numpy as np

def color_similarity(image, target_color):
    """
    Compute a grayscale image where intensity represents similarity to the target color.
    
    :param image: Input image (BGR format).
    :param target_color: Target color as (B, G, R) tuple.
    :return: Grayscale image with similarity scores.
    """
    # Convert image to float for calculations
    image = image.astype(np.float32)
    target_color = np.array(target_color, dtype=np.float32)

    # Compute Euclidean distance for each pixel
    distance = np.linalg.norm(image - target_color, axis=2)

    # Normalize distances to range [0, 255] (inverted so higher similarity -> brighter pixel)
    max_distance = np.sqrt(255**2 * 3)  # Max possible Euclidean distance in RGB
    similarity = 255 * (1 - (distance / max_distance))
    
    # Convert to uint8
    return similarity.astype(np.uint8)

# Load the image
image = cv2.imread("inputs/younger_crops.jpg")  # Replace with your image file

# Define the target color (in BGR format, since OpenCV loads in BGR)
target_color = (0, 255, 0)

# Compute the grayscale similarity map
similarity_map = color_similarity(image, target_color)

# Save and display the result
cv2.imwrite("outputs/similarity_map.jpg", similarity_map)
cv2.imshow("Similarity Map", similarity_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
