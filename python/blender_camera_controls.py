import bpy
import math
import numpy as np
import cv2

# Parameters
rendered_image_path = bpy.path.abspath("//rendered_image.png")
mask_path = bpy.path.abspath("//mask.jpg")

# Ensure there's a camera in the scene
if "Camera" not in bpy.data.objects:
    camera_data = bpy.data.cameras.new("Camera")
    camera_object = bpy.data.objects.new("Camera", camera_data)
    bpy.context.collection.objects.link(camera_object)
else:
    camera_object = bpy.data.objects["Camera"]

# Set camera pose
camera_object.location = (3, -0.2, 1)
camera_object.rotation_euler = (math.pi*(5/12), 0, math.pi/2)

# Set the camera as the active one
bpy.context.scene.camera = camera_object

# Render and save the image
bpy.context.scene.render.filepath = rendered_image_path
bpy.ops.render.render(write_still=True)

# Read the image into OpenCV
image = cv2.imread(rendered_image_path)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
s_mean = cv2.mean(hsv)[1]

# Create a mask
s_max = int(s_mean*0.1)
lower = np.array([0, 0, 0])
upper = np.array([179, s_max, 255])
mask = cv2.inRange(hsv, lower, upper)

# Clean the mask (reduce noise)
open_kernel = np.ones((3, 3), np.uint8)  
close_kernel = np.ones((5, 5), np.uint8)
dilate_kernel = np.ones((3, 3), np.uint8)
erode_kernel = np.ones((3, 3), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
mask = cv2.dilate(mask, dilate_kernel, iterations = 20)
mask = cv2.erode(mask, erode_kernel, iterations = 20)

## Crop the mask
#x_start, y_start = 0, 1920
#x_end, y_end = 0, 1080
#mask_cropped = mask_cleaned[y_start:y_end, x_start:x_end]

# Save the mask
cv2.imwrite(mask_path, mask)