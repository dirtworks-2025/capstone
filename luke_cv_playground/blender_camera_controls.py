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
mask_raw = cv2.inRange(hsv, lower, upper)

# Clean the mask (reduce noise)
kernel = np.ones((3, 3), np.uint8)  
mask_opened = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, kernel)
mask_cleaned = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel)

# Crop the mask
x_start, y_start = 100, 50
x_end, y_end = 300, 200
mask_cropped = mask_cleaned[y_start:y_end, x_start:x_end]

# Save the mask
cv2.imwrite(mask_path, mask_cropped)