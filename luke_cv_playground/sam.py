import cv2
import numpy as np
from segment_anything import SamPredictor, sam_model_registry, SamAutomaticMaskGenerator

def get_image_ndarray(path: str):
    data = open(path, "rb").read()
    image = np.frombuffer(data, np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image

sam = sam_model_registry["vit_b"](checkpoint="sam_vit_b_01ec64.pth")
image_array = get_image_ndarray("images/older_crops.jpg")

# This doesn't work; instead need to prompt by coordinates
# predictor = SamPredictor(sam)
# predictor.set_image(image_array)
# masks, _, _ = predictor.predict("plants")

mask_generator = SamAutomaticMaskGenerator(sam)
masks = mask_generator.generate(image_array)