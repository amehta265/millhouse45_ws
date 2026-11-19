"""Random image augmentation for training"""
""" This is a way to add more variety to the dataset and make it generaliziable"""
import cv2
import numpy as np

LEFT, RIGHT = 1, 2
# We flip the images cause left arrow  = right arrow
# We do a couple of small rotations to the images
# We change the brightness and saturation of the images so its under different lighting
def augment(image, label, rng):
    h, w = image.shape[:2]

    if label in (LEFT, RIGHT) and rng.random() < 0.5:
        image = cv2.flip(image, 1)
        label = RIGHT if label == LEFT else LEFT

    matrix = cv2.getRotationMatrix2D((w / 2, h / 2), rng.uniform(-10, 10), rng.uniform(0.85, 1.15))
    matrix[:, 2] += (rng.uniform(-0.08, 0.08) * w, rng.uniform(-0.08, 0.08) * h)
    image = cv2.warpAffine(image, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] *= rng.uniform(0.8, 1.2)
    hsv[..., 2] *= rng.uniform(0.75, 1.25)
    image = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    return image, label
