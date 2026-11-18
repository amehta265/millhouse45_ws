"""Shared preprocessing for the Lab 6 sign classifier.

Used by train.py (training) and model_grader.py (grading), so both see
exactly the same features.

Pipeline: color masks -> largest sign-like blob -> square crop -> feature vector.
"""
import cv2
import numpy as np

# Pink signs (do not enter, stop, goal), in HSV (OpenCV hue is 0-179).
# Hue starts at 150 so the orange wall (hue ~0-15) is never included.
PINK_LOW = (150, 100, 60)
PINK_HIGH = (179, 255, 255)
# Light arrows, in Lab. Arrows are pale green/blue/cream: bright (L) and on the
# green side of the a* axis, while walls, ceilings and the floor are all redder
# (a* above ~132), even under the pinkish light of the 2026F_Timgs photos.
LIGHT_LOW = (140, 0, 0)
LIGHT_HIGH = (255, 129, 255)

MIN_AREA_FRAC = 0.006   # smaller blobs are noise or far-away signs (smallest real arrow is ~0.8%)
SHAPE_SIZE = 20         # crop is resized to SHAPE_SIZE x SHAPE_SIZE
COLOR_WEIGHT = 6.0      # how strongly the pink/light flag counts in distances

KERNEL = np.ones((5, 5), np.uint8)


def color_masks(image):
    """Return (pink_mask, light_mask) as uint8 images with the sign in white."""
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    pink = cv2.inRange(cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV), PINK_LOW, PINK_HIGH)
    light = cv2.inRange(cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB), LIGHT_LOW, LIGHT_HIGH)
    return [clean(pink), clean(light)]


def clean(mask):
    """Remove speckle, then fill small holes."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)


def is_floor_or_ceiling(x, y, w, h, img_w, img_h):
    """Bright floor/ceiling bands touch the bottom edge or span the full width."""
    touches_bottom = y + h >= img_h - 1
    spans_width = x <= 0 and x + w >= img_w - 1
    return touches_bottom or spans_width


def find_sign(image):
    """Find the largest sign-like blob.

    Returns (blob_mask, is_pink, (x, y, w, h)) or None if nothing is found.
    blob_mask is a full-size image containing only the chosen blob.
    """
    img_h, img_w = image.shape[:2]
    min_area = MIN_AREA_FRAC * img_w * img_h
    best = None
    for is_pink, mask in zip([True, False], color_masks(image)):
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < min_area:
                continue
            if not is_pink and is_floor_or_ceiling(x, y, w, h, img_w, img_h):
                continue
            if best is None or area > best[0]:
                best = (area, is_pink, labels == i, (x, y, w, h))
    if best is None:
        return None
    _, is_pink, blob, box = best
    return blob.astype(np.uint8) * 255, is_pink, box


def crop_square(blob_mask, box, margin=4):
    """Cut the blob out with a small margin and pad it to a square."""
    x, y, w, h = box
    img_h, img_w = blob_mask.shape
    x0, y0 = max(x - margin, 0), max(y - margin, 0)
    x1, y1 = min(x + w + margin, img_w), min(y + h + margin, img_h)
    crop = blob_mask[y0:y1, x0:x1]
    side = max(crop.shape)
    top = (side - crop.shape[0]) // 2
    left = (side - crop.shape[1]) // 2
    return cv2.copyMakeBorder(crop, top, side - crop.shape[0] - top,
                              left, side - crop.shape[1] - left,
                              cv2.BORDER_CONSTANT, value=0)


def extract_features(image):
    """Turn a BGR image into a feature vector, or None if no sign blob is found."""
    found = find_sign(image)
    if found is None:
        return None
    blob, is_pink, box = found
    x, y, w, h = box
    shape = cv2.resize(crop_square(blob, box), (SHAPE_SIZE, SHAPE_SIZE),
                       interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    area_frac = np.count_nonzero(blob) / blob.size
    fill = np.count_nonzero(blob[y:y + h, x:x + w]) / float(w * h)
    extras = [COLOR_WEIGHT * float(is_pink),
              10.0 * np.sqrt(area_frac),   # sqrt so far and near signs are not too far apart
              2.0 * fill,
              2.0 * np.log(w / float(h))]
    return np.concatenate([shape.ravel(), np.array(extras, np.float32)])
