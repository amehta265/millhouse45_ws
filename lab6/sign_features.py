"""Preprocessing
Pipeline: color masks -> largest sign-like blob -> square crop -> feature vector.
"""
import cv2
import numpy as np

# Pink signs: (U turn, stop, goal)
PINK_LOW = (150, 100, 60)
PINK_HIGH = (179, 255, 255)
# Arrows have lighter colors i.e. green, blue, cream
# Empty walls are brownish red
LIGHT_LOW = (140, 0, 0)
LIGHT_HIGH = (255, 129, 255)

MIN_AREA_FRAC = 0.006   # remove small blobs after masking to focus on target feat i.e. the sign
SHAPE_SIZE = 20         # crop image to SHAPE_SIZE x SHAPE_SIZE
COLOR_WEIGHT = 6.0      # how strongly the pink/light flag counts in distances

# Defining mask shape/size
KERNEL = np.ones((5, 5), np.uint8)

def color_masks(image):
    blurred = cv2.GaussianBlur(image, (5, 5), 0) # Some smoothing
    pink = cv2.inRange(cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV), PINK_LOW, PINK_HIGH) # separate image by pink range
    light = cv2.inRange(cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB), LIGHT_LOW, LIGHT_HIGH) # separate image by light
    return [clean(pink), clean(light)]

def clean(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL) # Erosion followed my dilation i.e. remove small speckles around main image
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL) # Dilation followed by erosion i.e. remove any "black" dots from INSIDE target image so it appears whole

# Throw out blob if it touches the ceiling or bottom or extends full width of the image. No sign should be doing that
def is_floor_or_ceiling(x, y, w, h, img_w, img_h):
    touches_bottom = y + h >= img_h - 1
    spans_width = x <= 0 and x + w >= img_w - 1
    return touches_bottom or spans_width

# Extract largest "sign like" blob 
def find_sign(image):
    img_h, img_w = image.shape[:2]
    min_area = MIN_AREA_FRAC * img_w * img_h
    best = None
    # Combining the iterables into a list of tuples so its easier to assert conditions later on
    for is_pink, mask in zip([True, False], color_masks(image)):
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask) # Put bounding boxes on connected regions
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < min_area: # Same thing as lab 1 - discard any mask that is too small to count as a sign
                continue
            if not is_pink and is_floor_or_ceiling(x, y, w, h, img_w, img_h): # Light
                continue
            if best is None or area > best[0]:
                best = (area, is_pink, labels == i, (x, y, w, h))
    if best is None:
        return None
    _, is_pink, blob, box = best
    return blob.astype(np.uint8) * 255, is_pink, box # blob = the image we found, is_pink is so we can separate signs into 2 buckets, box = so we can crop on this outline

def crop_square(blob_mask, box):
    x, y, w, h = box
    crop = blob_mask[y:y + h, x:x + w]
    side = max(w, h)
    top, left = (side - h) // 2, (side - w) // 2
    return cv2.copyMakeBorder(crop, top, side - h - top, left, side - w - left, cv2.BORDER_CONSTANT, value=0)

# Turn image to feature vector for KNN
def extract_features(image):
    found = find_sign(image)
    if found is None: # None = class 0 = empty wall
        return None
    blob, is_pink, box = found
    x, y, w, h = box # x, y = row and col of top left corner of box and w, h = width and height of the box

    # crop image and into square AND resize image to 20 x 20 
    shape = cv2.resize(crop_square(blob, box), (SHAPE_SIZE, SHAPE_SIZE), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0 # This rescales values of 0 - 255 to 0 - 1

    area_frac = np.count_nonzero(blob) / blob.size # Ratio of the blob's white pixels to entire image

    # How much of the sign's shape is filled by white pixels coz a solid octagon fills most of its box; a star or curved arrow leaves big empty corners.
    fill = np.count_nonzero(blob[y:y + h, x:x + w]) / float(w * h)

    # Adding extra features now:
    # Color - 6 if pink, 0 if light ; size ; aspect ratio; fill
    extras = [COLOR_WEIGHT * float(is_pink),
              10.0 * np.sqrt(area_frac),   # sqrt so far and near signs are not too far apart
              2.0 * fill,
              2.0 * np.log(w / float(h))]
    return np.concatenate([shape.ravel(), np.array(extras, np.float32)])
