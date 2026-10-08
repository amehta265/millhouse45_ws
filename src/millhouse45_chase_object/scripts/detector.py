#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Date: 09/10/2026 (updated for lab 3)
Porting this over from lab 2, which actually used the lab 1 implementation and tweaked it a little.
Lab 3 changes: thresholds sized for the robot's 320x240 camera, and we pick the ROUNDEST red blob
instead of the BIGGEST one, so red/brown/pink/maroon stuff in the background (legs, clothes) doesn't win.
"""

import math
import cv2
import numpy as np

# Defining HSV constants that help distinguish red colored object in a picture
# Obtained these values via trial and error and visualizations (use hsv_tuner.py on the laptop to re-tune)
# Red color falls within two "Hue" bands 0 - 10 and 170 - 179. Therefore in order to capture both
# We define 2 H sections and later combine them with a bitwise OR operator
HSV = {
    'h1_lo': 0,   'h1_hi': 7,
    'h2_lo': 170, 'h2_hi': 179,
    's_lo': 80,   's_hi': 255,    # Saturation - intensity. Raise s_lo to reject washed-out pink/skin
    'v_lo': 88,   'v_hi': 180,    # Value - how bright/dark. Raise v_lo to reject dark maroon/brown
}

# Smallest blob we'll call a ball, in pixels. On the 320x240 robot camera a palm sized ball is about
# ~1000 px at 0.5 m, ~270 px at 1 m and ~70 px at 2 m. (The old 300 was tuned for the 640x480 laptop webcam
# and was throwing the ball away past ~0.9 m)
MIN_AREA = 50

# Shape checks - this is what stops legs, arms, clothes from being picked even if they're red-ish.
# We measure the shape of the blob's OUTLINE with the dents filled in (its "convex hull" - imagine a rubber band
# stretched around the blob). The ball's white logo, our fingers and its shadowed side take bites out of the
# red area, but the rubber band around what's left is still roughly round. A leg or an arm stays long and thin.
# Measured on frames from our test video: ball 0.73-0.95 / 0.46-0.81, leg/arm shapes <= 0.57 / <= 0.36
#
# circularity = 4*pi*area / perimeter^2 -> 1.0 for a perfect circle, long thin things (legs) ~0.3-0.5
MIN_CIRCULARITY = 0.65
# fill = outline area / area of the smallest circle around it -> ball ~0.45-0.8, stretched blobs ~0.1-0.35
MIN_FILL = 0.4

# takes in a frame and spits out an image where the object being detected is white in color
# and then rest of the background is black
def create_mask(frame, hsv=HSV):
    # perform some smoothing using a gaussian blur. Smaller kernel than lab 1 since the image is 4x smaller
    blurred_img = cv2.GaussianBlur(frame, (5, 5), 0)
    # convert the img from RGB to HSV. Two reasons: First HSV holds up better in diff kinds of lighting
    # Second it is easy to perform thresholding as I only have to tune the "Hue" for color vs tuning R, G, B
    hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

    # Two ranges
    lower_range_1 = np.array([hsv['h1_lo'], hsv['s_lo'], hsv['v_lo']])
    upper_range_1 = np.array([hsv['h1_hi'], hsv['s_hi'], hsv['v_hi']])
    lower_range_2 = np.array([hsv['h2_lo'], hsv['s_lo'], hsv['v_lo']])
    upper_range_2 = np.array([hsv['h2_hi'], hsv['s_hi'], hsv['v_hi']])

    kerneled_img = cv2.bitwise_or(cv2.inRange(hsv_img, lower_range_1, upper_range_1),
                                  cv2.inRange(hsv_img, lower_range_2, upper_range_2))

    # Opening which is erosion + dilation. Removes little specks of noise.
    # 3x3 instead of 5x5 - at 320x240 a 5x5 opening was eating a far-away ball
    kerneled_img = cv2.morphologyEx(kerneled_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    # Closing - dilation then erosion. Fills in the white writing on the ball.
    # 7x7 instead of 15x15 - 15 was so big it glued the ball to nearby red-ish stuff (hand, legs) into one blob,
    # and then that blob isn't round anymore. We only look at the outer edge of each blob anyway,
    # so holes in the middle from the writing don't matter much
    kerneled_img = cv2.morphologyEx(kerneled_img, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return kerneled_img

# Finds every red blob and measures how ball-like it is. Returns the mask + a list of candidates
def find_candidates(frame, hsv=HSV):
    masked_img = create_mask(frame, hsv)

    # Only want external boundaries of found objects
    contours, _ = cv2.findContours(masked_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue

        # Rubber band around the blob - fills in the bites from the logo / fingers / shadow
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        hull_perimeter = cv2.arcLength(hull, True)
        if hull_perimeter == 0:
            continue
        circularity = 4.0 * math.pi * hull_area / (hull_perimeter * hull_perimeter)

        (x, y), radius = cv2.minEnclosingCircle(hull)
        fill = hull_area / (math.pi * radius * radius) if radius > 0 else 0.0

        candidates.append({
            'x': x, 'y': y, 'r': radius, 'area': area,
            'circularity': circularity, 'fill': fill,
            'is_ball_shaped': circularity >= MIN_CIRCULARITY and fill >= MIN_FILL,
            # How ball-like it is. Roundness first; area only breaks ties between equally round blobs
            'score': circularity * fill,
            'contour': c,
        })
    return masked_img, candidates

# Picks the ball out of the candidates.
# previous = (x, y, r) of the ball in the last frame, or None. If we saw the ball last frame we prefer
# the round blob closest to where it was - the ball can't teleport across the image in 1/30 of a second,
# but a red-ish chair leg can suddenly look round for one frame
def pick_ball(candidates, previous=None):
    round_ones = [c for c in candidates if c['is_ball_shaped']]
    if not round_ones:
        return None

    if previous is not None:
        px, py, pr = previous
        # How far the ball is allowed to move between frames (pixels). Generous - scales with ball size
        max_jump = 40 + 3 * pr
        nearby = [c for c in round_ones if math.hypot(c['x'] - px, c['y'] - py) <= max_jump]
        if nearby:
            return min(nearby, key=lambda c: math.hypot(c['x'] - px, c['y'] - py))

    # No history (or nothing near the old spot) -> roundest blob wins, bigger one if it's a tie
    return max(round_ones, key=lambda c: (round(c['score'], 2), c['area']))

# Same interface as lab 2 so old code still works: returns (x, y), radius, mask
def detect_object(frame, previous=None, hsv=HSV):
    masked_img, candidates = find_candidates(frame, hsv)
    ball = pick_ball(candidates, previous)
    if ball is None:
        # Didn't find anything
        return None, None, masked_img
    return (ball['x'], ball['y']), ball['r'], masked_img

# Draws what the detector saw: green = picked ball, red = red blob rejected for its shape.
# Returns frame and mask side by side so disp_cam shows both
def draw_debug(frame, masked_img, candidates, ball, label):
    out = frame.copy()
    for c in candidates:
        if ball is not None and c is ball:
            continue
        cv2.drawContours(out, [c['contour']], -1, (0, 0, 255), 1)
        cv2.putText(out, f"{c['circularity']:.2f}/{c['fill']:.2f}", (int(c['x']) - 20, int(c['y'])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    if ball is not None:
        cv2.circle(out, (int(ball['x']), int(ball['y'])), int(ball['r']), (0, 255, 0), 2)
        cv2.circle(out, (int(ball['x']), int(ball['y'])), 2, (0, 255, 0), -1)
    cv2.putText(out, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    mask_bgr = cv2.cvtColor(masked_img, cv2.COLOR_GRAY2BGR)
    return np.hstack([out, mask_bgr])
