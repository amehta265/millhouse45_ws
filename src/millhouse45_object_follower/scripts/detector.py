#!/usr/bin/env python3
"""
Brian Huntley
Ankit Mehta
Date: 09/10/2026
"""

import cv2
import numpy as np

# Defining HSV constants that help distinguish red colored object in a picture
# Obtained these values via trial and error and visualizations
# Red color falls within two "Hue" bands 0 - 10 and 170 - 179. Therefore in order to capture both
# We define 2 H sections and later combine them with a bitwise OR operator

H1_LO, H1_HI = 0, 7        
H2_LO, H2_HI = 170, 179    
S_LO, S_HI = 80, 255    # Saturation - intensity
V_LO, V_HI = 88, 180    # value - how bright/dark the color is

# takes in a frame and spits out an image where the object being detected is white in color
# and then rest of the background is black
def create_mask(frame):
    # perform some smoothing using a gaussian blur
    blurred_img = cv2.GaussianBlur(frame, (11,11), 0)
    # convert the img from RGB to HSV. Two reasons: First HSV holds up better in diff kinds of lighting
    # Second it is easy to perform thresholding as I only have to tune the "Hue" for color vs tuning R, G, B
    hsv_img = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
    # Use the tuned params and perform a "kernel" like operation on the hsv image
    # Two ranges
    lower_range_1 = np.array([H1_LO, S_LO, V_LO])
    lower_range_2 = np.array([H2_LO, S_LO, V_LO])
    upper_range_1 = np.array([H1_HI, S_HI, V_HI])
    upper_range_2 = np.array([H2_HI, S_HI, V_HI])

    kerneled_img = cv2.bitwise_or(cv2.inRange(hsv_img, lower_range_1, upper_range_1),
                                  cv2.inRange(hsv_img, lower_range_2, upper_range_2))

    # Now we perform more "advanced" convolutions
    # Opening which is erosion + dilation. It helps remove background noise e.g. if there are "white" dots in my
    # background but I only want the red ball to be white, this should help with that
    kerneled_img = cv2.morphologyEx(kerneled_img, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    # closing  - dilation then opening. We need this. The object I am tracking (spherical red ball)
    # has white colored writing in it. I need these writings to be "masked" and not appear in the foreground object
    kerneled_img = cv2.morphologyEx(kerneled_img, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return kerneled_img

def detect_object(frame):
    masked_img = create_mask(frame)

    # Now we leverage the fact that a sphere is circular to confidently find the ball
    # Can use houghCircles or findContours (which finds the nearest neighbors on a given edge)
    # According to the interent houghCircles can be slow 
    
    # Only want external boundaries of found objects
    # don't need to store all boundary points so CHAIN_APPROX_SIMPLE leaves only horizontal/vertical/diagonal end points
    contours, _ = cv2.findContours(masked_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Now we need to obtain OUR circle (cause there can be other circles in the image)
    best_circle = None
    best_area = 0.0
    for c in contours:
        # We assume that our ball will appear bigger than a certain size
        # so we disregard circular contours small than that size
        area = cv2.contourArea(c)
        if area < 300:
            continue
        if area > best_area:
            best_circle = c
            best_area = area

    if best_circle is None:
        # Didn't find anything
        return None, None, masked_img

    # Get the coordinates of the circle
    (x, y), radius = cv2.minEnclosingCircle(best_circle)
    return (x,y), radius, masked_img
