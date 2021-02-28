#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy, keras_ocr

from geometry_msgs.msg import Point

# will have functions to determine locations of db and blocks
# bds function given initial image/position
# blocks function takes in image (execute movement rotates robot 
# to appropriate angles) and determines which block it's looking at
# calls to db functions return a dictionary where key is the db
# and value is the Point() location
# call to block function returns the block number of block in image

# position of dbs
db_locations = {"left": Point(x=1.0635, y=-0.5, z=0.1905), "middle": Point(x=1.0635, y=0.0, z=0.1905),\
    "right": Point(x=1.0635, y=0.5, z=0.1905)}

# position of blocks
block_locations = {"left": Point(x=-2.4, y=-2.0, z=0.4), "middle": Point(x=-2.4, y=0.0, z=0.4), \
    "right": Point(x=-2.4, y=2.0, z=0.4)}


def identify_dbs(image):
    """
    Takes in an Image message and identifies the locations of the three dumbbells
    Returns: dictionary with key = dumbbell/color, value = Point() location
    """
    locations = {"red": Point(), "green": Point(), "blue": Point()}
    masks = {"red": [], "green": [], "blue": []}

    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(image, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # upper and lower bounds for red
    # using python 3 bgr [0,0,188] = hsv [0, 255, 188]
    lower_red = numpy.array([0, 100, 100]) 
    upper_red = numpy.array([10, 255, 255])
    masks["red"] = cv2.inRange(hsv, lower_red, upper_red)

        # upper and lower bounds for green
    # using python 3 bgr [0,175,0] = hsv [60, 255, 175]
    lower_green = numpy.array([50, 100, 100]) 
    upper_green = numpy.array([70, 255, 255])
    masks["green"] = cv2.inRange(hsv, lower_green, upper_green)

    # upper and lower bounds for blue
    # using python 3 bgr [176, 0, 17] = hsv [123, 255, 176]
    lower_blue = numpy.array([113, 100, 100])
    upper_blue = numpy.array([133, 255, 255])
    masks["blue"] = cv2.inRange(hsv, lower_blue, upper_blue)

    x, y, w, h = 0, 0, image.shape[1]//3, image.shape[0]

    for color, mask in masks.items():
        pixels = {"left": 0, "middle": 0, "right": 0}
        
        left = mask[y:y+h, x:x+w]
        middle = mask[y:y+h, x+w:x+w+w]
        right = mask[y:y+h, x+w+w:x+3*w]
        pixels["left"] = cv2.countNonZero(left)
        pixels["middle"] = cv2.countNonZero(middle)
        pixels["right"] = cv2.countNonZero(right)
        location = max(pixels, key=pixels.get)
        locations[color] = db_locations[location]
    
    return locations


def identify_blocks(images):
    """
    Takes in a list of Image messages and identifies the block number in each image
    Returns: dictionary with key = block id/number, value = Point() location
    """
    locations = {1: Point(), 2: Point(), 3: Point()}
    blocks = {"left": 0, "middle": 0, "right": 0}
    pipeline = keras_ocr.pipeline.Pipeline()
    cv2_images = []

    for image in images:
        bridge = cv_bridge.CvBridge()
        cv2_images.append(bridge.imgmsg_to_cv2(image, "bgr8"))

    predictions = pipeline.recognize(cv2_images)

    blocks["left"] = int(predictions[0][0][0])
    blocks["middle"] = int(predictions[1][0][0])
    blocks["right"] = int(predictions[2][0][0])

    for position, block in blocks.items():
        locations[block] = block_locations[position]
    
    return locations




