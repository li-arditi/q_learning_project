#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# position of dbs
db_locations = {"left": Point(x=1.0635, y=-0.5, z=0.1905), "middle": Point(x=1.0635, y=0.0, z=0.1905),\
    "right": Point(x=1.0635, y=0.5, z=0.1905)}

class IdentifyDbs(object):
    def __init__(self):
        self.initialized = False
        # initialize node
        rospy.init_node('identify_dbs')
        self.identified = False
        self.bridge = cv_bridge.CvBridge()


        # dictionary with color as keys and Point() as values
        self.locations = {"red": Point(), "green": Point(), "blue": Point()}
        rospy.loginfo(self.locations)

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.image = rospy.wait_for_message('camera/rgb/image_raw', Image)
        self.initialized = True

        self.process_image(self.image)


    def process_image(self, data):
        # pass
        if (not self.initialized):
            return

        rospy.loginfo("got image")
        
        
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        masks = {"red": [], "green": [], "blue": []}
        

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
            self.locations[color] = db_locations[location]
        
        self.identified = True
        rospy.loginfo(self.locations)


    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = IdentifyDbs()
    node.run()