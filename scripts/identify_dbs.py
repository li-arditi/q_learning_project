#!/usr/bin/env python3

import rospy, cv2, cv_bridge

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock

from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RobotPerception(object):
    def __int__(self):
        # initialize node
        rospy.init_node('identify_dbs_blocks')

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)

    def get_block_location(self):
        pass
        pipeline = keras_ocr.pipeline.Pipeline()



    def process_image(self, data):
        pass
        bridge = cv_bridge.CvBridge()
        
        image = bridge.cv2_to_imgmsg(data, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # upper and lower bounds for red hsv [0,229,135]
        lower_red = numpy.array([81, 100, 100]) #TODO
        upper_red = numpy.array([40, 255, 255]) #TODO
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # upper and lower bounds for green hsv [81, 227, 135]
        lower_green = numpy.array([20, 100, 100]) #TODO
        upper_green = numpy.array([40, 255, 255]) #TODO
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # upper and lower bounds for blue hsv [160,225,135]
        lower_blue = numpy.array([20, 100, 100]) #TODO
        upper_blue = numpy.array([40, 255, 255]) #TODO
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)


    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

   