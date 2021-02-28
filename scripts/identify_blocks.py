#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import keras_ocr

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# position of blocks:
# Point(x=-2.4, y=-2.0, z=0.4),
# Point(x=-2.4, y=0.0, z=0.4),
# Point(x=-2.4, y=2.0, z=0.4)

class IdentifyBlocks(object):
    def __int__(self):
        # initialize node
        rospy.init_node('identify_blocks')

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)

    def get_block_location(self):
        pass
        pipeline = keras_ocr.pipeline.Pipeline()



    def process_image(self, data):
        pass
        pipeline = keras_ocr.pipeline.Pipeline()
        

    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = IdentifyBlocks()
    node.run()