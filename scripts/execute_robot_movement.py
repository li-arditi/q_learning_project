#!/usr/bin/env python3

import rospy, cv2, cv_bridge

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import moveit_commander
from msg import RobotMoveDBToBlock

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# subscribe to /q_learning/robot_action topic
# that topic has custom message q_learning/RObotMoveDBToBlock
# the msg has attributes robot_db and block_id



class ExecuteRobotActions(object):
    def __init__(self):
        # initialize node
        rospy.init_node('execute_actions')

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.move_robot)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.process)image)

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # self.arm_joint_goal = [0.0,0.0,0.0,0.0]
        # self.gripper_joint_goal = [0.0,0.0]
    
            
    
    def get_db_location(self, robot_db):
        pass

    def get_block_location(self, block_id):
        pass

    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

        
    
    def process_image(self, msg):
        pass
        img = cv2.imread(filename)
        bridge = cv_bridge.CvBridge()
        imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")



    def move_robot(self, msg):
        pass
        robot_db = msg.robot_db
        block_id = msg.block_id

        # determine location of dumbell and block
        db_location = self.get_db_location(robot_db)
        block_location = self.get_block_location(block_id)

        # go to in front of dumbbell

        # move arm to dumbbell
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        # move gripper
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # lift dumbbell
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()













if __name__=="__main__":
    actions = ExecuteRobotActions()

    rospy.spin()