#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock

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

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.arm_joint_goal = [0.0,0.0,0.0,0.0]
        self.gripper_joint_goal = [0.0,0.0]
    
    def move_arm(self):
        self.move_group_arm.go(self.arm_joint_goal, wait=True)
        self.move_group_arm.stop()
    
    def move_gripper(self):
        self.move_group_gripper.go(self.gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
    
    def move_robot(self, data):
        pass

    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

        robot_db = data.robot_db
        block_id = data.block_id













if __name__=="__main__":
    actions = ExecuteRobotActions()

    rospy.spin()