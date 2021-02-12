#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math


class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_dance')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def run(self):
    
        arm_joint_goal = [0.0,
                    math.radians(5.0),
                    math.radians(10.0),
                    math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        # move gripper
        gripper_joint_goal = [0.009,0.0009] #open
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    


        


if __name__ == '__main__':
    node = Robot()
    node.run()

    #rospy.spin()