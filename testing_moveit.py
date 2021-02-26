#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_dance')
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def run(self):
        r = rospy.Rate(5)
        self.twist.angular.z = math.radians(45)
        for x in range(0,20):
            self.cmd_vel_pub.publish(self.twist)
            r.sleep()
        self.cmd_vel_pub.publish(Twist())

    
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