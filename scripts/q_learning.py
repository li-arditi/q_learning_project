#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveDBToBlock

from random import randint, random, choice, uniform
from tf.transformations import quaternion_from_euler, euler_from_quaternion



class QLearning(object):
    def __init__(self):
        # initialize node
        rospy.init_node('q_learning')

        # publish to /q_learning/q_matrix with message type q_learning_QMatrix each time Q matrix is updated
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # publish to robot_action 
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # subscribe to /q_learning_reward
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

    





    def process_scan(self, data):
        pass












if __name__=="__main__":
    ql = QLearning()

    rospy.spin()