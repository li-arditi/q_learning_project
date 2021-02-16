#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QMatrix, QLearningReward, RobotMoveDBToBlock

from random import randint, random, choice, uniform
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math

def all_actions():
    actions = {}
    db_b_list = []
    dbs = ["red", "green", "blue"]
    blocks = ["b1", "b2", "b3"]

    
    for db in dbs:
        for b in blocks:
            db_b_list.append((db,b))
    
    for i, a in enumerate(db_b_list):
        actions[a] = i
    
    return actions

def all_states():
    states = {}
    combinations = []
    locations = ["origin", "b1", "b2", "b3"]

    for b in locations:
        for g in locations:
            for r in locations:
                combinations.append([r,g,b])
    
    for i, s in enumerate(combinations):
        states[i] = s
    
    return states

def create_action_matrix(all_states, all_actions):

    # i = row index = current state
    # j = col index = next state
    # action_matrix[i,j] = action index = action transition from i to j represents
    dbs = ["red", "green", "blue"]
    action_matrix = []


    for i, i_state in all_states.items():
        action_matrix.append([])
        for j, j_state in all_states.items():
            action_matrix[i].append(-1)
            # check if next state is valid (can't have same block numbers)
            if ((j_state.count('b1') > 1) | (j_state.count('b2') > 1) | (j_state.count('b3') > 1)):
                #action_matrix[i][j] = -1
                
                continue

            # compare current state i to next state j
            loc_change = [0, 0, 0]
            for d, loc in enumerate(loc_change):
                if i_state[d] != j_state[d]:
                    loc_change[d] = 1 # location of db d has changed btwn i and j states

            # check if valid transition (can't have 2 dbs change position)
            if sum(loc_change) > 1:
                # invalid next state
                #action_matrix[i][j] = -1
                
                continue

            # if valid transition find action index to use as value
            if sum(loc_change) == 0:
                #action_matrix[i][j] = -1
                
                continue

            db_ind = loc_change.index(1)
            db = dbs[db_ind]
            block = j_state[db_ind]
            if block == 'origin':
                print("error is here with i,j", i,j)
                continue
            action_matrix[i][j] = all_actions[(db, block)]
    
    return action_matrix



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

        # get all possible actions and states
        self.all_actions = all_actions() # a dictionary
        self.all_states = all_states() # a dictionary
        # self.action_matrix = create_action_matrix()



    def process_scan(self, data):
        pass








