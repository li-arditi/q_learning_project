#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock

from random import randint, random, choice, uniform
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from copy import deepcopy

import math

def all_actions():
    """
    returns a list of tuples where the index is the action number/id
    """
    actions = []
    # db_b_list = []
    dbs = ["red", "green", "blue"]
    blocks = [1, 2, 3]

    
    for db in dbs:
        for b in blocks:
            actions.append((db,b))
    
    return actions

def all_states():
    """
    returns a list of states where index is state number
    """
    states = []
    locations = ["origin", 1, 2, 3]

    for b in locations:
        for g in locations:
            for r in locations:
                states.append([r,g,b])
    
    return states

def create_action_matrix(all_states, all_actions):

    # i = row index = current state
    # j = col index = next state
    # action_matrix[i,j] = action index = action transition from i to j represents
    dbs = ["red", "green", "blue"]
    action_matrix = []


    for i, i_state in enumerate(all_states):
        action_matrix.append([])
        for j, j_state in enumerate(all_states):
            action_matrix[i].append(-1)

            #can't "transition" to same state
            if i == j:
                continue
            # check if next state is valid (can't have same block numbers)
            if ((j_state.count(1) > 1) | (j_state.count(2) > 1) | (j_state.count(3) > 1)):                
                continue

            # compare current state i to next state j
            loc_change = [0, 0, 0]
            for d, loc in enumerate(loc_change):
                if i_state[d] != j_state[d]:
                    loc_change[d] = 1 # location of db d has changed btwn i and j states

            # check if valid transition (can't have 2 dbs change position)
            if sum(loc_change) > 1:
                # invalid next state
                continue

            # if valid transition find action index to use as value
            db_ind = loc_change.index(1)
            db = dbs[db_ind]
            from_loc = i_state[db_ind]
            to_loc = j_state[db_ind]
            
             # can't have db move from block to block
             # and can't move db back to origin
            if from_loc != 'origin':
                continue
            
            if to_loc == 'origin':
                continue
            action_matrix[i][j] = all_actions.index((db, to_loc))
    
    return action_matrix

def get_possible_actions(action_matrix):
    possible_actions = []
    for state in action_matrix:
        # possible_actions.append([])
        state_np = np.array(state)
        state_np = state_np[state_np != -1]
        possible_actions.append(state_np)
    return possible_actions



class QLearning(object):
    def __init__(self):
        # initialize node
        rospy.init_node('q_learning')

        # 

        # publish to /q_learning/q_matrix with message type q_learning_QMatrix each time Q matrix is updated
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # publish to robot_action 
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

        # subscribe to /q_learning_reward
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        self.action = RobotMoveDBToBlock()

        # get all possible actions and states
        self.all_actions = all_actions() # a list 
        self.all_states = all_states() # a list
        self.action_matrix = create_action_matrix(self.all_states, self.all_actions)

        self.possible_actions = get_possible_actions(self.action_matrix) # array of numpy arrays

        self.reward = 0
        self.initialized = False
        #initialize q_matrix
        self.q_matrix = QMatrix()
        # rospy.loginfo(self.q_matrix)

        self.initialize_q_matrix()
        # rospy.loginfo(self.q_matrix)
        # rospy.loginfo(self.q_matrix.q_matrix)

        self.current_q_matrix = None
        self.converged = False
        self.action_seq = []
        self.initialized = True

        self.converge_q_matrix()
        self.converged = True

        self.get_action_sequence()
        
    
    def initialize_q_matrix(self):
        for i in range(0, len(self.all_states)):
            qrow = QMatrixRow()

            for j in range(0, len(self.all_actions)):
                qrow.q_matrix_row.append(0)
            
            self.q_matrix.q_matrix.append(qrow)
            

    
    def get_next_state(self, current_state, a_t):
        next_state = current_state.copy()
        # rospy.loginfo(current_state)
        # rospy.loginfo(next_state)
        db = self.all_actions[a_t][0] # string red, blue, or green
        block = self.all_actions[a_t][1] # int 1, 2, or 3
        if db == "red":
            next_state[0] = block
        elif db == "green":
            next_state[1] = block
        elif db == "blue":
            next_state[2] = block
        
        s_tp1 = self.all_states.index(next_state)

        # rospy.loginfo(current_state)
        # rospy.loginfo(next_state)

        return (s_tp1, next_state)

    
    def converge_q_matrix(self):
        if not self.initialized:
            return

        t = 0
        s_t = 0
        alpha = 1
        gamma = 0.5
        while self.converged == False:
            if self.possible_actions[s_t].size == 0:
                rospy.loginfo(self.all_states[s_t])
                rospy.loginfo("reset")
                s_t = 0
                rospy.sleep(.5)
                continue

            if self.current_q_matrix == None:
                self.current_q_matrix = deepcopy(self.q_matrix)

            if t != 0 and t % 50 == 0:
                rospy.loginfo(t)
                # check if converged
                equal = False
                
                for i, row in enumerate(self.q_matrix.q_matrix):
                    last = np.array(row.q_matrix_row)
                    # rospy.loginfo(last)
                    current = np.array(self.current_q_matrix.q_matrix[i].q_matrix_row)
                    # rospy.loginfo(current)
                    if np.allclose(last, current, atol = 10):
                        equal = True
                        continue
                    else:
                        equal = False
                        break
                
                if equal:
                    rospy.loginfo("converged")
                    self.converged = True
                    return
                
                self.q_matrix = deepcopy(self.current_q_matrix)


            # rospy.loginfo("new iteration")
            
        
            current_state = self.all_states[s_t] # list of location of r, b, g db
            # rospy.loginfo(current_state)
            
            # select action based on current state 
            a_t = choice(self.possible_actions[s_t])
            # rospy.loginfo(a_t)
            
            # Q_sa = self.current_q_matrix.q_matrix[s_t].q_matrix_row[a_t]

            # perform action
            self.action.robot_db = self.all_actions[a_t][0]
            self.action.block_id = self.all_actions[a_t][1]
            self.robot_action_pub.publish(self.action)
            rospy.sleep(1)
            


            # get next state index and value
            (s_tp1, next_state) = self.get_next_state(current_state, a_t)
            # 
            # if self.reward == 100:
            #     rospy.loginfo(current_state)
            #     rospy.loginfo(s_tp1)
            #     rospy.loginfo(next_state)

            

            # update q_matrix and publish
            max_Q_s_tp1 = max(self.current_q_matrix.q_matrix[s_tp1].q_matrix_row)
            # self.current_q_matrix.q_matrix[s_t].q_matrix_row[a_t] = int(Q_sa + alpha * (self.reward + (gamma * max_Q_s_tp1) - Q_sa))
            self.current_q_matrix.q_matrix[s_t].q_matrix_row[a_t] = int(self.reward + (gamma * max_Q_s_tp1))
            # rospy.loginfo(self.current_q_matrix.q_matrix[s_t].q_matrix_row[a_t])
            self.q_matrix_pub.publish(self.current_q_matrix)
            # rospy.sleep(.5)

            
            s_t = s_tp1
            # rospy.loginfo(s_t)
            
            t += 1

            

    def get_reward(self, reward_msg):
        rospy.loginfo(reward_msg.reward)
        self.reward = reward_msg.reward
    
    def get_action_sequence(self):
        if self.converged == False:
            return
        s_t = 0
        while self.possible_actions[s_t].size != 0:
            current_state = self.all_states[s_t]
            max_q = max(self.current_q_matrix.q_matrix[s_t].q_matrix_row)
            action_idx = self.current_q_matrix.q_matrix[s_t].q_matrix_row.index(max_q)
            robot_action = self.all_actions[action_idx]

            # get next state index and value
            (s_tp1, next_state) = self.get_next_state(current_state, action_idx)

            self.action.robot_db = robot_action[0]
            self.action.block_id = robot_action[1]
            rospy.loginfo(self.action.robot_db)
            rospy.loginfo(self.action.block_id)
            self.robot_action_pub.publish(self.action)

            s_t = s_tp1


    
    def run(self):
        rospy.spin()
            






if __name__ == '__main__':
    node = QLearning()
    node.run()



