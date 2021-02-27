#!/usr/bin/env python3

import rospy, cv2, cv_bridge

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock

from tf.transformations import quaternion_from_euler, euler_from_quaternion


from q_learning import QLearning
from identify_dbs import IdentifyDbs
from identify_blocks import IdentifyBlocks

# subscribe to /q_learning/robot_action topic
# that topic has custom message q_learning/RObotMoveDBToBlock
# the msg has attributes robot_db and block_id

# start 0.0, -0.386, 1.315, -0.862 
# ready to pick up joint space 0.0, 0.370, 0.523, -0.900
# pick up 0.0, 0.207, 0.253, -0.951 

class ExecuteRobotActions(object):
    def __init__(self):
        # initialize node
        rospy.init_node('execute_actions')

        # get the converged q_matrix
        # self.q_matrix = self.QLearning.q_matrix.q_matrix
        # self.all_actions = self.QLearning.all_actions

        # get action sequence
        self.QLearning = QLearning()
        # self.action_seq = self.QLearning.action_seq # list of action ids

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.add_action_to_queue)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        # rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)

        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # self.arm_joint_goal = [0.0,0.0,0.0,0.0]
        # self.gripper_joint_goal = [0.0,0.0]

        self.action_queue = []

        # determine location of dbs
        self.db_locations = IdentifyBlocks()
        self.redDB = self.db_locations.redDB
        self.greenDB = self.db_locations.greenDB
        self.blueDB = self.db_locations.blueDB


        # determine location of blocks
        self.block_locations = IdentifyBlocks()
        self.block1 = self.block_locations.block1
        self.block2 = self.block_locations.block2
        self.block3 = self.block_locations.block3

        self.move_robot()


    
            
    
    # def get_db_location(self, robot_db):
    #     pass

    # def get_block_location(self, block_id):
    #     pass
    def add_action_to_queue(self, action_msg):
        self.action_queue.append(action_msg)
    

    def move_robot(self):
        pass
        if len(self.action_queue) > 0:
            take_action = self.action_queue[0] # a RobotMoveDBToBlock message

            robot_db = take_action.robot_db
            block_id = take_action.block_id

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

            # move dumbbell to in front of correct block



            # action complete so remove from queue
            self.action_queue.pop(0)













if __name__=="__main__":
    actions = ExecuteRobotActions()

    rospy.spin()