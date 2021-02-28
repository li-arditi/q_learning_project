#!/usr/bin/env python3

import rospy, cv2, cv_bridge, math

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock

from tf.transformations import quaternion_from_euler, euler_from_quaternion


from q_learning import QLearning
import robot_perception
# from identify_dbs import IdentifyDbs
# from identify_blocks import IdentifyBlocks

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
        self.initialized = False

        # get the converged q_matrix
        # self.q_matrix = self.QLearning.q_matrix.q_matrix
        # self.all_actions = self.QLearning.all_actions

        # get action sequence
        # self.QLearning = QLearning()
        # self.action_seq = self.QLearning.action_seq # list of action ids

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.add_action_to_queue)
        # rospy.Subscriber("/scan", LaserScan, self.process_scan)
        # rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)
        rospy.Subscriber("/odom", Odometry, self.get_position)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # self.arm_joint_goal = [0.0,0.0,0.0,0.0]
        # self.gripper_joint_goal = [0.0,0.0]

        self.action_queue = []

        # determine location of dbs
        self.db_identified = False
        self.db_image = rospy.wait_for_message('camera/rgb/image_raw', Image)
        self.db_locations = robot_perception.identify_dbs(self.db_image)
        # self.redDB = self.db_locations["red"]
        # self.greenDB = self.db_locations["green"]
        # self.blueDB = self.db_locations["blue"]
        # rospy.loginfo(self.redDB)
        # 
        self.db_identified = True
        rospy.loginfo(self.db_locations)
        
        self.yaw = 0
        self.x = 0
        self.y = 0

        # determine location of blocks
        self.block_locations = {}
        # self.block1 = self.block_locations.blocks[1]
        # self.block2 = self.block_locations.blocks[2]
        # self.block3 = self.block_locations.blocks[3]
        
        # self.identify_locations = IdentifyDbs()
        # self.get_db_locations()

        # self.block_locations = IdentifyBlocks()
        self.blocks_identified = False
        self.get_block_locations()
        rospy.loginfo(self.block_locations)

        self.blocks_identified = True

        self.move_robot()

            
    
    def get_db_locations(self):
        pass

    def get_position(self, odom_msg):
        odom_quat = odom_msg.pose.pose.orientation
        self.yaw = euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])[2]
        

    def get_block_locations(self):
        if not self.db_identified:
            return
        angles = [-135, -180, 140]
        images = []
        for ang in angles:
            # for each angle (left, middle, right) get an image
            self.rotate(ang)

            get_image = rospy.wait_for_message('camera/rgb/image_raw', Image)
            images.append(get_image)
            rospy.loginfo(images)
            # p = Pose()
            # quat = quaternion_from_euler(0.0, 0.0, math.radians(ang))
            # p.orientation.x = quat[0]
            # p.orientation.y = quat[1]
            # p.orientation.z = quat[2]
            # p.orientation.w = quat[3]
        rospy.loginfo(len(images))
        self.block_locations = robot_perception.identify_blocks(images)
        

            
            

    def rotate(self, ang):
        r = rospy.Rate(10)
        cmd =Twist()

        while not rospy.is_shutdown() and round(math.radians(ang)-self.yaw, 2) != 0:
            cmd.angular.z = -0.4 * abs(math.radians(ang)-self.yaw)
            self.cmd_vel_pub.publish(cmd)
            r.sleep()






        


    def add_action_to_queue(self, action_msg):
        self.action_queue.append(action_msg)
    


    def move_robot(self):
        pass
        if not self.blocks_identified:
            return 

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