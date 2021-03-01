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
rest_arm_joint_goal = [0.0, -0.386, 1.315, -0.862]

# ready to pick up joint space 0.0, 0.370, 0.523, -0.900
grab_arm_joint_goal = [0.0, 0.370, 0.523, -0.900]

# pick up 0.0, 0.207, 0.253, -0.951 
pickup_arm_joint_goal = [0.0, 0.207, 0.253, -0.951]

# gripper position
gripper_joint_goal = [0.009,0.009]

class ExecuteRobotActions(object):
    def __init__(self):
        # initialize node
        rospy.init_node('execute_actions')
        self.initialized = False


        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.add_action_to_queue)
        # rospy.Subscriber("/scan", LaserScan, self.process_scan)
        # rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)
        rospy.Subscriber("/odom", Odometry, self.get_position)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # put in resting position
        self.move_group_arm.go(rest_arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(3)

        # gripper position
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(3)

        
        self.action_queue = []
        self.yaw = 0
        self.x = 0
        self.y = 0

        # determine location of dbs
        self.db_identified = False
        # self.db_image = rospy.wait_for_message('camera/rgb/image_raw', Image)
        # self.db_locations = robot_perception.identify_dbs(self.db_image)
        self.db_locations = {"red": Point(x=1.0635, y=-0.5, z=0.1905), "blue": Point(x=1.0635, y=0.0, z=0.1905),\
    "green": Point(x=1.0635, y=0.5, z=0.1905)}
        self.db_identified = True
        rospy.loginfo(self.db_locations)
        
        

        # determine location of blocks
        self.block_locations = {}
        
        self.blocks_identified = False
        # self.get_block_locations()
        self.block_locations = {1: Point(x=-2.4, y=-2.0, z=0.4), 2: Point(x=-2.4, y=0.0, z=0.4), \
    3: Point(x=-2.4, y=2.0, z=0.4)}
        rospy.loginfo(self.block_locations)

        self.blocks_identified = True

        self.initialized = True

        self.move_robot()

            
    

    def get_position(self, odom_msg):
        odom_pos = odom_msg.pose.pose.position
        self.x = odom_pos.x
        self.y = odom_pos.y

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
            # rospy.loginfo(images)

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
    
    def move_to_db(self, db_loc):
        (goal_x, goal_y) = (db_loc.x - 0.25, db_loc.y)
        move = Twist()
        r = rospy.Rate(10)
        # move to correct location
        # round(goal_x - self.x, 2) != 0
        while not math.isclose(goal_x, self.x, abs_tol = 0.01) and not math.isclose(goal_y, self.y, abs_tol = 0.01):
            diff_x = goal_x - self.x
            diff_y = goal_y - self.y

            angle_to_goal = math.atan2(diff_y, diff_x)
            # rospy.loginfo(angle_to_goal)
            rospy.loginfo(math.degrees(angle_to_goal - self.yaw))

            if round(angle_to_goal - self.yaw, 1) != 0:
                move.angular.z = 0.4 * (angle_to_goal - self.yaw)
            else:
                move.linear.x = 0.2
                move.angular.z = 0
            # if angle_to_goal < 0:
            #     if abs(angle_to_goal - self.yaw) > 0.1:
            #         move.linear.x = 0
            #         move.angular.z = 0.3
            #     else:
            #         move.linear.x = 0.3
            #         move.angular.z = 0.0
            # else:
            #     if abs(angle_to_goal - self.yaw) > 0.1:
            #         move.linear.x = 0
            #         move.angular.z = -0.3
            #     else:
            #         move.linear.x = 0.3
            #         move.angular.z = 0.0

            
            self.cmd_vel_pub.publish(move)
            r.sleep()
        
        while round(0- self.yaw, 1) != 0:
            move.angular.z = 0.4 * (0 - self.yaw)
            move.linear.x = 0
            self.cmd_vel_pub.publish(move)
            r.sleep()

        rospy.loginfo("got to correct location")
        self.cmd_vel_pub.publish(Twist())


    def move_robot(self):
        """
        source for moving with odom https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
        """
        
        if not self.initialized:
            return 
        self.action_queue.append(RobotMoveDBToBlock(robot_db = "green", block_id = 1))

        


        if len(self.action_queue) > 0:
            take_action = self.action_queue[0] # a RobotMoveDBToBlock message

            robot_db_loc = self.db_locations[take_action.robot_db] # Point()
            block_id_loc = self.block_locations[take_action.block_id] # Point()

            

            # go to in front of dumbbell
            self.move_to_db(robot_db_loc)
            
            # rotate to face dumbbell (if have time change to use camera)
            # while round(0 - self.yaw, 2) != 0:
            #     move.angular.z = 0.4 * (0 - self.yaw)
            #     self.cmd_vel_pub.publish(move)
            #     r.sleep()
            

            # move arm to dumbbell
            

            # move gripper
            

            # lift dumbbell
            self.move_group_arm.go(grab_arm_joint_goal, wait=True)
            self.move_group_arm.stop()
            rospy.sleep(3)

            # move dumbbell to in front of correct block



            # action complete so remove from queue
            self.action_queue.pop(0)













if __name__=="__main__":
    actions = ExecuteRobotActions()

    rospy.spin()