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
rest_arm_joint_goal = [0.0, -0.386, 1.280, -0.915]

# ready to pick up joint space 0.0, 0.370, 0.523, -0.900
grab_arm_joint_goal = [0.0, 0.370, 0.523, -0.900]

# pick up 0.0, 0.207, 0.253, -0.951 
pickup_arm_joint_goal = [0.0, 0.207, 0.253, -0.951]

# gripper position
gripper_joint_goal = [0.015,0.015]

class ExecuteRobotActions(object):
    def __init__(self):
        # initialize node
        rospy.init_node('execute_actions')
        self.initialized = False


        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.move = Twist()

        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.add_action_to_queue)
        # rospy.Subscriber("/scan", LaserScan, self.process_scan)
        # rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)
        rospy.Subscriber("/odom", Odometry, self.get_position)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # put in resting position
        self.move_group_arm.go(rest_arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        # rospy.sleep(3)

        # gripper position
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        # rospy.sleep(3)

        
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
        while not rospy.is_shutdown() and round(math.radians(ang)-self.yaw, 2) != 0:
            self.move.angular.z = -0.4 * abs(math.radians(ang)-self.yaw)
            self.cmd_vel_pub.publish(self.move)
            r.sleep()   


    def add_action_to_queue(self, action_msg):
        self.action_queue.append(action_msg)
    
    def move_to_db(self, db_loc):
        (goal_x, goal_y) = (db_loc.x - 0.18, db_loc.y)
        at_location = False
        
        r = rospy.Rate(20)
        # move to correct location
        # round(goal_x - self.x, 2) != 0
        while not at_location:
            rospy.loginfo("yaw")
            rospy.loginfo(self.yaw)
            diff_x = goal_x - self.x
            diff_y = goal_y - self.y

            angle_to_goal = math.atan2(diff_y, diff_x)
            rospy.loginfo("angle to goal")
            rospy.loginfo(angle_to_goal)
            # rospy.loginfo(math.degrees(angle_to_goal - self.yaw))
            # rospy.loginfo(round(angle_to_goal - self.yaw, 1))
            if round(angle_to_goal - self.yaw, 2) != 0 and not math.isclose(goal_y, self.y, abs_tol = 0.01):
                # rospy.loginfo("angular z")
                # rospy.loginfo(0.4 * (angle_to_goal - self.yaw))
                self.move.angular.z = 0.4 * (angle_to_goal - self.yaw)
                # move.linear.x = 0
            elif not math.isclose(goal_y, self.y, abs_tol = 0.01):
                self.move.linear.x = 0.4 * abs(goal_y - self.y)
                self.move.angular.z = 0
                # rospy.loginfo("linear x")
                # rospy.loginfo(0.4 * abs(goal_y - self.y))

            elif round(0 - self.yaw, 1) != 0:
                self.move.angular.z = 0.4 * (0 - self.yaw)
                self.move.linear.x = 0
            else:
                self.cmd_vel_pub.publish(Twist())
                at_location = True

            self.cmd_vel_pub.publish(self.move)
            r.sleep()
        
        # self.cmd_vel_pub.publish(Twist())

        rospy.loginfo("got to correct location")
        
        
    
    def place_at_block(self, block_loc):
        
        at_location = False
        (goal_x, goal_y) = (block_loc.x + 1, block_loc.y)
        diff_x = goal_x - self.x
        diff_y = goal_y - self.y

        goal_angle = math.atan2(diff_y, diff_x)
        
        r = rospy.Rate(20) 
        while not at_location:
            rospy.loginfo("yaw")
            rospy.loginfo(self.yaw)
            rospy.loginfo("goal angle")
            rospy.loginfo(goal_angle)
            if round(goal_angle - self.yaw, 1) != 0 and not math.isclose(goal_y, self.y, abs_tol = 0.01):
                self.move.angular.z = 0.4 * (goal_angle - self.yaw)

            elif not math.isclose(goal_y, self.y, abs_tol = 0.01):
                self.move.linear.x = 0.5 * abs(goal_y - self.y)
                self.move.angular.z = 0
                # rospy.loginfo("linear x")
                # rospy.loginfo(0.4 * abs(goal_y - self.y))

            elif round(math.radians(-180) - self.yaw, 1) != 0:
                self.move.angular.z = 0.4 * (math.radians(-180) - self.yaw)
                self.move.linear.x = 0
            else:
                self.cmd_vel_pub.publish(Twist())
                at_location = True

            self.cmd_vel_pub.publish(self.move)
            r.sleep()



    def move_robot(self):
        """
        source for moving with odom https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
        """
        
        if not self.initialized:
            return 
        self.action_queue.append(RobotMoveDBToBlock(robot_db = "green", block_id = 3))

        


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
            self.move_group_arm.go(grab_arm_joint_goal, wait=True)
            self.move_group_arm.stop()

            # lift dumbbell
            self.move_group_arm.go(pickup_arm_joint_goal, wait=True)
            self.move_group_arm.stop()

            # move dumbbell to in front of correct block
            self.place_at_block(block_id_loc)

            # place dumbbell at block
            self.move_group_arm.go(grab_arm_joint_goal, wait=True)
            self.move_group_arm.stop()

            self.move_group_arm.go(rest_arm_joint_goal, wait=True)
            self.move_group_arm.stop()

            # while self.x < -1.60:
            #     self.cmd_vel_pub.publish(Twist(linear=Vector3(x=-.08)))
            # self.cmd_vel_pub.publish(Twist())


            # action complete so remove from queue
            self.action_queue.pop(0)













if __name__=="__main__":
    actions = ExecuteRobotActions()

    rospy.spin()