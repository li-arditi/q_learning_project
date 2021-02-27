#!/usr/bin/env python3

import rospy, cv2, cv_bridge

from sensor_msgs.msg import LaserScan, Image

from tf.transformations import quaternion_from_euler, euler_from_quaternion

# position of dbs 
# Point(x=1.0635, y=-0.5, z=0.1905),
# Point(x=1.0635, y=0.0, z=0.1905),
# Point(x=1.0635, y=0.5, z=0.1905) 

class IdentifyDbs(object):
    def __int__(self):
        # initialize node
        rospy.init_node('identify_dbs')

        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)


    def process_image(self, data):
        # pass
        bridge = cv_bridge.CvBridge()
        
        image = bridge.cv2_to_imgmsg(data, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        

        # upper and lower bounds for red
        # using python 3 bgr [0,0,188] = hsv [0, 255, 188]
        lower_red = numpy.array([0, 100, 100]) 
        upper_red = numpy.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        res_red = cv2.bitwise_and(image, image, mask = mask_red)

        # upper and lower bounds for green
        # using python 3 bgr [0,175,0] = hsv [60, 255, 175]
        lower_green = numpy.array([50, 100, 100]) 
        upper_green = numpy.array([70, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        res_green = cv2.bitwise_and(image, image, mask = mask_green)

        # upper and lower bounds for blue
        # using python 3 bgr [176, 0, 17] = hsv [123, 255, 176]
        lower_blue = numpy.array([113, 100, 100])
        upper_blue = numpy.array([133, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        res_blue = cv2.bitwise_and(image, image, mask = mask_blue)

        # SOURCE: https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/
        # kernal = np.ones((5, 5), "uint8") 
      
        # For red db
        # mask_red = cv2.dilate(mask_red, kernal) 
        contours, _ = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # For green db
        # mask_green = cv2.dilate(mask_green, kernal) 
        #  
        
        # For blue db
        # mask_blue = cv2.dilate(mask_blue, kernal) 
        # 

        

        # visualize db locations in the image
        cv2.imshow('window',image)
        # cv2.imshow('mask_red',mask_red)
        cv2.waitKey(3)
        # cv2.destroyAllWindows()

    def process_scan(self, data):
        # data is msg type RobotMoveDBToBlock which has attrs
        # robot_db and block_id
        pass

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('identify_dbs')
    node = IdentifyDbs()
    node.run()