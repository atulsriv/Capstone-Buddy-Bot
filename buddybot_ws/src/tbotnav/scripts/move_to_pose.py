#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import sys
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from math import pi, radians, atan, sqrt, cos, sin

class HRControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):
	
	print("hello")

        # Default values for turtlebot_simulator
        self.speed = 0.2
        # Intializing message type
        self.msg = Twist()

        # initialize node
        rospy.init_node("handgest_control")
        rospy.on_shutdown(self.shutdown)

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribe to kws output
        rospy.Subscriber('num_fingers', Int32, self.num_fingers_callback)
        rospy.spin()

    def num_fingers_callback(self, data):
        self.num_fingers = data.data

	print("detected gesture", self.num_fingers)
        if self.num_fingers == 5: #full speed
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x * 2
                self.msg.angular.z = self.msg.angular.z * 2
                self.speed = 0.4

        elif self.num_fingers == 3: #forward
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0

        elif self.num_fingers == 1: #left
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z -= 0.75
            else:
                self.msg.angular.z = self.speed * 2

        elif self.num_fingers == 2: #right
	    print("right")
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z += 0.75
            else:
                self.msg.angular.z = -self.speed * 2

        elif self.num_fingers == 4: #back
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0


        # Publish required message
        self.pub_.publish(self.msg)


    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop HRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    HRControl()
    
