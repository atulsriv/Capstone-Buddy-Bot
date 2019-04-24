#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import sys
import numpy as np 
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from math import pi, radians, atan, sqrt, cos, sin

data_fingers = []

NAVI_MAX_LIN_VEL = 5.0
NAVI_MAX_ANG_VEL = 5.0

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 3.0
control_angular_vel = 3.0

command_speed = 1.0 # initialized to 1 m/s
navi_lock = 1

class HRControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):
        self.num_fingers = 0
	
	print("hello")

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
        global data_fingers
        if(len(data_fingers)<5):
            #print()
            data_fingers.append(self.num_fingers)

        elif((len(data_fingers)==5) and (len(set(data_fingers)) == 1)):
            #print("detected",data_fingers)
            data_fingers.pop(0)
            
            if self.num_fingers == 5: #full speed
                print('full speed')
                command_speed = NAVI_MAX_LIN_VEL
                print("Changing speed to max.")
                #change speed to 5
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif self.num_fingers == 3: #forward
                t_end = time.time() + 2
                print("forward")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)

            elif self.num_fingers == 1: #left
                print('quarter left turn')
                t_end = time.time() + 1.4
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = 18
                    pub.publish(twist)

            elif self.num_fingers == 2: #right
                print('quarter right turn')
                t_end = time.time() + 1.4
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = -18
                    pub.publish(twist)

            elif self.num_fingers == 4: #back
                t_end = time.time() +  2
                print("back")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = -command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)

            elif self.num_fingers == 0: #stop
                twist = Twist()
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
                pub.publish(twist)

            else:
                print("done")

        else:
            data_fingers.pop(0)
            #print("data popped", data_fingers.pop(0))
        print("detected",data_fingers)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop HRControl")
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == "__main__":
    HRControl()
    
