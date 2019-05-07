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

NAVI_MAX_LIN_VEL = 7.0
NAVI_MAX_ANG_VEL = 7.0

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
	
	print("HRControl Started")

        # initialize node
        
        rospy.on_shutdown(self.shutdown)

        # Initializing publisher with buffer size of 10 messages
        

        # Subscribe to kws output
        rospy.Subscriber('num_fingers', Int32, self.num_fingers_callback)
        rospy.spin()
        

    def num_fingers_callback(self, data):
        self.num_fingers = data.data
        global data_fingers
        global target_linear_vel
        global target_angular_vel
        global control_linear_vel
        global navi_lock #1:locked, 0:unlocked
        global command_speed
        
        if(len(data_fingers)<5):
            #print()
            data_fingers.append(self.num_fingers)
            print("I have less than 5 repeats")

        elif((len(data_fingers)==5) and (len(set(data_fingers)) == 1)):
            # max(data_fingers,key = data_fingers.count)   # this gets the maximum occurence of any element in a list but we don't need that. 
            #print("detected",data_fingers)
            x = data_fingers.pop(0)
            navi_lock = 1 # locked

            if x == 5: #full speed
                print('full speed')
                command_speed = NAVI_MAX_LIN_VEL
                print("Changing speed to max.")
                #change speed to 5
                data_fingers = [0, 0, 0, 0, 0]
                navi_lock = 0
            elif x == 3: #forward
                navi_lock = 1

                t_end = time.time() + 4
                print("forward")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)
                data_fingers = [0, 0, 0, 0, 0]

            elif x == 1: #left
                navi_lock = 1
                print('quarter left turn')
                t_end = time.time() + 3
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = 13
                    pub.publish(twist)
                data_fingers = [0, 0, 0, 0, 0]
            elif x == 2: #right
                navi_lock = 1
                print('quarter right turn')
                t_end = time.time() + 3
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = -13
                    pub.publish(twist)
                data_fingers = [0, 0, 0, 0, 0]
            elif x == 4: #back
                navi_lock = 1
                t_end = time.time() +  4
                print("back")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = -command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)
                data_fingers = [0, 0, 0, 0, 0]
            elif x == 0: #stop
                twist = Twist()
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                twist.linear.x = 0
                twist.angular.z = 0
                pub.publish(twist)

            else:
                print("idk how i got here... not technically possible...")

        else:
            data_fingers.pop(0)
            print("get rid of first")
            #print("data popped", data_fingers.pop(0))
        print("detected",data_fingers)
        # time.sleep(1)
        

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop HRControl")
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == "__main__":
    print("HRControl Started")
    rospy.init_node("handgest_control")
    pub = rospy.Publisher("/diff_controller/cmd_vel", Twist, queue_size=10)
    HRControl()
    print("Done...")
    
