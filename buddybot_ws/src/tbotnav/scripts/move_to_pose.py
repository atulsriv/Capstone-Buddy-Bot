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

data_fingers = []

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


            elif self.num_fingers == 3: #forward


            elif self.num_fingers == 1: #left


            elif self.num_fingers == 2: #right


            elif self.num_fingers == 4: #back


            elif self.num_fingers == 0: #stop


            else:
                print("done")

            # Publish required message
            self.pub_.publish(self.msg)


        else:
            data_fingers.pop(0)
            #print("data popped", data_fingers.pop(0))
        print("detected",data_fingers)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop HRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    HRControl()
    
