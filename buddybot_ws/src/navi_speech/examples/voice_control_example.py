#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

NAVI_MAX_LIN_VEL = 5.0
NAVI_MAX_ANG_VEL = 5.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

status = 0
target_linear_vel   = 3.0
target_angular_vel  = 0.0
control_linear_vel  = 3.0
control_angular_vel = 0.0

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    return constrain(vel, -NAVI_MAX_LIN_VEL, NAVI_MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -NAVI_MAX_ANG_VEL, NAVI_MAX_ANG_VEL)


class ASRControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):
	
	print("hello")


        # Default values for turtlebot_simulator
        # self.speed = 0.2
        # Intializing message type
        # self.msg = Twist()

        # initialize node
        rospy.init_node("asr_control")
        rospy.on_shutdown(self.shutdown)

        # Initializing publisher with buffer size of 10 messages
        # self.pub_ = rospy.Publisher("/diff_controller/cmd_vel", Twist, queue_size=10)
        
        # Subscribe to kws output
        rospy.Subscriber("kws_data", String, self.parse_asr_result)
        rospy.spin()

    def parse_asr_result(self, detected_words): #pylint: disable=too-many-branches
        
        
        """Function to perform action on detected word"""
    	print("detected words", detected_words.data)

        if detected_words.data.find("full speed") > -1:
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + NAVI_MAX_LIN_VEL)

            print(target_linear_vel)
            
            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = 5; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            print(target_linear_vel)
            pub.publish(twist)
            print("test: " + str(target_linear_vel))
 #    elif detected_words.data.find("half speed") > -1:
 #        if self.speed == 0.4:
 #            self.msg.linear.x = self.msg.linear.x / 2
 #            self.msg.angular.z = self.msg.angular.z / 2
 #            self.speed = 0.2
 #    elif detected_words.data.find("forward") > -1:
 #        self.msg.linear.x = self.speed
 #        self.msg.angular.z = 0
 #    elif detected_words.data.find("left") > -1:
 #        if self.msg.linear.x != 0:
 #            if self.msg.angular.z < self.speed:
 #                self.msg.angular.z -= 0.75
 #        else:
 #            self.msg.angular.z = self.speed * 2
 #    elif detected_words.data.find("right") > -1:
 #    print("right")
 #        if self.msg.linear.x != 0:
 #            if self.msg.angular.z > -self.speed:
 #                self.msg.angular.z += 0.75
 #        else:
 #            self.msg.angular.z = -self.speed * 2
 #    elif detected_words.data.find("back") > -1:
 #        self.msg.linear.x = -self.speed
 #        self.msg.angular.z = 0
 #    elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
 #        self.msg = Twist()
	# elif detected_words.data.find("pop") > -1:
 #        self.msg.angular.z = 0
	#     print("straighten out")

    # Publish required message
        

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == "__main__":
    print('im in')

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('asr_control')
    pub = rospy.Publisher('/diff_controller/cmd_vel', Twist, queue_size=10)

    ASRControl()
    print('im out')