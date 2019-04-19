#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Thread

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

NAVI_MAX_LIN_VEL = 5.0
NAVI_MAX_ANG_VEL = 5.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

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

        self.speed = 0.2

        # Intializing message type
        self.msg = Twist()

        # initialize node
        rospy.init_node("asr_control")
        rospy.on_shutdown(self.shutdown)

        # Subscribe to kws output
        rospy.Subscriber("kws_data", String, self.parse_asr_result)
        rospy.sleep(0.1)
        rospy.spin()
    

    def parse_asr_result(self, detected_words): #pylint: disable=too-many-branches
        global target_linear_vel
        global target_angular_vel
        global control_linear_vel
        global control_angular_vel

        """Function to perform action on detected word"""
        print("Detected Word: ", detected_words.data)

        current_word = detected_words.data

        

        # while(detected_words.data == current_word):
        if detected_words.data.find("full speed") > -1:
            print('fs')
            target_linear_vel = checkLinearLimitVelocity(target_linear_vel + NAVI_MAX_LIN_VEL)
            
            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = 5; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            
            pub.publish(twist)

        elif detected_words.data.find("half speed") > -1:
            print('hs')
            
            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = 2.5; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            
            pub.publish(twist)



        elif detected_words.data.find("forward") > -1:
            print("forward")
            twist = Twist()

            twist.linear.x = 0.2

            twist.angular.z = 0
            pub.publish(twist)


        elif detected_words.data.find("left") > -1:
            twist = Twist()
            print("i'm inside while and this is left")
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
            pub.publish(twist)

        elif detected_words.data.find("right") > -1:
            print("HI im inside while and this is right")
            twist = Twist()
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
            pub.publish(twist)

        elif detected_words.data.find("back") > -1:
            twist = Twist()
            print("HI im inside while and this is back")
            target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
            print vels(target_linear_vel,target_angular_vel)
            pub.publish(twist)

        elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
            twist = Twist()
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0
            print vels(target_linear_vel, target_angular_vel)
            pub.publish(twist)


    	else:
            print("End of reading")

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stopping ASRControl")

        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == "__main__":
    print('ASRControl Started')

    rospy.init_node('asr_control')
    pub = rospy.Publisher('/diff_controller/cmd_vel', Twist, queue_size=10)

    ASRControl()
    print('Done...')