#!/usr/bin/python
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Thread
import time

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

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
    def __init__(self):
        
	print("ASRControl Started")

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
        global navi_lock #1:locked, 0:unlocked
        global command_speed

        """Function to perform action on detected word"""
        print("Detected Word: ", detected_words.data)

        if detected_words.data.find("navi") > -1:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)

            navi_lock = 0

            rospy.loginfo("Navi Unlocked. Searching for a command.")

        if navi_lock == 0: #if unlocked, look for another kw.
            if detected_words.data.find("full speed") > -1:
                print('full speed')
                command_speed = NAVI_MAX_LIN_VEL
                print("Changing speed to max.")
                #change speed to 5
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("half speed") > -1:
                print('half speed')
                command_speed = NAVI_MAX_LIN_VEL/2
                print("Changing speed to half(max).")
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("forward") > -1:
                t_end = time.time() + 2
                print("forward")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)

                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("left") > -1:
                print('quarter left turn')
                t_end = time.time() + 1.5
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = 18
                    pub.publish(twist)
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("right") > -1:
                print('quarter right turn')
                t_end = time.time() + 1.5
                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.angular.z = -18
                    pub.publish(twist)
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("back") > -1:
                t_end = time.time() +  2
                print("back")

                while time.time() - t_end < 0:
                    twist = Twist()
                    twist.linear.x = -command_speed
                    # twist.angular.z = 0
                    pub.publish(twist)

                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again

            elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
                twist = Twist()
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
                pub.publish(twist)
                print("Locking Navi. Say 'navi' for another command")
                navi_lock = 1 #locks navi again
            else:
                pass

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