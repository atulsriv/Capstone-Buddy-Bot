#!/usr/bin/env python
# license removed for brevity
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

msg = """
Control Your Robot
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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
        self.speed = 0.2
        # Intializing message type
        self.msg = Twist()

        # initialize node
        rospy.init_node("asr_control")
        rospy.on_shutdown(self.shutdown)

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Subscribe to kws output
        rospy.Subscriber("kws_data", String, self.parse_asr_result)
        rospy.spin()

    def parse_asr_result(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action on detected word"""
    print("detected words", detected_words.data)
        if detected_words.data.find("full speed") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x * 2
                self.msg.angular.z = self.msg.angular.z * 2
                self.speed = 0.4
        elif detected_words.data.find("half speed") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x / 2
                self.msg.angular.z = self.msg.angular.z / 2
                self.speed = 0.2
        elif detected_words.data.find("forward") > -1:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z -= 0.75
            else:
                self.msg.angular.z = self.speed * 2
        elif detected_words.data.find("right") > -1:
        print("right")
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z += 0.75
            else:
                self.msg.angular.z = -self.speed * 2
        elif detected_words.data.find("back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
            self.msg = Twist()
    elif detected_words.data.find("pop") > -1:
            self.msg.angular.z = 0
        print("straighten out")

        # Publish required message
        self.pub_.publish(self.msg)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('navi_speech')
    pub = rospy.Publisher('/diff_controller/cmd_vel', Twist, queue_size=10)
    print("asr control bootup")
    ASRControl()
    print("asr running")
    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    print("values set")
    try:
        print msg
        while(1):

            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print vels(target_linear_vel, target_angular_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
