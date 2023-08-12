#!/usr/bin/env python
from __future__ import print_function
import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np 
import sys
import select
import termios
import tty

msg = """
Reading from the keyboard !
---------------------------
*: to enter angle to move, wrt x axis of bot

anything else : stop

q/z : increase/decrease max speeds by 10%
a/d : rotate anti/clockwise 
CTRL-C to quit
"""

rotateBindings = {
    'a': 1,
    'd': -1
}

speedBindings = {
    'q': 1.1,
    'z': 0.9,
}

speed = 0.2
cmd_vel = Twist()

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # print(key)
    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('vel_Publisher')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    status = 0

    try:
        print(msg)
        print(vels(speed))
        while (1):
            key = getKey()
            if key == '*':
                print("Enter angle to move in the direction:")
                status+=2
                angle = float(input())*math.pi/180
                cmd_vel.linear.x = speed*math.cos(angle)
                cmd_vel.linear.y = speed*math.sin(angle)
                pub.publish(cmd_vel)
                # print(vel_arr[0][0], vel_arr[1][0])
                
            elif key in rotateBindings.keys():
                cmd_vel.angular.z = speed * rotateBindings[key]
                pub.publish(cmd_vel)
                
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]
                cmd_vel.linear.x *= speedBindings[key]
                cmd_vel.linear.y *= speedBindings[key]
                cmd_vel.angular.z *= speedBindings[key]
                pub.publish(cmd_vel)
                print(vels(speed))
                if (status >= 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                cmd_vel = Twist()
                pub.publish(cmd_vel)
            if (key == '\x03'):
                break

    except Exception as e:
        print("Oo An Error Occured!!!")

    finally:
        cmd_vel = Twist()
        pub.publish(cmd_vel)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)