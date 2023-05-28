#!/usr/bin/env python
from __future__ import print_function
import rospy, math
from std_msgs.msg import Float64
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
wheel_radii = 0.04
lx = 0.25
ly = 0.25
_lx_ly = lx+ly

v_to_w_mat = np.array([[1, 1, _lx_ly],
                      [1, -1, _lx_ly],
                      [1, 1, -_lx_ly],
                      [1, -1, -_lx_ly]])
# to match the rotation direction
v_to_w_mat[1] *= -1
v_to_w_mat[2] *= -1
# w_to_v_mat = np.array([[1, 1, 1, 1],
#                       [1, -1, 1, -1],
#                       [1, 1, -1, -1]/_lx_ly])

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
    pub1 = rospy.Publisher(
        '/mecanum/one_joint_velocity_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher(
        '/mecanum/two_joint_velocity_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher(
        '/mecanum/three_joint_velocity_controller/command', Float64, queue_size=1)
    pub4 = rospy.Publisher(
        '/mecanum/four_joint_velocity_controller/command', Float64, queue_size=1)
    pub_arr = [pub1, pub2, pub3, pub4]
    
    vel_arr = np.array([[0.0], [0.0], [0.0]])
    omega_arr = np.array([[Float64()] for i in range(4)])
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
                vel_arr[0][0] = speed*math.cos(angle)
                vel_arr[1][0] = speed*math.sin(angle)
                print(vel_arr[0][0], vel_arr[1][0])
                mul = np.matmul(v_to_w_mat, vel_arr) * 1/wheel_radii
                for i in range(4):
                    omega_arr[i][0] = mul[i][0]
                for i in range(4):
                    pub_arr[i].publish(omega_arr[i][0])
                    
            elif key in rotateBindings.keys():
                vw = speed * rotateBindings[key]
                omega_arr[0][0] = vw
                omega_arr[1][0] = -vw
                omega_arr[2][0] = vw
                omega_arr[3][0] = -vw
                for i in range(4):
                    pub_arr[i].publish(omega_arr[i][0])
                    
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]
                print(vels(speed))
                if (status >= 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                omega_arr = np.array([[0.0], [0.0], [0.0], [0.0]])
                for i in range(4):
                    pub_arr[i].publish(omega_arr[i][0])
            if (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        for i in range(4):
            pub_arr[i].publish(Float64(0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)