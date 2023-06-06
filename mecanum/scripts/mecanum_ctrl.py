#!/usr/bin/env python
# subscribe to /cmd_vel, /joint_states, /imu
# pulish to /odom, transforms on /odom, wheels command
import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu, JointState
from gazebo_msgs.msg import LinkStates
import csv


File_ = open('/home/rajeev/mydata.xlsx', 'w')
file_ = csv.writer(File_)
file_.writerow(['N_steps', 'X', 'Y', 'Z', 'x_move?', 'y_move?', 'z_move?'])


class Mecanum_Ctrl():
    def __del__(self):
        print('What Happened Huh??')
    def __init__(self):
        # physical parameters
        self.base_lx = 0.25
        self.base_ly = 0.25
        self.lx_ly = self.base_lx + self.base_ly
        self.wheel_radii = 0.04
        self.wheel_posn = [[0.2, 0.056, -0.236], [-0.2, 0.056, -0.34], [-0.2, 0.056, 0.236], [0.2, 0.056, 0.34]]
        self.wheel_orient = [[1.57, 0, 0], [-1.57, 0, 0], [-1.57, 0, 0], [1.57, 0, 0]]
        self.wheel_joint_names = ["" for i in range(4)]
        self.wheel_frames = ['left_rim_one_link', 'right_rim_two_link', 'left_rim_three_link', 'right_rim_four_link']
            # wrt GND frame
        self.curr_pos = [0.0, 0.0, 0.0]
        self.bf_x_pos = 0
        self.bf_y_pos = 0
        self.bf_yaw_z = 0
        self.motor_rotn = [[0.0] for i in range(4)]
        self.motor_prev_rotn = [[0.0] for i in range(4)]
        self.motor_vel = np.array([[0.0] for i in range(4)])
        self.bot_vel = np.array([[0.0] for i in range(3)])
        # playing with sensor data
        self.last_time = rospy.Time.now()
        self.imu_offset = 0
        self.bool_imu_offset = 1
        self.motor_theta_offset = [0.0 for i in range(4)]
        self.motor_vel_offset = [0.0 for i in range(4)]
        self.bool_motor_offset = 1
        self.bf_index = -1
        self.N_steps = 0
        self.x_error = 0
        self.y_error = 0
        self.w_error = 0
        self.is_moving = [0, 0, 0]
        self.pose_offset = [0.002006136, 0.001387497, 0.0]
        self.ds_displacement = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]
        # topics
        self.cmd_topic = '/cmd_vel'
        self.odom_topic = '/odom'
        self.imu_topic = '/imu'
        self.odom_transform_topic = '/odom'
        self.odom_frame = 'odom'
        self.robot_base_frame = 'base_footprint'
        
        # kinematics
        self.v_to_w_mat = 1/self.wheel_radii * np.array([[1, 1, self.lx_ly],
                      [1, -1, self.lx_ly],
                      [1, 1, -self.lx_ly],
                      [1, -1, -self.lx_ly]])
        # to match the rotation direction
        self.v_to_w_mat[1] *= -1
        self.v_to_w_mat[2] *= -1
        # self.w_to_v_mat = 1/4*self.wheel_radii * np.array([[1, 1, 1, 1],
        #                       [1, -1, 1, -1],
        #                       [1/self.lx_ly, 1/self.lx_ly, -1/self.lx_ly, -1/self.lx_ly]])
        # to match the rotation direction
        # self.w_to_v_mat = 1/4*self.wheel_radii * np.array([[1, -1, -1, 1],
        #                       [1, 1, -1, -1],
        #                       [1/self.lx_ly, -1/self.lx_ly, 1/self.lx_ly, -1/self.lx_ly]])
        # to match the joint order in joint states--------->
        self.w_to_v_mat = 1/4*self.wheel_radii * np.array([[1, -1, 1, -1],
                                                            [1, -1, -1, 1],
                                                            [1/self.lx_ly, 1/self.lx_ly, -1/self.lx_ly, -1/self.lx_ly]])
        
        # subs and pubs
        self.cmd_subs = rospy.Subscriber(self.cmd_topic, Twist, self.callback_cmd)
        self.imu_subs = rospy.Subscriber(self.imu_topic, Imu, self.callback_imu)
        self.joint_state_subs = rospy.Subscriber('/mecanum/joint_states', JointState, self.callback_joint_states)
        self.link_state_subs = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback_link_states)
        
        # tfs 
        self.base_br = tf.TransformBroadcaster()
        self.wheel_br = [tf.TransformBroadcaster() for i in range(4)]
        
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.pub1 = rospy.Publisher(
        '/mecanum/one_joint_velocity_controller/command', Float64, queue_size=1)
        self.pub2 = rospy.Publisher(
            '/mecanum/two_joint_velocity_controller/command', Float64, queue_size=1)
        self.pub3 = rospy.Publisher(
            '/mecanum/three_joint_velocity_controller/command', Float64, queue_size=1)
        self.pub4 = rospy.Publisher(
            '/mecanum/four_joint_velocity_controller/command', Float64, queue_size=1)
        self.wheel_cmd_arr = [self.pub1, self.pub2, self.pub3, self.pub4]

        
    def callback_cmd(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        for k in range(3):
            if abs(self.cmd_vel[k])>=0.1:
                self.is_moving[k] = 1
            else:
                self.is_moving[k] = 0
        vrr = np.array([[self.cmd_vel[0]], [self.cmd_vel[1]], [self.cmd_vel[2]]])
        prr = np.matmul(self.v_to_w_mat, vrr)
        for i in range(4):
            self.wheel_cmd_arr[i].publish(prr[i][0])
            
    def callback_imu(self, msg):
        orientation = msg.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z 
        # radians : math.degrees(radian)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        pitch = math.asin(2 * (w * y - x * z))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        if self.bool_imu_offset == 1:
            self.imu_offset = yaw
            self.bool_imu_offset = 0
        self.curr_pos[2] = yaw - self.imu_offset

    def callback_joint_states(self, msg):
        position = msg.position
        velocity = msg.velocity
        for i in range(4):
            self.motor_prev_rotn[i][0] = self.motor_rotn[i][0]
        if (self.bool_motor_offset == 1):
            self.wheel_joint_names = msg.name
            self.last_time = rospy.Time.now()
            for i in range(4):
                self.motor_theta_offset[i] = position[i]
            for i in range(4):
                self.motor_vel_offset[i] = velocity[i]
            self.bool_motor_offset = 0
        for i in range(4):
            self.motor_rotn[i][0] = position[i] - self.motor_theta_offset[i]
            self.wheel_orient[i][1] = self.motor_rotn[i][0]
        for i in range(4):
            self.motor_vel[i][0] = velocity[i]
        vel_wrt_bot = np.matmul(self.w_to_v_mat, self.motor_vel)
        self.bot_vel[0][0] = (vel_wrt_bot[0][0]*math.cos(self.curr_pos[2]) - vel_wrt_bot[1][0]*math.sin(self.curr_pos[2]))
        self.bot_vel[1][0] = (vel_wrt_bot[0][0]*math.sin(self.curr_pos[2]) + vel_wrt_bot[1][0]*math.cos(self.curr_pos[2]))
        self.bot_vel[2][0] = vel_wrt_bot[2][0]
        dth_motor = [[self.motor_rotn[i][0] - self.motor_prev_rotn[i][0]] for i in range(4)]
        ds_wrt_bot = np.matmul(self.w_to_v_mat, dth_motor)
        self.ds_displacement[0] = (ds_wrt_bot[0][0]*math.cos(self.curr_pos[2]) - ds_wrt_bot[1][0]*math.sin(self.curr_pos[2]))
        self.ds_displacement[1] = (ds_wrt_bot[0][0]*math.sin(self.curr_pos[2]) + ds_wrt_bot[1][0]*math.cos(self.curr_pos[2]))
        self.ds_displacement[2] = ds_wrt_bot[2][0]
        for j in range(3):
            self.curr_pos[j]+=self.ds_displacement[j]
            # if self.is_moving[j]:
            self.curr_pos[j]+=self.pose_offset[j]*self.bot_vel[j][0]
        rospy.loginfo("bot vel wrt gnd: %f, %f, %f", self.bot_vel[0], self.bot_vel[1], self.bot_vel[2])
        self.N_steps+=1
        self.PoseErrorCalc()
        # rospy.loginfo('POSE: %f, %f, %f:', self.curr_pos[0], self.curr_pos[1], self.curr_pos[2])
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.robot_base_frame
        odom.pose.pose.position.x = self.curr_pos[0]
        odom.pose.pose.position.y = self.curr_pos[1]
        odom.pose.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.curr_pos[2])
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        self.odom_pub.publish(odom)
        self.publish_base_tf()
        # self.publish_wheel_tf()
        
    def publish_base_tf(self):
        self.base_br.sendTransform((self.curr_pos[0], self.curr_pos[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.curr_pos[2]),
                         rospy.Time.now(),
                         self.robot_base_frame,
                         self.odom_frame)
    def publish_wheel_tf(self):
        for i in range(4):
            self.wheel_br[i].sendTransform(tuple(self.wheel_posn[i]),
                         tf.transformations.quaternion_from_euler(self.wheel_orient[i][0], self.wheel_orient[i][1], self.wheel_orient[i][2]),
                         rospy.Time.now(),
                         self.robot_base_frame,
                         self.wheel_frames[i])
        
    def callback_link_states(self, msg):
        if self.bf_index<0:
            self.bf_index = msg.name.index('mecanum::base_footprint')
        posi = Pose()
        posi = msg.pose[self.bf_index]
        self.bf_x_pos = posi.position.x
        self.bf_y_pos = posi.position.y
        self.bf_yaw_z = posi.orientation.z
        # rospy.loginfo('Bot Position wrt World: %f, %f, %f', self.bf_x_pos, self.bf_y_pos, self.bf_yaw_z)
    
    def PoseErrorCalc(self):
        # rospy.loginfo('header --------------------\nError in your calc = gazebo data - your calc =')
        err_x = self.bf_x_pos - self.curr_pos[0]
        err_y = self.bf_y_pos - self.curr_pos[1]
        err_w = self.bf_yaw_z - self.curr_pos[2]
        self.x_error=err_x
        self.y_error=err_y
        self.w_error=err_w
        # rospy.loginfo('x, y, yaw: %f, %f, %f \n----------------------', 
        #               err_x,
        #               err_y,
        #               err_w                    
        #               )
        file_.writerow([self.N_steps, self.x_error, self.y_error, self.w_error, self.is_moving[0], self.is_moving[1], self.is_moving[2]])
        
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('Mecanum_Ctrl', disable_signals=True)
        rospy.loginfo("Node started")
        mecanum_ctrl = Mecanum_Ctrl()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Stopping Node')
    except KeyboardInterrupt:
        print('Keyboard interrupt received')
    finally:
        File_.close()
        print('Thank you for using')
