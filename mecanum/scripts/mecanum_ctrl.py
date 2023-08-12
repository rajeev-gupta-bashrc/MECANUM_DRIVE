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

# File_ = open('/home/rajeev/mydata.xlsx', 'w')
# file_ = csv.writer(File_)
# file_.writerow(['N_steps', 'X', 'x_error?', 'Y', 'y_error?', 'Z', 'z_error?'])

class Mecanum_Ctrl():
    def __del__(self):
        print('What Happened Huh??')
    def __init__(self):
        # physical parameters
        self.base_lx = 0.2
        self.base_ly = 0.288
        self.lx_ly = self.base_lx + self.base_ly
        self.wheel_radii = 0.04848
        self.encoder_radii = 0.019    #0.016914175 calculated
        self.omni_radii = 0.04
        self.wheel_posn = [[0.2, 0.056, -0.236], [-0.2, 0.056, -0.34], [-0.2, 0.056, 0.236], [0.2, 0.056, 0.34]]
        self.wheel_orient = [[1.57, 0, 0], [-1.57, 0, 0], [-1.57, 0, 0], [1.57, 0, 0]]
        self.motor_joint_names = {'left_rim_one_joint':0, 'right_rim_two_joint':1, 'left_rim_three_joint':2, 'right_rim_four_joint':3}
        self.encoder_joint_names = {'encoder_one_joint':0, 'encoder_two_joint':1, 'encoder_three_joint':2}
        self.wheel_frames = ['left_rim_one_link', 'right_rim_two_link', 'left_rim_three_link', 'right_rim_four_link']
        self.encoder_joint_index = []
            # wrt GND frame
        self.curr_pos = np.array([0.0, 0.0, 0.0])
        self.prev_pos = np.array([0.0, 0.0, 0.0])
        self.bf_x_pos = 0
        self.bf_y_pos = 0
        self.bf_yaw_z = 0
        self.motor_rotn = [[0.0] for i in range(4)]
        self.encoder_rotn = np.array([[0.0] for i in range(3)])
        self.encoder_prev_rotn = np.array([[0.0] for i in range(3)])
        self.encoder_pi_prev_rotn = np.array([[0.0] for i in range(3)])
        # self.motor_prev_rotn = [[0.0] for i in range(4)]
        self.motor_vel = np.array([[0.0] for i in range(4)])
        self.bot_vel = Vector3()
        self.bot_prev_vel = Vector3()
        self.bot_acc = Vector3()
        self.bot_orientation = Quaternion() 
        # playing with sensor data
        self.last_time = rospy.Time.now()
        self.imu_offset = 0
        self.bool_imu_offset = 1
        self.encoder_offset = [0.0 for i in range(3)]
        self.motor_theta_offset = [0.0 for i in range(4)]
        self.bool_motor_offset = 1
        self.bf_index = -1
        self.N_steps = 0
        self.x_error = 0
        self.y_error = 0
        self.w_error = 0
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.ds_displacement = [0.0, 0.0, 0.0]
        # topics
        self.cmd_topic = '/cmd_vel'
        self.odom_topic = '/odom'
        self.imu_topic = '/imu'
        self.odom_transform_topic = '/odom'
        self.odom_frame = 'odom'
        self.robot_base_frame = 'base_footprint'
        
        # kinematics
        self.omni_w_to_v_mat = 1/3*self.encoder_radii*np.array([[0, -math.sqrt(3), math.sqrt(3)],
                                         [2, -1, -1],
                                         [1, 1, 1]])
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
        # self.w_to_v_mat = 1/4*self.wheel_radii * np.array([[1, -1, 1, -1],
        #                                                     [1, -1, -1, 1],
        #                                                     [1/self.lx_ly, 1/self.lx_ly, -1/self.lx_ly, -1/self.lx_ly]])
        
        # subs and pubs
        self.cmd_subs = rospy.Subscriber(self.cmd_topic, Twist, self.callback_cmd)
        self.imu_subs = rospy.Subscriber(self.imu_topic, Imu, self.callback_imu)
        self.joint_state_subs = rospy.Subscriber('/mecanum/joint_states', JointState, self.callback_joint_states)
        self.link_state_subs = rospy.Subscriber('/gazebo/link_states', LinkStates, self.callback_link_states)
        # self.get_radii_subs = rospy.Subscriber('/start_exp', Float64, self.callback_get_radii)
        
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
        self.en_slider_pub1 = rospy.Publisher(
            '/mecanum/one_slider_joint_position_controller/command', Float64, queue_size=1)
        self.en_slider_pub2 = rospy.Publisher(
            '/mecanum/two_slider_joint_position_controller/command', Float64, queue_size=1)
        self.en_slider_pub3 = rospy.Publisher(
            '/mecanum/three_slider_joint_position_controller/command', Float64, queue_size=1)
        # to maintain traction in encoder wheels
        self.en_slider_pub3.publish(Float64(1))
        self.en_slider_pub2.publish(Float64(1))
        self.en_slider_pub1.publish(Float64(1))
        self.wheel_cmd_arr = [self.pub1, self.pub2, self.pub3, self.pub4]

        
    def callback_cmd(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
            
    def callback_imu(self, msg):
        # rospy.loginfo(msg)
        self.bot_orientation = msg.orientation
        self.bot_acc.x = msg.linear_acceleration.x
        self.bot_acc.y = msg.linear_acceleration.y
        self.bot_acc.z = msg.linear_acceleration.z
        (_roll, _pitch, self.curr_pos[2]) = tf.transformations.euler_from_quaternion(
                                            [self.bot_orientation.x, self.bot_orientation.y, self.bot_orientation.z, self.bot_orientation.w])
        # rospy.loginfo('-----------\n%f, %f, %f', self.bot_acc.x, self.bot_acc.y, self.bot_acc.z)
        
    def callback_joint_states(self, msg):
        position = msg.position
        if (self.bool_motor_offset == 1):
            n = len(msg.name)
            names = msg.name
            rospy.loginfo(names)
            for z in range(n):
                if names[z] in self.encoder_joint_names.keys():
                    self.encoder_joint_names[names[z]] = z
                if names[z] in self.motor_joint_names.keys():
                    self.motor_joint_names[names[z]] = z
            self.motor_joint_index = list(self.motor_joint_names.values())
            self.encoder_joint_index = list(self.encoder_joint_names.values())
            # set motor offset values
            for i in range(4):
                self.motor_theta_offset[i] = position[self.motor_joint_index[i]]
            # set encoder offset values
            for i in range(3):
                self.encoder_offset[i] = position[self.encoder_joint_index[i]]
            # end the if block
            self.bool_motor_offset = 0
        for i in range(4):
            self.motor_rotn[i][0] = position[self.motor_joint_index[i]] - self.motor_theta_offset[i]
            self.wheel_orient[i][1] = self.motor_rotn[i][0]                     #for wheel tf
        for i in range(3):
            self.encoder_rotn[i][0] = position[self.encoder_joint_index[i]] \
                                        - self.encoder_offset[i]
        # rospy.loginfo(self.encoder_rotn)
        
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
        (_roll, _pitch, self.bf_yaw_z) = tf.transformations.euler_from_quaternion([posi.orientation.x, posi.orientation.y, posi.orientation.z, posi.orientation.w])
        # rospy.loginfo('Bot Position wrt World: %f, %f, %f', self.bf_x_pos, self.bf_y_pos, self.bf_yaw_z)
        
    
    def PoseErrorCalc(self):
        rospy.loginfo('header --------------------\nError in your calc = gazebo data - your calc =')
        err_x = self.bf_x_pos - self.curr_pos[0]
        err_y = self.bf_y_pos - self.curr_pos[1]
        err_w = self.bf_yaw_z - self.curr_pos[2]
        self.x_error=err_x
        self.y_error=err_y
        self.w_error=err_w
        rospy.loginfo('x, y, yaw: %f, %f, %f \n----------------------', 
                      err_x,
                      err_y,
                      err_w                    
                      )
        # self.N_steps+=1
        # file_.writerow([self.N_steps, self.curr_pos[0], err_x, self.curr_pos[1], err_y, self.curr_pos[2], err_w])
        
    def Update_Bot(self):
        self.update_rate = rospy.Rate(100)
        self.last_time = rospy.Time.now()
        for i in range(3):
            self.encoder_prev_rotn[i][0] = self.encoder_rotn[i][0]
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dth_encoder = [[self.encoder_rotn[i][0]-self.encoder_prev_rotn[i][0]] for i in range(3)]
            for i in range(3):
                self.encoder_prev_rotn[i][0] = self.encoder_rotn[i][0]
            ds_wrt_bot  = np.matmul(self.omni_w_to_v_mat, dth_encoder)
            self.ds_displacement[0] = (ds_wrt_bot[0][0]*math.cos(self.curr_pos[2]) - ds_wrt_bot[1][0]*math.sin(self.curr_pos[2]))
            self.ds_displacement[1] = (ds_wrt_bot[0][0]*math.sin(self.curr_pos[2]) + ds_wrt_bot[1][0]*math.cos(self.curr_pos[2]))
            self.ds_displacement[2] = ds_wrt_bot[2][0]/self.omni_radii
            for j in range(2):
                self.curr_pos[j]+=self.ds_displacement[j]
            # rospy.loginfo("curr posn %f, %f, %f", self.curr_pos[0], self.curr_pos[1],self.curr_pos[2])
            
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
            self.PoseErrorCalc()
            self.publish_base_tf()
            # self.publish_wheel_tf()
            vrr = np.array([[self.cmd_vel[0]], [self.cmd_vel[1]], [self.cmd_vel[2]]])
            prr = np.matmul(self.v_to_w_mat, vrr)
            for i in range(4):
                self.wheel_cmd_arr[i].publish(prr[i][0])
                
            self.update_rate.sleep()
        
    # def callback_get_radii(self, msg):
    #     while(1):
    #         # rospy.loginfo(self.encoder_rotn[2][0])
    #         if(self.encoder_rotn[2][0]-self.encoder_pi_prev_rotn[2][0] > 2*math.pi):
    #             for i in range(3):
    #                 self.encoder_pi_prev_rotn[i][0] = self.encoder_rotn[i][0]
    #             rospy.loginfo(self.curr_pos - self.prev_pos)
    #             for i in range(3):
    #                 self.prev_pos[i] = self.curr_pos[i]
                
            
if __name__ == '__main__':
    try:
        rospy.init_node('Mecanum_Ctrl', disable_signals=True)
        rospy.loginfo("Node started")
        mecanum_ctrl = Mecanum_Ctrl()
        mecanum_ctrl.Update_Bot()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Stopping Node')
    except KeyboardInterrupt:
        print('Keyboard interrupt received')
    finally:
        print('Thank you for using')
