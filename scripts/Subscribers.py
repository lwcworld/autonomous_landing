#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Range, Imu
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import os


class Subscribers():
    def __init__(self, q, m, p):
        # q : filter states
        # m : measured states
        self.q = q
        self.m = m
        self.p = p
        self.true_pose = Odometry()
        self.iter = 0

        # ======== ROS communicators declaration ========z
        # Subscriber
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.save_pose)
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.save_vel)
        rospy.Subscriber("/mavros/px4flow/ground_distance", Range, self.save_LRF)
        rospy.Subscriber("/target_pixel", PointStamped, self.save_target_pixel)
        rospy.Subscriber("/mavros/state", State, self.save_state)
        rospy.Subscriber("/ground_truth_pose", Odometry, self.save_ground_truth)
        # rospy.Subscriber("/image_circled", Image, self.save_circled_image)
        # rospy.Subscriber("/image_original", Image, self.save_original_image)
        # rospy.Subscriber('/mavros/imu/data', Imu, self.save_imu)

    '''
    def save_circle_image(self, msg):
        path = '/home/lics/Documents/image/'
        directory = 'image_%s' % self.iter
        if not os.path.exists(path + directory):
            os.makedirs(path + directory)
            self.iter += 1
        

    def save_original_image(self, msg):
        pass
    '''


    def save_ground_truth(self, msg):
        self.true_pose = msg

    def save_imu(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (self.m['imu_roll'], self.m['imu_pitch'], self.m['imu_yaw']) = euler_from_quaternion(orientation_list)

    def save_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z

        # attitude transformation from Quaternion to Euler
        quaternion_list = [qx, qy, qz, qw]
        self.m['x_o'], self.m['y_o'], self.m['z_o'] = x, y, z
        (self.m['roll_o'], self.m['pitch_o'], self.m['yaw_o']) = euler_from_quaternion(quaternion_list)
        self.m['T_o'] = msg.header.stamp.to_sec()

    def save_vel(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z

        d_roll = msg.twist.angular.x
        d_pitch = msg.twist.angular.y
        d_yaw = msg.twist.angular.z

        self.m['vx_o'], self.m['vy_o'], self.m['vz_o'] = vx, vy, vz
        self.m['d_roll_o'], self.m['d_pitch_o'], self.m['d_yaw_o'] = d_roll, d_pitch, d_yaw

    def save_LRF(self, msg):
        self.m['r'] = msg.range
        self.m['T_r'] = msg.header.stamp.to_sec()

    def save_target_pixel(self, msg):
        # self.m['px_t'] = msg.point.x
        # self.m['py_t'] = msg.point.y
        self.m['px_t'] = -(msg.point.y - self.p['cam_bound'][0] / 2)
        self.m['py_t'] = (msg.point.x - self.p['cam_bound'][1] / 2)
        self.m['T_vis'] = msg.header.stamp.to_sec()  ##

    def save_state(self, msg):
        self.m['armed'] = msg.armed
