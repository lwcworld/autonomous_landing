#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import MountControl
import math

class Publishers():
    def __init__(self):
        # ======== ROS communicators declaration ========z
        self.pub_cmd_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.pub_cmd_mount = rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)

    def assign_cmd_vel(self, c):
        msg = Twist()
        msg.linear.x = c['vx']
        msg.linear.y = c['vy']
        msg.linear.z = c['vz']
        msg.angular.x = c['d_roll']
        msg.angular.y = c['d_pitch']
        msg.angular.z = c['d_yaw']
        return msg

    def assign_cmd_mount(self, m):
        roll = m['roll_o']
        pitch = m['pitch_o']

        # roll = m['imu_roll']
        # pitch = m['imu_pitch']

        msg = MountControl()
        msg.header.frame_id = 'map'
        msg.mode = 2
        msg.roll = - roll / math.pi * 180.
        msg.pitch = - 90. - (pitch) / math.pi * 180.
        return msg
