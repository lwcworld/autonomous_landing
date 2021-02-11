#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import MountControl
from visualization_msgs.msg import Marker
import math

class Publishers():
    def __init__(self):
        # ======== ROS communicators declaration ========z
        self.pub_cmd_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=2)
        self.pub_cmd_mount = rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=2)
        self.pub_state_helipad = rospy.Publisher('helipad_marker', Marker, queue_size=2)
        self.pub_state_ownship = rospy.Publisher('ownship_marker', Marker, queue_size=2)

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

        msg = MountControl()
        msg.header.frame_id = 'map'
        msg.mode = 2
        msg.roll = - roll / math.pi * 180.
        msg.pitch = - 90. - (pitch) / math.pi * 180.

        return msg

    def assign_marker_helipad(self, x, P):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp    = rospy.Time.now()
        msg.ns = "marker_helipad"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1.0
        msg.scale.x = P[0,0]
        msg.scale.y = P[1,1]
        msg.scale.z = P[2,2]

        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        return msg

    def assign_marker_ownship(self, x, P):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp    = rospy.Time.now()
        msg.ns = "marker_ownship"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1.0
        msg.scale.x = P[0,0]*100.
        msg.scale.y = P[1,1]*100.
        msg.scale.z = P[2,2]*100.

        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 1.0

        return msg