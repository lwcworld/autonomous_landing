#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, Point
from mavros_msgs.msg import MountControl, PositionTarget
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

import math
import tf
import numpy as np

class Publishers():
    def __init__(self):
        # ======== ROS communicators declaration ========z
        self.pub_cmd_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=2)
        self.pub_cmd_sp = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=2)
        self.pub_cmd_mount = rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=2)
        self.pub_state_helipad = rospy.Publisher('helipad_marker', Marker, queue_size=2)
        self.pub_state_ownship = rospy.Publisher('ownship_marker', Marker, queue_size=2)
        self.pub_state_trajectory_x = rospy.Publisher('trajectory_marker_x', Marker, queue_size=2)
        self.pub_state_trajectory_y = rospy.Publisher('trajectory_marker_y', Marker, queue_size=2)
        self.pub_state_trajectory_z = rospy.Publisher('trajectory_marker_z', Marker, queue_size=2)

        self.pub_plot_target = rospy.Publisher('plot_target', Float64, queue_size=2)
        self.pub_plot_covariance = rospy.Publisher('plot_covariance', Vector3Stamped, queue_size=2)
        self.pub_plot_mu = rospy.Publisher('plot_mu', Vector3Stamped, queue_size=2)
        # self.pub_plot_covariance = rospy.Publisher('plot_covariance', Float64, queue_size=2)
    def assign_cmd_sp(self, pos):
        msg = PositionTarget()
        msg.type_mask = int('010111111000', 2)
        msg.coordinate_frame = 1
        msg.position = Point(pos[0], pos[1], pos[2])
        return msg

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
        # roll = m['roll_o']
        # pitch = m['pitch_o']
        # yaw = m['yaw_o']

        msg = MountControl()
        msg.header.frame_id = 'map'
        msg.mode = 2
        msg.roll = 0
        msg.pitch = - 90.
        msg.yaw = 0

        return msg

    def assign_marker_helipad(self, x, P):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
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
        msg.scale.x = P[0, 0]
        msg.scale.y = P[1, 1]
        msg.scale.z = P[2, 2]

        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        return msg

    def assign_marker_ownship(self, x, P):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
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
        msg.scale.x = P[0, 0] * 100.
        msg.scale.y = P[1, 1] * 100.
        msg.scale.z = P[2, 2] * 100.

        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 1.0

        return msg

    def tf_broad(self, x):
        quaternion = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
        br = tf.TransformBroadcaster()
        br.sendTransform((x[0], x[1], x[2]), [quaternion[0], quaternion[1], quaternion[2], quaternion[3]],
                         rospy.Time.now(), "base_link", "map")

    def assign_marker_trajectory_x(self, q, v):
        x = [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']]
        count = v['traj_count']

        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.ns = 'x_%d' % (count)

        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        euler = tf.transformations.quaternion_from_euler(x[3], x[4], x[5])
        msg.pose.orientation.x = euler[0]
        msg.pose.orientation.y = euler[1]
        msg.pose.orientation.z = euler[2]
        msg.pose.orientation.w = euler[3]
        msg.scale.x = 0.3  # arrow length
        msg.scale.y = 0.05  # arrow width
        msg.scale.z = 0.05  # arrow height

        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 0.2

        msg.id = count
        msg.lifetime = rospy.Duration(100)  # marker duration

        return msg

    def assign_marker_trajectory_y(self, q, v):
        x = [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']]
        count = v['traj_count']

        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.ns = 'y_%d' % (count)

        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        euler = tf.transformations.quaternion_from_euler(x[3], x[4], x[5] + math.pi / 2)
        msg.pose.orientation.x = euler[0]
        msg.pose.orientation.y = euler[1]
        msg.pose.orientation.z = euler[2]
        msg.pose.orientation.w = euler[3]
        msg.scale.x = 0.3  # arrow length
        msg.scale.y = 0.05  # arrow width
        msg.scale.z = 0.05  # arrow height

        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 0.2

        msg.id = count
        msg.lifetime = rospy.Duration(100)  # marker duration

        return msg

    def assign_marker_trajectory_z(self, q, v):
        x = [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']]
        count = v['traj_count']

        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.ns = 'z_%d' % (count)

        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        euler = tf.transformations.quaternion_from_euler(x[3], x[4] + math.pi / 2, x[5])
        msg.pose.orientation.x = euler[0]
        msg.pose.orientation.y = euler[1]
        msg.pose.orientation.z = euler[2]
        msg.pose.orientation.w = euler[3]
        msg.scale.x = 0.3  # arrow length
        msg.scale.y = 0.05  # arrow width
        msg.scale.z = 0.05  # arrow height

        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 0.2

        msg.id = count
        msg.lifetime = rospy.Duration(100)  # marker duration

        return msg

    def assign_plot_target(self, q):
        x = [q['x_t'], q['y_t'], q['z_t']]
        msg = Float64()
        e_x = 10 - x[0]
        e_y = - x[1]
        e_z = - x[2]
        msg.data = math.sqrt(e_x**2 + e_y**2 + e_z**2)

        return msg

    def assign_plot_covariance(self, q):
        P = q['P']
        msg = Vector3Stamped()
        P_diag = np.diag(P)
        msg.vector.x = P_diag[0]
        msg.vector.y = P_diag[1]
        msg.vector.z = P_diag[2]

        return msg

    def assign_plot_mu(self, mu):
        msg = Vector3Stamped()

        msg.vector.x = mu[0]
        msg.vector.y = mu[1]
        msg.vector.z = mu[2]

        return msg
