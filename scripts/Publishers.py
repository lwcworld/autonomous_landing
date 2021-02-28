#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, Point, PoseWithCovarianceStamped, TwistWithCovarianceStamped, PoseStamped
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

        self.pub_filter_ownship_pose = rospy.Publisher('filter_ownship_pose', PoseWithCovarianceStamped, queue_size=2)
        self.pub_filter_ownship_vel = rospy.Publisher('filter_ownship_vel', TwistWithCovarianceStamped, queue_size=2)
        self.pub_filter_target_pose = rospy.Publisher('filter_target_pose', PoseWithCovarianceStamped, queue_size=2)
        self.pub_imm_mu = rospy.Publisher('imm_mu', Vector3Stamped, queue_size=2)
        self.pub_fcc_ownship_pose = rospy.Publisher('fcc_ownship_pose', PoseStamped, queue_size=2)
        self.pub_phase = rospy.Publisher('phase', Float64, queue_size=2)


    def assign_fcc_ownship_pose(self, m):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = m['x_o']
        msg.pose.position.y = m['y_o']
        msg.pose.position.z = m['z_o']

        return msg

    def assign_phase(self, s):
        msg = Float64()
        msg.data = s['phase']

        return msg

    def assign_filter_ownship_pose(self, q):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = q['x_o']
        msg.pose.pose.position.y = q['y_o']
        msg.pose.pose.position.z = q['z_o']
        msg.pose.pose.orientation.x = q['roll_o']
        msg.pose.pose.orientation.y = q['pitch_o']
        msg.pose.pose.orientation.z = q['yaw_o']
        msg.pose.pose.orientation.w = 0.

        aa = np.reshape(q['P'][0:11:2, 0:11:2], (1, 36))
        msg.pose.covariance = aa[0]

        return msg

    def assign_filter_ownship_vel(self, q):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.twist.linear.x = q['vx_o']
        msg.twist.twist.linear.y = q['vy_o']
        msg.twist.twist.linear.z = q['vz_o']
        msg.twist.twist.angular.x = q['d_roll_o']
        msg.twist.twist.angular.y = q['d_pitch_o']
        msg.twist.twist.angular.z = q['d_yaw_o']

        aa = np.reshape(q['P'][1:12:2, 1:12:2], (1, 36))
        msg.twist.covariance = aa[0]

        return msg

    def assign_filter_target_pose(self, q):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = q['x_t']
        msg.pose.pose.position.y = q['y_t']
        msg.pose.pose.position.z = q['z_t']

        P_6_6 = np.zeros((6,6))
        P_3_3 = q['P'][12:15, 12:15]
        P_6_6[0:3, 0:3] = P_3_3
        P = np.reshape(P_6_6, (1,36))[0]
        P = list(P)
        msg.pose.covariance = P

        return msg

    def assign_imm_mu(self, mu):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = mu[0]
        msg.vector.y = mu[1]
        msg.vector.z = mu[2]

        return msg

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
        msg.data = math.sqrt(e_x ** 2 + e_y ** 2 + e_z ** 2)

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
