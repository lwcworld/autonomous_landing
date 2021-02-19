#!/usr/bin/env python
from __future__ import print_function

from Variables import filter_state, meas_state, alg_state, ctrl_state, status, param
from Subscribers import *
from Algorithms import *
from Publishers import Publishers
from Scenario import *
from Utils import *

from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import Point
import rospy
import math


class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)


if __name__ == '__main__':
    rospy.init_node('auto_land')

    # flight mode object
    modes = fcuModes()

    # obj_AL : Autonomous_landing
    q = filter_state()
    m = meas_state()
    a = alg_state()
    c = ctrl_state()
    p = param()
    s = status()

    sub = Subscribers(q=q, m=m, p=p)
    pub = Publishers()
    scenario = Scenario()

    # start point
    sp_init = PositionTarget()
    sp_init.type_mask = int('010111111000', 2)
    sp_init.coordinate_frame = 1
    sp_init.position = Point(0.0, 0.0, 5.0)
    rate_init = rospy.Rate(20.0)

    # save data for plot
    data = []

    # Make sure the drone is armed
    while not m.armed:
        modes.setArm()
        rate_init.sleep()

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # move to the start altitude
    k = 0
    while k < 300:
        pub.pub_cmd_sp.publish(sp_init)
        rate_init.sleep()
        k = k + 1

    # set frequency
    freq_est = p['freq_est']
    freq_ctrl = p['freq_ctrl']
    freq_rviz = p['freq_rviz']
    freq = lcm(freq_est, freq_ctrl)

    rate = rospy.Rate(freq)

    count = 1
    traj_count = 1
    T_0 = rospy.get_rostime().to_time()
    while not rospy.is_shutdown():
        T_now = rospy.get_rostime().to_time()

        # estimation
        if count % (freq / freq_est) == 0:
            T_vis = m['T_vis']

            a = update_H(a=a, m=m, p=p, T_now=T_now)

            if a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] == False:
                q, ukf = init_KF(q=q, s=s, m=m, p=p)
                s['phase'] = 1
                s['flag_KF_init'] = True

            elif a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] == True:
                q, ukf = update_KF(q=q, m=m, p=p, ukf=ukf, T_now=T_now)
                s['phase'] = 1
                msg_plot_target = pub.assign_plot_target([q['x_t'], q['y_t'], q['z_t']])
                msg_plot_covariance = pub.assign_plot_covariance(ukf.P[12:, 12:])
                pub.pub_plot_target.publish(msg_plot_target)
                pub.pub_plot_covariance.publish(msg_plot_covariance)

            elif q['z_o'] < 2 and s['flag_KF_init'] == True:
                q, ukf = update_KF(q=q, m=m, p=p, ukf=ukf, T_now=T_now)
                msg_plot_target = pub.assign_plot_target([q['x_t'], q['y_t'], q['z_t']])
                msg_plot_covariance = pub.assign_plot_covariance(ukf.P[12:, 12:])
                pub.pub_plot_target.publish(msg_plot_target)
                pub.pub_plot_covariance.publish(msg_plot_covariance)

                q['x_o'] = m['x_o']
                q['y_o'] = m['y_o']
                q['z_o'] = m['z_o']
                q['roll_o'] = m['roll_o']
                q['pitch_o'] = m['pitch_o']
                q['yaw_o'] = m['yaw_o']
                s['phase'] = 1

            elif a['P_H'][0] < p['th_L'] and s['phase'] == 0:
                q['x_o'] = m['x_o']
                q['y_o'] = m['y_o']
                q['z_o'] = m['z_o']
                q['roll_o'] = m['roll_o']
                q['pitch_o'] = m['pitch_o']
                q['yaw_o'] = m['yaw_o']
                s['phase'] = 0
                s['flag_KF_init'] = False

        # control
        if count % (freq / freq_ctrl) == 0:
            p_t = scenario.target_pos(T_now - T_0, q)
            c = ctrl(c=c, q=q, p_t=p_t, phase=s['phase'])
            msg_cmd_vel = pub.assign_cmd_vel(c)
            pub.pub_cmd_vel.publish(msg_cmd_vel)
            # if the phase status is landing phase and altitude is less than 0.2m,
            # disarm the propellers
            if s['phase'] == 1 and q['z_o'] < 0.05:
                modes.setDisarm()
                # plot_data(data)
                break
            '''
            if s['phase'] == 1:
                msg_state_helipad = pub.assign_marker_helipad(ukf.x[12:], ukf.P[12:, 12:])
                pub.pub_state_helipad.publish(msg_state_helipad)

                msg_state_ownship = pub.assign_marker_ownship(ukf.x[0:5:2], ukf.P[0:5:2, 0:5:2])
                pub.pub_state_ownship.publish(msg_state_ownship)
            '''

        msg_cmd_mount = pub.assign_cmd_mount(q)
        pub.pub_cmd_mount.publish(msg_cmd_mount)


        # append trajectories to the data if count % 10 == 0: data.append([T_now - T_0, m['x_o'], m['y_o'], m['z_o'],
        # m['vx_o'], m['vy_o'], m['vz_o'], m['roll_o'], m['pitch_o'], m['yaw_o'], m['d_roll_o'], m['d_pitch_o'],
        # m['d_yaw_o'], msg_cmd_mount.pitch, msg_cmd_mount.roll])

        if count % (freq / freq_rviz) == 0:
            msg_state_trajectory_x = pub.assign_marker_trajectory_x(
                [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']], traj_count)
            msg_state_trajectory_y = pub.assign_marker_trajectory_y(
                [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']], traj_count)
            msg_state_trajectory_z = pub.assign_marker_trajectory_z(
                [q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']], traj_count)
            pub.pub_state_trajectory_x.publish(msg_state_trajectory_x)
            pub.pub_state_trajectory_y.publish(msg_state_trajectory_y)
            pub.pub_state_trajectory_z.publish(msg_state_trajectory_z)

            pub.tf_broad([q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']])

            traj_count += 1

        if count >= freq:
            count = 1
        else:
            count = count + 1

        rate.sleep()
