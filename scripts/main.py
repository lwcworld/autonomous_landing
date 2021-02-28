#!/usr/bin/env python
from __future__ import print_function

from Variables import filter_state, meas_state, alg_state, ctrl_state, status, param, vis
from Subscribers import *
from Algorithms import *
from Publishers import Publishers
from Services import Services
from Scenario import *
from Utils import *
import rospy

if __name__ == '__main__':
    rospy.init_node('auto_land')

    # flight mode object
    modes = Services()

    # obj_AL : Autonomous_landing
    q = filter_state()
    m = meas_state()
    a = alg_state()
    c = ctrl_state()
    p = param()
    s = status()
    v = vis()

    sub = Subscribers(q=q, m=m, p=p)
    pub = Publishers()
    scenario = Scenario()

    # Make sure the drone is armed
    while not m.armed:
        modes.setArm()
        rospy.sleep(0.1)

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # start point
    msg_cmd_sp = pub.assign_cmd_sp(scenario.sp_pos)

    # move to the start altitude
    k = 0
    while k < 200:
        pub.pub_cmd_sp.publish(msg_cmd_sp)
        rospy.sleep(0.05)
        k = k + 1

    # set frequency
    freq_est = p['freq_est']
    freq_ctrl = p['freq_ctrl']
    freq_rviz = p['freq_rviz']
    freq = lcm(freq_est, freq_ctrl)

    rate = rospy.Rate(freq)

    count = 1
    T_0 = rospy.get_rostime().to_time()
    while not rospy.is_shutdown():
        T_now = rospy.get_rostime().to_time()

        # estimation
        if count % (freq / freq_est) == 0:
            T_vis = m['T_vis']

            a = update_H(a=a, m=m, p=p, T_now=T_now)

            if a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] is False:
                q, ukf = init_KF(q=q, s=s, m=m, p=p)
                s['phase'] = 1
                s['flag_KF_init'] = True
                print('kf_update start')

            elif a['P_H'][0] >= p['th_L'] and s['flag_KF_init']:
                q, ukf = update_KF(q=q, m=m, p=p, ukf=ukf, T_now=T_now)
                s['phase'] = 1
                print('kf_update')

            elif a['P_H'][0] < p['th_L'] and q['z_o'] < 2 and s['flag_KF_init']:
                q = assign_m_o_2_q_o(q, m)
                s['phase'] = 2
                print('terminal')

            elif a['P_H'][0] < p['th_L'] and s['phase'] == 0:
                q = assign_m_o_2_q_o(q, m)
                s['phase'] = 0
                s['flag_KF_init'] = False
                print('ctrl')

            # publish target info
            if s['phase'] >= 1:
                msg_plot_target = pub.assign_plot_target(q)
                msg_plot_covariance = pub.assign_plot_covariance(q)
                pub.pub_plot_target.publish(msg_plot_target)
                pub.pub_plot_covariance.publish(msg_plot_covariance)

        # control
        if count % (freq / freq_ctrl) == 0:
            p_t = scenario.target_pos(T_now - T_0)
            c = ctrl(c=c, q=q, p_t=p_t, phase=s['phase'])
            msg_cmd_vel = pub.assign_cmd_vel(c)
            pub.pub_cmd_vel.publish(msg_cmd_vel)
            # if the phase status is landing phase and altitude is less than 0.2m,
            # disarm the propellers
            if s['phase'] == 1 and q['z_o'] < 0.05:
                modes.setDisarm()
                break

        msg_cmd_mount = pub.assign_cmd_mount(q)
        pub.pub_cmd_mount.publish(msg_cmd_mount)

        if count % (freq / freq_rviz) == 0:
            pass
            '''
            if s['phase'] == 1:
                msg_state_helipad = pub.assign_marker_helipad(ukf.x[12:], ukf.P[12:, 12:])
                pub.pub_state_helipad.publish(msg_state_helipad)

                msg_state_ownship = pub.assign_marker_ownship(ukf.x[0:5:2], ukf.P[0:5:2, 0:5:2])
                pub.pub_state_ownship.publish(msg_state_ownship)
            '''
            msg_state_trajectory_x = pub.assign_marker_trajectory_x(q, v)
            msg_state_trajectory_y = pub.assign_marker_trajectory_y(q, v)
            msg_state_trajectory_z = pub.assign_marker_trajectory_z(q, v)
            pub.pub_state_trajectory_x.publish(msg_state_trajectory_x)
            pub.pub_state_trajectory_y.publish(msg_state_trajectory_y)
            pub.pub_state_trajectory_z.publish(msg_state_trajectory_z)

            pub.tf_broad([q['x_o'], q['y_o'], q['z_o'], q['roll_o'], q['pitch_o'], q['yaw_o']])

            v['traj_count'] += 1

        if count >= freq:
            count = 1
        else:
            count = count + 1

        rate.sleep()
