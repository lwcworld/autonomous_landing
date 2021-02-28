#!/usr/bin/env python
from __future__ import print_function

from Variables import filter_state, meas_state, alg_state, ctrl_state, status, param, vis
from Subscribers import *
from Algorithms import *
from Publishers import Publishers
from Services import Services
from Scenario import *
from Utils import *
from imm_estimator import IMMEstimator
import rospy

from matplotlib import pyplot as plt
import numpy as np

import subprocess

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
    return_flag = 0
    arrival_list = []
    iii = 0
    land_flag = 0
    stop_flag = 0
    rosbag_flag = 0

    while not rospy.is_shutdown():
        T_now = rospy.get_rostime().to_time()
        # for loop control, initialize time
        if (T_now - T_0) > 55:
            T_0 = T_now
            return_flag = 1
            iii += 1
            land_flag = 0
            stop_flag = 0
            rosbag_flag = 0


        elif (T_now - T_0) > 15:
            return_flag = 0
            if rosbag_flag is 0:
                cmd_arg = ['rosbag', 'record', '__name:=my_bag', '-O', 'loop_imm_mc_%d' % iii,
                           '/mavros/local_position/pose', '/mavros/local_position/velocity_body',
                           '/mavros/px4flow/ground_distance', '/target_pixel', '/mavros/state',
                           'filter_ownship_pose', 'filter_ownship_vel', 'filter_target_pose', 'imm_mu',
                           'fcc_ownship_pose', 'phase', '/ground_truth_pose']
                subprocess.Popen(cmd_arg)
                rosbag_flag = 1

        # estimation
        if count % (freq / freq_est) == 0:
            T_vis = m['T_vis']

            a = update_H(a=a, m=m, p=p, T_now=T_now)

            if a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] is False and q['z_o'] > 1 and return_flag is 0:
                imm = IMMEstimator(p=p, q=q, m=m)
                s['phase'] = 1
                s['flag_KF_init'] = True

            elif a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] and q['z_o'] > 1 and return_flag is 0:
                imm.predict()
                q = imm.update(q, m, T_now, p)
                msg_state_imm_mu = pub.assign_imm_mu(imm.mu)
                msg_state_filter_target_pose = pub.assign_filter_target_pose(q)

                pub.pub_imm_mu.publish(msg_state_imm_mu)
                pub.pub_filter_target_pose.publish(msg_state_filter_target_pose)

                s['phase'] = 1

            elif s['flag_KF_init'] and q['z_o'] < 1:
                q = assign_m_o_2_q_o(q, m)

                if stop_flag is 0 and q['z_o'] < 0.5:
                    stop_arg = 'rosnode kill /my_bag'
                    subprocess.call([stop_arg], shell=True)
                    s['phase'] = 2
                    stop_flag = 1

            elif s['phase'] is 0:
                q = assign_m_o_2_q_o(q, m)
                s['phase'] = 0
                s['flag_KF_init'] = False

        # control
        if count % (freq / freq_ctrl) == 0:
            p_t = scenario.target_pos_loop(T_now - T_0)
            if return_flag is 1:
                s['flag_KF_init'] = False
                s['phase'] = 0
            c = ctrl(c=c, q=q, p_t=p_t, phase=s['phase'])
            msg_cmd_vel = pub.assign_cmd_vel(c)
            pub.pub_cmd_vel.publish(msg_cmd_vel)

        # mount ctrl
        msg_cmd_mount = pub.assign_cmd_mount(q)
        pub.pub_cmd_mount.publish(msg_cmd_mount)

        if count % (freq / freq_rviz) == 0:
            msg_state_filter_ownship_pose = pub.assign_filter_ownship_pose(q)
            msg_state_filter_ownship_vel = pub.assign_filter_ownship_vel(q)
            msg_state_fcc_ownship_pose = pub.assign_fcc_ownship_pose(m)
            msg_state_phase = pub.assign_phase(s)

            pub.pub_filter_ownship_pose.publish(msg_state_filter_ownship_pose)
            pub.pub_filter_ownship_vel.publish(msg_state_filter_ownship_vel)
            pub.pub_fcc_ownship_pose.publish(msg_state_fcc_ownship_pose)
            pub.pub_phase.publish(msg_state_phase)

        if count >= freq:
            count = 1
        else:
            count = count + 1

        if land_flag is 0 and s['flag_KF_init'] and q['z_o'] < 1:
            arrival_list.append([q['x_o'], q['y_o']])
            print(iii, '###################')
            print(q['x_o'], q['y_o'])
            print(q['z_o'])
            land_flag = 1

        if iii is 50:
            break
        rate.sleep()

    print(arrival_list)
    arrival_list = np.array(arrival_list)
    plt.figure(1)
    plt.scatter(arrival_list[:, 0], arrival_list[:, 1])
    plt.show()
