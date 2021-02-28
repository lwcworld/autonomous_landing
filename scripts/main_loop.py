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
import os
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

    topic = '/clock', '/diagnostics', '/gazebo/link_states', '/gazebo/model_states', '/gazebo/parameter_descriptions', '/gazebo/parameter_updates', '/gazebo/set_link_state', '/gazebo/set_model_state', '/gazebo_gui/parameter_descriptions', '/gazebo_gui/parameter_updates', '/helipad_marker', '/mavlink/from', '/mavlink/gcs_ip', '/mavlink/to', '/mavros/actuator_control', '/mavros/adsb/send', '/mavros/adsb/vehicle', '/mavros/altitude', '/mavros/battery', '/mavros/cam_imu_sync/cam_imu_stamp', '/mavros/companion_process/status', '/mavros/debug_value/debug', '/mavros/debug_value/debug_vector', '/mavros/debug_value/named_value_float', '/mavros/debug_value/named_value_int', '/mavros/debug_value/send', '/mavros/esc_info', '/mavros/esc_status', '/mavros/estimator_status', '/mavros/extended_state', '/mavros/fake_gps/mocap/tf', '/mavros/global_position/compass_hdg', '/mavros/global_position/global', '/mavros/global_position/gp_lp_offset', '/mavros/global_position/gp_origin', '/mavros/global_position/home', '/mavros/global_position/local', '/mavros/global_position/raw/fix', '/mavros/global_position/raw/gps_vel', '/mavros/global_position/raw/satellites', '/mavros/global_position/rel_alt', '/mavros/global_position/set_gp_origin', '/mavros/gps_rtk/rtk_baseline', '/mavros/gps_rtk/send_rtcm', '/mavros/gpsstatus/gps1/raw', '/mavros/gpsstatus/gps1/rtk', '/mavros/gpsstatus/gps2/raw', '/mavros/gpsstatus/gps2/rtk', '/mavros/hil/actuator_controls', '/mavros/hil/controls', '/mavros/hil/gps', '/mavros/hil/imu_ned', '/mavros/hil/optical_flow', '/mavros/hil/rc_inputs', '/mavros/hil/state', '/mavros/home_position/home', '/mavros/home_position/set', '/mavros/imu/data', '/mavros/imu/data_raw', '/mavros/imu/diff_pressure', '/mavros/imu/mag', '/mavros/imu/static_pressure', '/mavros/imu/temperature_baro', '/mavros/imu/temperature_imu', '/mavros/landing_target/lt_marker', '/mavros/landing_target/pose', '/mavros/landing_target/pose_in', '/mavros/local_position/accel', '/mavros/local_position/odom', '/mavros/local_position/pose', '/mavros/local_position/pose_cov', '/mavros/local_position/velocity_body', '/mavros/local_position/velocity_body_cov', '/mavros/local_position/velocity_local', '/mavros/log_transfer/raw/log_data', '/mavros/log_transfer/raw/log_entry', '/mavros/manual_control/control', '/mavros/manual_control/send', '/mavros/mission/reached', '/mavros/mission/waypoints', '/mavros/mocap/pose', '/mavros/mount_control/command', '/mavros/mount_control/orientation', '/mavros/obstacle/send', '/mavros/odometry/in', '/mavros/odometry/out', '/mavros/onboard_computer/status', '/mavros/param/param_value', '/mavros/play_tune', '/mavros/px4flow/ground_distance', '/mavros/px4flow/raw/optical_flow_rad', '/mavros/px4flow/raw/send', '/mavros/px4flow/temperature', '/mavros/radio_status', '/mavros/rc/in', '/mavros/rc/out', '/mavros/rc/override', '/mavros/setpoint_accel/accel', '/mavros/setpoint_attitude/cmd_vel', '/mavros/setpoint_attitude/thrust', '/mavros/setpoint_position/global', '/mavros/setpoint_position/global_to_local', '/mavros/setpoint_position/local', '/mavros/setpoint_raw/attitude', '/mavros/setpoint_raw/global', '/mavros/setpoint_raw/local', '/mavros/setpoint_raw/target_attitude', '/mavros/setpoint_raw/target_global', '/mavros/setpoint_raw/target_local', '/mavros/setpoint_trajectory/desired', '/mavros/setpoint_trajectory/local', '/mavros/setpoint_velocity/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped', '/mavros/state', '/mavros/statustext/recv', '/mavros/statustext/send', '/mavros/target_actuator_control', '/mavros/time_reference', '/mavros/timesync_status', '/mavros/trajectory/desired', '/mavros/trajectory/generated', '/mavros/trajectory/path', '/mavros/vfr_hud', '/mavros/vision_pose/pose', '/mavros/vision_pose/pose_cov', '/mavros/vision_speed/speed_twist_cov', '/mavros/wind_estimation', '/ownship_marker', '/plot_covariance', '/plot_mu', '/plot_target', '/rosout', '/rosout_agg', '/target_pixel', '/tf', '/tf_static', '/trajectory_marker_x', '/trajectory_marker_y', '/trajectory_marker_z'


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
    landing_flag = 0
    stop_flag = 0
    rosbag_flag = 0
    iii = 0

    while not rospy.is_shutdown():
        T_now = rospy.get_rostime().to_time()
        # for loop control, initialize time
        if (T_now - T_0) > 55:
            T_0 = T_now
            return_flag = 1
            iii += 1
            landing_flag = 0
            stop_flag = 0
            rosbag_flag = 0

        elif (T_now - T_0) > 15:
            return_flag = 0
            if rosbag_flag is 0:
                cmd_arg = ['rosbag', 'record', '__name:=my_bag', '-O', 'loop_kf_mc_%d' % iii,
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
                q, ukf = init_KF(q=q, s=s, m=m, p=p)
                s['phase'] = 1
                s['flag_KF_init'] = True

            elif a['P_H'][0] >= p['th_L'] and s['flag_KF_init'] and q['z_o'] > 1 and return_flag is 0:
                q, ukf = update_KF(q=q, m=m, p=p, ukf=ukf, T_now=T_now)
                msg_state_filter_target_pose = pub.assign_filter_target_pose(q)

                pub.pub_filter_target_pose.publish(msg_state_filter_target_pose)
                s['phase'] = 1

            elif s['flag_KF_init'] and q['z_o'] < 1:
                q = assign_m_o_2_q_o(q, m)

                if stop_flag is 0 and q['z_o'] < 0.5:
                    stop_arg = 'rosnode kill /my_bag'
                    subprocess.call([stop_arg], shell=True)
                    s['phase'] = 2
                    stop_flag = 1
            elif s['phase'] == 0:
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

        if landing_flag is 0 and s['flag_KF_init'] and q['z_o'] < 1:
            arrival_list.append([q['x_o'], q['y_o']])
            print(iii, '###################')
            print(q['x_o'], q['y_o'])
            print(q['z_o'])
            landing_flag = 1

        if iii is 50:
            break

        rate.sleep()

    print(arrival_list)
    arrival_list = np.array(arrival_list)
    plt.figure(1)
    plt.scatter(arrival_list[:, 0], arrival_list[:, 1])
    plt.show()
