#!/usr/bin/env python
from Variables import filter_state, meas_state, alg_state, ctrl_state, status, param
from Subscribers import *
from Algorithms import *
from Publishers import Publishers
from Scenario import *
from Utils import *

if __name__ == '__main__':
    rospy.init_node('auto_land')

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

            if   a['P_H'][0] >= p['th_L'] and s['flag_KF_init']==False:
                q, ukf = init_KF(q=q, s=s, m=m, p=p)
                s['phase'] = 1
                s['flag_KF_init'] = True

            elif a['P_H'][0] >= p['th_L'] and s['flag_KF_init']==True:
                q, ukf = update_KF(q=q, m=m, p=p, ukf=ukf, T_now=T_now)
                s['phase'] = 1

            elif a['P_H'][0] <  p['th_L']:
                s['phase'] = 0
                s['flag_KF_init'] = False

        # control
        msg_cmd_mount = pub.assign_cmd_mount(m)
        pub.pub_cmd_mount.publish(msg_cmd_mount)

        if count % (freq / freq_ctrl) == 0:
            if s['phase'] == 1:
                msg_state_helipad = pub.assign_marker_helipad(ukf.x[12:], ukf.P[12:,12:])
                pub.pub_state_helipad.publish(msg_state_helipad)

                msg_state_ownship = pub.assign_marker_ownship(ukf.x[0:5:2], ukf.P[0:5:2, 0:5:2])
                pub.pub_state_ownship.publish(msg_state_ownship)

        if count % (freq / freq_rviz) == 0:
            pass

        if count >= freq:
            count = 1
        else:
            count = count + 1

        rate.sleep()


