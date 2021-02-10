#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped

from tf.transformations import euler_from_quaternion

def update_H(a, m, p, T_now, T_vis):
    # check
    # 1) target is centered in camera view
    # 2) measurement is obtained succesively

    c_1, c_2 = False, False

    # condition 1

    if T_vis - T_now < p['freq_est']:
        c_1 = True

    # condition 2
    b_x, b_y = p['cam_bound'][0], p['cam_bound'][1]
    px_t, py_t = m['px_t'], m['py_t']
    if abs(px_t-b_x/2)<p['th_viscen'] and abs(py_t-b_y/2)<p['th_viscen']:
        c_2 = True

    if c_1==True and c_2 == True:
        idx_status = 0  # good measure
    else:
        idx_status = 1  # not good measure

    a['P_H'] = np.multiply(p['cm'][:,idx_status], a['P_H']) / np.matmul(p['cm'][:,idx_status], a['P_H'])

    a['P_H'] = np.maximum(p['P_H_min'], a['P_H'])
    a['P_H'] = a['P_H'] / np.sum(a['P_H'])

    return a

def init_KF(q, s, m):
    q['x_o'] = 3
    s['flag_KF_init'] = True
    return q, s

def update_KF(q, m):
    return q

def ctrl(c, q, p_t, phase):
    if phase ==0 : # waypoint
        pass
    else: # landing
        pass

    return c