#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped

from tf.transformations import euler_from_quaternion

def update_H(a, T_now, T_vis):
    P_H = a['P_H']
    a['P_H'] = P_H

    return a

def init_KF(q, a, m):
    q['x_o'] = 3
    a['flag_KF_init'] = True
    return q, a

def update_KF(q, m):
    return q

def ctrl(c, q, p_t, phase):
    if phase ==0 : # waypoint
        pass
    else: # landing
        pass

    return c