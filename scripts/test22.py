#!/usr/bin/env python
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common.discretization import Q_discrete_white_noise
from scipy.linalg import block_diag
import numpy as np
from numpy import dot, asarray, zeros, outer
from filterpy.common import pretty_str
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common.discretization import Q_discrete_white_noise
from scipy.linalg import block_diag
from Variables import filter_state, meas_state, alg_state, ctrl_state, status, param, vis
from Subscribers import *
from Algorithms import *
from Publishers import Publishers
from Services import Services
from Scenario import *
from Utils import *
import rospy

import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped

from tf.transformations import euler_from_quaternion




def fx(x, dt):
    # state transition function - predict next state based
    # on constant velocity model x = vt + x_0
    F = np.array([[1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]], dtype=float)
    return np.dot(F, x)


def hx(x, alpha, beta):
    # measurement function - convert state into a measurement
    # where measurements are [x_pos, y_pos]

    I_12_12 = np.eye(12)
    zero_12_3 = np.zeros((12, 3))

    cam_coeff = np.array([[alpha, 0, 0],
                          [0, beta, 0],
                          [0, 0, 1]], dtype=float)
    phi = x[6]
    theta = x[8]
    psi = x[10]
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    c_phi = np.cos(phi)
    c_theta = np.cos(theta)
    H_1 = np.array([[-c_psi, 0, s_psi, 0, 0, 0, 0, 0, 0, 0, 0, 0, c_psi, -s_psi, 0],
                    [s_psi, 0, c_psi, 0, 0, 0, 0, 0, 0, 0, 0, 0, -s_psi, -c_psi, 0],
                    [0, 0, 0, 0, 1 / (c_phi * c_theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 / (c_phi * c_theta)]],
                   dtype=float)

    H_2 = np.matmul(cam_coeff, H_1)
    H_3 = np.concatenate((np.concatenate((I_12_12, zero_12_3), axis=1), H_2), axis=0)
    return np.matmul(H_3, x)

q = filter_state()
m = meas_state()
a = alg_state()
c = ctrl_state()
p = param()
s = status()
v = vis()
dt = 1 / p['freq_est']

points = MerweScaledSigmaPoints(q['dim'], alpha=.1, beta=2., kappa=-1)
ukf = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)

print(ukf.P)


