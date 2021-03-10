#!/usr/bin/env python
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common.discretization import Q_discrete_white_noise
from scipy.linalg import block_diag

import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped

from tf.transformations import euler_from_quaternion


def update_H(a, m, p, T_now):
    # check
    # 1) target is centered in camera view
    # 2) measurement is obtained succesively

    c_1, c_2, c_3, c_4 = False, False, False, False

    T_vis = m['T_vis']
    T_r = m['T_r']
    T_o = m['T_o']

    # condition 1
    if T_now - T_vis < 1 / p['freq_est']:
        c_1 = True

    # condition 2
    if T_now - T_r < 1 / p['freq_est']:
        c_2 = True

    # condition 3
    if T_now - T_o < 1 / p['freq_est']:
        c_3 = True

    # print(c_1, c_2, c_3)

    # condition 4
    px_t, py_t = m['px_t'], m['py_t']
    if abs(px_t) < p['th_viscen'] and abs(py_t) < p['th_viscen']:
        c_4 = True

    if c_1 == True and c_2 == True and c_3 == True:
        idx_status = 0  # good measure
    else:
        idx_status = 1  # not good measure

    a['P_H'] = np.multiply(p['cm'][:, idx_status], a['P_H']) / np.matmul(p['cm'][:, idx_status], a['P_H'])
    a['P_H'] = np.maximum(p['P_H_min'], a['P_H'])
    a['P_H'] = a['P_H'] / np.sum(a['P_H'])

    return a


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


def init_KF(q, s, m, p):
    dt = 1 / p['freq_est']

    points = MerweScaledSigmaPoints(q['dim'], alpha=.1, beta=2., kappa=-1)
    ukf = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)

    x_0 = np.array([m['x_o'],
                    m['vx_o'],
                    m['y_o'],
                    m['vy_o'],
                    m['z_o'],
                    m['vz_o'],
                    m['roll_o'],
                    m['d_roll_o'],
                    m['pitch_o'],
                    m['d_pitch_o'],
                    m['yaw_o'],
                    m['d_yaw_o'],
                    m['px_t'],
                    m['py_t'],
                    m['r']])

    ukf.x = x_0  # initial state

    # ukf.P *= 0.2  # initial uncertainty

    Q_pos_o = Q_discrete_white_noise(dim=2, dt=dt, var=1. ** 2, block_size=3)
    Q_ang_o = Q_discrete_white_noise(dim=2, dt=dt, var=0.1 ** 2, block_size=3)
    Q_pos_t = np.eye(3) * 1e-1
    ukf.Q = block_diag(Q_pos_o, Q_ang_o, Q_pos_t)

    ukf.predict()

    (q['x_o'],
     q['vx_o'],
     q['y_o'],
     q['vy_o'],
     q['z_o'],
     q['vz_o'],
     q['roll_o'],
     q['d_roll_o'],
     q['pitch_o'],
     q['d_pitch_o'],
     q['yaw_o'],
     q['d_yaw_o'],
     q['x_t'],
     q['y_t'],
     q['z_t']) = ukf.x

    q['P'] = ukf.P

    return q, ukf


def update_KF(q, m, p, ukf, T_now):
    T_o = m['T_o']
    T_vis = m['T_vis']
    T_r = m['T_r']

    idx_o = list(range(0, 12))
    idx_vis = list(range(12, 14))
    idx_r = list(range(14, 15))

    R_o = np.eye(m['dim_o']) * (m['z_std_o'] ** 2)
    R_vis = np.eye(m['dim_vis']) * (m['z_std_vis'] ** 2)
    R_r = np.eye(m['dim_r']) * (m['z_std_r'] ** 2)
    ukf.R = block_diag(R_o, R_vis, R_r)

    if T_now - T_o > 1 / p['freq_est']:
        ukf.R[idx_o, idx_o] = ukf.R[idx_o, idx_o] * (m['z_std_bad'] ** 2)  # 1 standard
        print('FCC state is missing')
    if T_now - T_vis > 1 / p['freq_est']:
        ukf.R[idx_vis, idx_vis] = ukf.R[idx_vis, idx_vis] * (m['z_std_bad'] ** 2)  # 1 standard
        print('Vision is missing')
    if T_now - T_r > 1 / p['freq_est']:
        ukf.R[idx_r, idx_r] = ukf.R[idx_r, idx_r] * (m['z_std_bad'] ** 2)  # 1 standard
        print('LRF is missing')

    z = np.array([m['x_o'],
                  m['vx_o'],
                  m['y_o'],
                  m['vy_o'],
                  m['z_o'],
                  m['vz_o'],
                  m['roll_o'],
                  m['d_roll_o'],
                  m['pitch_o'],
                  m['d_pitch_o'],
                  m['yaw_o'],
                  m['d_yaw_o'],
                  m['px_t'],
                  m['py_t'],
                  m['r']])

    ukf.predict()
    ukf.update(z, alpha=1000. / (q['z_o'] - q['z_t']), beta=1000. / (q['z_o'] - q['z_t']))

    (q['x_o'],
     q['vx_o'],
     q['y_o'],
     q['vy_o'],
     q['z_o'],
     q['vz_o'],
     q['roll_o'],
     q['d_roll_o'],
     q['pitch_o'],
     q['d_pitch_o'],
     q['yaw_o'],
     q['d_yaw_o'],
     q['x_t'],
     q['y_t'],
     q['z_t']) = ukf.x

    q['P'] = ukf.P

    print('Q value: ', ukf.Q)
    return q, ukf


def ctrl(c, q, p_t, phase):
    # distance values to the target point
    e_x = p_t[0] - q['x_o']
    e_y = p_t[1] - q['y_o']
    e_z = p_t[2] - q['z_o']
    e_tot = np.sqrt(e_x ** 2 + e_y ** 2 + e_z ** 2)

    if phase == 0:  # waypoint
        # if total distance to the target exceeds 3m, move at the constant velocity
        if e_tot > 1.0:
            v_tot = 1.
        # if total distance to the target is less than 3m, move gradually to the target
        else:
            v_tot = e_tot
        v_x_cmd = e_x / e_tot * v_tot
        v_y_cmd = e_y / e_tot * v_tot
        v_z_cmd = e_z / e_tot * v_tot

    elif phase == 1:  # landing
        # yaw = q['yaw_o']
        # if yaw angle exceeds 15 degree, yaw control
        # if yaw > 0.2617 or yaw < -0.2617:
        #     c['d_yaw'] = - yaw / 3.0
        alt = q['z_o']

        # if altitude exceeds 2m, descent at the constant velocity
        # while move gradually in a horizontal

        if alt > 1:
            v_x_cmd = e_x
            v_y_cmd = e_y
            v_z_cmd = - 1.
        # if altitude is less than 2m, descent gradually without horizontal movement
        elif alt > 0.5:
            v_x_cmd = e_x
            v_y_cmd = e_y
            v_z_cmd = - alt

        else:
            v_x_cmd = e_x
            v_y_cmd = e_y
            v_z_cmd = 0

    elif phase == 2:
        v_x_cmd = e_x
        v_y_cmd = e_y
        v_z_cmd = 0

    c['vx'] = v_x_cmd
    c['vy'] = v_y_cmd
    c['vz'] = v_z_cmd

    return c

def assign_m_o_2_q_o(q, m):
    q['x_o'] = m['x_o']
    q['y_o'] = m['y_o']
    q['z_o'] = m['z_o']
    q['roll_o'] = m['roll_o']
    q['pitch_o'] = m['pitch_o']
    q['yaw_o'] = m['yaw_o']
    return q
