#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=invalid-name, too-many-instance-attributes

from __future__ import (absolute_import, division)
import numpy as np
from numpy import dot, asarray, zeros, outer
from filterpy.common import pretty_str
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common.discretization import Q_discrete_white_noise
from scipy.linalg import block_diag


class IMMEstimator(object):
    def __init__(self, p, q, m):
        dt = 1 / p['freq_est']
        points = MerweScaledSigmaPoints(q['dim'], alpha=.1, beta=2., kappa=-1)

        # declare 3 ukfs
        self.ukf1 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf2 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf3 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf4 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf5 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf6 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf7 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf8 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)
        self.ukf9 = UnscentedKalmanFilter(dim_x=q['dim'], dim_z=m['dim'], dt=dt, fx=fx, hx=hx, points=points)

        self.filters = [self.ukf1, self.ukf2, self.ukf3, self.ukf4, self.ukf5, self.ukf6, self.ukf7, self.ukf8, self.ukf9]
        if len(self.filters) < 2:
            raise ValueError('filters must contain at least two filters')
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

        # update Q for each filters
        Q_pos_o = Q_discrete_white_noise(dim=2, dt=dt, var=1. ** 2, block_size=3)
        Q_ang_o = Q_discrete_white_noise(dim=2, dt=dt, var=0.1 ** 2, block_size=3)
        Q_pos_t = np.eye(3) * 1e-1
        for f in self.filters:
            f.Q = block_diag(Q_pos_o, Q_ang_o, Q_pos_t)
            # f.P = f.P*0.2

        mu = [1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0, 1.0 / 9.0]
        self.mu = asarray(mu / np.sum(mu))
        self.M = q['TM']

        x_shape = self.filters[0].x.shape
        for f in self.filters:
            if x_shape != f.x.shape:
                raise ValueError(
                    'All filters must have the same state dimension')

        self.x = x_0  # initial state
        self.P = zeros(self.filters[0].P.shape)

        self.N = len(self.filters)  # number of filters
        self.likelihood = zeros(self.N)
        self.omega = zeros((self.N, self.N))
        self._compute_mixing_probabilities()

        # initialize imm state estimate based on current filters
        self._compute_state_estimate()
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def predict(self):
        # compute mixed initial conditions
        xs, Ps = [], []
        for i, (f, w) in enumerate(zip(self.filters, self.omega.T)):
            x = zeros(self.x.shape)
            for kf, wj in zip(self.filters, w):
                x += kf.x * wj
            xs.append(x)

            P = zeros(self.P.shape)
            for kf, wj in zip(self.filters, w):
                y = kf.x - x
                P += wj * (outer(y, y) + kf.P)
            Ps.append(P)

        #  compute each filter's prior using the mixed initial conditions
        for i, f in enumerate(self.filters):
            # propagate using the mixed state estimate and covariance
            f.x = xs[i].copy()
            f.P = Ps[i].copy()
            f.predict()

        # compute mixed IMM state and covariance and save posterior estimate
        self._compute_state_estimate()
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()


    def update(self, q, m, T_now, p):
        tuning_param = [[850. / (q['z_o'] - q['z_t']), 850. / (q['z_o'] - q['z_t'])],
                        [850. / (q['z_o'] - q['z_t']), 1000. / (q['z_o'] - q['z_t'])],
                        [850. / (q['z_o'] - q['z_t']), 1150. / (q['z_o'] - q['z_t'])],
                        [1000. / (q['z_o'] - q['z_t']), 850. / (q['z_o'] - q['z_t'])],
                        [1000. / (q['z_o'] - q['z_t']), 1000. / (q['z_o'] - q['z_t'])],
                        [1000. / (q['z_o'] - q['z_t']), 1150. / (q['z_o'] - q['z_t'])],
                        [1150. / (q['z_o'] - q['z_t']), 850. / (q['z_o'] - q['z_t'])],
                        [1150. / (q['z_o'] - q['z_t']), 1000. / (q['z_o'] - q['z_t'])],
                        [1150. / (q['z_o'] - q['z_t']), 1150. / (q['z_o'] - q['z_t'])]
                        ]

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

        R_o = np.eye(m['dim_o']) * (m['z_std_o'] ** 2)
        R_vis = np.eye(m['dim_vis']) * (m['z_std_vis'] ** 2)
        R_r = np.eye(m['dim_r']) * (m['z_std_r'] ** 2)

        T_o = m['T_o']
        T_vis = m['T_vis']
        T_r = m['T_r']

        idx_o = list(range(0, 12))
        idx_vis = list(range(12, 14))
        idx_r = list(range(14, 15))

        if T_now - T_o > 1 / p['freq_est']:
            for f in self.filters:
                f.R[idx_o, idx_o] = f.R[idx_o, idx_o] * (m['z_std_bad'] ** 2)  # 1 standard
        if T_now - T_vis > 1 / p['freq_est']:
            for f in self.filters:
                f.R[idx_vis, idx_vis] = f.R[idx_vis, idx_vis] * (m['z_std_bad'] ** 2)  # 1 standard
                print('Vision is missing')
        if T_now - T_r > 1 / p['freq_est']:
            for f in self.filters:
                f.R[idx_r, idx_r] = f.R[idx_r, idx_r] * (m['z_std_bad'] ** 2)  # 1 standard
                print('LRF is missing')

        # run update on each filter, and save the likelihood
        for i, f in enumerate(self.filters):
            f.R = block_diag(R_o, R_vis, R_r)
            f.update(z, alpha=tuning_param[i][0], beta=tuning_param[i][1])
            self.likelihood[i] = f.likelihood

        # update mode probabilities from total probability * likelihood
        self.mu = self.cbar * self.likelihood
        self.mu /= np.sum(self.mu)  # normalize
        self._compute_mixing_probabilities()

        # compute mixed IMM state and covariance and save posterior estimate
        self._compute_state_estimate()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

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
         q['z_t']) = self.x.copy()
        q['P'] = self.P_post.copy()

        return q

    def _compute_state_estimate(self):
        """
        Computes the IMM's mixed state estimate from each filter using
        the the mode probability self.mu to weight the estimates.
        """
        self.x.fill(0)
        for f, mu in zip(self.filters, self.mu):
            self.x += f.x * mu

        self.P.fill(0)
        for f, mu in zip(self.filters, self.mu):
            y = f.x - self.x
            self.P += mu * (outer(y, y) + f.P)

    def _compute_mixing_probabilities(self):
        """
        Compute the mixing probability for each filter.
        """

        self.cbar = dot(self.mu, self.M)
        for i in range(self.N):
            for j in range(self.N):
                self.omega[i, j] = (self.M[i, j] * self.mu[i]) / self.cbar[j]

    def __repr__(self):
        return '\n'.join([
            'IMMEstimator object',
            pretty_str('x', self.x),
            pretty_str('P', self.P),
            pretty_str('x_prior', self.x_prior),
            pretty_str('P_prior', self.P_prior),
            pretty_str('x_post', self.x_post),
            pretty_str('P_post', self.P_post),
            pretty_str('N', self.N),
            pretty_str('mu', self.mu),
            pretty_str('M', self.M),
            pretty_str('cbar', self.cbar),
            pretty_str('likelihood', self.likelihood),
            pretty_str('omega', self.omega)])


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
