from __future__ import print_function

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def gcd(a, b):
    if b > a:
        tmp = a
        a = b
        b = tmp
    while b > 0:
        c = b
        b = a % b
        a = c
    return a


def lcm(a, b):
    return a * b // gcd(a, b)


def diag_block_mat_slicing(L):
    shp = L[0].shape
    N = len(L)
    r = range(N)
    out = np.zeros((N, shp[0], N, shp[1]), dtype=int)
    out[r, :, r, :] = L
    return out.reshape(np.asarray(shp) * N)


def plot_data(data):
    data = np.array(data)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]
    vx = data[:, 4]
    vy = data[:, 5]
    vz = data[:, 6]
    roll = data[:, 7]
    pitch = data[:, 8]
    yaw = data[:, 9]
    d_roll = data[:, 10]
    d_pitch = data[:, 11]
    d_yaw = data[:, 12]
    mount_pitch = data[:, 13]
    mount_roll = data[:, 14]


    # t vs. x y z vx vy vz
    plt.figure(1)
    plt.subplot(231)
    plt.plot(t, x)
    plt.xlabel('t')
    plt.ylabel('x')
    plt.subplot(232)
    plt.plot(t, y)
    plt.xlabel('t')
    plt.ylabel('y')
    plt.subplot(233)
    plt.plot(t, z)
    plt.xlabel('t')
    plt.ylabel('z')
    plt.subplot(234)
    plt.plot(t, vx)
    plt.xlabel('t')
    plt.ylabel('vx')
    plt.subplot(235)
    plt.plot(t, vy)
    plt.xlabel('t')
    plt.ylabel('vy')
    plt.subplot(236)
    plt.plot(t, vz)
    plt.xlabel('t')
    plt.ylabel('vz')
    plt.show()

    # t vs. roll pitch yaw d_roll d_pitch d_yaw
    plt.figure(2)
    plt.subplot(231)
    plt.plot(t, roll)
    plt.xlabel('t')
    plt.ylabel('roll')
    plt.subplot(232)
    plt.plot(t, pitch)
    plt.xlabel('t')
    plt.ylabel('pitch')
    plt.subplot(233)
    plt.plot(t, yaw)
    plt.xlabel('t')
    plt.ylabel('yaw')
    plt.subplot(234)
    plt.plot(t, d_roll)
    plt.xlabel('t')
    plt.ylabel('d_roll')
    plt.subplot(235)
    plt.plot(t, d_pitch)
    plt.xlabel('t')
    plt.ylabel('d_pitch')
    plt.subplot(236)
    plt.plot(t, d_yaw)
    plt.xlabel('t')
    plt.ylabel('d_yaw')
    plt.show()

    # t vs. mount control of pitch and roll
    plt.figure(3)
    plt.subplot(221)
    plt.plot(t, pitch)
    plt.xlabel('t')
    plt.ylabel('pitch')
    plt.subplot(222)
    plt.plot(t, mount_pitch)
    plt.xlabel('t')
    plt.ylabel('mount_pitch')
    plt.subplot(223)
    plt.plot(t, roll)
    plt.xlabel('t')
    plt.ylabel('roll')
    plt.subplot(224)
    plt.plot(t, mount_roll)
    plt.xlabel('t')
    plt.ylabel('mount_roll')
    plt.show()

    # x y z
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, alpha=0.6, marker='o')
    ax.set_title('3D trajectory of Typhoon h480')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()


def plot_data_2(data):
    data = np.array(data)
    t = data[:, 0]
    x_t = data[:, 1]
    y_t = data[:, 2]
    z_t = data[:, 3]
    P = data[:, 4]
    # print('P dimension: ', np.shape(P))
    # print(P)
    P_diag_list = []
    for iii in range(len(P)):
        temp = P[iii].diagonal()
        P_diag_list.append(temp)
    P_diag_list = np.array(P_diag_list)
    P_0 = P_diag_list[:, 0]
    P_1 = P_diag_list[:, 1]
    P_2 = P_diag_list[:, 2]
    mu = data[:, 5]
    mu = np.array(mu)
    mu_0 = []
    mu_1 = []
    mu_2 = []
    for muu in mu:
        mu_0.append(muu[0])
        mu_1.append(muu[1])
        mu_2.append(muu[2])




    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_t, y_t, z_t, alpha=0.6, marker='o')
    ax.set_title('3D trajectory of filtered target position')
    ax.set_xlabel('x_t')
    ax.set_ylabel('y_t')
    ax.set_zlabel('z_t')
    plt.show()

    plt.figure(1)
    plt.plot(t, P_0, t, P_1, t, P_2)
    plt.xlabel('t')
    plt.ylabel('covariances')
    plt.show()

    plt.figure(2)
    plt.plot(t, mu_0, t, mu_1, t, mu_2)
    plt.xlabel('t')
    plt.ylabel('mode probabilities')
    plt.show()