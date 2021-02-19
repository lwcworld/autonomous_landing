import rosbag
import numpy as np
from matplotlib import pyplot as plt


def plot_target_imm():
    plt.figure(1)
    plt.plot(time_list, target_list)
    plt.title('target position')
    plt.xlabel('time stamp')
    plt.ylabel('error for target position estimation')
    plt.show()


def plot_cov_imm():
    plt.figure(2)
    plt.plot(time_list, cov_list[:, 0], time_list, cov_list[:, 1], time_list, cov_list[:, 2])
    plt.title('covariance')
    plt.xlabel('time stamp')
    plt.ylabel('covariance')
    plt.show()


def plot_mu_imm():
    plt.figure(3)
    plt.plot(time_list, mu_list[:, 0], time_list, mu_list[:, 1], time_list, mu_list[:, 2])
    plt.title('Mode Probability')
    plt.xlabel('time stamp')
    plt.ylabel('mu')
    plt.show()


def plot_target_kf():
    plt.figure(4)
    plt.plot(time_list_kf, target_list_kf)
    plt.title('target position')
    plt.xlabel('time stamp')
    plt.ylabel('error for target position estimation')
    plt.show()


def plot_cov_kf():
    plt.figure(5)
    plt.plot(time_list_kf, cov_list_kf[:, 0], time_list_kf, cov_list_kf[:, 1], time_list_kf, cov_list_kf[:, 2])
    plt.title('covariance')
    plt.xlabel('time stamp')
    plt.ylabel('covariance')
    plt.show()


def plot_target():
    plt.figure(6)
    plt.plot(time_list, target_list, time_list_kf, target_list_kf)
    plt.title('target position estimation error')
    plt.xlabel('time stamp')
    plt.ylabel('error for target position estimation')
    plt.legend(['imm', 'kf'])
    plt.show()


def plot_cov():
    plt.figure(7)
    plt.plot(time_list, cov_list[:, 0], time_list, cov_list[:, 1], time_list, cov_list[:, 2], time_list_kf,
             cov_list_kf[:, 0], time_list_kf, cov_list_kf[:, 1], time_list_kf, cov_list_kf[:, 2])
    plt.title('covariance')
    plt.xlabel('time stamp')
    plt.ylabel('covariance')
    plt.legend(['imm_x', 'imm_y', 'imm_z', 'kf_x', 'kf_y', 'kf_z'])
    plt.show()


# #############################################################################################
# ############################# IMM ###########################################################
# #############################################################################################

bag_dir = '/home/lics/Hyeonmun/rosbag/imm_3.bag'
bag = rosbag.Bag(bag_dir)

target_list = []
cov_list = []
mu_list = []
time_list = []

for topic, msg, t in bag.read_messages(topics=['plot_target']):
    target_list.append(msg.data)
    time_list.append(t.to_sec())

for topic, msg, t in bag.read_messages(topics=['plot_covariance']):
    cov_list.append([msg.vector.x, msg.vector.y, msg.vector.z])

for topic, msg, t in bag.read_messages(topics=['plot_mu']):
    mu_list.append([msg.vector.x, msg.vector.y, msg.vector.z])

bag.close()

time_stamp = range(len(target_list))
target_list = np.array(target_list)
cov_list = np.array(cov_list)
mu_list = np.array(mu_list)

# #############################################################################################
# ############################# KF ############################################################
# #############################################################################################

bag_dir = '/home/lics/Hyeonmun/rosbag/kf_3.bag'
bag = rosbag.Bag(bag_dir)

target_list_kf = []
cov_list_kf = []
time_list_kf = []
for topic, msg, t in bag.read_messages(topics=['plot_target']):
    target_list_kf.append(msg.data)
    time_list_kf.append(t.to_sec())

for topic, msg, t in bag.read_messages(topics=['plot_covariance']):
    cov_list_kf.append([msg.vector.x, msg.vector.y, msg.vector.z])

bag.close()

time_stamp_kf = range(len(target_list_kf))
target_list_kf = np.array(target_list_kf)
cov_list_kf = np.array(cov_list_kf)

print('target shape', np.shape(target_list_kf))
print('time shape', np.shape(time_list_kf))

temp = time_list[0]
for ii in range(len(time_list)):
    time_list[ii] -= temp - time_list_kf[0]

#############################################################################################

plot_target_imm()
plot_cov_imm()
plot_mu_imm()

plot_target_kf()
plot_cov_kf()

plot_target()
plot_cov()
