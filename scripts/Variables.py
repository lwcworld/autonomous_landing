import numpy as np

class filter_state:
    def __init__(self):
        # ownship (enu)
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.yaw_o = 0

        self.Px_o = 0
        self.Py_o = 0
        self.Pz_o = 0
        self.Pyaw_o = 0

        # target  (enu)
        self.x_t = 0
        self.y_t = 0
        self.z_t = 0

        self.Px_t = 0
        self.Py_t = 0
        self.Pz_t = 0

    def __getitem__(self,key):
        return getattr(self, key)

    def __setitem__(self,key,value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class meas_state:
    def __init__(self):
        # ownship
        # (enu)
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.vx_o = 0
        self.vy_o = 0
        self.vz_o = 0
        # (ned)
        self.roll_o = 0
        self.pitch_o = 0
        self.yaw_o = 0
        self.d_roll_o = 0
        self.d_pitch_o = 0
        self.d_yaw_o = 0

        # imu
        self.imu_roll = 0
        self.imu_pitch = 0
        self.imu_yaw = 0

        # target
        # (ned)
        self.px_t = 0
        self.py_t = 0
        self.r = 0

        # timestamp
        self.T_vis = 0

    def __getitem__(self,key):
        return getattr(self, key)

    def __setitem__(self,key,value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class param:
    def __init__(self):
        self.freq_est = 20
        self.freq_ctrl = 20

        self.N_H = 2 # number of hypothesis (0:landing condition, 1:not landing condition)
        self.th_L = 0.8 # threshold for landing possible

        self.cm = np.array([[0.6, 0.4], [0.4, 0.5]]) # confusion matrix
        self.P_H_min = 0.01

        self.cam_bound = [1920, 1080] # camera resolution
        self.th_viscen = 200 # threshold for target centered

    def __getitem__(self,key):
        return getattr(self, key)

class alg_state:
    def __init__(self):
        self.P_H = np.array([0.1, 0.9]) # hypothetical probabilities

    def __getitem__(self,key):
        return getattr(self, key)

    def __setitem__(self,key,value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class ctrl_state:
    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.d_roll = 0
        self.d_pitch = 0
        self.d_yaw = 0

    def __getitem__(self,key):
        return getattr(self, key)

    def __setitem__(self,key,value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class status:
    def __init__(self):
        self.phase = 0 # 0:scenario, 1:landing
        self.flag_KF_init = False # checker for KF initiation

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')