import numpy as np

class filter_state:
    def __init__(self):
        # ownship
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.yaw_o = 0

        self.Px_o = 0
        self.Py_o = 0
        self.Pz_o = 0
        self.Pyaw_o = 0

        # target
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
        self.x_o = 0
        self.y_o = 0
        self.z_o = 0
        self.vx_o = 0
        self.vy_o = 0
        self.vz_o = 0
        self.roll_o = 0
        self.pitch_o = 0
        self.yaw_o = 0
        self.d_roll_o = 0
        self.d_pitch_o = 0
        self.d_yaw_o = 0

        # target
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

class alg_state:
    def __init__(self):
        self.N_H = 2
        self.P_H = np.ones((self.N_H))/self.N_H
        self.th_L = 0.8 # threshold for landing possible


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