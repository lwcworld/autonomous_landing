#!/usr/bin/env python
import numpy as np


class Scenario():
    def __init__(self):
        # ======== ROS communicators declaration ========
        self.sp_pos = [0.0, 0.0, 5.0]

        self.timeline = np.array([0, 10, 20, 30])
        self.N_wp = np.shape(self.timeline)[0]
        self.goal = np.zeros((self.N_wp, 3))
        self.goal[0] = [0, 0, 5]
        self.goal[1] = [0, 0, 10]
        self.goal[2] = [10, 0, 10]
        self.goal[3] = [10, 0, 10]

    def target_pos(self, T_now, q):
        idx = np.argmax(np.where(self.timeline <= T_now))
        return self.goal[idx]
