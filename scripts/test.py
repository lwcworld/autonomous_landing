#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    rospy.init_node("test")
    T = rospy.get_time()
    print(T)