#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

# def smooth_start(force, thresh):
#     n = np.linalg.norm(force)
#     if n < thresh:
#         f = (-1/thresh**2)*(n**3) + (2/thresh)*(n**2)
#     else:
#         f = n
#     return f * (force / n)


# x = np.linspace(0, 2, 100)
# y = np.zeros_like(x)
# for i in range(len(x)):
#     y[i] = smooth_start(x[i], 1.2)

# plt.plot(x, y)
# plt.show()


import sys
print(sys.path)


[
    '/home/spot/yumi_ws/src/yumi/controller/src/examples', 
    '/home/spot/yumi_ws/devel/lib/python3/dist-packages', 
    '/opt/ros/melodic/lib/python2.7/dist-packages', 
    '/usr/lib/python2.7', 
    '/usr/lib/python2.7/plat-x86_64-linux-gnu', 
    '/usr/lib/python2.7/lib-tk', 
    '/usr/lib/python2.7/lib-old', 
    '/usr/lib/python2.7/lib-dynload', 
    '/home/spot/.local/lib/python2.7/site-packages', 
    '/usr/local/lib/python2.7/dist-packages', 
    '/usr/lib/python2.7/dist-packages', 
    '/usr/lib/python2.7/dist-packages/wx-3.0-gtk3'
]