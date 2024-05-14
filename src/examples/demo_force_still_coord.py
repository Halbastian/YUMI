#!/usr/bin/env python
import rospy

import numpy as np
import tf

from controller.msg import Trajectory_point, Trajectory_msg


def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True)
    pub = rospy.Publisher('/trajectory', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'coordinated'  # now controlling with the coordinated motion mode
    msg.trajectory = []

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.30, 0]
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 20.0
    trajectoryPoint.gripperRight = 20.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.30, 0]
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    pub.publish(msg)

    print('sent msg 1 (coordinated)')
    rospy.sleep(31)


if __name__ == '__main__':
    main()


