#!/usr/bin/env python
import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np


def main():
    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True)
    pub = rospy.Publisher('/trajectory', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'individual'  # control mode
    msg.trajectory = []

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.5, -0.2, 0.2]  # poition right arm [m], yumi_base_link is the origin
    trajectoryPoint.positionLeft = [0.5, 0.2, 0.2]  # poition left arm [m]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]  # orientation left arm, quaterniorns [x, y, z, w]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]  # orientation right arm
    trajectoryPoint.gripperLeft = 20.0  # gripper width for the fingers [mm]
    trajectoryPoint.gripperRight = 20.0
    trajectoryPoint.pointTime = 8.0  # time to get to this point [s]
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.5, -0.2, 0.2]  # poition right arm [m], yumi_base_link is the origin
    trajectoryPoint.positionLeft = [0.5, 0.2, 0.2]  # poition left arm [m]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]  # orientation left arm, quaterniorns [x, y, z, w]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]  # orientation right arm
    trajectoryPoint.gripperLeft = 0.0  # gripper width for the fingers [mm]
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 12.0  # time to get to this point [s]
    msg.trajectory.append(trajectoryPoint)

    pub.publish(msg)

    print('sent msg 1 (individual)')
    rospy.sleep(31)


if __name__ == '__main__':
    main()


