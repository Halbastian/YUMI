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

    msg = Trajectory_msg()  # message will contain list of trajectory points
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'individual'  # control mode
    msg.trajectory = []

    # ---------------
    trajectoryPoint = Trajectory_point()  # point
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2]  # poition right arm [m], yumi_base_link is the origin
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]  # poition left arm [m]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]  # orientation left arm, quaterniorns [x, y, z, w]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]  # orientation right arm
    trajectoryPoint.gripperLeft = 20.0  # gripper width for the fingers [mm]
    trajectoryPoint.gripperRight = 20.0
    trajectoryPoint.pointTime = 12.0  # time to get to this point [s]
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.04]
    trajectoryPoint.positionLeft = [0.35, 0.1, 0.04]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0
    trajectoryPoint.gripperRight = 0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.04]
    trajectoryPoint.positionLeft = [0.35, 0.15, 0.04]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 20
    trajectoryPoint.gripperRight = 20
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.04]
    trajectoryPoint.positionLeft = [0.35, 0.15, 0.04]
    trajectoryPoint.orientationLeft = [1, 0, 0, 0]
    trajectoryPoint.orientationRight = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0
    trajectoryPoint.gripperRight = 0
    trajectoryPoint.pointTime = 2.0
    msg.trajectory.append(trajectoryPoint)

    pub.publish(msg)
    print('sent individual')
    rospy.sleep(31)

    # --------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'coordinated'  # now controlling with the coordinated motion mode
    msg.trajectory = []

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.35, 0.0, 0.2]  # absolute  position, the avg of the gripper positions
    trajectoryPoint.positionRelative = [0, 0.25, 0]  # relatibe  position, the difference of the gripper positions in the absolute frame
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.1, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.1, 0.4]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.35, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.orientationAbsolute = [1, 0, 0, 0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    pub.publish(msg)
    print('sent msg 2, coordinated ')
    rospy.sleep(33)

    # --------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'coordinated'
    msg.trajectory = []

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 210 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 150 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)
    
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(40 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(-40 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0, 40 * np.pi / 180, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0, -40 * np.pi / 180, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 16.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = [0, 0, 0, 1]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 60 * np.pi / 180, 'rzyx')
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationAbsolute = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 180 * np.pi / 180, 'rzyx')
    trajectoryPoint.orientationRelative = tf.transformations.quaternion_from_euler(0 * np.pi / 180, 0, 0 * np.pi / 180, 'rzyx')
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0
    msg.trajectory.append(trajectoryPoint)

    pub.publish(msg)
    print('sent msg 3, coordinated ')
    rospy.sleep(96)

    # --------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'resetPose'
    pub.publish(msg)
    print('resetPose')
    rospy.spin()


if __name__ == '__main__':
    main()


