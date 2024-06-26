

import numpy as np
import rospy
import tf
from trajectory import Trajectory
import utils
from parameters import Parameters


class TaskSpaceVelocityControlTarget(object):  # generates target velocity in task space taking into account external forces
    """ Generates velocity commands in cartesian space for following a trajectory"""
    def __init__(self, deltaTime):
        self.mode = 'individual'  # can be either 'individual' or 'coordinated'
        self.trajectory = Trajectory(deltaTime)  # instance of trajectory class
        self.targetVelocities = np.zeros(12)  # in [m/s] and [rad/s]
        self.error = np.zeros(12)  # in [m] and [rad]
        self.trajectorySegment = 0  # keeps track of which trajectory segment currently executed
        # listen to pose
        self.gripperLeft = 0  # Gripper position in [mm]
        self.gripperRight = 0
        self.dataReceived = False
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

    def getIndividualTargetVelocity(self, k_p, k_o):
        """Calculates the target velocities for individual control.
        :param k_p: float for position gain.
        :param k_o: float for orientation gain.
        """

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.error[0:3] = utils.PositionError(self.translationRightArm, self.targetPosition[0:3], 0.1)
        self.error[3:6] = utils.RotationError(self.rotationRightArm, self.targetOrientation[0:4], 0.2)
        self.error[6:9] = utils.PositionError(self.translationLeftArm, self.targetPosition[3:6], 0.1)
        self.error[9:12] = utils.RotationError(self.rotationLeftArm, self.targetOrientation[4:8], 0.2)

        K = np.array([k_p, k_p, k_p, k_o, k_o, k_o, k_p, k_p, k_p, k_o, k_o, k_o])
        
        self.targetVelocities = self.desiredVelocity + K*self.error

        return self.targetVelocities  # pos_R, rot_R, pos_L, rot_L

    def getAbsoluteTargetVelocity(self, k_p, k_o):
        """Calculates the target velocities for absolute motion i.e. controlling
        the average of the grippers.
        :param k_p: float for position gain.
        :param k_o: float for orientation gain.
        """

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.error[0:3] = utils.PositionError(self.absolutePosition, self.targetPosition[0:3], 0.1)
        self.error[3:6] = utils.RotationError(self.absoluteOrientation, self.targetOrientation[0:4], 0.2)

        K = np.array([k_p, k_p, k_p, k_o, k_o, k_o])

        self.targetVelocities[0:6] = self.desiredVelocity[0:6] + K*self.error[0:6]

        return self.targetVelocities[0:6]  # pos_abs, rot_abs, (pos_abs, rot_abs)

    def getRelativeTargetVelocity(self,  k_p, k_o):
        """Calculates the target velocities for relative motion i.e. controlling
        the grippers relative to each other in absolute frame.
        :param k_p: float for position gain.
        :param k_o: float for orientation gain.
        """

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)
        
        self.error[6:9] = utils.PositionError(self.relativePosition, self.targetPosition[3:6], 0.1)
        self.error[9:12] = utils.RotationError(self.rotationRelative, self.targetOrientation[4:8], 0.2)

        K = np.array([k_p, k_p, k_p, k_o, k_o, k_o])

        self.targetVelocities[6:12] = self.desiredVelocity[6:12] + K*self.error[6:12]

        return self.targetVelocities[6:12]  # (pos_abs, rot_abs), pos_rel, rot_rel

    def updateTarget(self):
        """ updates the desired velocities and target position from the trajectory """
        if len(self.trajectory.trajectory) < 2: 
            return
        self.targetPosition, self.targetOrientation, self.desiredVelocity, self.gripperLeft, self.gripperRight = self.trajectory.getTarget()

    def updatePose(self, yumiGripPoseR, yumiGripPoseL):
        """ updates the pose and calculates the pose for relative and absolute motion as well """
        self.translationRightArm = yumiGripPoseR.getPosition()
        self.translationLeftArm = yumiGripPoseL.getPosition()
        self.rotationRightArm = yumiGripPoseR.getQuaternion()
        self.rotationLeftArm = yumiGripPoseL.getQuaternion()

        # absolute pose, avg of the grippers
        tfMatrixRight = self.transformer.fromTranslationRotation(self.translationRightArm, self.rotationRightArm)
        tfMatrixLeft = self.transformer.fromTranslationRotation(self.translationLeftArm, self.rotationLeftArm)

        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        self.absoluteOrientation = utils.averageQuaternions(avgQ)  
        self.absolutePosition = 0.5*(self.translationRightArm + self.translationLeftArm)

        # relative pose, difference of the grippers in absolute frame
        transformationAbsolute = self.transformer.fromTranslationRotation(self.absolutePosition, self.absoluteOrientation)
        transformationAbsoluteInv = np.linalg.pinv(transformationAbsolute)

        transformationRightFromAbs = transformationAbsoluteInv.dot(tfMatrixRight)
        transformationLeftFromAbs = transformationAbsoluteInv.dot(tfMatrixLeft)
        quatRightAbs = tf.transformations.quaternion_from_matrix(transformationRightFromAbs)
        posRightAbs = tf.transformations.translation_from_matrix(transformationRightFromAbs)
        quatLeftAbs = tf.transformations.quaternion_from_matrix(transformationLeftFromAbs)
        posLeftAbs = tf.transformations.translation_from_matrix(transformationLeftFromAbs)

        self.rotationRelative = tf.transformations.quaternion_multiply(quatRightAbs, tf.transformations.quaternion_conjugate(quatLeftAbs))
        self.relativePosition = posRightAbs - posLeftAbs
        self.dataReceived = True

    def checkNewTrajectorySegment(self):
        """ Returns True if a new segment has been entered, only shows true
         once for each segment. Used to check if new gripper commands should be sent."""
        if self.trajectorySegment != self.trajectory.index:
            self.trajectorySegment = self.trajectory.index
            return True
        else:
            return False

    def checkTrajectoryDeviation(self, maxTrajectoryDeviation):
        """
        Returns true if any of the deviation limits for trajectory following has been violated.
        :param maxTrajectoryDeviaiton: np.array([maxRightT, maxRightR, maxLeftT, maxLeftR]), shape(4)
        """
        
        if self.mode == 'individual' and Parameters.feasibilityObjectives['gripperCollision']:
            return False
        
        errorRightPos = np.linalg.norm(self.error[0:3])
        errorRightRot = np.linalg.norm(self.error[3:6])
        errorLeftPos = np.linalg.norm(self.error[6:9])
        errorLeftRot = np.linalg.norm(self.error[9:12])
        deviation = np.max(np.array([errorRightPos, errorRightRot, errorLeftPos, errorLeftRot]) - maxTrajectoryDeviation)
        
        if deviation > 0:
            return True

        return False

