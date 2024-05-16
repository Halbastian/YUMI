#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt


from Controller.controller import YumiController
from Controller.control_target import TaskSpaceVelocityControlTarget
from Controller.utils import TrajectoryPoint
from parameters import Parameters

from abb_robot_msgs.msg import SystemState
from abb_robot_msgs.srv import TriggerWithResultCode
from controller.msg import Trajectory_point, Trajectory_msg
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Int64

# k is the gain for vel = vel + k*[error, wrench]
# Gain for individual Control
K_P_I = 4  # Gain for positional error
K_O_I = 2  # Gain for angular error
#NYTT
K_F_I = 0.03  # Gain for force error
K_M_I = 0.7  # Gain for moment error

# Gain for absolute control
K_P_A = 3  # Gain for positional error
K_O_A = 3  # Gain for angular error
#NYTT
K_F_A = 0.02  # Gain for force error
K_M_A = 0.4  # Gain for moment error

# Gain for relative control
K_P_R = 4  # Gain for positional error
K_O_R = 4  # Gain for angular error
#NYTT
K_F_R = 0.02  # Gain for force error
K_M_R = 0.4  # Gain for moment error

#NYTT
thresh_force = 3.0 #N
thresh_torque = 0.8 #Nm

FxL = []
FyL = []
FzL = []
MxL = []
MyL = []
MzL = []

FxR = []
FyR = []
FzR = []
MxR = []
MyR = []
MzR = []

class TrajectoryController(YumiController):
    """Class for running trajectory control, trajectory parameters are sent with ros and
    from those a trajectory is constructed and followed."""
    def __init__(self):
        super(TrajectoryController, self).__init__()
        self.control_target = TaskSpaceVelocityControlTarget(Parameters.dT)
        self.lockTrajectory = threading.Lock()
        self.maxDeviation = np.array([0.015, 0.25, 0.015, 0.25])*4
        self.reset = False
        self.pubSubTask = rospy.Publisher('/controller/sub_task', Int64, queue_size=1)
        # trajectory-reset callback
        self.auto_mode = True
        self.restart_rapid = rospy.ServiceProxy('/yumi/rws/start_rapid', TriggerWithResultCode)
        rospy.Subscriber("/yumi/rws/system_states", SystemState, self.callback_system_state, queue_size=3, tcp_nodelay=True)
        
        #NYTT
        # read force sensors
        self.wrenches = np.zeros(12)  # f_R, m_R, f_L, m_L
        rospy.Subscriber("/ftsensor_r/world_tip_force", WrenchStamped, self.callback_ext_force_r, queue_size=3, tcp_nodelay=True)
        rospy.Subscriber("/ftsensor_l/world_tip_force", WrenchStamped, self.callback_ext_force_l, queue_size=3, tcp_nodelay=True)

        #NyttOss
        #self.pub = rospy.Publisher('/objectpos', Trajectory_point, queue_size=1, tcp_nodelay=True)

    def callback_system_state(self, data):
        auto_mode = data.auto_mode

        if not self.auto_mode and auto_mode:
            self.reset_trajectory()
            self.restart_rapid()
            print('trajectory reset')

        self.auto_mode = auto_mode
    
    def reset_trajectory(self):
        self.lockTrajectory.acquire()
        self.control_target = TaskSpaceVelocityControlTarget(Parameters.dT)
        self.lockTrajectory.release()

    #NYTT Right hand
    def callback_ext_force_r(self, data):
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        mx = data.wrench.torque.x
        my = data.wrench.torque.y
        mz = data.wrench.torque.z
        self.wrenches[0:3] = np.array([fx, fy, fz])
        self.wrenches[3:6] = np.array([mx, my, mz])
        FxR.append(fx)
        FyR.append(fy)
        FzR.append(fz)
        MxR.append(mx)
        MyR.append(my)
        MzR.append(mz)

    #NYTT Left Hand
    def callback_ext_force_l(self, data):
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        mx = data.wrench.torque.x
        my = data.wrench.torque.y
        mz = data.wrench.torque.z
        self.wrenches[6:9] = np.array([fx, fy, fz])
        self.wrenches[9:12] = np.array([mx, my, mz])
        FxL.append(fx)
        FyL.append(fy)
        FzL.append(fz)
        MxL.append(mx)
        MyL.append(my)
        MzL.append(mz)
    
    #NYTTUseless
    @staticmethod
    def smooth_start(force, thresh):
        n = np.linalg.norm(force)
        if n < thresh:
            if np.allclose(n, 0):
                return force
            f = (-1/thresh**2)*(n**3) + (2/thresh)*(n**2)
        else:
            f = n
        return f * (force / n)

    #NyttOss
    def plot():
        time = range(0,len(FxL),1)
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        # Forces on left arm
        axs[0, 0].plot(time, FxL, label='FxL')
        axs[0, 0].plot(time, FyL, label='FyL')
        axs[0, 0].plot(time, FzL, label='FzL')
        axs[0, 0].set_title('Forces on Left Arm')
        axs[0, 0].set_xlabel('Time')
        axs[0, 0].set_ylabel('Force')
        axs[0, 0].legend()

        # Forces on right arm
        axs[0, 1].plot(time, FxR, label='FxR')
        axs[0, 1].plot(time, FyR, label='FyR')
        axs[0, 1].plot(time, FzR, label='FzR')
        axs[0, 1].set_title('Forces on Right Arm')
        axs[0, 1].set_xlabel('Time')
        axs[0, 1].set_ylabel('Force')
        axs[0, 1].legend()

        # Moments on left arm
        axs[1, 0].plot(time, MxL, label='MxL')
        axs[1, 0].plot(time, MyL, label='MyL')
        axs[1, 0].plot(time, MzL, label='MzL')
        axs[1, 0].set_title('Moments on Left Arm')
        axs[1, 0].set_xlabel('Time')
        axs[1, 0].set_ylabel('Moment')
        axs[1, 0].legend()

        # Moments on right arm
        axs[1, 1].plot(time, MxR, label='MxR')
        axs[1, 1].plot(time, MyR, label='MyR')
        axs[1, 1].plot(time, MzR, label='MzR')
        axs[1, 1].set_title('Moments on Right Arm')
        axs[1, 1].set_xlabel('Time')
        axs[1, 1].set_ylabel('Moment')
        axs[1, 1].legend()

        plt.tight_layout()
        plt.show()


    #NyttOss (Ha ifsats innan calculate...)
    #Mata in m, YL, XL, rz
    def calculateObjectPos(self):
        [fxr,fyr,fzr, mxr,myr,mzr,fxl,fyl,fzl,mxl,myl,mzl] = self.wrenches[0:12]
        Fnr = [fxr,fyr,fzr]#[0,0,fzr]
        Fnl = [fxl,fyl,fzl]#[0,0,fzl]
        Fb=m*9.82
        #Antar att origo ligger vid höger endeffector.Låt Yl, Xl beteckna avståndet i C,Y led från origo till left endeffector
        #Man söker Xb,Yb positionkoordinater på objekgtet.
        ry=(Yl*fzl)/Fb
        rx=(Xl*fzl)/Fb
        #Fn1 + Fn2 = F Och Mz=0, Mx= ry * F + 0*Fn2 Och My = rx * F + Traylängd * Fn2

        #rz kan gettas för att den är samma hela tiden

        msg = Trajectory_point()
        msg.positionRight = [rx, ry, rz]

        return msg
        #För horisontella krafter verkande på kanten: Fn1z + Fn2z = Fz Och Mx = 0 Och My = 0 Och Mz = F*rx



    def policy(self):
        """Gets called for each time step and calculates the target velocity"""
        if self.auto_mode == False:
            action = dict()  # used to store the desired action
            action['controlSpace'] = 'jointSpace'
            action['jointVelocities'] = np.zeros(Parameters.Dof)
            #print('control loop off')
            return
        
        # Update the pose for controlTarget class
        self.lockTrajectory.acquire()

        self.control_target.updatePose(self.yumiGripPoseR, self.yumiGripPoseL)
        # resets yumi to init pose
        if self.reset:
            self.plot()
            self.reset = self.resetPose()
            if self.reset == False:
                self.control_target = TaskSpaceVelocityControlTarget(Parameters.dT)
            self.lockTrajectory.release()
            return

        # calculates target velocities and positions
        self.control_target.updateTarget()

        action = dict()  # used to store the desired action
        # send commands to the grippers
        if self.control_target.checkNewTrajectorySegment():
            action['gripperRight'] = self.control_target.gripperRight
            action['gripperLeft'] = self.control_target.gripperLeft

        # sets the control mode
        action['controlSpace'] = self.control_target.mode

        # k is the gain for vel = vel + k*error
        if self.control_target.mode == 'individual':
            # pos_R, rot_R, pos_L, rot_L
            action['cartesianVelocity'] = self.control_target.getIndividualTargetVelocity(k_p=K_P_I, k_o=K_O_I)            
           
            #NYTT Useless
            # wrench compensation
            # K_F = np.array([K_F_I, K_F_I, K_F_I])
            # K_M = np.array([K_M_I, K_M_I, K_M_I])
            # # right arm
            # action['cartesianVelocity'][0:3] -= K_F * (self.smooth_start(self.wrenches[0:3], thresh_force))
            # action['cartesianVelocity'][3:6] -= K_M * self.smooth_start(self.wrenches[3:6], thresh_torque)
            # # left arm
            # action['cartesianVelocity'][6:9] -= K_F * (self.smooth_start(self.wrenches[6:9], thresh_force))
            # action['cartesianVelocity'][9:12] -= K_M * self.smooth_start(self.wrenches[9:12], thresh_torque)

        elif self.control_target.mode == 'coordinated':
            # pos_abs, rot_abs, pos_abs, rot_abs
            action['absoluteVelocity'] = self.control_target.getAbsoluteTargetVelocity(k_p=K_P_A, k_o=K_O_A)
            action['relativeVelocity'] = self.control_target.getRelativeTargetVelocity(k_p=K_P_R, k_o=K_O_R)
            
            #NYTT Useless
            # # wrench absolute
            # K_F = np.array([K_F_A, K_F_A, K_F_A])
            # K_M = np.array([K_M_A, K_M_A, K_M_A])
            # wrench_abs = self.wrenches[0:6] + self.wrenches[6:12]
            # action['absoluteVelocity'][0:3] -= K_F * self.smooth_start(wrench_abs[0:3], thresh_force)
            # action['absoluteVelocity'][3:6] -= K_M * self.smooth_start(wrench_abs[3:6], thresh_torque)
            # # wrench relative
            # K_F = np.array([K_F_R, K_F_R, K_F_R])
            # K_M = np.array([K_M_R, K_M_R, K_M_R])
            # wrench_rel = np.zeros(6) #self.wrenches[0:6] - self.wrenches[6:12]
            # action['relativeVelocity'][6:9] -= K_F * self.smooth_start(wrench_rel[0:3], thresh_force)
            # action['relativeVelocity'][9:12] -= K_M * self.smooth_start(wrench_rel[3:6], thresh_torque)

        # check so deviation from the trajectory is not too big, stop if it is
        # (turned of if gripperCollision is active for individual mode)
        if self.control_target.checkTrajectoryDeviation(self.maxDeviation):
            print('Deviation from trajectory too large, stopping')
            action['controlSpace'] = 'jointSpace'
            action['jointVelocities'] = np.zeros(Parameters.Dof)

        self.setAction(action)

        #NyttOss
        # if np.any(self.wrenches[0:12]):
        #     p = self.calculateObjectPos()
        #     self.pub.publish(p)

        # sends information about which part of the trajectory is being executed
        msgSubTask = Int64()
        msgSubTask.data = self.control_target.trajectory.index - 1
        self.lockTrajectory.release()
        self.pubSubTask.publish(msgSubTask)

    def callback_trajectory(self, data):
        """ Gets called when a new set of trajectory parameters is received
         The variable names in this function and the the trajectory class follows
         individual motion with left and right . This means when coordinate manipulation
         is usd, right is absolute motion and left becomes relative motion. """

        if not self.control_target.dataReceived:
            print('No data received, start robot or simulation before sending trajectory')
            return
        # get current pose, used as first trajectory parameter
        if data.mode == 'coordinated':
            positionRight = np.copy(self.control_target.absolutePosition)
            positionLeft  = np.copy(self.control_target.relativePosition)
            orientationRight = np.copy(self.control_target.absoluteOrientation)
            orientationLeft = np.copy(self.control_target.rotationRelative)
            self.reset = False
        elif data.mode == 'individual':
            positionRight = np.copy(self.control_target.translationRightArm)
            positionLeft  = np.copy(self.control_target.translationLeftArm)
            orientationRight = np.copy(self.control_target.rotationRightArm)
            orientationLeft = np.copy(self.control_target.rotationLeftArm)
            self.reset = False
        elif data.mode == 'resetPose':
            self.reset = True
            return
        else:
            print('Error, mode not matching combined or individual')
            return
        # current gripper position 
        gripperLeft = self.control_target.gripperLeft
        gripperRight = self.control_target.gripperRight
        # add current state as a trajectory point
        currentPoint = TrajectoryPoint(
            positionRight=positionRight,
            positionLeft=positionLeft,
            orientationRight=orientationRight,
            orientationLeft=orientationLeft,
            gripperLeft=gripperLeft,
            gripperRight=gripperRight)
        trajectory = [currentPoint]

        # append trajectory points from msg
        for i in range(len(data.trajectory)):
            if data.mode == 'coordinated':
                positionRight = np.asarray(data.trajectory[i].positionAbsolute)
                positionLeft  = np.asarray(data.trajectory[i].positionRelative)
                orientationRight = np.asarray(data.trajectory[i].orientationAbsolute)
                orientationLeft = np.asarray(data.trajectory[i].orientationRelative)
            else:
                positionRight = np.asarray(data.trajectory[i].positionRight)
                positionLeft  = np.asarray(data.trajectory[i].positionLeft)
                orientationRight = np.asarray(data.trajectory[i].orientationRight)
                orientationLeft = np.asarray(data.trajectory[i].orientationLeft)

            gripperLeft = data.trajectory[i].gripperLeft
            gripperRight = data.trajectory[i].gripperRight
            pointTime = np.asarray(data.trajectory[i].pointTime)
            trajectoryPoint = TrajectoryPoint(
                positionRight=positionRight,
                positionLeft=positionLeft,
                orientationRight=orientationRight,
                orientationLeft=orientationLeft,
                gripperLeft=gripperLeft,
                gripperRight=gripperRight,
                pointTime=pointTime)
            trajectory.append(trajectoryPoint)

        # use current velocity for smoother transitions, (only for translation)
        if self.control_target.checkTrajectoryDeviation(self.maxDeviation):
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)
        elif self.control_target.mode == data.mode:  # no change in control mode
            velLeftInit = np.copy(self.control_target.targetVelocities[6:9])
            velRightInit = np.copy(self.control_target.targetVelocities[0:3])
        elif self.control_target.mode == 'individual':  # going from individual to coordinate motion
            # simple solution, not fully accurate transition
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.control_target.targetVelocities[0:3]) + np.copy(self.control_target.targetVelocities[6:9]))
        elif self.control_target.mode == 'coordinated':  # going from coordinated to individual motion
            # simple solution, not fully accurate transition
            velLeftInit = np.copy(self.control_target.targetVelocities[0:3])
            velRightInit = np.copy(self.control_target.targetVelocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)

        self.lockTrajectory.acquire()
        # set mode
        self.control_target.mode = data.mode

        # update the trajectory
        self.control_target.trajectory.updateTrajectory(trajectory, velLeftInit, velRightInit)
        self.control_target.trajIndex = 0
        self.control_target.trajectorySegment = 0
        self.lockTrajectory.release()


def main():
    """main function"""
    # starting ROS node and subscribers
    rospy.init_node('trajectoryController', anonymous=True) 

    ymuiContoller = TrajectoryController()
    rospy.Subscriber("/trajectory", Trajectory_msg, ymuiContoller.callback_trajectory, queue_size=1, tcp_nodelay=True)

    rospy.spin()

if __name__ == '__main__':
    main()