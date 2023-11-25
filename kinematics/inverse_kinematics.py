'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from numpy import sin, cos, pi, matrix, random, linalg, asarray
from scipy.linalg import pinv
from math import atan2

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def from_trans(self, m):
        '''get x, y, z, theta from 4x4 transform matrix'''
        return [m[3, 0], m[3, 1], m[3, 2], atan2(m[0, 1], m[0, 0])]
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        #using jacobian from the jupyter notebook
        N = len(self.chains[effector_name])
        # joint_angles = np.random.random(N) * 1e-5 this is way too small
        joint_angles = np.random.random(N)
        # print(joint_angles)
        lambda_ = 1
        max_step = 0.1
        
        target = np.matrix([self.from_trans(transform)]).T
        for i in range(1000):
            Ts = [self.transforms[n] for n in self.chains[effector_name]]
            Te = np.matrix([self.from_trans(Ts[-1])]).T
            error = target - Te
            error[error > max_step] = max_step
            error[error < -max_step] = -max_step
            T = np.matrix([self.from_trans(i) for i in Ts[1:-1]]).T
            J = Te -T
            dT = Te - T
            J[0, :] = -dT[2, :] # x
            J[1, :] = dT[1, :] # y
            J[2, :] = dT[0, :] # z
            J[-1, :] = 1  # angular
            d_theta = lambda_ * pinv(J) * error
            joint_angles += asarray(d_theta)[0]
            if(linalg.norm(d_theta)< 1e-4):
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        '''
        * Keyframe data format:
            keyframe := (names, times, keys)
            names := [str, ...]  # list of joint names
            times := [[float, float, ...], [float, float, ...], ...]
            # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
            keys := [[float, [int, float, float], [int, float, float]], ...]
            # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
            # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
            # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
            # preceding the point, the second describes the curve following the point.
        '''
        names = []
        times = []
        keys = []
        for index,joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([1.0,2.0])
            keys.append([[self.perception.joint[joint],[3,0,0],[3,0,0]],
                         [angles[index],[3,0,0],[3,0,0]]])
        self.keyframes = (names,times,keys)
        # print(self.keyframes)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()