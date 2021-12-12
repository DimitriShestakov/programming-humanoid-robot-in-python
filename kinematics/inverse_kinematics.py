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
import scipy as scip
from math import atan2


def from_trans(T):
    x = T[3, 0]
    y = T[3, 1]
    z = T[3, 2]

    if T[0, 0] == 1:
        return [x, y, z, atan2(T[2, 1], T[1, 1])]
    elif T[1, 1] == 1:
        return [x, y, z, atan2(T[0, 2], T[0, 0])]
    elif T[2, 2] == 1:
        return [x, y, z, atan2(T[1, 0], T[0, 0])]
    else:
        return [T[3, 0], T[3, 1], T[3, 2], 0]


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics


        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        lambda_ = 0.001
        maxMove = 0.1
        for name in self.chains[effector_name]:
            joint_angles[name] = self.perception.joint[name]

        target = np.matrix([from_trans(transform)]).T

        for i in range(5000):
            Ts = self.forward_kinematics(joint_angles)
            Te = np.matrix([from_trans(Ts[-1])]).T
            e = target - Te
            e[e > maxMove] = maxMove
            e[e < -maxMove] = -maxMove

            T = np.matrix([from_trans(i) for i in Ts[0:]]).T

            J = Te - T
            J[0, :] = -(Te - T)[1, :]
            J[1, :] = (Te - T)[0, :]
            J[-1, :] = 1
            joint_angles[name] += np.asarray((lambda_ *
                                              scip.linalg.pinv(J) * e).T)[0]

            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(
                    (lambda_ * scip.linalg.pinv(J) * e).T)[0][i]

            if np.linalg.norm(lambda_ * scip.linalg.pinv(J) * e) < 1e-3:
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        self.keyframes = ([], [], [])
        angles = self.inverse_kinematics(effector_name, transform)
        for i, joint in enumerate(self.chains[effector_name]):
            self.keyframes[0].append(joint)
            self.keyframes[1].append([0, 3])
            self.keyframes[2].append([[self.perception.joint[joint], [3, 0, 0], [
                                     3, 0, 0]], [angles[i], [3, 0, 0], [3, 0, 0]]])


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
