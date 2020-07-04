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
import math


class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def get_values(self, matrix):
        x, y, z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
     
        t_x = math.atan2(matrix[2, 1], matrix[2, 2])
        t_y = math.atan2(-matrix[2, 0], math.sqrt(matrix[2, 1] * matrix[2, 1] + matrix[2, 2] * matrix[2, 2]))
        t_z = math.atan2(matrix[1, 0], matrix[0, 0])
            
        return np.array([x, y, z, t_x, t_y, t_z])
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        lambda_ = 0.002

        print("START")
        
        for name in self.chains[effector_name]:
            joint_angles[name] = self.perception.joint[name]

        target = np.array(self.get_values(transform.T))
        
        for i in range(1000):
            self.forward_kinematics(joint_angles)
            Ts = []
            for i in range(len(self.chains[effector_name])):
                Ts.append(0)
            for i, name in enumerate(self.chains[effector_name]):
                Ts[i] = self.transforms[name]
            Te = np.array(self.get_values(Ts[-1]))
            e = target - Te
            T = np.array([self.get_values(k) for k in Ts[:]])
            J = Te - T
            J[:,-1] = 1
            d_theta = np.dot(np.dot(J, np.linalg.pinv(np.dot(J.T, J))), e.T) * lambda_
            
            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(d_theta)[i]
            if  np.linalg.norm(d_theta) < 1e-3:
                break
            
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        names = []
        times = []
        keys = []
        self.forward_kinematics(self.perception.joint)
        angles = self.inverse_kinematics(effector_name, transform)
        print(angles)

        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([1.0, 3.0])
            keys.append([[angles[joint] - 0.01, [3, 0, 0], [3, 0, 0]], [angles[joint], [3, 0, 0], [3, 0, 0]]]) 
        self.keyframes = (names, times, keys) # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
