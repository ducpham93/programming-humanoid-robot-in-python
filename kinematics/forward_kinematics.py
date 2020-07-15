'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

#from angle_interpolation import AngleInterpolationAgent
from recognize_posture import PostureRecognitionAgent

# below are listed the joints and their rotation relatively to their local coordinate system
# the joint names themselves denote the rotation in the global coordinate system
rollJoints = ['LElbowYaw', 'LHipRoll', 'LAnkleRoll', 'RElbowYaw', 'RHipRoll', 'RAnkleRoll']
pitchJoints = ['HeadPitch', 'LShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RShoulderPitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch']
yawJoints = ['HeadYaw', 'LShoulderRoll', 'LElbowRoll',  'RShoulderRoll', 'RElbowRoll']
wristJoints = ['LWristYaw', 'RWristYaw']
specialHipJoints = ['LHipYawPitch', 'RHipYawPitch']

class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            # YOUR CODE HERE
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                      }
        
        self.jointLengths = {
            'HeadYaw': [0, 0, 126.5], 'HeadPitch': [0, 0, 0],
            'LShoulderPitch': [0, 98, 100], 'LShoulderRoll': [0, 0, 0], 'LElbowYaw': [105, 15, 0], 'LElbowRoll': [0, 0, 0], 'LWristYaw': [55.95, 0, 0],
		    'LHipYawPitch': [0, 50, -85], 'LHipRoll': [0, 0, 0], 'LHipPitch': [0, 0, 0], 'LKneePitch': [0, 0, -100], 'LAnklePitch': [0, 0, -102.9], 'LAnkleRoll': [0, 0, 0],     
		    'RHipYawPitch': [0, -50, -85], 'RHipRoll': [0, 0, 0], 'RHipPitch': [0, 0, 0], 'RKneePitch': [0, 0, -100], 'RAnklePitch': [0, 0, -102.9], 'RAnkleRoll': [0, 0, 0],
		    'RShoulderPitch': [0, -98, 100], 'RShoulderRoll': [0, 0, 0], 'RElbowYaw': [105, -15, 0], 'RElbowRoll': [0, 0, 0], 'RWristYaw': [55.95, 0, 0]
                            }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)

        # create corresponding rotation matrices and multiply them with T
        if (joint_name in rollJoints):
            T = np.dot(T, np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]))
        elif (joint_name in pitchJoints):
            T = np.dot(T, np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]))
        elif (joint_name in yawJoints):
            T = np.dot(T, np.array([[c, s, 0, 0], [-s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
        # deal with hipyawpitch joints differently, as the yaw rotation is tilted by 45° 
        elif (joint_name in specialHipJoints):
            s_45 = np.sin(np.pi/4)
            c_45 = np.cos(np.pi/4)
            # create matrix to compensate leg tilt (in roll) first before applying yaw rotation
            comp = np.array([[1, 0, 0, 0], [0, c_45, -s_45, 0], [0, s_45, c_45, 0], [0, 0, 0, 1]])
            R_z = np.array([[c, s, 0, 0], [-s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            # tilt leg again after applying rotation
            # use inverse rotation (transposed matrix) for other leg (due to symmetry)
            if (joint_name == 'LHipYawPitch'):
                T = np.dot(T, comp)
                T = np.dot(T, R_z)
                T = np.dot(T, comp.T)
            elif (joint_name == 'RHipYawPitch'):
                T = np.dot(T, comp.T)
                T = np.dot(T, R_z.T)
                T = np.dot(T, comp)
            
        # apply translation according to given (fixed) dimensions of the joints
        T[:-1][:,-1] = np.asarray(self.jointLengths[joint_name]).reshape(3,1)
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        #for chain_joints in self.chains.values():
        T = identity(4)
        for joint in joints.keys():
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            # YOUR CODE HERE
            T = np.dot(T, Tl)
            self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
