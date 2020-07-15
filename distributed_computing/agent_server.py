'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
import time
from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client as client
import threading

from inverse_kinematics import InverseKinematicsAgent

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print("GETTING ANGLE")
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print("SETTING ANGLE TO " + str(angle) + " FOR JOINT " + joint_name)
        self.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("GETTING POSTURE")
        self.posture = self.recognize_posture(self.perception)
        print(self.posture)
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("EXECUTING KEYFRAME")
        # set keyframes
        self.keyframes = keyframes
        self.angle_interpolation(keyframes, self.perception)
        # blocking procedure
        start_time = time.time()
        current_time = time.time()
        all_times = keyframes[1]
        finished = False
        # measure time since execution start
        while not finished:
            current_time = time.time()
            finished = True
            # keep executing if there is one joint that has not finished yet
            for i in range(len(all_times)):
                if (current_time - start_time < all_times[i][-1]):
                    finished = False
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print("GETTING TRANSFORM")
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("SETTING TRANSFORM")
        self.set_transforms(self, effector_name, transform)
        return True

def start_server(address, port):
    server = SimpleXMLRPCServer((address, port))
    server.register_instance(ServerAgent(), allow_dotted_names=True)
    server.register_introspection_functions()
    server.register_multicall_functions()
    print('Serving XML-RPC on localhost port 8000')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting. Shutting down server.")
        sys.exit(0)

if __name__ == '__main__':
    agent = ServerAgent()
    thread = threading.Thread(target=start_server, args=["localhost", 8000])
    thread.start()
    time.sleep(5)
    print("Starting Agent")
    #thread = threading.Thread(target=agent.run(), args=[])
    #thread.start()
    #start_server("localhost", 8000)
    agent.run()


