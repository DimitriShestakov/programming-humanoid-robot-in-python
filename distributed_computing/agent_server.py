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
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
from recognize_posture import PostureRecognitionAgent
from inverse_kinematics import InverseKinematicsAgent
import os
import sys
import numpy as np
import pickle
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'kinematics'))


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC servic
    '''
    # YOUR CODE HERE

    def __init__(self):
        super(ServerAgent, self).__init__()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE

        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE

        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return pickle.dumps(self.recognize_posture(self.perception))

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE

        self.keyframes = pickle.loads(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

        return pickle.dumps(self.local_trans(name, self.perception.joint[name]))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        self.set_transforms(effector_name, pickle.loads(transform))


server = SimpleXMLRPCServer(('localhost', 9000))
server.register_introspection_functions()
server.register_instance(ServerAgent())
server.serve_forever()


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()
