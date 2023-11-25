'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
import threading
import sys
import os
from numpy.matlib import identity
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import *

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = obj

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform)).start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.proxy = xmlrpc.client.ServerProxy('http://localhost:8000', allow_none=True)
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        return self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.proxy.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        return self.proxy.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # Test the functions
    keyframes = leftBackToStand()
    agent.execute_keyframes(keyframes)
    print('Executed keyframe')

    # joint_angle = agent.get_angle('HeadYaw')
    # print(f'Current angle of HeadYaw: {joint_angle}')

    # agent.set_angle('HeadYaw', 10.0)
    # print('Set angle of HeadYaw to 10.0 degrees')

    # posture = agent.get_posture()
    # print(f'Current posture: {posture}')

    #this somehow gets a weird error i can't seem to fix
    # T = identity(4)
    # T[-1, 1] = 0.05
    # T[-1, 2] = -0.26  
    # agent.set_transform("LLeg", T)



