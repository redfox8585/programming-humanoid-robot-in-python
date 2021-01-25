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

from inverse_kinematics import InverseKinematicsAgent
from threading import Thread, Lock
from xmlrpc.server import SimpleXMLRPCServer
import time

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(InverseKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        
        self.mutex = Lock()
        self.mutex.acquire()
        
        self.rpcServer = SimpleXMLRPCServer(('127.0.0.1', 8000))
        
        self.rpcServer.register_function(self.get_angle)
        self.rpcServer.register_function(self.set_angle)
        self.rpcServer.register_function(self.get_posture)
        self.rpcServer.register_function(self.execute_keyframes)
        self.rpcServer.register_function(self.get_transform)
        self.rpcServer.register_function(self.set_transform)
        
        self.serverThread = Thread(target=self.rpcServer.serve_forever)
        self.serverThread.start()
        print("XML RPC server listening on port 8000")
        
    
    def think(self, perception):
        
        
        self.tmpPerception = perception
        #self.posture = PostureRecognitionAgent.recognize_posture(perception)
        
        self.mutex.release()
        
        # allow RPC server thread to execute methods of ServerAgent
        time.sleep(0.01)
        
        self.mutex.acquire()
        
        return super(InverseKinematicsAgent, self).think(perception)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        self.mutex.acquire()
        
        angle = self.tmpPerception.joint[joint_name]
        
        self.mutex.release()
        
        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.mutex.acquire()
        
        #self.target_joints: target positions (dict: joint_id -> position (target))
        self.target_joints[joint_name] = angle        
        
        self.mutex.release()
        return 0

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        self.mutex.acquire()
        p = self.posture
        self.mutex.release()
        
        return p

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        
        self.mutex.acquire()

        # if keyframe not finished        
        if self.tmpPerception.time <= self.keyframe_offset_time + 12.0:
            return 0
        
        self.keyframes = keyframes
        self.keyframe_offset_time = self.tmpPerception.time
        
        
        self.mutex.release()
        
        return 1

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.mutex.acquire()
        
        # updated in ForwardKinematicsAgent.think()
        t = self.transforms[name]
        
        self.mutex.release()
        return t

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.mutex.acquire()
        
        self.set_transform(effector_name, transform)
        
        self.mutex.release()
        return 0

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

