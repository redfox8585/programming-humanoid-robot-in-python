'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
from threading import Thread
from keyframes import hello
import time

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        
    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

        def threadFunc():
            sp = xmlrpc.client.ServerProxy('http://localhost:8000')
            sp.execute_keyframes(keyframes)
        
        t = Thread(target=threadFunc)
        t.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        
        def threadFunc():
            sp = xmlrpc.client.ServerProxy('http://localhost:8000')
            sp.set_transform(effector_name, transform)
        
        t = Thread(target=threadFunc)
        t.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.proxy = xmlrpc.client.ServerProxy('http://localhost:8000')
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()
        

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.proxy.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.proxy.set_transform(effector_name, transform)


if __name__ == '__main__':
    
    agent = ClientAgent()
    # TEST CODE HERE

    print(agent.get_angle("LShoulderPitch"))
    agent.set_angle("LShoulderPitch", 0.4)
    print(agent.get_posture())


    agent.post.execute_keyframes(hello.hello())
    print("doing something else ...")
    
    agent.execute_keyframes(hello.hello())
    
    
    agent.get_transform("LShoulderPitch")
    
    