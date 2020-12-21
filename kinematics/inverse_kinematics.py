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

import autograd.numpy as np
from autograd import grad



class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def leg_forward_kinematics(self, joints, leg_name):
        T = identity(4)
        for joint in self.chains[leg_name]:
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            
            T = T.dot(Tl)

        return T
        
    
    def error_func(self, angles, target, effector_name) : 
        
        joint_names = self.chains[effector_name]
        joints = {}
        for i in range(0, len(joint_names)):
            joints[joint_names[i]] = angles[i]
            i += 1
        
        current = self.leg_forward_kinematics(joints, effector_name)
        error = target - current
        
        res = np.sum(error*error)
                
        return res
    
    
    def normalizeAngles(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        
        while angle < -np.pi:
            angle += 2.0*np.pi
            
        return angle
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        
        
        # using the numerical approach with autograd
        # to improve accuracy decrease max_error and increase max_iterations below
        max_error = 0.01
        max_iterations = 300
        speed = 0.1
        
        
        # calculate symbolic gradient function with autograd for a fixed goal transform
        error_func2 = lambda angles: self.error_func(angles, transform, effector_name)
        symGradFunc = grad(error_func2)
        
        joint_angles = np.array([0.1,0.1,0.1,0.1,0.1,0.1])
        
        for i in range(0, max_iterations):
            
            for i in range(0, len(joint_angles)):
                joint_angles[i] = self.normalizeAngles(joint_angles[i])
            
            joint_angles -= symGradFunc(joint_angles) * speed
            error = error_func2(joint_angles)
            if error < max_error:
                break
            
        
            
        return joint_angles

    

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        
        joint_angles = self.inverse_kinematics(effector_name, transform)
        
        names = list()
        times = list()
        keys = list()
        
        
        for chain_names in self.chains:
            i = 0
            for joint_name in self.chains[chain_names]:
                if effector_name == chain_names:
                    names.append(joint_name)
                    times.append([0, 100000])
                    keys.append([[joint_angles[i], [3, -1.0, 0.0], [3, 1.0, 0.0]], [joint_angles[i], [3, -1.0, 0.0], [3, 1.0, 0.0]]])
                    i += 1
                else:
                    names.append(joint_name)
                    times.append([0, 100000])
                    keys.append([[0, [3, -1.0, 0.0], [3, 1.0, 0.0]],[0, [3, -1.0, 0.0], [3, 1.0, 0.0]]])
                    i += 1
            
    
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, -1] = 0.05
    T[2, -1] = 0.26
    
    T = agent.leg_forward_kinematics({'LHipYawPitch': 0.0,
                                  'LHipRoll' : 0.0,
                                  'LHipPitch' : 0.0,
                                  'LKneePitch': 0.001,
                                  'LAnklePitch': 0.0,
                                  'LAnkleRoll': 0.0
                                  }, 'LLeg')
    
    agent.set_transforms('LLeg', T)
    agent.run()
