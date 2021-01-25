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

from autograd.numpy import matrix, identity, sin, cos, pi

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def rotX(self, angle):
        return matrix([[1,0,0,0],
                       [0,cos(angle),-sin(angle),0],
                       [0,sin(angle),cos(angle),0],
                       [0,0,0,1]])
    
    def rotY(self, angle):
        return matrix([[cos(angle),0,sin(angle),0],
                       [0,1,0,0],
                       [-sin(angle),0,cos(angle),0],
                       [0,0,0,1]])
    
    def rotZ(self, angle):
        return matrix([[cos(angle),-sin(angle),0,0],
                       [sin(angle),cos(angle),0,0],
                       [0,0,1,0],
                       [0,0,0,1]])

    def translation(self, x, y, z):
        return matrix([[1,0,0,x],
                       [0,1,0,y],
                       [0,0,1,z],
                       [0,0,0,1]])
    
    def rotation(self, axis, angle):
        if axis == 'x' :
            return self.rotX(angle)
        elif axis == 'y': 
            return self.rotY(angle)
        else:
            return self.rotZ(angle)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        
        # translation offset in mm from previous jont or torso to joint with joint_name {joint_name: [x,y,z]}
        trans_offsets = {'HeadYaw':  [0, 0, 126.5 ],
                   'HeadPitch':      [0, 0, 0],
                   'LShoulderPitch': [0, 98, 100],
                   'LShoulderRoll':  [0, 0, 0],
                   'LElbowYaw':      [105, 15, 0],
                   'LElbowRoll':     [0, 0, 0],
                   'RShoulderPitch': [0, -98, 100],
                   'RShoulderRoll':  [0, 0, 0],
                   'RElbowYaw':      [105, -15, 0],
                   'RElbowRoll':     [0, 0, 0],
                   'LHipYawPitch':   [0, 50, -85],
                   'LHipRoll':       [0, 0, 0],
                   'LHipPitch':      [0, 0, 0],
                   'LKneePitch':     [0, 0, -100],
                   'LAnklePitch':    [0,0,-102.9],
                   'LAnkleRoll':     [0, 0, 0],
                   'RHipYawPitch':   [0, -50, -85],
                   'RHipRoll':       [0, 0, 0],
                   'RHipPitch':      [0, 0, 0],
                   'RKneePitch':     [0, 0, -100],
                   'RAnklePitch':    [0,0,-102.9],
                   'RAnkleRoll':     [0, 0, 0]
                   }
        
        # first array parameter is the rotation axis and second parameter is a factor for rotation direction (CW vs. CCW)
        rot_axis_counterClockwiseFactor = {
                   'HeadYaw':        ['z', 1],
                   'HeadPitch':      ['y', 1],
                   'LShoulderPitch': ['y', 1],
                   'LShoulderRoll':  ['z', 1],
                   'LElbowYaw':      ['y', 1],
                   'LElbowRoll':     ['z', 1],
                   'RShoulderPitch': ['y', 1],
                   'RShoulderRoll':  ['z', 1],
                   'RElbowYaw':      ['x', 1],
                   'RElbowRoll':     ['z', 1],
                   'LHipYawPitch':   ['z', -1],
                   'LHipRoll':       ['x', 1],
                   'LHipPitch':      ['y', 1],
                   'LKneePitch':     ['y', 1],
                   'LAnklePitch':    ['y', 1],
                   'LAnkleRoll':     ['x', 1],
                   'RHipYawPitch':   ['z', 1],
                   'RHipRoll':       ['x', 1],
                   'RHipPitch':      ['y', 1],
                   'RKneePitch':     ['y', 1],
                   'RAnklePitch':    ['y', 1],
                   'RAnkleRoll':     ['x', 1]
                   }
    
        # translational part
        o = trans_offsets[joint_name]
        T = self.translation(o[0], o[1], o[2])
    
        # special joint orientations
        if joint_name == "LHipYawPitch":
            T = T.dot(self.rotation('x', pi/4))
            
        if joint_name == "RHipYawPitch":
            T = T.dot(self.rotation('x', -pi/4))
        
        if joint_name == "LHipRoll":
            T = T.dot(self.rotation('x', -pi/4))
        
        if joint_name == "RHipRoll":
            T = T.dot(self.rotation('x', pi/4))
        
    
        # rotate joint / add rotational part
        r = rot_axis_counterClockwiseFactor[joint_name]
        T = T.dot(self.rotation(r[0], joint_angle * r[1]))
    
        # joint rotation
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T.dot(Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
