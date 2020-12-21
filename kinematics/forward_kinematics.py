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

from numpy.matlib import matrix, identity, sin, cos, pi

from angle_interpolation import AngleInterpolationAgent

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

'''
change this parameter for 2D output
'''
plotIn3D = True

if plotIn3D :
    ax = plt.axes(projection='3d')


# point and vector structure [[x],[y], [...]]
def plotVector(point, direction, color):
    if plotIn3D :
        ax.plot3D([point[0,0], direction[0,0]+point[0,0]],
              [point[1,0], direction[1,0]+point[1,0]],
              [point[2,0], direction[2,0]+point[2,0]],
              color)
    else:
        plt.plot([point[0,0], direction[0,0]+point[0,0]], [point[1,0], direction[1,0]+point[1,0]], color=color)


"""
coo is a standard transform matrix
coo is a 4x4 matrix. columns represent: {x-vector, y-vector, z-vector, point of origin}
a point is represented by (x,y,z,1) and a vector by (x,y,z,0)

size is a scaling factor for coordinate axes
"""
def plotCoordinateSystem(coo, size = 1):
    x = coo[:,0]
    y = coo[:,1]
    z = coo[:,2]
    origin = coo[:,3]
    
    plotVector(origin, x * size, 'r')
    plotVector(origin, y * size, 'g')
    plotVector(origin, z * size, 'b')

class ForwardKinematicsAgent(AngleInterpolationAgent):
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
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'LAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       #'Body': ['Head', 'LArm', 'LLeg', 'RLeg', 'RArm']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def rotX(self, angle):
        T = identity(4)
        T[1,1] = cos(angle)
        T[1,2] = -sin(angle)
        T[2,1] = sin(angle)
        T[2,2] = cos(angle)
        
        return T
    
    def rotY(self, angle):
        T = identity(4)
        T[0,0] = cos(angle)
        T[0,2] = sin(angle)
        T[2,0] = -sin(angle)
        T[2,2] = cos(angle)
        
        return T
    
    def rotZ(self, angle):
        T = identity(4)
        T[0,0] = cos(angle)
        T[0,1] = -sin(angle)
        T[1,0] = sin(angle)
        T[1,1] = cos(angle)
        
        return T
    
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
                   'LElbowYaw':      [0, 15, -105],
                   'LElbowRoll':     [0, 0, 0],
                   'RShoulderPitch': [0, -98, 100],
                   'RShoulderRoll':  [0, 0, 0],
                   'RElbowYaw':      [0, 15, -105],
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
        
        for chain_joints in self.chains.values():
            for joint in chain_joints:
                offset = trans_offsets[joint]
                trans_offsets[joint] = self.translation(offset[0], offset[1], offset[2])
        
        # rotational offsets
        rot_offsets = {}
        for chain_joints in self.chains.values():
            for joint in chain_joints:
                 rot_offsets[joint] = identity(4)
                 
        rot_offsets['LShoulderRoll'] = self.rotY(-pi/2)
                 
        rot_offsets['RShoulderRoll'] = matrix([[1,0,0,0],
                                              [0,-1,0,0],
                                              [0,0,1,0],
                                              [0,0,0,1]]).dot(self.rotY(-pi/2))
    
        rot_offsets['LHipYawPitch'] = self.rotX(pi/4)
    
        rot_offsets['RHipYawPitch'] = matrix([[1,0,0,0],
                                             [0,-1,0,0],
                                             [0,0,1,0],
                                             [0,0,0,1]]).dot(self.rotX(pi/4))
    
        rot_offsets['LHipRoll'] = self.rotX(-pi/4)
        rot_offsets['RHipRoll'] = self.rotX(-pi/4)
                
        
        rot_axis = {'HeadYaw':       'z',
                   'HeadPitch':      'y',
                   'LShoulderPitch': 'y',
                   'LShoulderRoll':  'x',
                   'LElbowYaw':      'z',
                   'LElbowRoll':     'x',
                   'RShoulderPitch': 'y',
                   'RShoulderRoll':  'x',
                   'RElbowYaw':      'z',
                   'RElbowRoll':     'x',
                   'RWristYaw':      'z',
                   'LHipYawPitch':   'y',
                   'LHipRoll':       'x',
                   'LHipPitch':      'y',
                   'LKneePitch':     'y',
                   'LAnklePitch':    'y',
                   'LAnkleRoll':     'x',
                   'RHipYawPitch':   'y',
                   'RHipRoll':       'x',
                   'RHipPitch':      'y',
                   'RKneePitch':     'y',
                   'RAnklePitch':    'y',
                   'RAnkleRoll':     'x'
                   }
        
        # rotational offset combined with translational offset
        T = trans_offsets[joint_name].dot(rot_offsets[joint_name])

        # joint rotation
        return T.dot(self.rotation(rot_axis[joint_name], joint_angle))

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        test_joints = {}
        for chain_joints in self.chains.values():
            for joint in chain_joints:
                test_joints[joint] = 0.0
                
        joints = test_joints
        
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T.dot(Tl)

                self.transforms[joint] = T
        
        # plotting the result
        
        names = self.chains['Head']
        
        # show y coordinate on x axis and z coordinate on y axis
        view_transform = matrix([[0, 1, 0, 0],
                                 [-1, 0, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
        
        self.plotSkeleton(view_transform)
        self.plotRobot(view_transform)
        plt.show()
        
    def plotJoints(self, names, view_transform = identity(4)):
        for joint in names:
            plotCoordinateSystem(view_transform.dot(self.transforms[joint]), 50)
    
    def plotSkeleton(self, view_transform = identity(4)):
        for chain_joints in self.chains.values():
            points_x = [0]
            points_y = [0]
            points_z = [0]
            for joint in chain_joints:
                T = view_transform.dot(self.transforms[joint])
                points_x.append(T[0,3])
                points_y.append(T[1,3])
                points_z.append(T[2,3])
        
            if plotIn3D :
                ax.plot3D(points_x, points_y, points_z)
            else:
                plt.plot(points_x, points_y)
                        
    def plotRobot(self, view_transform = identity(4)):
        # origin / torso
        plotCoordinateSystem(view_transform, 50)
        
        for chain_joints in self.chains.values():
             for joint in chain_joints:
                 plotCoordinateSystem(view_transform.dot(self.transforms[joint]), 50)


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
