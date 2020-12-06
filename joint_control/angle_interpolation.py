'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

import numpy as np
from pid import PIDAgent
from keyframes import leftBackToStand

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        
        self.keyframe_offset_time = 0
        self.keyframe_start = 1
        self.repeat_keyframe = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezier_interpolation(self, time_point, times, keys):
        
        # find part of bezier curve wich needs to be calculated
        t = np.zeros(4)
        a = np.zeros(4)
        
        for i in range(0, len(times)):
            if times[i] < time_point:
                t[0] = times[i]
                t[1] = keys[i][2][1]
                a[0] = keys[i][0]
                a[1] = keys[i][2][2]
            else:
                t[2] = keys[i][1][1]
                t[3] = times[i]
                a[2] = keys[i][1][2]
                a[3] = keys[i][0]
                break
            
        if t[0] > t[3]:
            return keys[-1][0]
            
        # increase steps if movement is not smooth enough
        steps = 100
        i = np.linspace(0, 1, steps)
        
        c1 = (1 - i)**3
        c2 = 3*(1-i)**2*i
        c3 = 3*(1-i)*i**2
        c4 = i**3
        
        
        t[1] += t[0]
        t[2] += t[3]
        a[1] += a[0]
        a[2] += a[3]
        
        # time and value pairs (bez_x, bez_y) make bezier curve
        bez_x = c1* t[0] + c2 * t[1] + c3 * t[2] + c4 * t[3]
        bez_y = c1* a[0] + c2 * a[1] + c3 * a[2] + c4 * a[3]
        
        # fill very small gaps between points calculated bezier curve
        angle = np.interp(time_point, bez_x, bez_y)
            
        return angle

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        
        
        if keyframes == ([], [], []) or self.keyframe_start == 0:
            return  target_joints
        
        # for every joint
        for i in range(0, len(keyframes[0])):
            name = keyframes[0][i]
            times = keyframes[1][i]
            keys = keyframes[2][i]
            points = []
            
            # simplify keys
            for k in keys:
                points.append(k[0])
            
            t = perception.time - self.keyframe_offset_time
            if t < 0:
                t = 0
            
            # repeat every 12s
            if self.repeat_keyframe == 1:
                t = np.mod(t, 12.0)
            
            angle = self.bezier_interpolation(t, times, keys)         
            
            if name == "LHipYawPitch":
                target_joints["RHipYawPitch"] = angle
                
            target_joints[name] = angle
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand.leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.repeat_keyframe = 1
    agent.run()
