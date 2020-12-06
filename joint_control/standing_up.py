'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent

from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand


class StandingUpAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(StandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.update_posture = 1
         
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        # YOUR CODE HERE
        
        if self.update_posture == 0:
            return

        # mapping postures to keyframes
        posture_to_keyframe = {'Back':0, 'Belly':1, 'Crouch':0, 'Frog':0, 'HeadBack':0, 'Knee':0, 'Left':0, 'Right':2, 'Sit':2, 'Stand':0, 'StandInit':0, 'unknown':4}
        
        keyframe_num = posture_to_keyframe[posture]
        
        if keyframe_num == 0:
            self.keyframes= leftBackToStand.leftBackToStand()
        if keyframe_num == 1:
            self.keyframes = leftBellyToStand.leftBellyToStand()
        if keyframe_num == 2:
            self.keyframes = rightBackToStand.rightBackToStand()
        if keyframe_num == 3:
            self.keyframes = rightBellyToStand.rightBellyToStand()
        if keyframe_num == 4:
            self.keyframes = ([], [], [])


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
            self.update_posture = 1
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
            self.keyframe_offset_time = self.stiffness_on_off_time + self.stiffness_off_cycle
            self.update_posture = 0
            
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
