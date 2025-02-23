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


from pid import PIDAgent
from keyframes import *
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.timerToZero = True
        self.startTime = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names, times, keys = keyframes
        if(self.timerToZero):
            self.startTime = perception.time
            self.timerToZero = False
        timer = perception.time - self.startTime

        for index, joint in enumerate(names):
            jointTimes = times[index]
            jointKeys = keys[index]
            for timeIndex in range(len(jointTimes)-1):
                keyframeStartTime = jointTimes[timeIndex]
                keyframeEndTime = jointTimes[timeIndex + 1]
                if (keyframeStartTime < timer < keyframeEndTime):
                    keyframeDuration = keyframeEndTime - keyframeStartTime
                    t = (timer - keyframeStartTime) / keyframeDuration
                    P0 = jointKeys[timeIndex][0] 
                    #after testing out the other keyframe test I found out that we apparently need to add P0 for it to work
                    P1 = P0+jointKeys[timeIndex][1][2] 
                    P2 = P0+jointKeys[timeIndex][2][2]  
                    P3 = jointKeys[timeIndex+1][0]     
                    #formula from https://www.youtube.com/watch?v=pnYccz1Ha34
                    bezier = (1-t)**3 * P0 + 3*(1-t)**2 * t * P1 + 3*(1-t)*t**2 * P2 + t**3 * P3
                    target_joints[joint] = bezier
                    if joint == 'LHipYawPitch':
                        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
