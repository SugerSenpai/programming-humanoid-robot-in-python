'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


import pickle
from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import numpy as np


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        #self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))
        posture = 'unknown'
        features = np.array([self.perception.joint['LHipYawPitch'], self.perception.joint['LHipRoll'], self.perception.joint['LHipPitch'],
                             self.perception.joint['LKneePitch'], self.perception.joint['RHipYawPitch'], self.perception.joint['RHipRoll'],
                             self.perception.joint['RHipPitch'], self.perception.joint['RKneePitch'], self.perception.imu[0], self.perception.imu[1]])
        features = features.reshape(1, -1)
        #prediction = self.posture_classifier.predict(features)
        #print(prediction)
        posture = features
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
