#! /usr/bin/env python
import rospy
import tf
import csv
import matlab.engine
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import ast
from math import e
import pandas as pd

class Traj:
    def __init__(self, eng, pub):
        self.obsv = []
        self.matlab = eng
        self.pub = pub
        self.pred = None
        self.count = 0
        self.transform = tf.transformations.euler_matrix(0.020, 0.427, -1.448)[:3, :3]#tf.transformations.euler_matrix(-2.185351651204997, 0.0025157588914009534, 2.9853950940245584)[:3, :3] #np.eye(3) #np.array([[0.175213, 0.381727, 0.545808],[-0, 0.389293, 0.344862], [-5.78241*e**-17, 1.33333, 1]])
    
    def add(self, obv):
        self.obsv.append(obv)
    def discard(self):
        self.obsv.pop(0)
    def clear(self):
        self.obsv = []
    def publish(self):
        self.pub.publish(self.pred)

# April 10
#Translation: [0.077, 0.407, 1.122]
#RPY [-2.140, -0.013, 3.135]

    def generate_prediction(self):
        df = pd.read_csv ('/home/hritiksapra/catkin_ws/src/comanipulation/co-manipulation/human_traj_pred/src/UOLA/training_data/data_traj_motion_4.csv')
        traj = df.to_list()
        for row in traj:
            row.insert(0, 0)
        expData, expSigma = self.matlab.UOLA_predict('newtrainedGMM.mat', matlab.double(traj), 'prediction.csv', nargout=2)
        for row in traj:
            del row[0]
        self.pred = str(traj) + "splitTag" + str(expData)+"splitTag"+str(expSigma)


def callback(data, curr_obsv):
    curr_obsv.generate_prediction()
    print("Done")
            
def listener(eng):
    rospy.init_node('predictor', anonymous=False)
    pub = rospy.Publisher('human_traj_pred', String, queue_size=10)
    curr_obsv = Traj(eng, pub)
    callback("", curr_obsv)
    #/front/body_tracking_data
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if curr_obsv.pred is not None:
            curr_obsv.publish()
            rate.sleep()

if __name__ == '__main__':
    print("Starting Matlab engine")
    eng = matlab.engine.start_matlab()
    print("Started Matlab. Calling Listner")
    eng.cd('UOLA', nargout=0)
    listener(eng)
