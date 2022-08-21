#! /usr/bin/env python
import sys
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
    def __init__(self, eng, pub, count):
        self.obsv = []
        self.matlab = eng
        self.pub = pub
        self.pred = None
        self.count = count#104
        self.campos = [-0.019, 0.420, 1.112]
        self.transform = tf.transformations.euler_matrix(-2.161, 0.046, -3.110)[:3, :3]#tf.transformations.euler_matrix(-2.185351651204997, 0.0025157588914009534, 2.9853950940245584)[:3, :3] #np.eye(3) #np.array([[0.175213, 0.381727, 0.545808],[-0, 0.389293, 0.344862], [-5.78241*e**-17, 1.33333, 1]])
    
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
        shoulder_elbow_wrist_palm = [12,13,14,15] #for right hand https://docs.microsoft.com/en-us/azure/kinect-dk/body-joints
        traj = []
        for obv in self.obsv:
            curr_frame = [0]
            for index in shoulder_elbow_wrist_palm:
                # -0.1380266 , -0.41772856, -1.04825784
                # 0.047, 0.303, 1.181
                old_coords = np.array([obv[index].pose.position.x, obv[index].pose.position.y, obv[index].pose.position.z])
                new_coords = np.matmul(self.transform, old_coords.T)
                # -0.022, 0.450, 1.069
                curr_frame.append(new_coords[0] + self.campos[0])#1.4)#
                curr_frame.append(new_coords[1] + self.campos[1])#0.5)#
                curr_frame.append(new_coords[2] + self.campos[2])#-0.5)#
            traj.append(curr_frame)
        expData, expSigma = self.matlab.UOLA_predict('GMMtry.mat', matlab.double(traj), 'prediction.csv', nargout=2)
        for row in traj:
            del row[0]
        df1 = pd.DataFrame(traj)
        df1.to_csv('traj_' + str(self.count) + '.csv',index=False, header=False)
        # df2 = pd.DataFrame(traj[:51])
        # df3 = pd.DataFrame(traj[51:])
        # df2.to_csv('traj_' + str(self.count) + '_trimmed.csv',index=False, header=False)
        # df3.to_csv('predSampledtraj_' + str(self.count) + '_trimmed.csv',index=False, header=False)
        self.pred = str(traj) + "splitTag" + str(expData)+"splitTag"+str(expSigma)
        print("Sent")


def callback(data, curr_obsv):
    curr_obsv.add(data.markers)
    if len(curr_obsv.obsv) > 100:
        curr_obsv.generate_prediction()
        curr_obsv.clear()
        # print("Done " + str(curr_obsv.count))
            
def listener(eng, count):
    rospy.init_node('predictor', anonymous=False)
    pub = rospy.Publisher('human_traj_pred', String, queue_size=10)
    curr_obsv = Traj(eng, pub, count)
    print("Subscribing")
    rospy.Subscriber("/front/body_tracking_data", MarkerArray, callback, (curr_obsv))
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
    listener(eng, int(sys.argv[1]))
