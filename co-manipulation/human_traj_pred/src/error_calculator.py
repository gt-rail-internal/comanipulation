#! /usr/bin/env python
import rospy
import csv
import matlab.engine
from std_msgs.msg import String
import numpy as np
import ast
import json
from sklearn.metrics import mean_squared_error

class Pair:
    def __init__(self, eng):
        self.eng = eng
        self.ground_truth = ""
        self.prediction = ""
    
    def get_dtw(self):
        gt = matlab.double(json.loads(self.ground_truth)[:100])
        try:
            pred = matlab.double(json.loads(str(self.prediction))[:100])
            DTW = self.eng.DTW_dis(gt, pred, 0)
            RMSE = mean_squared_error(gt, pred)
            print(str(DTW) + " " + str(RMSE))
        except ValueError:
            pass
    
    def try_dtw(self):
        if len(self.ground_truth) != 0 and len(self.prediction) != 0:
            self.get_dtw()
            self.ground_truth = ""
            self.prediction = ""


def callback_prediction(data, pair):
    if data.data != "Waiting for enough data":
        pair.prediction = data.data
        pair.try_dtw()
    else:
        print("Waiting for enough data")

def callback_gt(data, pair):
    pair.ground_truth = data.data
    pair.try_dtw()
            
def listener(eng):
    rospy.init_node('error_calculator', anonymous=False)
    pair = Pair(eng)
    rospy.Subscriber("human_traj_pred", String, callback_prediction, (pair))
    rospy.Subscriber("human_traj_future", String, callback_gt, (pair))
    rospy.spin()


if __name__ == '__main__':
    print("Starting Matlab engine")
    eng = matlab.engine.start_matlab()
    print("Started Matlab. Calling Listner")
    eng.cd('UOLA', nargout=0)
    listener(eng)