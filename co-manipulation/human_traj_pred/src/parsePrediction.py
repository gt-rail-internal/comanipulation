#! /usr/bin/env python
import rospy
import csv
import matlab.engine
from std_msgs.msg import String
import numpy as np
import ast
import json
import math
from sklearn.metrics import mean_squared_error

def callback(data):
    if len(data.data) != 0 and data.data != "Waiting for enough data":
        expData, expSigma = data.data.split("splitTag")
        expData, expSigma = matlab.double(json.loads(str(expData))[:100]), matlab.double(json.loads(str(expSigma))[:100])
        predictedMeans, variances = [],[]

        for row in expData:
            for col in row:
                predictedMeans.append(col)
        
        for timestep in range(len(expSigma)):
            for row in range(len(expSigma[timestep])):
                colStart = row%3 * 3
                for col in range(len(expSigma[timestep][row])):
                    if col >= colStart and col < (colStart + 3):
                        variances.append(expSigma[timestep][row][col])
            
        
        print("Mean length = " + str(len(predictedMeans)) + " Variance length = " + str(len(variances)))
        # print(str(len(expSigma)) + " " + str(len(expSigma[0])) + " " + str(len(expSigma[0][0])))
            
def listener():
    rospy.init_node('parse', anonymous=False)
    rospy.Subscriber("human_traj_pred", String, callback)
    rospy.spin()

if __name__ == '__main__':
    print("Calling Listener")
    listener()