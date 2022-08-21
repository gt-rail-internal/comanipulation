#!/usr/bin/env python

# For simple data rostopic echo -b file.bag -p /topic > data.csv
# http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

import os
import sys
import csv
import rospy
import rospkg
import argparse
import tf
import std_msgs.msg

import cv2
from cv_bridge import CvBridge
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, JointState 
from std_msgs.msg import Bool
from learning_tf2.msg import Num, MAstamp
from cmath import e, sqrt

import tf2_ros
import geometry_msgs.msg 
from geometry_msgs.msg import TwistStamped


def transform_data(old_coords):                                                 #is this being used?
    transform = tf.transformations.euler_matrix(-2.161, 0.046, -3.110)[:3, :3]
    campos = [-0.019, 0.420, 1.112]
    new_coords = np.matmul(transform, old_coords.T)

    return new_coords

def save2csv(tag, skeleton_trajectory, dump_folder, name):

    with open('{}/{}_{}.csv'.format(dump_folder, tag, name), 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        for t in skeleton_trajectory:
            sklen = len(t)
            if sklen >0:
                writer.writerow([(sklen*'{},').format(*t.ravel())])
            else:
                writer.writerow(['NONE'])

def distan(base,coord):
        dist_x = abs(base[0]- coord[0])**2.0
        dist_y = abs(base[1]- coord[1])**2.0
        dist_z = abs(base[2]- coord[2])**2.0

        dist = sqrt(dist_x + dist_y + dist_z)
        print("Distance: {}".format(dist))

        return dist



class Rostopic2Data:
    
    ID = 0
    cnt = 0
    cnt1 = 0
    runstate = 1
    skeleton_joints = []
    skeleton_orientation = []

    def __init__(self, skeleton_topic, robot_topic, flag_topic_start, flag_topic_stop, user_id, tf2_topic, dump_folder='', extra_folder=[],):  
        
        self.bridge = CvBridge()
        self.dump_folder = dump_folder
        self.extra_folder = extra_folder
        self.tag = 'User-{}'.format(user_id)

        self.skeleton_trajectory = []
        self.skeleton_orient = []
        self.robot_trajectory = []
        self.robot_point = []

        # self.skeleton_subscriber = Subscriber(skeleton_topic, MarkerArray)
        self.skeleton_subscriber = Subscriber(skeleton_topic, MAstamp)
        self.robot_subscriber = Subscriber(robot_topic, JointState)
        self.flag_subscriber_start = Subscriber(flag_topic_start, Num)
        self.flag_subscriber_stop = Subscriber(flag_topic_stop, Num)
        self.tf2_subscriber = Subscriber(tf2_topic, TwistStamped)

        
        sync = ApproximateTimeSynchronizer([self.skeleton_subscriber, self.robot_subscriber, self.flag_subscriber_start, self.flag_subscriber_stop, self.tf2_subscriber], queue_size=5, slop=0.1, allow_headerless=False) # slop=0.0667
        # sync = ApproximateTimeSynchronizer([self.skeleton_subscriber, self.image_subscriber, self.robot_subscriber, self.flag_subscriber_start, self.flag_subscriber_stop], queue_size=10, slop=10000000, allow_headerless=True)
        sync.registerCallback(self.callback)
        print("---Only for recording ROSBAG---")
        print('Collecting data')
        self.count_it = 1
        self.count = 1
        self.stop_flag = False
        rospy.spin()

            
    
    def callback(self, skeleton_msg, robot_msg, flag_msg_start, flag_msg_stop, tf2_msg):
        markers = skeleton_msg.markers.markers
        tf2 = tf2_msg

        if flag_msg_start.statement:
            Rostopic2Data.runstate = 1

        if Rostopic2Data.cnt == 0 and Rostopic2Data.runstate == 1:
            #print(type(markers[0].header.stamp))
            self.begin = float(str(markers[0].header.stamp))
            self.beg = float(str(tf2.header.stamp))

            base = [markers[26].pose.position.x, markers[26].pose.position.y, markers[26].pose.position.z]
            print(base)
            dista = distan([0,0,0],base)

            if dista.real > 2.77:
                return
            else:
                Rostopic2Data.ID = (markers[0].id)/100

                

        
        if Rostopic2Data.runstate == 1:
            print(Rostopic2Data.cnt)
            Rostopic2Data.cnt += 1                              #added -counter

        #----------------------------------------------------------------------------------------------------------------

        if not flag_msg_start.statement:                         
            self.stop_flag = flag_msg_stop.statement
            print("not saving")                        #added -ck

        else:
            end = float(str(markers[0].header.stamp))
            end1 = float(str(tf2.header.stamp))
            time = (end - self.begin)
            time1 = (end1 - self.beg)
            time = float(time)/1000000000
            time1 = float(time1)/1000000000

            #ends the callback if the marker id's don't match end with time skip if missing data
            # if (markers[0].id)/100 != Rostopic2Data.ID:
            #     return

            for idx, marker in enumerate(skeleton_msg.markers.markers):
                #marker id may be redundant with condition above
                if marker.pose.position.x < 1 and marker.id >= (Rostopic2Data.ID*100) and marker.id <= ((Rostopic2Data.ID*100)+31):
                    Rostopic2Data.skeleton_joints = np.append(Rostopic2Data.skeleton_joints, np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,]), axis=0)
                    Rostopic2Data.skeleton_orientation = np.append(Rostopic2Data.skeleton_orientation, np.array([marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z,]), axis=0)
            
            Rostopic2Data.skeleton_joints = np.append(Rostopic2Data.skeleton_joints, [round(time,9)])
            Rostopic2Data.skeleton_orientation = np.append(Rostopic2Data.skeleton_orientation, [round(time,9)])

            self.skeleton_trajectory.append(Rostopic2Data.skeleton_joints)

            self.skeleton_orient.append(Rostopic2Data.skeleton_orientation)

            self.robot_trajectory.append(robot_msg.position)

            pos_rob = [tf2_msg.twist.linear.x, tf2_msg.twist.linear.y, tf2_msg.twist.linear.z, time1]
            self.robot_point.append(pos_rob)

            self.stop_flag = flag_msg_stop.statement
            Rostopic2Data.skeleton_joints = []
            Rostopic2Data.skeleton_orientation = []
            # print("End of else")                            #added -ck
            Rostopic2Data.runstate = 1
            # print(self.stop_flag)

        #----------------------------------------------------------------------------------------------------------------

        if self.stop_flag == True:

            if Rostopic2Data.runstate == 1:
                print("Start of saving .csv")                   #added -ck
            
                self.skeleton_trajectory = np.array(self.skeleton_trajectory)
                self.skeleton_orient = np.array(self.skeleton_orient)
                self.robot_trajectory = np.array(self.robot_trajectory)
                self.robot_point = np.array(self.robot_point)

                # print(file_name)
                # print("sk: {}".format(len(self.skeleton_trajectory)))                                #added -ck
                # print("rp: {}".format(len(self.robot_point)))
                # print("rt: {}".format(len(self.robot_trajectory)))

                file_name = "run_" + str(self.count)
                self.count += 1

                Rostopic2Data.cnt1 += 1 

                save2csv(self.tag, self.skeleton_trajectory, self.extra_folder[3], "skeletal_"+file_name)
                save2csv(self.tag, self.skeleton_orient, self.extra_folder[0], "orientation_"+file_name)
                save2csv(self.tag, self.robot_trajectory, self.extra_folder[2], "robot_"+file_name)
                save2csv(self.tag, self.robot_point, self.extra_folder[1], "point_"+file_name)
                
                Rostopic2Data.cnt = 0

                self.skeleton_trajectory = []
                self.skeleton_orient = []
                self.robot_trajectory = []
                self.robot_point = []

            else:
                return

            print("Run: {}".format(Rostopic2Data.cnt1))

            Rostopic2Data.runstate = 0


def main(user_id):
    rospy.init_node("topic2data", anonymous=True)
    rospack = rospkg.RosPack()

    logdir_path = '/home/hritiksapra/Documents/comoto.jl/src/expt_logs/user_{}'.format(user_id)
    logdir_path1 = '{}/orientation'.format(logdir_path)
    logdir_path2 = '{}/point'.format(logdir_path)
    logdir_path3 = '{}/robot'.format(logdir_path)
    logdir_path4 = '{}/skeleton'.format(logdir_path)
    if not os.path.exists(logdir_path):
        os.mkdir(logdir_path)
        print('Created directory for User {}'.format(user_id))
    if not os.path.exists(logdir_path1):
        os.mkdir(logdir_path1)
        os.mkdir(logdir_path2)
        os.mkdir(logdir_path3)
        os.mkdir(logdir_path4)
    
    extra_folder = [logdir_path1,logdir_path2,logdir_path3,logdir_path4]

    
    #skeleton_topic = '/front/body_tracking_data'        #15hz   #not work
    skeleton_topic = '/MAstamp/publisher'               #15hz   
    robot_topic = '/j2s7s300_driver/out/joint_state'    #100hz  #works
    flag_topic_start = '/flag_topic/start'                      #not work
    flag_topic_stop = '/flag_topic/stop'                        #not work
    tf2_topic = '/tf2_listener/publisher'

    Rostopic2Data(skeleton_topic=skeleton_topic, robot_topic=robot_topic, flag_topic_start=flag_topic_start, flag_topic_stop=flag_topic_stop, user_id=user_id, tf2_topic = tf2_topic, dump_folder=logdir_path, extra_folder=extra_folder)

if __name__ == '__main__':
    main(sys.argv[1])