#!/usr/bin/env python

# For simple data rostopic echo -b file.bag -p /topic > data.csv
# http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

from curses import raw
from ntpath import join
from operator import truediv
from codecs import raw_unicode_escape_decode
from pickle import TRUE
import sys
sys.path.insert(1, '/home/hritiksapra/Documents/comoto.jl/src')
from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import Tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool, String
import json
from pygame import mixer
import time
import sys
import os

from threading import Thread                #importing threads
from message_filters import Subscriber, ApproximateTimeSynchronizer

mixer.init()

def run1(stop, case):
    #raw_input("1")
    #raw_input(rospy.is_shutdown())
    #print(case)
    if case == "true":
        while not rospy.is_shutdown():
            rate = rospy.Rate(5) #5 hz
            pub_start.publish(Bool(True))
            pub_stop.publish(Bool(False))
            rate.sleep()
            #raw_input("c1")
            if stop():
                    break
    else:
        while not rospy.is_shutdown():
            rate = rospy.Rate(5) #5 hz
            pub_start.publish(Bool(False))
            pub_stop.publish(Bool(True))
            rate.sleep()
            #raw_input("c2")
            if stop():
                    break

class topic_subscriber:

    def __init__(self, flag_topic_start, flag_topic_stop):
        
        self.flag_subscriber_start = Subscriber(flag_topic_start, Bool)
        self.flag_subscriber_stop = Subscriber(flag_topic_stop, Bool)
        
        sync = ApproximateTimeSynchronizer([self.flag_subscriber_start, self.flag_subscriber_stop], queue_size=10, slop=1, allow_headerless=True)
        sync.registerCallback(self.callback)
        print("in run_experiment")
        rospy.spin()

    def callback(self, flag_msg_start, flag_msg_stop):
        
        stop_thread1 = False
        # stop_thread2 = True
        
        t1 = Thread(target= run1, args=( lambda: stop_thread1, "false")) #start outputing False
        t1.start()
        
        
        print(flag_msg_start.data)
        print(flag_msg_stop.data)

        while not rospy.is_shutdown():
            if flag_msg_stop.data:
                stop_thread1 = True
                break
            print(stop_thread1)
            

        
        # stop_thread1 = True
        t1.join()
        raw_input("??")
        # stop_thread2 = False
        
        # t2 = Thread(target= run1, args=( lambda: stop_thread2, "false"))
        # t2.start()
        # time.sleep(1)
        # stop_thread2 = True
        # t2.join()
    
        print('thread killed')


def main(user_id, pub_start, pub_stop):
    flag_topic_start = '/flag_topic/start'
    flag_topic_stop = '/flag_topic/stop'

    topic_subscriber(flag_topic_start, flag_topic_stop)

if __name__ == '__main__':
    rospy.init_node('flag_pub', anonymous=False)
    pub_start = rospy.Publisher('/flag_pub/start', Bool, queue_size=10)
    pub_stop = rospy.Publisher('/flag_pub/stop', Bool, queue_size=10)

    main(sys.argv[1], pub_start, pub_stop)