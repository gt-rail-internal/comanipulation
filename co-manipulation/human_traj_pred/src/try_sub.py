#! /usr/bin/env python

from sensor_msgs.msg import JointState
import rospy
robot_topic = '/j2s7s300_driver/out/joint_state'

def callback(data):
    print(data.position)

rospy.Subscriber(robot_topic, JointState, callback)
