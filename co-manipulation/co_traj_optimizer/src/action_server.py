#! /usr/bin/env python

import actionlib
from actionlib_msgs.msg import *
import co_traj_optimizer.msg
from geometry_msgs.msg import *
import math
from motion_planning import *
import rospy
import tf
from tf import TransformListener

Test = False


class ChairAssemblyActionServer(object):

    def __init__(self, name):
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name, co_traj_optimizer.msg.DemoAction,
                                execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()
        self.transformer = tf.Transformer(True, rospy.Duration(10.0))
        self.tf = TransformListener()
        self.mp = ChairAssemblyMotionPlanner()


    def execute_cb(self, goal):
        '''
        goal = primitive ID
             = part ID
        '''
        part = goal.partID
        r = rospy.Rate(1)
        success = True
        
        target_obj_position = ""
        position = None
        orientation = Quaternion(math.sqrt(2)/2, 0, -math.sqrt(2)/2, 0)
        if part == 0:  # Right leg
            target_obj_position = "/ar_marker_3"
            position, _ = self.tf.lookupTransform("/base_link", target_obj_position,
                                    self.tf.getLatestCommonTime(target_obj_position, "/base_link"))
            position = Point(position[0], position[1] - 0.095, position[2] + 0.15)
            # b = tf.TransformBroadcaster()
            # b.sendTransform(position, orientation, rospy.Time(), '/TEST_ar_marker_3', '/base_link')
        elif part == 1:  # Left leg
            target_obj_position = "/ar_marker_7"
            position, _ = self.tf.lookupTransform("/base_link", target_obj_position,
                                    self.tf.getLatestCommonTime(target_obj_position, "/base_link"))
            position = Point(position[0], position[1] + 0.10, position[2] + 0.15)
            # b = tf.TransformBroadcaster()
            # b.sendTransform(position, orientation, rospy.Time(), '/TEST_ar_marker_7', '/base_link')
        elif part == 2:  # Chair base
            target_obj_position = "/ar_marker_5"
            position, _ = self.tf.lookupTransform("/base_link", target_obj_position,
                                    self.tf.getLatestCommonTime(target_obj_position, "/base_link"))
            position = Point(position[0], position[1] + 0.15, position[2] + 0.15)
            # b = tf.TransformBroadcaster()
            # b.sendTransform(position, orientation, rospy.Time(), '/TEST_ar_marker_5', '/base_link')


        if goal.primitiveID == 0:
            # Pick up
            print('position of object at: ', position)
            pose = Pose(position=position, orientation=orientation)
            self.mp.pickup_object(pose)
            if Test:
                print("Target object located at AR tag: ", target_obj_position)
                print(pose)
                print("Executing primitive pickup, ID: ", goal.primitiveID)
        elif goal.primitiveID == 1:
            # Hand off
            print("Handing off")
            position = Point(-0.5, -0.2, 0.4)
            pose = Pose(position=position, orientation=orientation)
            self.mp.handoff_object(pose)
            if Test:
                print("Target object located at AR tag: ", target_obj_position)
                print(pose)
                print("Executing primitive pickup, ID: ", goal.primitiveID)
        elif goal.primitiveID == 2:
            # Stabilize
            pose = Pose(position=position, orientation=orientation)
            self.mp.stabilize(pose)
            if Test:
                print("Target object located at AR tag: ", target_obj_position)
                print(pose)
                print("Executing primitive pickup, ID: ", goal.primitiveID)
        elif goal.primitiveID == 3:
            # Prealign
            position.x = position.x - 0.12
            orientation = Quaternion(0, 1, 0, 0)
            pose = Pose(position=position, orientation=orientation)
            self.mp.prealign(pose)
            if Test:
                print("Target object located at AR tag: ", target_obj_position)
                print(pose)
                print("Executing primitive pickup, ID: ", goal.primitiveID)
        else:
            success = False
            print("NO PRIMITIVES FOUND")
            # raise Exception('Invalid primitive')

        result = co_traj_optimizer.msg.DemoResult()
        result.success = 1
        self.action_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('demo_server')
    demo = ChairAssemblyActionServer("task_september_demo")
    rospy.spin()
