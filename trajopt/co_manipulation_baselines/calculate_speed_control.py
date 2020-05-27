#! /usr/bin/env python

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
import math
from std_msgs.msg import Float64
import rospy
import tf
from tf import TransformListener
import numpy as np


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import trajoptpy.kin_utils as ku

from scipy.interpolate import CubicSpline


class SpeedControl:

    def __init__(self):
        self.joint_1_pub = rospy.Publisher('/j2s7s300/joint_1_position_controller/command', Float64, queue_size=3)
        self.joint_2_pub = rospy.Publisher('/j2s7s300/joint_2_position_controller/command', Float64, queue_size=3)
        self.joint_3_pub = rospy.Publisher('/j2s7s300/joint_3_position_controller/command', Float64, queue_size=3)
        self.joint_4_pub = rospy.Publisher('/j2s7s300/joint_4_position_controller/command', Float64, queue_size=3)
        self.joint_5_pub = rospy.Publisher('/j2s7s300/joint_5_position_controller/command', Float64, queue_size=3)
        self.joint_6_pub = rospy.Publisher('/j2s7s300/joint_6_position_controller/command', Float64, queue_size=3)
        self.joint_7_pub = rospy.Publisher('/j2s7s300/joint_7_position_controller/command', Float64, queue_size=3)
        self.transformer = tf.Transformer(True, rospy.Duration(10.0))
        self.tf = TransformListener()



        self.env = openravepy.Environment()
        self.env.StopSimulation()
        self.env.Load("../data/jaco-test.dae")
        self.env.Load("../data/table.xml")
        # self.env.SetDefaultViewer()

        # trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
        self.robot = self.env.GetRobots()[0]
        print(self.robot.__dict__)
        print(self.robot.GetManipulator('test_arm'))

        joint_start = [0, 3.14, 0, 3.14, 0, 3.14, 0]
        self.robot.SetDOFValues(joint_start, self.robot.GetManipulator('test_arm').GetArmIndices())

        joint_target = [0.5, 1.8, 0, 3.14, 0, 3.14, 0]



        quat_target = [1,0,0,0] # wxyz
        xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
        hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

        # BEGIN ik
        self.manip = self.robot.GetManipulator("test_arm")
        self.robot.SetActiveManipulator(self.manip)
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        init_joint_target = ku.ik_for_link(hmat_target, self.manip, "j2s7s300_ee_link",
            filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
        # END ik

        print(init_joint_target)
        joint_target = init_joint_target.tolist()

        # TODO call tests here

    def get_separation_dist(human_pos, robot_joints):
        return 0

    def cubic_interpolation(dof_vals):
        x = [i for i in range(0, len(dof_vals))]
        y = dof_vals
        return CubicSpline(x,y)



    def calculate_new_trajectory(self, dof_vals, human_pos):
        '''
        dof_vals: time-sampled joint space trajectory (timesteps x 7 matrix)
        human_pos: human position from vision system (timesteps x 4 matrix)
        ideally, dof_values_timesteps >= human_pos_timesteps
        TODO: assume blackbox function to calculate separation distance // get_separation_dist(time, configuration)
        '''
        scaling_factor = 1  # TODO choose based on separation dist
        d_slow = # choose threshold
        d_stop = # choose threshold
        beta = # parameter
        gamma = # parameter
        traj_interpolation = cubic_interpolation(dof_vals)

        num_timesteps = len(human_pos)  
        new_exec_traj = [] 
        robot_timestep = 0
        for human_timestep in range(0, num_timesteps):
            robot_joints = traj_interpolation(robot_timestep)
            new_exec_traj.append([robot_timestep, robot_joints])

            d = get_separation_dist(human_pos[human_timestep], dof_vals[robot_timestep])
            if (d_stop <= d <= d_slow):
                scaling_factor = 1 - beta * ((d - d_stop) ** gamma)
            elif (d_slow < d):# change to new timestamp and trajectory pts
                scaling_factor = 0
            else:
                scaling_factor = 1
            
            robot_timestep += 1 - scaling_factor
        
        if robot_timestep < num_timesteps:
            new_exec_traj.extend(dof_vals[math.ceil(robot_timestep)::])
        
        return new_exec_traj