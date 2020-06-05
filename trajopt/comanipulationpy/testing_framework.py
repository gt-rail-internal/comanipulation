#! /usr/bin/env python


import math
import numpy as np
import csv
import glob

import openravepy
import trajoptpy
import json
import time
import trajoptpy.kin_utils as ku

from comanipulationpy import *
from plots import *
import sys

from scipy.interpolate import CubicSpline


    

    



####################################################
######  Helper Functions
####################################################









####################################################
######  Baseline Functions
####################################################

def nominal_baseline_test(self, init_joint, final_joint, traj_num=303, exec_traj = False):

    # Read pose prediction files
    full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
    # Expand ground truth human trajectory
    full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)
    obs_complete_test_traj = create_human_plot_traj(obs_rightarm_test_traj)

    object_pos = [0, 0.2, 0.83]

    # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
    n_human_joints = 11
    num_timesteps = 20
    num_human_timesteps = len(full_complete_test_traj) / (n_human_joints * 3)
    num_obs_timesteps = len(obs_complete_test_traj) / (n_human_joints * 3)

    exec_complete_test_traj = full_complete_test_traj[num_obs_timesteps * n_human_joints * 3 : ]

    # # Setup coefficients
    # coeff_optimal_traj = 5.0
    # coeff_dist = []
    # coeff_vel = []
    # coeff_vis = []
    # coeff_leg = 1.0
    # coeffs_reg = []
    # for i in range(num_timesteps):
    #     coeff_dist.append(100.0)
    #     coeff_vel.append(100.0)
    #     coeff_vis.append(1.0)
    # for i in range(num_timesteps - 1):
    #     coeffs_reg.append(1.0)

    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
    default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)

    if (exec_traj):
        self.execute_full_trajectory(default_traj, full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)

    return self.evaluate_metrics(default_traj, full_complete_test_traj, len(obs_rightarm_test_traj)/ 12, object_pos, default_traj)


def dist_vis_baselines_test(self, init_joint, final_joint, traj_num=303, exec_traj = False):

    # Read pose prediction files
    full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
    # Expand ground truth human trajectory
    full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)
    obs_complete_test_traj = create_human_plot_traj(obs_rightarm_test_traj)

    # Set object position
    object_pos = [0, 0.2, 0.83]

    # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
    n_human_joints = 11
    num_timesteps = 20
    num_human_timesteps = len(full_complete_test_traj) / (n_human_joints * 3)
    num_obs_timesteps = len(obs_complete_test_traj) / (n_human_joints * 3)

    # Get human head and torso pose mean from full_human_poses
    head_ind = 5
    torso_ind = 6
    final_obs_timestep_ind = num_obs_timesteps
    head_pos = full_complete_test_traj[final_obs_timestep_ind * n_human_joints * 3 + head_ind * 3 : final_obs_timestep_ind * n_human_joints * 3 + head_ind * 3 + 3]
    torso_pos = full_complete_test_traj[final_obs_timestep_ind * n_human_joints * 3 + torso_ind * 3 : final_obs_timestep_ind * n_human_joints * 3 + torso_ind * 3 + 3]
    feet_pos = [torso_pos[0], torso_pos[1], torso_pos[2] - 0.5]

    
    # Setup coefficients
    coeff_optimal_traj = 5.0
    coeff_dist = []
    coeff_vel = []
    coeff_vis = []
    coeff_leg = 1.0
    coeffs_reg = []
    for i in range(num_timesteps):
        coeff_dist.append(500.0)
        coeff_vel.append(100.0)
        coeff_vis.append(1.0)
    for i in range(num_timesteps - 1):
        coeffs_reg.append(1.0)

    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
    default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)

    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

    combined_request = create_empty_request(20, final_joint, self.manipulator_name)
    add_distance_baseline_cost(combined_request, head_pos, torso_pos, feet_pos, self.eef_link_name, 20, 5)
    add_visibility_baseline_cost(combined_request, head_pos, object_pos, self.eef_link_name, 20, 5)
    add_regularize_cost(combined_request, coeffs_reg, self.eef_link_name)
    add_collision_cost(combined_request, [20], [0.025])
    add_smoothing_cost(combined_request, 10, 2)

    comb_result = self.optimize_problem(combined_request)
    if (exec_traj):
        self.execute_full_trajectory(comb_result.GetTraj(), full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)

    return self.evaluate_metrics(comb_result.GetTraj(), full_complete_test_traj, len(obs_rightarm_test_traj)/ 12, object_pos, default_traj)


def calculate_adaptive_trajectory(self, robot_joints, human_pos, human_num_joints, robot_num_joints):
    '''
    robot_joints: time-sampled JOINT space trajectory (vectorized - timesteps*robot_num_joints*3 matrix)
    human_pos: human position from vision system (vectorized - timesteps*human_num_joints*3matrix)
    the order of human_pos is - right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
    ideally, robot_joints >= human_pos_timesteps
    TODO: assume blackbox function to calculate separation distance // get_separation_dist(human_joints, robots_joints)
    '''
    scaling_factor = 1  # TODO choose based on separation dist
    d_slow = 0.15# choose threshold
    d_stop = 0.6# choose threshold
    beta = 0.5# parameter
    gamma = 0.5# parameter
    traj_interpolation = self.cubic_interpolation(robot_joints, robot_num_joints)
    num_timesteps = len(human_pos)/(human_num_joints*3)
    robot_total_timesteps = len(robot_joints)
    new_exec_traj = [] 
    robot_timestep = 0
    done = False
    human_timestep = 0
    while not done:
        print("Curr Human Timesteps = ", human_timestep)
        print("Curr Robot Timesteps = ", robot_timestep)
        new_robot_joints = traj_interpolation(robot_timestep).tolist()
        new_exec_traj.append(new_robot_joints)

        d = self.get_separation_dist(human_pos[human_timestep*human_num_joints*3:(human_timestep + 1)*human_num_joints*3], new_robot_joints)
        if (d_stop <= d <= d_slow):
            scaling_factor = 1 - beta * ((d - d_stop) ** gamma)
        elif (d_slow < d):# change to new timestamp and trajectory pts
            scaling_factor = 0
        else:
            scaling_factor = 1
        
        robot_timestep += 0.1 - scaling_factor/10
        human_timestep += 0 if human_timestep >= (num_timesteps - 1) else 1

        if robot_total_timesteps <= robot_timestep:
            done = True
        elif human_timestep == (num_timesteps - 1) and scaling_factor == 1:
            done = True
        else:
            done = False
    # if robot_timestep < num_timesteps:
    #     new_exec_traj.extend(robot_joints[math.ceil(robot_timestep)::])
    
    return new_exec_traj


def speed_control_baseline_test(self, init_joint, final_joint, traj_num=303, exec_traj = False):

    # Read pose prediction files
    full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
    # Expand ground truth human trajectory
    full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)
    obs_complete_test_traj = create_human_plot_traj(obs_rightarm_test_traj)

    
    object_pos = [0, 0.2, 0.83]

    # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
    n_human_joints = 11
    num_timesteps = 20
    num_human_timesteps = len(full_complete_test_traj) / (n_human_joints * 3)
    num_obs_timesteps = len(obs_complete_test_traj) / (n_human_joints * 3)

    exec_complete_test_traj = full_complete_test_traj[num_obs_timesteps * n_human_joints * 3 : ]

    # Setup coefficients
    coeff_optimal_traj = 5.0
    coeff_dist = []
    coeff_vel = []
    coeff_vis = []
    coeff_leg = 1.0
    coeffs_reg = []
    for i in range(num_timesteps):
        coeff_dist.append(100.0)
        coeff_vel.append(100.0)
        coeff_vis.append(1.0)
    for i in range(num_timesteps - 1):
        coeffs_reg.append(1.0)

    # Generate default trajectory using trajopt
    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
    default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

    adapted_robot_traj_fast = self.calculate_adaptive_trajectory(default_traj, exec_complete_test_traj, 11, 7)

    if len(adapted_robot_traj_fast) < 200:
        last_joints = adapted_robot_traj_fast[-1]
        for i in range(200 - len(adapted_robot_traj_fast)):
            adapted_robot_traj_fast.append(last_joints)

    new_num_timesteps_fast = len(adapted_robot_traj_fast)
    print(len(default_traj))
    print(new_num_timesteps_fast)

    adapted_robot_fast_spline = self.cubic_interpolation(adapted_robot_traj_fast, 7)

    adapted_robot_traj_slow = []
    for i in range(20):
        adapted_robot_traj_slow.append(adapted_robot_fast_spline(i * 10))


    if (exec_traj):
        self.execute_full_trajectory(adapted_robot_traj_slow, full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)

    return self.evaluate_metrics(adapted_robot_traj_slow, full_complete_test_traj, len(obs_rightarm_test_traj)/ 12, object_pos, default_traj)

def legibility_baseline_test(self, init_joint, final_joint, traj_num=303, exec_traj = True):

    # Read pose prediction files
    full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
    # Expand ground truth human trajectory
    full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)
    obs_complete_test_traj = create_human_plot_traj(obs_rightarm_test_traj)

    object_pos = [0, 0.2, 0.83]

    # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
    n_human_joints = 11
    num_timesteps = 20
    num_human_timesteps = len(full_complete_test_traj) / (n_human_joints * 3)
    num_obs_timesteps = len(obs_complete_test_traj) / (n_human_joints * 3)

    exec_complete_test_traj = full_complete_test_traj[num_obs_timesteps * n_human_joints * 3 : ]

    coeffs_reg = []
    for i in range(num_timesteps - 1):
        coeffs_reg.append(1.0)

    self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

    leg_request = create_empty_request(20, final_joint, self.manipulator_name)
    add_legibility_cost(leg_request, coeff_leg, self.eef_link_name)
    # add_regularize_cost(leg_request, coeffs_reg, self.eef_link_name)
    add_collision_cost(leg_request, [20], [0.025])
    # add_smoothing_cost(leg_request, 10, 2)

    leg_result = self.optimize_problem(leg_request)
    if (exec_traj):
        self.execute_full_trajectory(leg_result.GetTraj(), full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)

    # TODO run metrics
    return self.evaluate_metrics(leg_result.GetTraj(), full_complete_test_traj, len(obs_rightarm_test_traj)/ 12, object_pos, default_traj)


def test_baselines(self, init_joint, final_joint, traj_num=303, baselines=[1, 1, 1, 1], exec_traj = False):

    if baselines[0]:
        print("Starting nominal baseline test...")
        nominal_baseline_metrics = self.nominal_baseline_test(init_joint, final_joint, traj_num, exec_traj)
    else:
        nominal_baseline_metrics = [0, 0, 0, 0]
    
    if baselines[1]:
        print("Start distance visiblity baseline test...")
        dist_vis_baseline_metrics = self.dist_vis_baselines_test(init_joint, final_joint, traj_num, exec_traj)
        # print("Distance visibility baseline metric: \n", dist_vis_baseline_metrics)
    else:
        dist_vis_baseline_metrics = [0, 0, 0, 0]
    
    if baselines[2]:
        print("Start legibility baseline test...")
        legibility_baseline_metrics = self.legibility_baseline_test(init_joint, final_joint, traj_num, exec_traj)
        # print("Legibility baseline metric: \n", legibility_baseline_metrics)
    else:
        legibility_baseline_metrics = [0, 0, 0, 0]
    
    if baselines[3]:
        print("Start speed control baseline test...")
        speed_control_baseline_metrics = self.speed_control_baseline_test(init_joint, final_joint, traj_num, exec_traj)
        # print("Speed control baseline metric: \n", speed_control_baseline_metrics)
    else:
        speed_control_baseline_metrics = [0, 0, 0, 0]

    return [nominal_baseline_metrics, dist_vis_baseline_metrics, legibility_baseline_metrics, speed_control_baseline_metrics]






if __name__ == '__main__':

    plot = True
    use_jaco = True # use franka by default, if this is true, use jaco
    ues_franka = False
    use_ros = True

    raw_input("Finished...")