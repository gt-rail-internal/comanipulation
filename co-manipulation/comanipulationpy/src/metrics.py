import traj_calc

import numpy as np
import math
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import pandas as pd
import os.path
from os import path

import traj_utils 

HUMAN_LINKS = np.array([[0, 1], [1, 2], [2, 3], [0, 4], [4, 5],
                        [4, 6], [4, 7], [7, 8], [8, 9], [9, 10]])
ROBOT_LINKS = np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6]])
METRIC_ORDER = ["Distance Metric", "Visibility Metric",
                "Legibility Metric", "Nominal Trajectory Metric"]

def metric_print_helper(metrics, heading):
    """
    Helper method for print_metrics. Prints in order specified by Metric order

    Metrics : 2D Array of 4 X Number of Trajectorys floats signifying score on each metric
    Heading: The Heading/Label for each score. Specified in METRIC_ORDER
    """
    print("\n")
    print(heading)
    for index, metric in enumerate(metrics):
        print(METRIC_ORDER[index] + ": " + str(np.mean(metric)) + " +/- " + str(np.std(metric)))
    print("\n")

def create_csv(file_name, our_metrics, baseline):
    """
    Helper method to create the csv with the proper header

    filename: the filename of the csv
    our_metrics: Scores of our metrics
    baseline: Scores of the baselines
    """
    headers = ["Test Case"]
    for index, metric in enumerate(our_metrics):
        headers.append("Our Metrics_"+METRIC_ORDER[index])
    for i in range(4):
        for index, metric in enumerate(baseline[i]):
            if (i == 0):
                headers.append("Distance + Visibility_"+METRIC_ORDER[index])
            if (i == 1):
                headers.append("Legibility_"+METRIC_ORDER[index])
            if (i == 2):
                headers.append("Nominal Trajectory_"+METRIC_ORDER[index])
            if (i == 3):
                headers.append("Speed Control_"+METRIC_ORDER[index])
    df = pd.DataFrame(columns=headers)
    df.to_csv(file_name, index=False)

def metrics_to_csv(test_case, our_metrics, baseline):
    """
    Helper method to output the metrics to a csv for easy interpretation

    test_case: the experiment that was run
    our_metrics: Scores of our metrics
    baseline: Scores of the baselines
    """
    file_name = '../human_prob_models/scripts/csvFiles/TestingResults.csv'
    data = [test_case]
    for index, metric in enumerate(our_metrics):
        data.append(np.mean(metric))
    for i in range(4):
        for index, metric in enumerate(baseline[i]):
            data.append(np.mean(metric))

    if not path.exists(file_name):
        create_csv(file_name, our_metrics, baseline)

    df = pd.DataFrame([data])

    # For the first run, set header=headers
    df.to_csv(file_name, mode='a', header=False, index=False)

def save_experiments(test_case, our_metrics, baseline):
    """
    Helper method to output our experiemtns for reference and for the future paper results 

    test_case: the experiments that were run
    our_metrics: Scores of our metrics
    baseline: Scores of the baselines
    """
    file_name = '../human_prob_models/scripts/csvFiles/ExperimentResults.csv'
    data = [test_case]
    for index, metric in enumerate(our_metrics):
        data.append(str(str(np.mean(metric)) + " +/- " + str(np.std(metric))))
    for i in range(4):
        for index, metric in enumerate(baseline[i]):
            data.append(str(str(np.mean(metric)) + " +/- " + str(np.std(metric))))

    if not path.exists(file_name):
        create_csv(file_name, our_metrics, baseline)

    df = pd.DataFrame([data])

    # For the first run, set header=headers
    df.to_csv(file_name, mode='a', header=False, index=False)

def print_metrics(comanipulationMetrics, baselineMetrics):
    metric_print_helper(comanipulationMetrics, "Our Metrics")
    metric_print_helper(baselineMetrics[0], "Distance + Visibility Baseline")
    metric_print_helper(baselineMetrics[1], "Legibility Baseline")
    metric_print_helper(baselineMetrics[2], "Nominal Trajectory Baseline")
    metric_print_helper(baselineMetrics[3], "Speed Control Baseline")


def calculate_distance_3d(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)


def get_visibility_angle(scene, head_pos, robot_joints, object_pos):
    """
    Returns the angle between the robot's end effector and the object from the perspective of the head

    head_pos: a vector representing the head position in cartesian space
    robot_joints: a vector representing the robot's joint angles
    object_pos: a vector representing the object position in cartesian space
    """
    eef_pos = scene.get_eef_position(robot_joints)
    vec_head_eef = np.array(head_pos) - np.array(eef_pos)
    vec_head_obj = np.array(head_pos) - np.array(object_pos)
    return math.acos(vec_head_obj.dot(vec_head_eef) / (np.linalg.norm(vec_head_obj) * np.linalg.norm(vec_head_eef)))


def get_separation_dist(scene, human_pos, robot_joints, human_sphere_radius=0.10, 
        human_sphere_num=10, robot_sphere_radius=0.10, robot_sphere_num=10, plot=False):
    """
    Returns the minimum separation distance between a human and a robot.

    human_pos: human position from vision system - one timestep, vectorized
    the order of human_pos joints is right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
    robot_joints: time-sampled JOINT space trajectory  - one timestep - vectorized
    human_sphere_radius: the radius of the spheres sampled on the human
    human_sphere_num: number of spheres per link for the human
    robot_sphere_radius: the radius of the spheres sampled on the robot
    robot_sphere_num: number of spheres per link for the robot
    """

    human_link_info = {
        0: [20, 0.1],
        1: [20, 0.1],
        2: [10, 0.1],
        3: [20, 0.1],
        4: [10, 0.1],
        5: [20, 0.2],
        6: [20, 0.1],
        7: [20, 0.1],
        8: [20, 0.1],
        9: [10, 0.1]
    }

    robot_link_info = {
        0: [20, 0.15],
        1: [20, 0.15],
        2: [20, 0.15],
        3: [20, 0.15],
        4: [20, 0.15],
        5: [20, 0.15],
    }

    human_pos = np.array(human_pos)
    robot_joints_pos = scene.performFK(robot_joints)
    robot_joints_pos = np.array(robot_joints_pos)
    robot_joints_pos = np.reshape(robot_joints_pos, 7*3)

    if plot:
        exp_setup = plt.figure()
        plotter = exp_setup.add_subplot(111, projection='3d')

    distance = float('inf')

    for human_link_index, curr_human_link in enumerate(HUMAN_LINKS):
        curr_human_sphere_num, curr_human_sphere_radius = human_link_info[human_link_index]
        human_link_start = human_pos[3*curr_human_link[0]:3*(1+curr_human_link[0])]
        human_link_end = human_pos[3*curr_human_link[1]:3*(1+curr_human_link[1])]
        human_sphere_sep = (human_link_end - human_link_start)/curr_human_sphere_num
        # create human_sphere_num spheres equally spaced along each human link
        human_spheres_centers = [human_link_start + i * human_sphere_sep for i in range(curr_human_sphere_num)]
        
        if plot:
            humanXLine = np.linspace(human_link_start[0], human_link_end[0], 25)
            humanYLine = np.linspace(human_link_start[1], human_link_end[1], 25)
            humanZLine = np.linspace(human_link_start[2], human_link_end[2], 25)
            plotter.plot3D(humanXLine, humanYLine, humanZLine, c='red')
            for center in human_spheres_centers:
                plotter.scatter([center[0]], [center[1]], [center[2]], c='y')

        for robot_link_index, curr_robot_link in enumerate(ROBOT_LINKS):
            curr_robot_sphere_num, curr_robot_sphere_radius = robot_link_info[robot_link_index]
            robot_link_start = robot_joints_pos[3 *
                                                curr_robot_link[0]:3*(1+curr_robot_link[0])]
            robot_link_end = robot_joints_pos[3 *
                                              curr_robot_link[1]:3*(1+curr_robot_link[1])]
            robot_sphere_sep = (robot_link_end - robot_link_start)/curr_robot_sphere_num
            # create robot_sphere_num spheres equally spaced along each robot link
            robot_spheres_centers = [robot_link_start + i * robot_sphere_sep for i in range(curr_robot_sphere_num)]

            if plot:
                robotXLine = np.linspace(robot_link_start[0], robot_link_end[0], 25)
                robotYLine = np.linspace(robot_link_start[1], robot_link_end[1], 25)
                robotZLine = np.linspace(robot_link_start[2], robot_link_end[2], 25)
                plotter.plot3D(robotXLine, robotYLine, robotZLine, c='blue')
                for center in robot_spheres_centers:
                    plotter.scatter([center[0]], [center[1]], [center[2]], c='g')

            for human_sphere_center in human_spheres_centers:
                for robot_sphere_center in robot_spheres_centers:
                    center_dist = calculate_distance_3d(
                        human_sphere_center, robot_sphere_center)
                    curr_distance = center_dist - curr_human_sphere_radius - curr_robot_sphere_radius
                    distance = min(distance, curr_distance)
    if plot:
        plt.show()
    return distance


def evaluate_metrics(scene, robot_traj, human_traj, num_obs_timesteps, object_pos, nominal_traj):
    """
    Evaluates each metric on the robot, then prints its results

    robot_traj: the robot trajectory (vectorized)
    human_traj: the human trajectory (full skeleton, vectorized)
    num_obs_timesteps: number of observed human timesteps
    object_pos: vector representing object position
    nominal_traj: nominal robot trajectory, vectorized
    """
    num_human_joints = 11

    human_traj_expanded = traj_utils.expand_human_test_traj(
        human_traj, 11, num_obs_timesteps, 20)

    num_timesteps_expanded = len(
        human_traj_expanded) / (num_human_joints * 3)

    full_head_test_traj_expanded = []
    head_ind = 5

    print("Num human timesteps expanded: ", num_timesteps_expanded)

    for i in range(num_timesteps_expanded):
        start_head_pos = (i * num_human_joints + head_ind) * 3
        full_head_test_traj_expanded.append(
            human_traj_expanded[start_head_pos])
        full_head_test_traj_expanded.append(
            human_traj_expanded[start_head_pos + 1])
        full_head_test_traj_expanded.append(
            human_traj_expanded[start_head_pos + 2])

    distance_metric = compute_distance_metric(scene,
        human_traj_expanded, num_obs_timesteps, num_timesteps_expanded, robot_traj)
    visibility_metric = compute_visibility_metric(scene,
        full_head_test_traj_expanded, num_obs_timesteps, num_timesteps_expanded, robot_traj, object_pos)
    legibility_metric = compute_legibility_metric(scene, robot_traj)
    nominal_traj_metric = compute_nominal_traj_metric(scene, robot_traj, nominal_traj)

    metrics = [distance_metric, visibility_metric,
                legibility_metric, nominal_traj_metric]

    # print(metrics)
    return metrics

def compute_distance_metric(scene, human_traj_expanded, num_obs_timesteps, num_total_timesteps, robot_traj):
    """
    Returns a distance metric, defined as the fraction of timesteps in which some part of the robot 
    gets closer to the human than a hardcoded threshold. Between 0 and 1.

    scene: an instance of a Scene object (from scene_utils)
    robot_traj: the robot trajectory (vectorized)
    human_traj: the human trajectory (full skeleton, vectorized and expanded)
    num_obs_timesteps: number of observed human timesteps
    num_total_timesteps: total timesteps in each trajectory
    """

    n_human_joints = 11
    n_robot_joints = 7

    distance_threshold = 0.2

    robot_traj_spline = traj_calc.cubic_interpolation(robot_traj, n_robot_joints)
    num_above_threshold = 0

    for t in range(num_obs_timesteps, num_total_timesteps):
        # robot trajectory is sampled at 10 Hz, human trajectory is sampled at 100 Hz
        # so robot discrete time is human discrete time / 10
        robot_timestep = (t - num_obs_timesteps) / 10.0
        robot_joints = robot_traj_spline(robot_timestep)

        dist_t = get_separation_dist(scene, human_traj_expanded[t*n_human_joints*3 : (t+1)*n_human_joints*3], robot_joints)
        if dist_t > distance_threshold:
            num_above_threshold += 1
    
    return num_above_threshold * 1.0 / (num_total_timesteps - num_obs_timesteps)

def compute_visibility_metric(scene, full_head_test_traj_expanded, num_obs_timesteps, num_total_timesteps, robot_traj, object_pos):
    """
    Returns a visibility metric, defined as the fraction of timesteps the 
    eef-head-object angle falls below a hardcoded threshold. Between 0 and 1.

    scene: an instance of a Scene object (from scene_utils)
    robot_traj: the robot trajectory (vectorized)
    full_head_test_traj_expanded: the human head trajectory (vectorized and expanded)
    num_obs_timesteps: number of observed human timesteps
    num_total_timesteps: total timesteps in each trajectory
    """
    n_robot_joints = 7

    visibility_threshold = 1.4

    robot_traj_spline = traj_calc.cubic_interpolation(robot_traj, n_robot_joints)
    num_below_thres = 0

    visibilities = []

    for t in range(num_obs_timesteps, num_total_timesteps):
        robot_timestep = (t - num_obs_timesteps) / 10.0
        robot_joints = robot_traj_spline(robot_timestep)
        
        curr_head_pos = full_head_test_traj_expanded[t * 3: (t + 1) * 3]
        vis_t = get_visibility_angle(scene, curr_head_pos, robot_joints, object_pos)

        visibilities.append(vis_t)

        if vis_t < visibility_threshold:
            num_below_thres += 1
    
    return float(num_below_thres) / float(num_total_timesteps - num_obs_timesteps)

def compute_legibility_metric(scene, robot_traj):
    """
    Returns a legibility metric based on the robot's average deviation
    from a linear path in cartesian space. 
    Bounded between 0 and 1.

    scene: an instance of a Scene object (from scene_utils)
    robot_traj: the robot trajectory (vectorized)
    """
    num_timesteps = np.array(robot_traj).shape[0]
    eef_pos_dist = np.zeros((num_timesteps - 1))
    legibility = 0.0
    f_t = np.ones((num_timesteps - 1)) # f_t allows different weighting of the average

    eef_goal_pos = scene.get_eef_position(robot_traj[-1])
    eef_start_pos = scene.get_eef_position(robot_traj[0])

    start_goal_dist = np.linalg.norm(eef_goal_pos - eef_start_pos)

    for i in range(num_timesteps - 1):
        f_t[i] = num_timesteps - i #f(t) = T - t where T is the complete time period
        curr_eef_pos = scene.get_eef_position(robot_traj[i])
        next_eef_pos = scene.get_eef_position(robot_traj[i + 1])
        eef_pos_dist[i] = np.linalg.norm(next_eef_pos - curr_eef_pos)

        dist_remaining = np.linalg.norm(eef_goal_pos - next_eef_pos)
        dist_traveled = np.sum(eef_pos_dist[:i])

        p_g_given_q = np.exp(-dist_traveled - dist_remaining) / np.exp(-start_goal_dist)
        legibility += p_g_given_q * f_t[i]

    legibility = 1 - (legibility / np.sum(f_t))

    return legibility

def compute_nominal_traj_metric(scene, robot_traj, nominal_traj):
    """
    Returns a metric based on the difference between the robot 
    trajectory and the nominal trajectory. No upper bound.

    scene: an instance of a Scene object (from scene_utils)
    robot_traj: the robot trajectory (vectorized)
    nominal_traj: the nominal trajectory (vectorized)
    """
    num_timesteps = len(robot_traj)

    deviation = 0
    
    for i in range(num_timesteps):
        p_eef_t = scene.get_eef_position(list(robot_traj[i]))
        p_nom_t = scene.get_eef_position(list(nominal_traj[i]))

        deviation += np.linalg.norm(p_eef_t - p_nom_t)

    return deviation