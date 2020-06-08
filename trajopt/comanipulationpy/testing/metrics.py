import traj_calc
import scene_utils

import numpy as np
import math

import scene_utils

import traj_utils 

HUMAN_LINKS = np.array([[0, 1], [1, 2], [2, 3], [0, 4], [4, 5],
                        [4, 6], [4, 7], [7, 8], [8, 9], [9, 10]])
ROBOT_LINKS = np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6]])


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


def get_separation_dist(scene, human_pos, robot_joints, human_sphere_radius=0.05, 
        human_sphere_num=5, robot_sphere_radius=0.05, robot_sphere_num=5):
    """
    Returns the minimum separation distance between a human and a robot.

    human_pos: human position from vision system - one timestep, vectorized
    the order of human_pos joints is right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
    robot_joint_pos: time-sampled JOINT space trajectory  - one timestep - vectorized
    human_sphere_radius: the radius of the spheres sampled on the human
    human_sphere_num: number of spheres per link for the human
    robot_sphere_radius: the radius of the spheres sampled on the robot
    robot_sphere_num: number of spheres per link for the robot
    """

    human_pos = np.array(human_pos)
    robot_joints_pos = scene.performFK(robot_joints)
    robot_joints_pos = np.array(robot_joints_pos)
    robot_joints_pos = np.reshape(robot_joints_pos, 7*3)

    distance = float('inf')

    for curr_human_link in HUMAN_LINKS:
        human_link_start = human_pos[3*curr_human_link[0]:3*(1+curr_human_link[0])]
        human_link_end = human_pos[3*curr_human_link[1]:3*(1+curr_human_link[1])]
        human_sphere_sep = (human_link_end - human_link_start)/human_sphere_num
        # create human_sphere_num spheres equally spaced along each human link
        human_spheres_centers = [human_link_start + i *
                                 human_sphere_sep for i in range(human_sphere_num)]

        for curr_robot_link in ROBOT_LINKS:
            robot_link_start = robot_joints_pos[3 *
                                                curr_robot_link[0]:3*(1+curr_robot_link[0])]
            robot_link_end = robot_joints_pos[3 *
                                              curr_robot_link[1]:3*(1+curr_robot_link[1])]
            robot_sphere_sep = (robot_link_end - robot_link_start)/robot_sphere_num
            # create robot_sphere_num spheres equally spaced along each robot link
            robot_spheres_centers = [robot_link_start + i *
                                     robot_sphere_sep for i in range(robot_sphere_num)]

            for human_sphere_center in human_spheres_centers:
                for robot_sphere_center in robot_spheres_centers:
                    center_dist = calculate_distance_3d(
                        human_sphere_center, robot_sphere_center)
                    curr_distance = center_dist - human_sphere_radius - robot_sphere_radius
                    distance = min(distance, curr_distance)

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

    print("Distance metric : ", distance_metric)
    print("Visibility metric : ", visibility_metric)
    print("Legibility metric : ", legibility_metric)
    print("Nominal traj metric : ", nominal_traj_metric)

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
        # print(vis_t)

        visibilities.append(vis_t)

        if vis_t < visibility_threshold:
            num_below_thres += 1
    
    return num_below_thres / (num_total_timesteps - num_obs_timesteps)

def compute_legibility_metric(scene, robot_traj):
    """
    Returns a legibility metric based on the robot's average deviation
    from a linear path in cartesian space. 
    No upper bound.

    scene: an instance of a Scene object (from scene_utils)
    robot_traj: the robot trajectory (vectorized)
    """
    num_timesteps = np.array(robot_traj).shape[0]
    eef_pos_dist = np.zeros((num_timesteps - 1))
    legibility = 0
    f_t = np.ones((num_timesteps - 1)) # f_t allows different weighting of the average

    eef_goal_pos = scene.get_eef_position(robot_traj[-1])
    eef_start_pos = scene.get_eef_position(robot_traj[0])

    start_goal_dist = np.linalg.norm(eef_goal_pos - eef_start_pos)

    for i in range(num_timesteps - 1):
        curr_eef_pos = scene.get_eef_position(robot_traj[i])
        next_eef_pos = scene.get_eef_position(robot_traj[i + 1])
        eef_pos_dist[i] = np.linalg.norm(next_eef_pos - curr_eef_pos)

        dist_remaining = np.linalg.norm(eef_goal_pos - next_eef_pos)
        dist_traveled = np.sum(eef_pos_dist[:i])

        p_g_given_q = np.exp(-dist_traveled - dist_remaining) / np.exp(-start_goal_dist)
        legibility += p_g_given_q * f_t[i]

    legibility = legibility / np.sum(f_t)

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
        p_eef_t = scene.get_eef_position(robot_traj[i])
        p_nom_t = scene.get_eef_position(nominal_traj[i])

        deviation += np.linalg.norm(p_eef_t - p_nom_t)

    return deviation