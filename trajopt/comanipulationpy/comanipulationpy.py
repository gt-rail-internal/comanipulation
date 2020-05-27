import openravepy
import trajoptpy
import json
import time
import numbers
import numpy as np
from operator import add

def create_empty_request(num_timesteps, joint_target, manipulator_name):
    request = {
        "basic_info" : {
            "n_steps" : num_timesteps,
            "manip" : manipulator_name, # see below for valid values
            "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
        },
        "constraints" : [
        {
            "type" : "joint", # joint-space target
            "params" : {"vals" : joint_target } # length of vals = # dofs of manip
        }
        ],
        "init_info" : {
            "type" : "straight_line", # straight line in joint space.
            "endpoint" : joint_target
        }
    }
    return request


def set_init_traj(request, init_traj):
    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = init_traj
    return request


def add_distance_cost(request, human_poses_mean, human_poses_var, coeffs, n_human_joints, links):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "distance_cost":
            print("ERROR distance cost already present in request")
            return request
    dist_cost = {"type" : "distance_cost", "params" : {}}
    # list of 3-vectors of size n_human_joints * num_timesteps
    dist_cost["params"]["human_poses_mean"] = human_poses_mean
    # list of 3-vectors of size n_human_joints * num_timesteps
    dist_cost["params"]["human_poses_var"] = human_poses_var
    # list of size num_timesteps
    dist_cost["params"]["coeffs"] = coeffs
    # number of human joints
    dist_cost["params"]["n_human_joints"] = n_human_joints
    dist_cost["params"]["links"] = links
    request["costs"].append(dist_cost)
    return request



def add_velocity_cost(request, human_poses_mean, human_poses_var, coeffs, n_human_joints, links):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "velocity_cost":
            print("ERROR velocity cost already present in request")
            return request
    vel_cost = {"type" : "velocity_cost", "params" : {}}
    vel_cost["params"]["human_poses_mean"] = human_poses_mean
    vel_cost["params"]["human_poses_var"] = human_poses_var
    vel_cost["params"]["coeffs"] = coeffs
    vel_cost["params"]["n_human_joints"] = n_human_joints
    vel_cost["params"]["links"] = links
    request["costs"].append(vel_cost)
    return request


def add_visibility_cost(request, head_pos_mean, head_pos_var, coeffs, object_pose, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "visibility_cost":
            print("ERROR visibility cost already present in request")
            return request
    vis_cost = {"type" : "visibility_cost", "params" : {}}
    vis_cost["params"]["head_pos_mean"] = head_pos_mean
    vis_cost["params"]["head_pos_var"] = head_pos_var
    vis_cost["params"]["obj_pos"] = object_pose
    vis_cost["params"]["coeffs"] = coeffs
    vis_cost["params"]["link"] = link
    request["costs"].append(vis_cost)
    return request


def add_legibility_cost(request, coeffs, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "legibility_cost":
            print("ERROR: Legibility cost already present in request")
            return request
    leg_cost = {"type" : "legibility_cost", "params" : {"link" : link, "coeffs" : coeffs}}
    request["costs"].append(leg_cost)
    return request


def add_optimal_trajectory_cost(request, target_eef_traj, link, num_timesteps, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for t in range(num_timesteps):
        pose_cost = {"type" : "pose", "params" : {}}
        pose_cost["params"]["timestep"] = t
        pose_cost["params"]["xyz"] = [target_eef_traj[t, 0], target_eef_traj[t, 1], target_eef_traj[t, 2]]
        pose_cost["params"]["wxyz"] = [1, 0, 0, 0]
        pose_cost["params"]["pos_coeffs"] = [coeffs, coeffs, coeffs]
        pose_cost["params"]["rot_coeffs"] = [0, 0, 0]
        pose_cost["params"]["link"] = link
        request["costs"].append(pose_cost)
    return request


def add_distance_baseline_cost(request, head_pos, torso_pos, feet_pos, link, num_timesteps, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "distance_baseline_cost":
            print("ERROR: Distance baseline cost already present in request")
            return request
    dist_baseline_cost = {"type" : "distance_baseline_cost", "params" : {}}
    dist_baseline_cost["params"]["head_pos"] = head_pos
    dist_baseline_cost["params"]["torso_pos"] = torso_pos
    dist_baseline_cost["params"]["feet_pos"] = feet_pos
    dist_baseline_cost["params"]["link"] = link
    dist_baseline_cost["params"]["coeffs"] = coeffs
    request["costs"].append(dist_baseline_cost)
    return request

def add_visibility_baseline_cost(request, head_pos, obj_pos, link, num_timesteps, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "visibility_baseline_cost":
            print("ERROR: Visibility baseline cost already present in request")
            return request
    dist_baseline_cost = {"type" : "visibility_baseline_cost", "params" : {}}
    dist_baseline_cost["params"]["head_pos"] = head_pos
    dist_baseline_cost["params"]["obj_pos"] = obj_pos
    dist_baseline_cost["params"]["link"] = link
    dist_baseline_cost["params"]["coeffs"] = coeffs
    request["costs"].append(dist_baseline_cost)
    return request


def add_regularize_cost(request, coeffs, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "regularize_cost":
            print("ERROR: Regularize cost already present in request")
            return request
    reg_cost = {"type" : "regularize_cost", "params" : {"link" : link, "coeffs" : coeffs}}
    request["costs"].append(reg_cost)
    return request

def add_smoothing_cost(request, coeff, type):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "smoothing_cost":
            print("ERROR: Smoothing cost already present in request")
            return request
    smooth_cost = {"type" : "smoothing_cost", "params" : {"coeffs" : coeff, "type" : type}}
    request["costs"].append(smooth_cost)
    return request


def add_collision_cost(request, coeffs, dist_pen):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "collision":
            print("ERROR: Collision cost already present in request")
            return request
    collision_cost = {"type" : "collision" ,"params" : {"coeffs" : coeffs, "dist_pen" : dist_pen}}
    request["costs"].append(collision_cost)
    return request


def add_cart_vel_cnst(request, max_displacement, first_step, last_step, link):
    if "constraints" not in request:
        request["constraints"] = []
    cart_vel_cnst = {"type" : "cart_vel", "name" : "cart_vel", "params" : {}}
    cart_vel_cnst["params"]["max_displacement"] = max_displacement
    cart_vel_cnst["params"]["first_step"] = first_step
    cart_vel_cnst["params"]["last_step"] = last_step
    cart_vel_cnst["params"]["link"] = link
    request["constraints"].append(cart_vel_cnst)
    return request

def add_joint_vel_cost(request, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "joint_vel":
            print("ERROR: Joint velocity cost already present in request")
            return request
    joint_vel_cost = {"type" : "joint_vel", "params" : {}}
    if isinstance(coeffs, numbers.Number):
        joint_vel_cost["params"]["coeffs"] = [coeffs]
    else:
        joint_vel_cost["params"]["coeffs"] = coeffs


def create_human_trajectory_tree(right_arm_trajectory):
    num_timesteps = len(right_arm_trajectory)/12

    human_trajectory = {}

    human_trajectory["right_shoulder"] = []
    human_trajectory["right_elbow"] = []
    human_trajectory["right_wrist"] = []
    human_trajectory["right_palm"] = []

    human_trajectory["neck"] = []
    human_trajectory["head"] = []
    human_trajectory["torso"] = []

    human_trajectory["left_shoulder"] = []
    human_trajectory["left_elbow"] = []
    human_trajectory["left_wrist"] = []
    human_trajectory["left_palm"] = []

    for i in range(num_timesteps):

        # human_trajectory["right_shoulder"].append([right_arm_trajectory[i * 12 + 0], right_arm_trajectory[i * 12 + 1], right_arm_trajectory[i * 12 + 2]])
        # human_trajectory["right_elbow"].append([right_arm_trajectory[i * 12 + 3] - right_arm_trajectory[i * 12 + 0], right_arm_trajectory[i * 12 + 4] -  right_arm_trajectory[i * 12 + 1], right_arm_trajectory[i * 12 + 5] - right_arm_trajectory[i * 12 + 2]])
        # human_trajectory["right_wrist"].append([right_arm_trajectory[i * 12 + 6] - right_arm_trajectory[i * 12 + 3], right_arm_trajectory[i * 12 + 7] -  right_arm_trajectory[i * 12 + 4], right_arm_trajectory[i * 12 + 8] - right_arm_trajectory[i * 12 + 5]])
        # human_trajectory["right_palm"].append([right_arm_trajectory[i * 12 + 9] - right_arm_trajectory[i * 12 + 6], right_arm_trajectory[i * 12 + 10] -  right_arm_trajectory[i * 12 + 7], right_arm_trajectory[i * 12 + 11] - right_arm_trajectory[i * 12 + 8]])



        human_trajectory["right_shoulder"].append([right_arm_trajectory[i * 12 + 0], right_arm_trajectory[i * 12 + 1], right_arm_trajectory[i * 12 + 2]])
        human_trajectory["right_elbow"].append([right_arm_trajectory[i * 12 + 3] - right_arm_trajectory[i * 12 + 0], right_arm_trajectory[i * 12 + 4] -  right_arm_trajectory[i * 12 + 1], right_arm_trajectory[i * 12 + 5] - right_arm_trajectory[i * 12 + 2]])
        human_trajectory["right_wrist"].append([right_arm_trajectory[i * 12 + 6] - right_arm_trajectory[i * 12 + 3], right_arm_trajectory[i * 12 + 7] -  right_arm_trajectory[i * 12 + 4], right_arm_trajectory[i * 12 + 8] - right_arm_trajectory[i * 12 + 5]])
        human_trajectory["right_palm"].append([right_arm_trajectory[i * 12 + 9] - right_arm_trajectory[i * 12 + 6], right_arm_trajectory[i * 12 + 10] -  right_arm_trajectory[i * 12 + 7], right_arm_trajectory[i * 12 + 11] - right_arm_trajectory[i * 12 + 8]])

        human_trajectory["neck"].append([0.2, 0, 0])
        human_trajectory["head"].append([0, 0, 0.15])

        human_trajectory["torso"].append([right_arm_trajectory[0] - right_arm_trajectory[i * 12 + 0], right_arm_trajectory[1] -  right_arm_trajectory[i * 12 + 1], right_arm_trajectory[2] - right_arm_trajectory[i * 12 + 2] - 0.6])
        # human_trajectory["torso"].append([0, 0, -0.4])

        human_trajectory["left_shoulder"].append([0.2, 0, 0])
        human_trajectory["left_elbow"].append([0, 0, -0.3])
        human_trajectory["left_wrist"].append([0, 0, -0.3])
        human_trajectory["left_palm"].append([0, 0, -0.1])

    return human_trajectory

# parse human pose means and vars and return full skeleton for optimization
# TODO update offsets
def create_human_means_vars(human_poses_mean, human_poses_var):

    num_timesteps = len(human_poses_mean)/12

    human_global_trajectory = []
    human_variance = []

    for i in range(num_timesteps):

        right_shoulder = [human_poses_mean[i * 12 + 0], human_poses_mean[i * 12 + 1], human_poses_mean[i * 12 + 2]]
        right_shoulder_variance = human_poses_var[i*36 : i*36 + 9]

        right_elbow = [human_poses_mean[i * 12 + 3], human_poses_mean[i * 12 + 4], human_poses_mean[i * 12 + 5]]
        right_elbow_variance = human_poses_var[i*36 + 9 : i*36 + 18]

        right_wrist = [human_poses_mean[i * 12 + 6], human_poses_mean[i * 12 + 7], human_poses_mean[i * 12 + 8]]
        right_wrist_variance = human_poses_var[i*36 + 18 : i*36 + 27]

        right_palm = [human_poses_mean[i * 12 + 9], human_poses_mean[i * 12 + 10], human_poses_mean[i * 12 + 11]]
        right_palm_variance = human_poses_var[i*36 + 27 : i*36 + 36]


        neck = list(map(add, [0.2, 0, 0], right_shoulder))
        head = list(map(add, [0, 0, 0.15], neck))
        torso = list(map(add, [0, 0, -0.6], neck))

        left_shoulder = list(map(add, [0.2, 0, 0], neck))
        left_elbow = list(map(add, [0, 0, -0.3], left_shoulder))
        left_wrist = list(map(add, [0, 0, -0.3], left_elbow))
        left_palm = list(map(add, [0, 0, -0.1], left_wrist))

        neck_variance = right_shoulder_variance
        head_variance = right_shoulder_variance
        torso_variance = right_shoulder_variance

        left_shoulder_variance = right_shoulder_variance
        left_elbow_variance = right_shoulder_variance
        left_wrist_variance = right_shoulder_variance
        left_palm_variance = right_shoulder_variance

        human_global_trajectory += right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
        human_variance += right_shoulder_variance + right_elbow_variance + right_wrist_variance + right_palm_variance + neck_variance + head_variance + torso_variance + left_shoulder_variance + left_elbow_variance + left_wrist_variance + left_palm_variance
    
    return human_global_trajectory, human_variance


# parse human pose and return full skeleton
# TODO update offsets
def create_human_plot_traj(human_poses):

    num_timesteps = len(human_poses)/12

    human_global_trajectory = []
    human_variance = []

    for i in range(num_timesteps):

        right_shoulder = [human_poses[i * 12 + 0], human_poses[i * 12 + 1], human_poses[i * 12 + 2]]

        right_elbow = [human_poses[i * 12 + 3], human_poses[i * 12 + 4], human_poses[i * 12 + 5]]

        right_wrist = [human_poses[i * 12 + 6], human_poses[i * 12 + 7], human_poses[i * 12 + 8]]

        right_palm = [human_poses[i * 12 + 9], human_poses[i * 12 + 10], human_poses[i * 12 + 11]]


        neck = list(map(add, [0.2, 0, 0], right_shoulder))
        head = list(map(add, [0, 0, 0.15], neck))
        torso = list(map(add, [0, 0, -0.], neck))

        left_shoulder = list(map(add, [0.2, 0, 0], neck))
        left_elbow = list(map(add, [0, 0, -0.3], left_shoulder))
        left_wrist = list(map(add, [0, 0, -0.3], left_elbow))
        left_palm = list(map(add, [0, 0, -0.1], left_wrist))

        human_global_trajectory += right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
    
    return human_global_trajectory


# parse full human pose means and vars, and return human head pose means and vars
def create_human_head_means_vars(full_human_poses_mean, full_human_poses_var):
    head_pose_mean = []
    head_pose_var = []

    num_timesteps = len(full_human_poses_mean)/33

    for i in range(num_timesteps):
        head_pose_mean += full_human_poses_mean[i*33 + 15: i*33 + 18]
        head_pose_var += full_human_poses_var[i*99 + 45: i*99 + 54:4]

    return head_pose_mean, head_pose_var


# expand human prediction to 20 timesteps
def expand_human_pred(full_human_poses_mean, full_human_poses_var):
    num_timesteps = len(full_human_poses_mean) / 33
    for i in range(20 - num_timesteps):
        full_human_poses_mean += full_human_poses_mean[-33:]
        full_human_poses_var += full_human_poses_var[-99:]
        # for j in range(11):
    return full_human_poses_mean, full_human_poses_var

def expand_human_test_traj(human_test_traj, n_human_joints, num_human_obs, robot_timesteps):
    num_timesteps = len(human_test_traj) / (n_human_joints * 3)


    extra_timesteps = robot_timesteps * 10 - (num_timesteps - num_human_obs)

    last_human_pose = human_test_traj[-(n_human_joints * 3):]
    for i in range(extra_timesteps):
        human_test_traj = human_test_traj + last_human_pose

    return human_test_traj