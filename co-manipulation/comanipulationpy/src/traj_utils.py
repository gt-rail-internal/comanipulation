import openravepy
import trajoptpy
import json
import time
import numbers
import numpy as np
from operator import add

def create_human_trajectory_tree(right_arm_trajectory):
    num_timesteps = len(right_arm_trajectory)/12
    print('Printing right arm traj')
    print(right_arm_trajectory)
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

        human_trajectory["neck"].append([-0.2, 0, 0])
        human_trajectory["head"].append([0, 0, 0.15])

        human_trajectory["torso"].append([right_arm_trajectory[0] - right_arm_trajectory[i * 12 + 0], right_arm_trajectory[1] -  right_arm_trajectory[i * 12 + 1], right_arm_trajectory[2] - right_arm_trajectory[i * 12 + 2] - 0.6])
        # human_trajectory["torso"].append([0, 0, -0.4])

        human_trajectory["left_shoulder"].append([-0.2, 0, 0])
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


        neck = list(map(add, [-0.2, 0, 0], right_shoulder))
        head = list(map(add, [0, 0, 0.15], neck))
        torso = list(map(add, [0, 0, -0.6], neck))

        left_shoulder = list(map(add, [-0.2, 0, 0], neck))
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


        neck = list(map(add, [-0.2, 0, 0], right_shoulder))
        head = list(map(add, [0, 0, 0.15], neck))
        torso = list(map(add, [0, 0, -0.], neck))

        left_shoulder = list(map(add, [-0.2, 0, 0], neck))
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

def get_subsampled_human_from_dict(human_traj):
        order = ['right_shoulder', 'right_elbow', 'right_wrist', 'right_palm', 'neck', 'head', 'torso', 'left_shoulder', 'left_elbow', 'left_wrist', 'left_palm']

        final_trajectory = np.zeros([len(human_traj[order[0]][::10]), len(order)*3])
        for index, joint in enumerate(order):
            final_trajectory[:, 3*index:3*(index + 1)] = human_traj[joint][::10] ##each timestep needs to be vectorized
        
        return final_trajectory
