import const
import glob
import csv
import os.path as path

import numpy as np
import math

from traj_utils import create_human_means_vars, create_human_trajectory_tree

CSV_FOLDER = "../../human_prob_models/scripts/csvFiles"
TRAIN_FOLDER = path.join(CSV_FOLDER, "Train")
TEST_FOLDER = path.join(CSV_FOLDER, "Test")


def create_still_trajectories(traj_files):
    """
    Repeats the last frame from every file in a list to create a "trajectory" 
    with no motion. Filenames are expected to be named "{something}_{number}.csv" 
    - only numbers less than 400 are considered. Output files are named 
    "{something}_{number + 400}.csv", each in the same folder as the corresponding 
    input file.

    traj_folder: a list of string paths to trajectory files
    """
    for traj_filepath in traj_files:
        basename = path.basename(traj_filepath)
        traj_folder = path.dirname(traj_filepath)
        file_number = int(basename.split("_")[1])
        if file_number < 400:
            new_basename = basename.split("_")[0]+"_"+str(file_number+400)
            last_row = ""
            with open(traj_filepath, 'r') as f:
                for row in reversed(list(csv.reader(f))):
                    last_row = row
                    break
            timesteps = 250
            print("Creating ", new_basename+".csv")
            traj_writer = csv.writer(
                open(traj_folder+new_basename+".csv", 'w'), delimiter=',', quotechar='|')
            for timesteps in range(timesteps):
                traj_writer.writerow(last_row)


def still_trajectories_wrapper():
    """
    Calls create_still_trajectories with hardcoded folder
    """
    # TODO: ensure relative path is still correct, make relative path unnecessary
    train_traj = glob.glob(path.join(TRAIN_FOLDER, "*.csv"))
    test_traj = glob.glob(path.join(TEST_FOLDER, "*.csv"))
    create_still_trajectories(train_traj+test_traj)


def prep_data_for_prediction(pose_mean_folder=TEST_FOLDER):
    """
    Ensures all trajectories in a folder are partitioned into "trimmed" and 
    "remainder" files.

    pose_mean_folder: the path to the folder containing the prediction data
    """
    # TODO: relative paths

    csv_glob = path.join(pose_mean_folder, "*.csv")
    traj_paths = glob.glob(csv_glob)
    for traj_path in traj_paths:
        means = read_human_poses_mean(traj_path, 2)
        file_name = path.basename(traj_path).split(".")[0]
        if not "trimmed" in file_name and not "remainder" in file_name:
            print(file_name)
            crop_human_pose(means, 100, 4, path.join(pose_mean_folder, file_name + "_trimmed.csv"),
                            path.join(pose_mean_folder, file_name+"_remainder.csv"))


def crop_human_pose(human_pose_mean, obs_timesteps, dof, csv_obs_path, csv_rem_path):
    """
    Partitions a list of poses and writes it to two different files

    human_pose_mean: a 1-D list of human poses with `dof` degrees of freedom
    obs_timesteps: the number of timesteps to write to `csv_obs_path`
    dof: the number of degrees of freedom (how many elements per pose)
    csv_obs_path: the first file to write to - gets `obs_timesteps` timesteps
    csv_rem_path: the second file to write to - gets remaining timesteps
    """
    with open(csv_obs_path, 'w') as obs_file:
        obs_writer = csv.writer(obs_file, delimiter=',', quotechar='|')
        for timestep in range(obs_timesteps):
            obs_writer.writerow(
                [str(human_pose_mean[timestep*dof*3 + i]) for i in range(dof*3)])

    with open(csv_rem_path, 'w') as rem_file:
        rem_writer = csv.writer(rem_file, delimiter=',', quotechar='|')
        total_timesteps = len(human_pose_mean)/(dof*3)
        for timestep in range(obs_timesteps, total_timesteps):
            rem_writer.writerow(
                [str(human_pose_mean[timestep*dof*3 + i]) for i in range(dof*3)])


# Takes in mean_pos which is a 2D matrix (Mode 1) and var_pos which is a 3D matrix (Mode 1)
def sample_human_trajectory(mean_pos, var_pos):
    """
    At each point in mean_pos, samples from a normal distribution with that point 
    as the mean and the corresponding entry of var_pos as the covariance matrix.
    Returns an array of these samples, a new trajectory.

    mean_pos: a 2D matrix of poses
    var_pos: an array of 2D covariance matrices, parallel to mean_pos
    """
    sample = []
    for i in range(0, len(mean_pos)):
        sample = sample + \
            list(np.random.multivariate_normal(mean_pos[i], var_pos[i]))
    return sample

# mean output will depend on mode


def read_human_poses_mean(csv_path, mode=2):
    """
    Reads a set of human poses from a csv file, returning in a format that depends 
    on the mode

    csv_path: The path to the CSV file containing a human pose set.
    mode: A number determining the return format. Accepted values are 1 and 2.

    Mode 1 - mean is a 2d matrix of size timesteps * (joints*3)
    Mode 2 - mean is an array of length timesteps * joints * 3
    """
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        mean = []
        if mode == 1:
            for row in csv_reader:
                curr_row = []
                for col in row:
                    curr_row.append(float(col))
                mean.append(curr_row)
        elif mode == 2:
            for row in csv_reader:
                for col in row:
                    mean.append(float(col))
        return mean


# var output will depend on the mode
#
def read_human_poses_var(csv_path, mode=2):
    """
    Reads a set of covariance matrices from a csv file and returns them in a format
    that depends on the mode

    csv_path: The path to the CSV file containing a set of covariance matrices associated with human poses.
    mode: A number determining the return format. Accepted values are 1, 2, and 3.

    Mode 1 - returns a 3D matrix of size timesteps * (joints * 3) * (joints * 3) >> all covariances
    Mode 2 - returns an array of length timesteps * joints * 9 >> only the variances along the 3x3 block matricies along the diagonal
    Mode 3 - returns an array of length timesteps * joints * 3 >> only the variances along the covariance matrix diagonal
    """
    with open(csv_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        var = []
        for row in csv_reader:
            # numCol is the number of columns in the covariance matrix, which is stored flattened
            # since the covariance matrix is square, it is also the number of rows
            numCol = int(math.sqrt(len(row)))
            if mode == 1:
                curr_matrix = []
                for i in range(0, numCol):
                    curr_row = []
                    for j in range(0, numCol):
                        curr_row.append(float(row[i*numCol + j]))
                    curr_matrix.append(curr_row)
                var.append(curr_matrix)
            elif mode == 2:
                for i in range(0, numCol):
                    for j in range(0, numCol):
                        if j < 3*(i//3):
                            continue
                        elif j >= (3*(i//3) + 3):
                            continue
                        else:
                            var.append(float(row[i*numCol + j]))
            elif mode == 3:
                for i in range(0, numCol):
                    for j in range(0, numCol):
                        if i == j:
                            var.append(float(row[i*numCol + j]))
    return var


def load_all_human_trajectories(traj_num):
    """
    Loads a full trajectory, its partitions from prep_data_for_prediction, and
    its variance from csv files. Returns the full trajectory, both partitions, 
    and the variance. Trajectories are assumed to be in TEST_FOLDER.

    traj_num: the number of the trajectory to load.
    """
    full_rightarm_test_traj_file = path.join(TEST_FOLDER, "traj_%d.csv" % traj_num)
    full_rightarm_test_traj = read_human_poses_mean(full_rightarm_test_traj_file)
    full_rightarm_test_traj = add_offset_human_traj(full_rightarm_test_traj, 4, traj_num)

    obs_rightarm_test_traj_file = path.join(TEST_FOLDER, "traj_%d_trimmed.csv" % traj_num)
    obs_rightarm_test_traj = read_human_poses_mean(obs_rightarm_test_traj_file)
    obs_rightarm_test_traj = add_offset_human_traj(obs_rightarm_test_traj, 4, traj_num)

    rightarm_pred_traj_means_file = path.join(
        CSV_FOLDER, "Predictions", "predSampledtraj_%d_trimmed.csv" % traj_num)
    rightarm_pred_traj_means = read_human_poses_mean(rightarm_pred_traj_means_file)
    rightarm_pred_traj_means = add_offset_human_traj(
        rightarm_pred_traj_means, 4, traj_num)

    rightarm_pred_traj_var_file = path.join(
        CSV_FOLDER, "Predictions", "varPredSampledtraj_%d_trimmed.csv" % traj_num)
    rightarm_pred_traj_var = read_human_poses_var(rightarm_pred_traj_var_file)

    complete_pred_traj_means, complete_pred_traj_vars = create_human_means_vars(
        rightarm_pred_traj_means, rightarm_pred_traj_var)

    return full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars


def add_offset_human_traj(human_traj, num_joints, traj_num):
    """
    Adds right shoulder offset to a trajectory array.

    human_traj: the 1-D trajectory array to be added to
    num_joints: number of joints in the human trajectory
    traj_num: the number of the trajectory
    """
    num_timesteps = len(human_traj) / (num_joints * 3)
    for i in range(num_timesteps):
        for j in range(num_joints):
            human_traj[i * num_joints + j * 3 + 0] = \
                human_traj[(i*num_joints + j) * 3 + 0] + \
                const.RIGHT_SHOULDER_OFFSETS[traj_num][0]
            human_traj[i * num_joints + j * 3 + 1] = \
                human_traj[(i*num_joints + j) * 3 + 1] + \
                const.RIGHT_SHOULDER_OFFSETS[traj_num][1]
            human_traj[i * num_joints + j * 3 + 2] = \
                human_traj[(i*num_joints + j) * 3 + 2] + \
                const.RIGHT_SHOULDER_OFFSETS[traj_num][2]
    return human_traj

def visualize_traj(follow_joint_trajectory_client,
                   traj_nums=[520, 524, 531, 544, 565, 604,
                              621, 640, 669, 674, 676, 681, 703],
                   show_pred=False):
    """
    Visualizes a set of trajectories, with the option to see them as recorded or the
    observed and predicted trajectories

    follow_joint_trajectory_client: a FollowTrajectoryClient (from the arm_control module)
    traj_nums: the numbers of the trajectories to display
    show_pred: whether to show observed/predicted or actual
    """
    i = 0
    while i < len(traj_nums):
        num = traj_nums[i]
        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = load_all_human_trajectories(
            num)

        raw_input("Ready to visualize trajectory number " + str(num) +
                  " with " + str(len(full_rightarm_test_traj)/12) + " timesteps")

        if show_pred:
            # Show the observed, then the predicted behavior
            obs_complete_test_traj_tree = create_human_trajectory_tree(
                obs_rightarm_test_traj)
            rightarm_pred_traj_mean = []

            for j in range(len(complete_pred_traj_means) / 33):
                rightarm_pred_traj_mean = rightarm_pred_traj_mean + \
                    complete_pred_traj_means[j * 33: j * 33 + 12]

            complete_pred_traj_mean_tree = create_human_trajectory_tree(
                rightarm_pred_traj_mean)

            follow_joint_trajectory_client.visualize_human_trajectory(
                0.01, len(obs_rightarm_test_traj)/12, obs_complete_test_traj_tree)
            follow_joint_trajectory_client.visualize_human_trajectory(
                0.01, len(complete_pred_traj_means)/33, complete_pred_traj_mean_tree)
        else:
            # Show the recorded behavior
            full_complete_test_traj_tree = create_human_trajectory_tree(
                full_rightarm_test_traj)
            follow_joint_trajectory_client.visualize_human_trajectory(
                0.01, len(full_rightarm_test_traj)/12, full_complete_test_traj_tree)

        in_key = raw_input("Press r to repeat traj num " +
                           str(num) + ", enter to continue\n")

        if in_key != 'r':
            i += 1
