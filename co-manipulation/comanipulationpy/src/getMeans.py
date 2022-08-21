import sys
import const
import glob
import csv
import os.path as path

import numpy as np
import math
import traj_utils
from traj_utils import create_human_means_vars, create_human_trajectory_tree
from traj_file_utils import read_human_poses_mean, read_human_poses_var, add_offset_human_traj

CSV_FOLDER = path.join(const.PARENT_DIR, "human_prob_models/scripts/csvFiles")
TRAIN_FOLDER = path.join(CSV_FOLDER, "Train")
TEST_FOLDER = path.join(CSV_FOLDER, "Test")

def write_to_csv(arr, filename):
    if filename.startswith("means"):
        arr.insert(0, ["right_shoulderx","right_shouldery","right_shoulderz","right_elbowx","right_elbowy","right_elbowz","right_wristx","right_wristy","right_wristz","right_palmx","right_palmy","right_palmz","neckx","necky","neckz","headx","heady","headz","torsox","torsoy","torsoz","left_shoulderx","left_shouldery","left_shoulderz","left_elbowx","left_elbowy","left_elbowz","left_wristx","left_wristy","left_wristz","left_palmx","left_palmy","left_palmz"])
    with open(filename, 'w') as csvfile: 
        # creating a csv writer object 
        csvwriter = csv.writer(csvfile) 
            
        # writing the data rows 
        csvwriter.writerows(arr)

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
        full_rightarm_test_traj, rightarm_pred_traj_var)

    complete_pred_traj_obs_means_expanded, complete_pred_traj_obs_vars_expanded = traj_utils.expand_human_pred(
            complete_pred_traj_means, complete_pred_traj_vars)
    
    deconstructed_means = []
    deconstructed_vars = []
    last_i = 0
    for i in range(len(complete_pred_traj_obs_means_expanded)):
        if i > 0 and i%33==0:
            deconstructed_means.append(complete_pred_traj_obs_means_expanded[last_i:i])
            last_i = i
    last_i = 0
    for i in range(len(complete_pred_traj_obs_vars_expanded)):
        if i > 0 and i%99==0:
            deconstructed_vars.append(complete_pred_traj_obs_vars_expanded[last_i:i])
            last_i = i
    
    new_deconstructed_means = []
    new_deconstructed_vars = []
    for i in range(len(deconstructed_means)):
        if i%10 == 0:
            new_deconstructed_means.append(deconstructed_means[i])
    
    write_to_csv(new_deconstructed_means, "means_" + str(traj_num) + "_52.csv")
    write_to_csv(deconstructed_vars, "vars_" + str(traj_num) + "_52.csv")

    print("Writing Done")
load_all_human_trajectories(int(sys.argv[1]))