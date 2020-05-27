import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import glob
import csv

#mean output will depend on mode
#Mode 1 - mean is a 2d matrix of size timesteps * (joints*3)
#Mode 2 - mean is an array of length timesteps * joints * 3
def read_human_poses_mean(csv_path, mode=2):
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

def plot_trajectory(pos_name, file_name, human_pos = None, human_dof = None):
    fig = plt.figure()
    ax = Axes3D(fig)
    human_pos = np.array(human_pos)
    ax.scatter(human_pos[0::3], human_pos[1::3], zs=human_pos[2::3], color="blue", label="Human Position")
    ax.plot(human_pos[0:human_dof*3:3], human_pos[1:human_dof*3:3], zs=human_pos[2:human_dof*3 + 1:3], color="magenta", label="Human Start Position")
    ax.plot(human_pos[len(human_pos) - human_dof*3::3], human_pos[len(human_pos) - human_dof*3 + 1::3], zs=human_pos[len(human_pos) - human_dof*3 + 2::3], color="pink", label="Human Stop Position")
    # ax.scatter(human_pos[0::12], human_pos[1::12], zs=human_pos[2::12], color="blue", label="Human Position")
    # ax.scatter([0], [0], zs=[0], color="blue", label="Human Position")
    # ax.scatter(human_pos[3::12] - human_pos[0::12], human_pos[4::12] - human_pos[1::12], zs=human_pos[5::12] - human_pos[2::12], color="blue", label="Human Position")
    # ax.scatter(human_pos[6::12] - human_pos[0::12], human_pos[7::12] - human_pos[1::12], zs=human_pos[8::12] - human_pos[2::12], color="blue", label="Human Position")
    # ax.scatter(human_pos[9::12] - human_pos[0::12], human_pos[10::12] - human_pos[1::12], zs=human_pos[11::12] - human_pos[2::12], color="blue", label="Human Position")
    ax.set_title(pos_name)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
    # plt.savefig(file_name, bbox_inches="tight")
    plt.show()



def plot_pred_obs_trajectory(pos_name, file_name, human_obs_pos = None, human_dof = None, human_pred_pos = None, trim_point = None):
    fig = plt.figure()
    ax = Axes3D(fig)
    human_obs_pos = np.array(human_obs_pos)
    human_pred_pos = np.array(human_pred_pos)
    
    ax.scatter(human_obs_pos[0::3], human_obs_pos[1::3], zs=human_obs_pos[2::3], color="blue", label="Observation Position")
    ax.plot(human_obs_pos[0:human_dof*3:3], human_obs_pos[1:human_dof*3:3], zs=human_obs_pos[2:human_dof*3 + 1:3], color="magenta", label="Observation Start Position")
    ax.plot(human_obs_pos[len(human_obs_pos) - human_dof*3::3], human_obs_pos[len(human_obs_pos) - human_dof*3 + 1::3], zs=human_obs_pos[len(human_obs_pos) - human_dof*3 + 2::3], color="pink", label="Observation Stop Position")
    
    ax.scatter(human_pred_pos[0::3], human_pred_pos[1::3], zs=human_pred_pos[2::3], color="red", label="Prediction Position")
    ax.plot(human_pred_pos[0:human_dof*3:3], human_pred_pos[1:human_dof*3:3], zs=human_pred_pos[2:human_dof*3 + 1:3], color="orange", label="Prediction Start Position")
    ax.plot(human_pred_pos[len(human_pred_pos) - human_dof*3::3], human_pred_pos[len(human_pred_pos) - human_dof*3 + 1::3], zs=human_pred_pos[len(human_pred_pos) - human_dof*3 + 2::3], color="green", label="Prediction Stop Position")
    
    ax.set_title(pos_name)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
    plt.savefig(file_name, bbox_inches="tight")
    plt.show()


obs_pose_mean = "../human_prob_models/scripts/csvFiles/Test/traj_703.csv"
human_obs_pos = read_human_poses_mean(obs_pose_mean)
pred_pose_mean = "../human_prob_models/scripts/csvFiles/Predictions/predtraj_703_trimmed.csv"
human_pred_pos = read_human_poses_mean(pred_pose_mean)

plot_pred_obs_trajectory("HumanPose", "plots/HumanPoseViz/traj_703_combined.png", human_obs_pos, 4, human_pred_pos)


# obs_pose_mean = "../human_prob_models/scripts/csvFiles/Test/traj_144.csv"
# human_obs_pos = read_human_poses_mean(obs_pose_mean)
# plot_trajectory("144", None, human_obs_pos, 4)


# pose_mean_folder = "../human_prob_models/scripts/csvFiles/otherTraj/*.csv"
# trajectories = glob.glob(pose_mean_folder)

# baseDir = "../human_prob_models/scripts/"
# trajectories = [baseDir + "csvFiles/otherTraj/traj_99.csv", baseDir + "csvFiles/otherTraj/traj_95.csv",baseDir + "csvFiles/otherTraj/traj_90.csv",baseDir + "csvFiles/otherTraj/traj_85.csv",baseDir + "csvFiles/otherTraj/traj_80.csv",baseDir + "csvFiles/otherTraj/traj_87.csv",baseDir + "csvFiles/otherTraj/traj_82.csv",baseDir + "csvFiles/otherTraj/traj_75.csv",baseDir + "csvFiles/otherTraj/traj_9.csv",baseDir + "csvFiles/otherTraj/traj_8.csv",baseDir + "csvFiles/otherTraj/traj_7.csv",baseDir + "csvFiles/otherTraj/traj_77.csv"]
# for trajectory in trajectories:
#     human_poses_mean = read_human_poses_mean(trajectory)
#     plot_trajectory(trajectory.split("/")[-1], "plots/HumanPoseViz/"+ trajectory.split("/")[-1] +".png", human_poses_mean, 4)