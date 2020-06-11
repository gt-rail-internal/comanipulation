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
import csv
import glob


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import trajoptpy.kin_utils as ku

from comanipulationpy import *
# from quantitative_tests import *
from plots import *
import sys

from scipy.interpolate import CubicSpline

class TestingFramework:
    def __init__(self, use_jaco, use_franka, use_ros, plot):

        self.use_jaco = use_jaco
        self.use_franka = use_franka
        self.use_ros = use_ros
        self.plot = plot



        ###################################
        ### Robot and Envrionment Setup ###
        ###################################
        
        self.env = openravepy.Environment()
        self.env.StopSimulation()
        if use_jaco:
            self.env.Load("../data/jaco-test.dae")
            self.manipulator_name = 'test_arm'
            self.eef_link_name = "j2s7s300_ee_link"
            self.all_links = ["j2s7s300_ee_link", "j2s7s300_link_6", "j2s7s300_link_4", "j2s7s300_link_7", "j2s7s300_link_5", "j2s7s300_link_3", "j2s7s300_link_2"]
        elif use_franka:
            self.env.Load("../data/panda_default.dae")
            self.manipulator_name = 'panda_arm'
            self.eef_link_name = "panda_hand"
            self.all_links = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7"]
        else:
            self.env.Load("../data/iiwa_env.dae")
            self.manipulator_name = 'iiwa_arm'
            self.eef_link_name = "iiwa_link_ee"
            self.all_links = ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"]

        # self.env.SetDefaultViewer()

        # trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
        self.robot = self.env.GetRobots()[0]
        self.manipulator = self.robot.GetManipulator(self.manipulator_name)
        self.eef_link = self.robot.GetLink(self.eef_link_name)


        self.body = openravepy.RaveCreateKinBody(self.env, '')
        self.body.SetName("Collision_Scene")

        if use_ros:
            from arm_control import *
            import actionlib
            import rospy

            from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            rospy.init_node("comanipulation_testing")

            if use_jaco:
                self.follow_joint_trajectory_client = FollowTrajectoryClient("jaco_trajectory_controller", ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"])
            elif use_franka:
                self.follow_joint_trajectory_client = FollowTrajectoryClient("panda_arm_controller", ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"])
            else:
                self.follow_joint_trajectory_client = FollowTrajectoryClient("iiwa/PositionJointInterface_trajectory_controller", ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"])


        ###################################
        ### Variables                   ###
        ###################################


        self.base_link_world_offset = [0.5, -0.2, -1.0]
        self.right_shoulder_base_link_offset = [-2.0, -0.5, 0]



        self.right_shoulder_offsets =  {120 : [-1.5, -0.4, 0.1], 
                                        124 : [-1.5, -0.5, 0.1],
                                        131 : [-1.9, -0.5, 0.1],
                                        144 : [-1.5, -0.5, 0],
                                        165 : [-1.9, -0.5, 0],
                                        204 : [-1.5, -0.4, 0],#
                                        221 : [-1.9, -0.5, 0],#
                                        240 : [-1.5, -0.5, 0],#
                                        269 : [-1.9, -0.5, 0],#
                                        274 : [-1.9, -0.5, 0],#
                                        276 : [-1.9, -0.5, 0],#
                                        281 : [-1.9, -0.5, 0],#
                                        303 : [-1.9, -0.5, 0],#
                                        520 : [-1.5, -0.4, 0.1], 
                                        524 : [-1.5, -0.5, 0.1],
                                        531 : [-1.9, -0.5, 0.1],
                                        544 : [-1.5, -0.5, 0],
                                        565 : [-1.9, -0.5, 0],
                                        604 : [-1.5, -0.4, 0],#
                                        621 : [-1.9, -0.5, 0],#
                                        640 : [-1.5, -0.5, 0],#
                                        669 : [-1.9, -0.5, 0],#
                                        674 : [-1.9, -0.5, 0],#
                                        676 : [-1.9, -0.5, 0],#
                                        681 : [-1.9, -0.5, 0],#
                                        703 : [-1.9, -0.5, 0],#
                                        }
        


        ###################################
        ### IK (not working...)         ###
        ###################################


        # quat_target = [1,0,0,0] # wxyz
        # xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
        # xyz_target = self.get_eef_position(joint_target)
        # hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

        # print("Starting IK")
        # # BEGIN ik
        # self.manip = self.robot.GetManipulator("test_arm")
        # self.robot.SetActiveManipulator(self.manip)
        # ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        #     self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        # if not ikmodel.load():
        #     ikmodel.autogenerate()
        # init_joint_target = ku.ik_for_link(hmat_target, self.manip, "j2s7s300_ee_link",
        #     filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
        # # END ik
        # print("Finished IK")

        # print(init_joint_target)
        # joint_target = init_joint_target.tolist()

        #############################################


        # baseDir = "../human_prob_models/scripts/"
        # trajectories = [baseDir + "csvFiles/otherTraj/traj_99.csv", baseDir + "csvFiles/otherTraj/traj_95.csv",baseDir + "csvFiles/otherTraj/traj_90.csv",baseDir + "csvFiles/otherTraj/traj_85.csv",baseDir + "csvFiles/otherTraj/traj_80.csv",baseDir + "csvFiles/otherTraj/traj_87.csv",baseDir + "csvFiles/otherTraj/traj_82.csv",baseDir + "csvFiles/otherTraj/traj_75.csv",baseDir + "csvFiles/otherTraj/traj_9.csv",baseDir + "csvFiles/otherTraj/traj_8.csv",baseDir + "csvFiles/otherTraj/traj_7.csv",baseDir + "csvFiles/otherTraj/traj_77.csv"]
        # for trajectory in trajectories:
        #     means = self.read_human_poses_mean(trajectory, 2)
        #     self.crop_human_pose(means, 100, 4, "../human_prob_models/scripts/csvFiles/otherTraj/"+trajectory.split("/")[-1].split(".")[0]+"_trimmed.csv")
        ###UNCOMMENT EVERYTHING BELOW LATER###


        #############################################
        ####     Straight Line Trajectory        ####
        #############################################

        # print("Calculating straight trajectory...")
        # straight_traj = np.zeros((10, 7))
        # for i in range(10):
        #     for j in range(7):
        #         straight_traj[i, j] = joint_start[j] + ((joint_target[j] - joint_start[j]) * i / 9)
        # print(straight_traj)
        # print("Following straight trajectory")
        # straight_eef_traj = self.follow_trajectory(straight_traj)
        # np.savetxt('trajectories/straight.txt', straight_eef_traj, delimiter=',')

        ################################################

    

    ####################################################
    ######  Helper Functions
    ####################################################

    def create_still_trajectories(self, traj_folder, traj_folder_path):
        for traj in traj_folder:
            file_name = traj.split("/")[-1].split(".")[0]
            file_number = int(file_name.split("_")[1])
            if file_number < 400:
                new_file_name = file_name.split("_")[0]+"_"+str(file_number+400)
                last_row = ""
                with open(traj_folder_path+file_name+".csv", 'r') as f:
                    for row in reversed(list(csv.reader(f))):
                        last_row = row
                        break
                timesteps = 250
                print("Creating ", new_file_name+".csv")
                spamWriter = csv.writer(open(traj_folder_path+new_file_name+".csv", 'w'), delimiter=',', quotechar='|')
                for timesteps in range(timesteps):
                    spamWriter.writerow(last_row)

    def still_trajectories_wrapper(self):
        train_folder = "../human_prob_models/scripts/csvFiles/Train/"
        test_folder = "../human_prob_models/scripts/csvFiles/Test/"
        train_traj = glob.glob(train_folder+"*.csv") 
        test_traj = glob.glob(test_folder+"*.csv")
        self.create_still_trajectories(train_traj, train_folder)
        self.create_still_trajectories(test_traj, test_folder)        


    def prep_data_for_prediction(self):
        pose_mean_folder = "../human_prob_models/scripts/csvFiles/Test/*.csv"
        trajectories = glob.glob(pose_mean_folder)
        for trajectory in trajectories:
            means = self.read_human_poses_mean(trajectory, 2)
            file_name = trajectory.split("/")[-1].split(".")[0]
            if not "trimmed"  in file_name and not "remainder"  in file_name:
                print(file_name)
                self.crop_human_pose(means, 100, 4, "../human_prob_models/scripts/csvFiles/Test/"+file_name+"_trimmed.csv", "../human_prob_models/scripts/csvFiles/Test/"+file_name+"_remainder.csv")



    def calculate_distance_3d(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)


    def get_visibility_angle(self, head_pos, robot_joints, object_pos):

        eef_pos = self.get_eef_position(robot_joints)
        # print(robot_joints)
        # self.execute_trajectory([robot_joints])
        # print(eef_pos)
        # print(head_pos)
        # print(object_pos)
        
        vec_head_eef = np.array(head_pos) - np.array(eef_pos)
        vec_head_obj = np.array(head_pos) - np.array(object_pos)

        # print(vec_head_eef)
        # print(vec_head_obj)
        # print(vec_head_obj.dot(vec_head_eef) / (np.linalg.norm(vec_head_obj) * np.linalg.norm(vec_head_eef)))
        # print(math.acos(vec_head_obj.dot(vec_head_eef) / (np.linalg.norm(vec_head_obj) * np.linalg.norm(vec_head_eef))))



        # import tf
        # br = tf.TransformBroadcaster()
        # br.sendTransform((head_pos[0], head_pos[1], head_pos[2]), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "head", "world")
        # br.sendTransform((eef_pos[0], eef_pos[1], eef_pos[2]), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "eef", "world")
        # br.sendTransform((object_pos[0], object_pos[1], object_pos[2]), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "object", "world")

        # raw_input()

        return math.acos(vec_head_obj.dot(vec_head_eef) / (np.linalg.norm(vec_head_obj) * np.linalg.norm(vec_head_eef)))


    
    def get_separation_dist(self, human_pos, robot_joints):
        '''
        human_pos: human position from vision system - one timestep, vectorized
        the order of human_pos joints is right_shoulder + right_elbow + right_wrist + right_palm + neck + head + torso + left_shoulder + left_elbow + left_wrist + left_palm
        robot_joint_pos: time-sampled JOINT space trajectory  - one timestep - vectorized
        '''
        human_links = np.array([[0,1],[1,2],[2,3],[0,4],[4,5],[4,6],[4,7],[7,8],[8,9],[9,10]])
        robot_links = np.array([[0,1],[1,2],[2,3], [3,4], [4,5], [5,6]])
        human_pos = np.array(human_pos)
        robot_joints_pos = self.performFK(robot_joints)
        robot_joints_pos = np.array(robot_joints_pos)
        robot_joints_pos = np.reshape(robot_joints_pos, 7*3)
        human_sphere_radius = 0.05
        human_sphere_num = 5
        robot_sphere_radius = 0.05
        robot_sphere_num = 5
        distance = float('inf')

        # plot_spheres(human_pos, robot_joints_pos)
        
        for curr_human_link in human_links:
            human_link_length = human_pos[3*curr_human_link[1]:3*(1+curr_human_link[1])] - human_pos[3*curr_human_link[0]:3*(1+curr_human_link[0])]
            for curr_robot_link in robot_links:
                robot_link_length = robot_joints_pos[3*curr_robot_link[1]:3*(1+curr_robot_link[1])] - robot_joints_pos[3*curr_robot_link[0]:3*(1+curr_robot_link[0])]

                human_spheres_centers = [human_pos[3*curr_human_link[0]:3*(1+curr_human_link[0])] + i*human_link_length/human_sphere_num for i in range(human_sphere_num)]
                robot_spheres_centers = [robot_joints_pos[3*curr_robot_link[0]:3*(1+curr_robot_link[0])] + i*robot_link_length/robot_sphere_num for i in range(robot_sphere_num)]
                
                # plot_spheres(human_pos, robot_joints_pos, human_spheres_centers, robot_spheres_centers)
                
                for human_sphere_center in human_spheres_centers:
                    for robot_sphere_center in robot_spheres_centers:
                        center_dist = self.calculate_distance_3d(human_sphere_center, robot_sphere_center)
                        curr_distance = center_dist - human_sphere_radius - robot_sphere_radius
                        distance = min(distance, curr_distance)

        # print("Separation Distance at this time = ", distance)
        return distance

    def cubic_interpolation(self, robot_joints, robot_num_joints):
        '''
        robot_joints = time-sampled JOINT space trajectory (vectorized - timesteps*7 matrix)
        '''
        robot_joints = np.array(robot_joints)
        robot_joints_pos = np.reshape(robot_joints, (-1, robot_num_joints))
        x = [i for i in range(0, len(robot_joints_pos))]
        y = robot_joints_pos
        return CubicSpline(x,y)


    def add_collision_objects(self):
        if not self.use_ros:
            return False
        
        from octomap_trajopt_bridge.msg import octomap_custom

        octomap_message = rospy.wait_for_message("/octomap_decrypted", octomap_custom)

        octomap_array = np.zeros((octomap_message.num_vox, 6))
        for i in range(octomap_message.num_vox):
            octomap_array[i] = [octomap_message.x[i], octomap_message.y[i], octomap_message.z[i], 0.025, 0.025, 0.025]

        self.env.Remove(self.body)
        
        self.body.InitFromBoxes(octomap_array)

        self.env.Add(self.body)
    
    def get_subsampled_human_from_dict(self, human_traj):
        order = ['right_shoulder', 'right_elbow', 'right_wrist', 'right_palm', 'neck', 'head', 'torso', 'left_shoulder', 'left_elbow', 'left_wrist', 'left_palm']

        final_trajectory = np.zeros([len(human_traj[order[0]][::10]), len(order)*3])
        for index, joint in enumerate(order):
            final_trajectory[:, 3*index:3*(index + 1)] = human_traj[joint][::10] ##each timestep needs to be vectorized
        
        return final_trajectory



    ####################################################
    ######  Robot Functions
    ####################################################

    # Takes joint values (dof_vales) and returns end effector 3D Pose in world frame
    def get_eef_position(self, dof_vals):
        
        self.robot.SetDOFValues(dof_vals, self.manipulator.GetArmIndices())
        posevec = openravepy.poseFromMatrix(self.eef_link.GetTransform())
        position = posevec[4:7]
        pos = np.array([position[0], position[1], position[2]])
        # raw_input(pos)
        return pos

    
    def follow_trajectory(self, traj):
        num_timesteps = traj.shape[0]
        eef_traj = np.zeros((num_timesteps, 3))
        for i in range(num_timesteps):
            p_eef_t = self.get_eef_position(traj[i])
            for j in range(3):
                eef_traj[i, j] = p_eef_t[j]
            # raw_input(eef_traj[i])
        return eef_traj

    def execute_trajectory(self, traj):
        # if self.use_ros and not self.use_jaco:
        print("Executing trajectory!")
        raw_input("Ready to move to initial position")
        # self.follow_joint_trajectory_client.move_to(traj[0], duration=1)
        self.follow_joint_trajectory_client.follow_trajectory([traj[0], traj[0]], duration=0.5)
        raw_input("Ready for Gazebo execution?")
        self.follow_joint_trajectory_client.follow_trajectory(traj, duration=0.5)

    def execute_full_trajectory(self, traj, human_traj, obs_traj_len, full_human_traj_len):
        # if self.use_ros and not self.use_jaco:
        self.follow_joint_trajectory_client.follow_trajectory([traj[0], traj[0]], duration=0.5)
        full_human_traj = create_human_trajectory_tree(human_traj)
        sub_human_traj = self.get_subsampled_human_from_dict(full_human_traj)

        execution_traj = []
        collision_threshold = 0.25

        print("Separation distances for real trajectory")
        last_pos = None
        for i in range(max(len(traj), len(sub_human_traj))):
            curr_distance = self.get_separation_dist(sub_human_traj[min(i, len(sub_human_traj) - 1)], traj[min(i, len(traj) - 1)])
            print("Curr Distance = " + str(curr_distance))
            if curr_distance > collision_threshold and last_pos is None:
                execution_traj.append(traj[i])
            else:                
                last_pos = i if last_pos is None else last_pos
                execution_traj.append(traj[last_pos])

        raw_input("Ready for Gazebo execution")
        self.follow_joint_trajectory_client.execute_full_trajectory(execution_traj, 0.1, 0.01, obs_traj_len, full_human_traj_len - obs_traj_len, len(execution_traj), full_human_traj)


    #use openrave's FK module to get robot_joints in cartesian space rather than joint space
    def performFK(self, robot_joints):
        self.robot.SetDOFValues(robot_joints, self.manipulator.GetArmIndices())

        robot_joint_cart_pos = np.zeros((len(self.all_links), 3))
        for i in range(len(self.all_links)):
            link = self.all_links[i]
            posevec = openravepy.poseFromMatrix(self.robot.GetLink(link).GetTransform())
            position = posevec[4:7]
            robot_joint_cart_pos[i] = np.array([position[0], position[1], position[2]])
        return robot_joint_cart_pos

    ####################################################
    ######  Human Trajectory Functions
    ####################################################


    def crop_human_pose(self, human_pose_mean, timesteps, dof, csv_save_path, csv_remainder_path):
        spamWriter = csv.writer(open(csv_save_path, 'w'), delimiter=',', quotechar='|')
        spamWriter2 = csv.writer(open(csv_remainder_path, 'w'), delimiter=',', quotechar='|')
        for timestep in range(timesteps):
            spamWriter.writerow([str(human_pose_mean[timestep*dof*3 + i]) for i in range(dof*3)])

        for timestep in range(timesteps, len(human_pose_mean)/(dof*3)):
            spamWriter2.writerow([str(human_pose_mean[timestep*dof*3 + i]) for i in range(dof*3)])

    
    #Takes in mean_pos which is a 2D matrix (Mode 1) and var_pos which is a 3D matrix (Mode 1)    
    def sample_human_trajectory(self, mean_pos, var_pos):
        sample = []
        for i in range(0, len(mean_pos)):
            sample = sample + list(np.random.multivariate_normal(mean_pos[i], var_pos[i]))
        return sample

    #mean output will depend on mode
    #Mode 1 - mean is a 2d matrix of size timesteps * (joints*3)
    #Mode 2 - mean is an array of length timesteps * joints * 3
    def read_human_poses_mean(self, csv_path, mode=2):

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


    #var output will depend on the mode
    #Mode 1 - returns a 3D matrix of size timesteps * (joints * 3) * (joints * 3) >> all covariances
    #Mode 2 - returns an array of length timesteps * joints * 9 >> only the variances along the 3x3 block matricies along the diagonal
    #Mode 3 - returns an array of length timesteps * joints * 3 >> only the variances along the covariance matrix diagonal 
    def read_human_poses_var(self, csv_path, mode=2):
        with open(csv_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            var = []
            for row in csv_reader:
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
                            if j < 3*(i/3):
                                continue
                            elif j >= (3*(i/3) + 3):
                                continue
                            else:
                                # print("coord = ", i, j)
                                var.append(float(row[i*numCol + j]))
                elif mode == 3:
                    for i in range(0, numCol):
                        for j in range(0, numCol):
                            if i==j:
                                # print("coord = ", i, j)
                                var.append(float(row[i*numCol + j]))
        return var

    def load_all_human_trajectories(self, traj_num):
        base_dir = "../human_prob_models/scripts/csvFiles/"

        full_rightarm_test_traj_file = base_dir + "Test/traj_" + str(traj_num) + ".csv"
        full_rightarm_test_traj = self.read_human_poses_mean(full_rightarm_test_traj_file)
        full_rightarm_test_traj = self.add_offset_human_traj(full_rightarm_test_traj, 4, traj_num)

        obs_rightarm_test_traj_file = base_dir + "Test/traj_" + str(traj_num) + "_trimmed.csv"
        obs_rightarm_test_traj = self.read_human_poses_mean(obs_rightarm_test_traj_file)
        obs_rightarm_test_traj = self.add_offset_human_traj(obs_rightarm_test_traj, 4, traj_num)

        rightarm_pred_traj_means_file = base_dir + "Predictions/predSampledtraj_" + str(traj_num) + "_trimmed.csv"
        rightarm_pred_traj_means = self.read_human_poses_mean(rightarm_pred_traj_means_file)
        rightarm_pred_traj_means = self.add_offset_human_traj(rightarm_pred_traj_means, 4, traj_num)

        rightarm_pred_traj_var_file = base_dir + "Predictions/varPredSampledtraj_" + str(traj_num) + "_trimmed.csv"
        rightarm_pred_traj_var = self.read_human_poses_var(rightarm_pred_traj_var_file)

        complete_pred_traj_means, complete_pred_traj_vars = create_human_means_vars(rightarm_pred_traj_means, rightarm_pred_traj_var)

        return full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars

    
    def add_offset_human_traj(self, human_traj, num_joints, traj_num):
        num_timesteps = len(human_traj) / (num_joints * 3)
        for i in range(num_timesteps):
            for j in range(num_joints):
                human_traj[i * num_joints * 3 + j * 3 + 0] = human_traj[i * num_joints * 3 + j * 3 + 0] + self.right_shoulder_offsets[traj_num][0]
                human_traj[i * num_joints * 3 + j * 3 + 1] = human_traj[i * num_joints * 3 + j * 3 + 1] + self.right_shoulder_offsets[traj_num][1]
                human_traj[i * num_joints * 3 + j * 3 + 2] = human_traj[i * num_joints * 3 + j * 3 + 2] + self.right_shoulder_offsets[traj_num][2]
        return human_traj


    def visualize_all_traj(self):
        # traj_nums = [120, 124, 131, 144, 165, 204, 221, 240, 269, 274, 276, 281, 303]
        traj_nums = [520, 524, 531, 544, 565, 604, 621, 640, 669, 674, 676, 681, 703]
        i = 0
        while i < len(traj_nums):
            num = traj_nums[i]
            full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(num)
            full_complete_test_traj_tree = create_human_trajectory_tree(full_rightarm_test_traj)
            raw_input("Ready to visualize trajectory number " + str(num) + " with " + str(len(full_rightarm_test_traj)/12) + " timesteps")
                
            self.follow_joint_trajectory_client.visualize_human_trajectory(0.01, len(full_rightarm_test_traj)/12, full_complete_test_traj_tree)

            in_key = raw_input("Press r to repeat traj num " + str(num) + ", enter to continue\n")

            if in_key != 'r':
                i += 1

    def visualize_traj_pred(self):
        # traj_nums = [120, 124, 131, 144, 165, 204, 221, 240, 269, 274, 276, 281, 303]
        traj_nums = [520, 524, 531, 544, 565, 604, 621, 640, 669, 674, 676, 681, 703]
        i = 0
        while i < len(traj_nums):
            num = traj_nums[i]
            full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(num)
            obs_complete_test_traj_tree = create_human_trajectory_tree(obs_rightarm_test_traj)

            rightarm_pred_traj_mean = []
            for j in range(len(complete_pred_traj_means) / 33):
                rightarm_pred_traj_mean = rightarm_pred_traj_mean + complete_pred_traj_means[j * 33 : j * 33 + 12]
            complete_pred_traj_mean_tree = create_human_trajectory_tree(rightarm_pred_traj_mean)


            raw_input("Ready to visualize trajectory number " + str(num) + " with " + str(len(full_rightarm_test_traj)/12) + " timesteps")
                
            self.follow_joint_trajectory_client.visualize_human_trajectory(0.01, len(obs_rightarm_test_traj)/12, obs_complete_test_traj_tree)
            self.follow_joint_trajectory_client.visualize_human_trajectory(0.01, len(complete_pred_traj_means)/33, complete_pred_traj_mean_tree)

            in_key = raw_input("Press r to repeat traj num " + str(num) + ", enter to continue\n")

            if in_key != 'r':
                i += 1

    def get_human_obs_and_prediction(self):
        if not self.use_ros:
            return False
        
        import rospy
        from std_msgs.msg import String
        import matlab.engine

        pred_msg_data = rospy.wait_for_message("/human_traj_pred", String)


        expData, expSigma = pred_msg_data.data.split("splitTag")
        expData, expSigma = matlab.double(json.loads(str(expData))[:100]), matlab.double(json.loads(str(expSigma))[:100])
        predictedMeans, variances = [],[]

        for row in expData:
            for col in row:
                predictedMeans.append(col)
        
        for timestep in range(len(expSigma)):
            for row in range(len(expSigma[timestep])):
                colStart = row%3 * 3
                for col in range(len(expSigma[timestep][row])):
                    if col >= colStart and col < (colStart + 3):
                        variances.append(expSigma[timestep][row][col])
        return predictedMeans, variances

        complete_pred_traj_means, complete_pred_traj_vars = create_human_means_vars(predictedMeans, variances)

        return complete_pred_traj_means, complete_pred_traj_vars




    ####################################################
    ######  Trajopt Functions
    ####################################################

    def optimize_problem(self, request):
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, self.env) # Create object that stores optimization problem
        t_start = time.time()
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        t_elapsed = time.time() - t_start
        print("optimization took %.3f seconds"%t_elapsed)

        # from trajoptpy.check_traj import traj_is_safe
        # prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        # assert traj_is_safe(result.GetTraj(), self.robot) # Check that trajectory is collision free
        return result

    def get_default_trajectory(self, init_joint, final_joint, num_timesteps):
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)

        add_joint_vel_cost(request, 1)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))

        return result.GetTraj(), eef_traj




    ####################################################
    ######  Testing Functions
    ####################################################


    def distance_test(self, init_joint, final_joint):
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = expand_human_pred(complete_pred_traj_means, complete_pred_traj_vars)

        n_human_joints = 11
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        print("Full trajectory timesteps:", len(full_rightarm_test_traj)/12)
        print("Observed trajectory timesteps:", len(obs_rightarm_test_traj)/12)
        print("Num timesteps:", num_timesteps)

        ref_joint_traj, ref_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())


        coeffs = []
        reg_coeffs = []
        for i in range(num_timesteps):
            coeffs.append(1000.0)

        for i in range(num_timesteps - 1):
            reg_coeffs.append(10.0)

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        add_distance_cost(request, complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded, coeffs, n_human_joints, self.all_links)
        add_collision_cost(request, [20], [0.025])
        add_optimal_trajectory_cost(request, ref_eef_traj, self.eef_link_name, num_timesteps, 0.5)
        # add_regularize_cost(request, reg_coeffs, self.eef_link_name)
        add_smoothing_cost(request, 10, 2)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))
        self.execute_full_trajectory(result.GetTraj(), full_rightarm_test_traj, len(obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj) / 12)

        full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)

        if plot:
            plot_trajectory(eef_traj, "Distance", ref_eef_traj, "Joint Space Linear", "plots/distance.png", full_complete_test_traj, 11)

        np.savetxt('trajectories/distance.txt', eef_traj, delimiter=',')

        return eef_traj


    def velocity_test(self, init_joint, final_joint):
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = expand_human_pred(complete_pred_traj_means, complete_pred_traj_vars)

        n_human_joints = 11
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        print("Full trajectory timesteps:", len(full_rightarm_test_traj)/12)
        print("Observed trajectory timesteps:", len(obs_rightarm_test_traj)/12)
        print("Num timesteps:", num_timesteps)

        ref_joint_traj, ref_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
        

        coeffs = []
        reg_coeffs = []
        for i in range(num_timesteps):
            coeffs.append(50000.0)

        for i in range(num_timesteps - 1):
            reg_coeffs.append(1.0)

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        add_velocity_cost(request, complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded, coeffs, n_human_joints, self.all_links)
        # add_collision_cost(request, [20], [0.025])
        # add_optimal_trajectory_cost(request, ref_eef_traj, self.eef_link_name, num_timesteps, 0.1)
        # add_regularize_cost(request, reg_coeffs, self.eef_link_name)
        # add_smoothing_cost(request, 2, 2)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))
        self.execute_full_trajectory(result.GetTraj(), full_rightarm_test_traj, len(obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj) / 12)

        full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)

        if plot:
            plot_trajectory(eef_traj, "Velocity", ref_eef_traj, "Joint Space Linear", "plots/velocity.png", full_complete_test_traj, 11)

        np.savetxt('trajectories/velocity.txt', eef_traj, delimiter=',')

        return eef_traj


    def visibility_test(self, init_joint, final_joint):
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(303)
        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = expand_human_pred(complete_pred_traj_means, complete_pred_traj_vars)
        full_rightarm_test_traj_expanded = expand_human_test_traj(full_rightarm_test_traj, 4, len(obs_rightarm_test_traj)/12, 20)

        n_human_joints = 11
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        print("Full trajectory timesteps:", len(full_rightarm_test_traj)/12)
        print("Observed trajectory timesteps:", len(obs_rightarm_test_traj)/12)
        print("Num timesteps:", num_timesteps)

        ref_joint_traj, ref_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        head_pred_traj_mean, head_pred_traj_var = create_human_head_means_vars(complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded)
        coeffs = []
        reg_coeffs = []
        for i in range(num_timesteps):
            coeffs.append(0.01)

        for i in range(num_timesteps - 1):
            reg_coeffs.append(10.0)
        object_pos = [0, 0.2, 0.83]

        full_head_test_traj = []

        for i in range(len(full_rightarm_test_traj_expanded) / 12):
            full_head_test_traj.append(full_rightarm_test_traj_expanded[i * 12 + 0] + 0.2)
            full_head_test_traj.append(full_rightarm_test_traj_expanded[i * 12 + 1])
            full_head_test_traj.append(full_rightarm_test_traj_expanded[i * 12 + 2] + 0.15)

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        # add_visibility_cost(request, head_pred_traj_mean, head_pred_traj_var, coeffs, object_pos, self.eef_link_name)
        add_collision_cost(request, [20], [0.025])
        add_optimal_trajectory_cost(request, ref_eef_traj, self.eef_link_name, num_timesteps, 10)
        # add_regularize_cost(request, reg_coeffs, self.eef_link_name)
        add_smoothing_cost(request, 10, 2)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))
        self.execute_full_trajectory(result.GetTraj(), full_rightarm_test_traj, len(obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj) / 12)

        # if plot:
        #     plot_trajectory(eef_traj, "Visibility", ref_eef_traj, "Joint Space Linear", "plots/visibility.png", full_head_test_traj, 1, object_pos)

        # np.savetxt('trajectories/visibility.txt', eef_traj, delimiter=',')

        print(self.compute_visibility_metric(full_head_test_traj, len(obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj_expanded) / 12, result.GetTraj(), object_pos))

        return eef_traj


    def legibility_test(self, init_joint, final_joint):


        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        full_human_poses, obs_human_poses, human_poses_mean, human_poses_var = self.load_all_human_trajectories(303)
        human_poses_mean, human_poses_var = expand_human_pred(human_poses_mean, human_poses_var)

        n_human_joints = 11
        num_timesteps = len(human_poses_mean) / (n_human_joints * 3)

        print("Full trajectory timesteps:", len(full_human_poses)/12)
        print("Observed trajectory timesteps:", len(obs_human_poses)/12)
        print("Num timesteps:", num_timesteps)

        ref_joint_traj, ref_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        coeffs = 100.0
        reg_coeffs = []

        for i in range(num_timesteps - 1):
            reg_coeffs.append(1.0)

        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        add_legibility_cost(request, coeffs, self.eef_link_name)
        add_collision_cost(request, [20], [0.025])
        # add_optimal_trajectory_cost(request, ref_eef_traj, self.eef_link_name, num_timesteps, 5)
        # add_regularize_cost(request, reg_coeffs, self.eef_link_name)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))
        # self.execute_trajectory(result.GetTraj(), full_human_poses, len(obs_human_poses) / 12, len(full_human_poses) / 12)
        self.execute_trajectory(result.GetTraj())

        if plot:
            plot_trajectory(eef_traj, "Legibility", ref_eef_traj, "Joint Space Linear", "plots/legibility.png")

        np.savetxt('trajectories/legibility.txt', eef_traj, delimiter=',')
        
        return eef_traj
        

    def optimal_trajectory_test(self, init_joint, final_joint, ref_traj, num_timesteps):
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)

        xyz_target = self.get_eef_position(final_joint)
        xyz_init = self.get_eef_position(init_joint)

        add_optimal_trajectory_cost(request, ref_traj, self.eef_link_name, num_timesteps, 5)

        result = self.optimize_problem(request)
        eef_traj = self.follow_trajectory(np.array(result.GetTraj()))

        if plot:
            plot_trajectory(eef_traj, "Optimal Trajectory", ref_traj, "Joint Space Linear", "plots/optimal.png")

        np.savetxt('trajectories/optimal.txt', eef_traj, delimiter=',')

        return eef_traj
    

    def setup_test_without_ros(self, init_joint, final_joint, traj_num = 303, exec_traj = False, cal_separation_with_pred_human = False):

        # Read pose prediction files
        # full_human_poses, obs_human_poses, human_poses_mean, human_poses_var = self.load_all_human_trajectories(traj_num)
        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = self.load_all_human_trajectories(traj_num)

        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = expand_human_pred(complete_pred_traj_means, complete_pred_traj_vars)

        # Get human head poses and means from full human poses and means
        head_pred_traj_means_expanded, head_pred_traj_vars_expanded = create_human_head_means_vars(complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded)

        full_complete_test_traj = create_human_plot_traj(full_rightarm_test_traj)

        # Set object position
        object_pos = [0, 0.2, 0.83]

        # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
        n_human_joints = 11
        n_robot_joints = 7
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        # Setup coefficients
        coeff_optimal_traj = 10.0
        coeff_dist = []
        coeff_vel = []
        coeff_vis = []
        coeff_leg = 100.0
        coeffs_reg = []
        for i in range(num_timesteps):
            coeff_dist.append(10000.0)
            coeff_vel.append(100.0)
            coeff_vis.append(0.5)
        for i in range(num_timesteps - 1):
            coeffs_reg.append(5.0)

        # Generate default trajectory using trajopt
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
        default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        # print("Default Joint Space = ", default_traj)
        # print("Human Pose = ", np.array(human_poses_mean).reshape((-1, n_human_joints*3)))
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        #Test Adaptive Control Baseline
        # print("Calculating Adaptive Trajectory now!")
        # adaptive_traj = self.calculate_adaptive_trajectory(default_traj, human_poses_mean, n_human_joints, n_robot_joints)
        # print(adaptive_traj)

        # Create empty request and setup initial trajectory

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        request = set_init_traj(request, default_traj.tolist())

        add_distance_cost(request, complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded, coeff_dist, n_human_joints, self.all_links)

        add_visibility_cost(request, head_pred_traj_means_expanded, head_pred_traj_vars_expanded, coeff_vis, object_pos, self.eef_link_name)

        add_legibility_cost(request, coeff_leg, self.eef_link_name)

        add_regularize_cost(request, coeffs_reg, self.eef_link_name)
        add_optimal_trajectory_cost(request, default_eef_traj, self.eef_link_name, num_timesteps, coeff_optimal_traj)
        add_collision_cost(request, [20], [0.025])
        add_smoothing_cost(request, 200, 2)

        result = self.optimize_problem(request)
        sub_human_traj = np.array(complete_pred_traj_means_expanded).reshape(num_timesteps, -1)
        robot_trajectory = result.GetTraj()

        if (cal_separation_with_pred_human):
            print("Separation Distances for predicted human trajectory: ")
            for i in range(max(len(robot_trajectory), len(sub_human_traj))):
                curr_distance = self.get_separation_dist(sub_human_traj[min(i, len(sub_human_traj) - 1)], robot_trajectory[min(i, len(robot_trajectory) - 1)])
                print(curr_distance)        


        if (exec_traj):
            self.execute_full_trajectory(result.GetTraj(), full_rightarm_test_traj, len(obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj) / 12)

        print("Num robot timesteps: ", len(result.GetTraj()))
        print("Num full human timesteps: ", len(full_rightarm_test_traj) / 12)
        print("Num observed human timesteps: ", len(obs_rightarm_test_traj) / 12)

        # return request
        return self.evaluate_metrics(result.GetTraj(), full_complete_test_traj, len(obs_rightarm_test_traj)/ 12, object_pos, default_traj)

    
    def setup_test(self, init_joint, final_joint, exec_traj = False):

        # Get prediction from human_traj_pred stream
        complete_pred_traj_means, complete_pred_traj_vars = self.get_human_obs_and_prediction()

        # Expand trajectory in time by placing static samples
        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = expand_human_pred(complete_pred_traj_means, complete_pred_traj_vars)

        # Get human head poses and means from full human poses and means
        head_pred_traj_means_expanded, head_pred_traj_vars_expanded = create_human_head_means_vars(complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded)

        # Set object position
        object_pos = [0, 0.2, 0.83]

        # 11 joints: 4 for each arm (8), 1 for neck, 1 for head, 1 for torso
        n_human_joints = 11
        n_robot_joints = 7
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        # Setup coefficients
        coeff_optimal_traj = 10.0
        coeff_dist = []
        coeff_vel = []
        coeff_vis = []
        coeff_leg = 100.0
        coeffs_reg = []
        for i in range(num_timesteps):
            coeff_dist.append(10000.0)
            coeff_vel.append(100.0)
            coeff_vis.append(0.5)
        for i in range(num_timesteps - 1):
            coeffs_reg.append(5.0)

        # Generate default trajectory using trajopt
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
        default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
        
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        # Create empty request and setup initial trajectory

        request = create_empty_request(num_timesteps, final_joint, self.manipulator_name)
        request = set_init_traj(request, default_traj.tolist())

        # Add costs
        add_distance_cost(request, complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded, coeff_dist, n_human_joints, self.all_links)

        add_visibility_cost(request, head_pred_traj_means_expanded, head_pred_traj_vars_expanded, coeff_vis, object_pos, self.eef_link_name)

        add_legibility_cost(request, coeff_leg, self.eef_link_name)

        add_regularize_cost(request, coeffs_reg, self.eef_link_name)
        add_optimal_trajectory_cost(request, default_eef_traj, self.eef_link_name, num_timesteps, coeff_optimal_traj)
        add_collision_cost(request, [20], [0.025])
        
        # Need to add again after fixing errors
        # add_smoothing_cost(request, 200, 2)
        
        result = self.optimize_problem(request)

        self.execute_trajectory(result.GetTraj())

        return result


    ####################################################
    ######  Metric Functions
    ####################################################
    
    def compute_distance_metric(self, full_complete_test_traj_expanded, num_obs_timesteps, num_total_timesteps, robot_traj):

        n_human_joints = 11

        distance_threshold = 0.2

        robot_traj_spline = self.cubic_interpolation(robot_traj, 7)
        num_above_threshold = 0

        for t in range(num_obs_timesteps, num_total_timesteps):
            robot_timestep = (t - num_obs_timesteps) / 10.0
            robot_joints = robot_traj_spline(robot_timestep)

            dist_t = self.get_separation_dist(full_complete_test_traj_expanded[t * n_human_joints * 3 : t * n_human_joints * 3 + n_human_joints * 3], robot_joints)
            if dist_t > distance_threshold:
                num_above_threshold += 1
        
        return num_above_threshold * 1.0 / (num_total_timesteps - num_obs_timesteps)

    def compute_visibility_metric(self, full_head_test_traj_expanded, num_obs_timesteps, num_total_timesteps, robot_traj, object_pos):

        visibility_threshold = 1.4

        robot_traj_spline = self.cubic_interpolation(robot_traj, 7)
        num_below_thres = 0

        visibilities = []

        # for t in range(len(robot_traj)):
        for t in range(num_obs_timesteps, num_total_timesteps):
            robot_timestep = (t - num_obs_timesteps) / 10.0
            robot_joints = robot_traj_spline(robot_timestep)
            # robot_joints = robot_traj[t]

            # human_timestep = num_obs_timesteps + (t * 10)

            # vis_t = self.get_visibility_angle(complete_test_head_traj[human_timestep * 3: human_timestep * 3 + 3], robot_joints, object_pos)
            vis_t = self.get_visibility_angle(full_head_test_traj_expanded[t * 3: t * 3 + 3], robot_joints, object_pos)
            # print(vis_t)

            visibilities.append(vis_t)

            if vis_t < visibility_threshold:
                num_below_thres += 1
            
        # import pandas as pd
        # import seaborn as sns
        # from scipy import stats
        # import matplotlib.pyplot as plt

        # # sns.set(color_codes=True)
        # # sns.distplot(visibilities)
        # plt.plot(visibilities)
        # plt.show()
        
        return num_below_thres / (num_total_timesteps - num_obs_timesteps)

    def compute_legibility_metric(self, robot_traj):
        num_timesteps = np.array(robot_traj).shape[0]
        d_eef_s_q = np.zeros((num_timesteps - 1))
        legibility = 0
        f_t = np.ones((num_timesteps - 1))

        p_eef_g = self.get_eef_position(robot_traj[-1])
        p_eef_s = self.get_eef_position(robot_traj[0])

        Cstar_s_g = np.linalg.norm(p_eef_g - p_eef_s)

        for i in range(num_timesteps - 1):
            p_eef_t = self.get_eef_position(robot_traj[i])
            p_eef_t1 = self.get_eef_position(robot_traj[i + 1])
            d_eef_s_q[i] = np.linalg.norm(p_eef_t1 - p_eef_t)

            Cstar_q_g = np.linalg.norm(p_eef_g - p_eef_t1)
            C_s_q = np.sum(d_eef_s_q[:i])

            p_g_given_q = np.exp(-C_s_q - Cstar_q_g) / np.exp(-Cstar_s_g)
            legibility += p_g_given_q * f_t[i]

        legibility = legibility / np.sum(f_t)

        return legibility

    def compute_nominal_traj_metric(self, robot_traj, nominal_traj):
        num_timesteps = len(robot_traj)

        deviation = 0
        
        for i in range(num_timesteps):
            p_eef_t = self.get_eef_position(robot_traj[i])
            p_nom_t = self.get_eef_position(nominal_traj[i])

            deviation += np.linalg.norm(p_eef_t - p_nom_t)

        return deviation


    def evaluate_metrics(self, robot_traj, full_complete_test_traj, num_obs_timesteps, object_pos, nominal_traj):
        num_human_joints = 11

        full_complete_test_traj_expanded = expand_human_test_traj(full_complete_test_traj, 11, num_obs_timesteps, 20)

        num_timesteps_expanded = len(full_complete_test_traj_expanded) / (num_human_joints * 3)

        full_head_test_traj_expanded = []
        head_ind = 5

        print("Num human timesteps expanded: ", num_timesteps_expanded)

        for i in range(num_timesteps_expanded):
            full_head_test_traj_expanded.append(full_complete_test_traj_expanded[i * num_human_joints * 3 + head_ind * 3 + 0])
            full_head_test_traj_expanded.append(full_complete_test_traj_expanded[i * num_human_joints * 3 + head_ind * 3 + 1])
            full_head_test_traj_expanded.append(full_complete_test_traj_expanded[i * num_human_joints * 3 + head_ind * 3 + 2])



        distance_metric = self.compute_distance_metric(full_complete_test_traj_expanded, num_obs_timesteps, num_timesteps_expanded, robot_traj)
        visibility_metric = self.compute_visibility_metric(full_head_test_traj_expanded, num_obs_timesteps, num_timesteps_expanded, robot_traj, object_pos)
        legibility_metric = self.compute_legibility_metric(robot_traj)
        nominal_traj_metric = self.compute_nominal_traj_metric(robot_traj, nominal_traj)
        

        print("Distance metric : ", distance_metric)
        print("Visibility metric : ", visibility_metric)
        print("Legibility metric : ", legibility_metric)
        print("Nominal traj metric : ", nominal_traj_metric)

        metrics = [distance_metric, visibility_metric, legibility_metric, nominal_traj_metric]

        # print(metrics)
        return metrics



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

        # self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        # dist_request = create_empty_request(20, final_joint, self.manipulator_name)
        # add_distance_baseline_cost(dist_request, head_pos, torso_pos, feet_pos, self.eef_link_name, 20, 5)
        # add_regularize_cost(dist_request, coeffs_reg, self.eef_link_name)
        # add_collision_cost(dist_request, [20], [0.025])
        # add_smoothing_cost(dist_request, 10, 2)

        # dist_result = self.optimize_problem(dist_request)
        # if (exec_traj):
        #     self.execute_full_trajectory(dist_result.GetTraj(), full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)


        # self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        # vis_request = create_empty_request(20, final_joint, self.manipulator_name)
        # add_visibility_baseline_cost(vis_request, head_pos, object_pos, self.eef_link_name, 20, 5)
        # add_regularize_cost(vis_request, coeffs_reg, self.eef_link_name)
        # add_collision_cost(vis_request, [20], [0.025])
        # add_smoothing_cost(vis_request, 10, 2)

        # vis_result = self.optimize_problem(vis_request)
        # if (exec_traj):
        #     self.execute_full_trajectory(vis_result.GetTraj(), full_rightarm_test_traj, num_obs_timesteps, num_human_timesteps)


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

        # Setup coefficients
        coeff_optimal_traj = 5.0
        coeff_dist = []
        coeff_vel = []
        coeff_vis = []
        coeff_leg = 100.0
        coeffs_reg = []
        for i in range(num_timesteps):
            coeff_dist.append(100.0)
            coeff_vel.append(100.0)
            coeff_vis.append(1.0)
        for i in range(num_timesteps - 1):
            coeffs_reg.append(1.0)

        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())
        default_traj, default_eef_traj = self.get_default_trajectory(init_joint, final_joint, num_timesteps)
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