from traj_calc import TrajectoryPlanner
import metrics
from arm_control import FollowTrajectoryClient
from scene_utils import Scene, get_human_obs_and_prediction

import json

import matlab.engine
import rospy
from std_msgs.msg import String

import traj_utils 

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from const import ROBOTS_DICT

OBJECT_POS = [0, 0.2, 0.83]

class TrajectoryFramework:
    def __init__(self, robot_type, plot, num_human_joints=11):
        self.num_human_joints = num_human_joints
        self.robot_type = robot_type
        self.plot = plot

        self.robot_info = ROBOTS_DICT[robot_type]
        self.is_real = self.robot_type.endswith('real')
        self.scene = Scene(self.robot_info)
    
        self.ros_initialized = False
        self.trajectory_solver = TrajectoryPlanner(self.scene, n_human_joints=self.num_human_joints)

    def setup_ros(self):
        """
        Sets up a ROS node, if applicable
        """
        rospy.init_node("comanipulation_testing")

        self.scene.follow_joint_trajectory_client = FollowTrajectoryClient(
            self.robot_info.controller_name, self.robot_info.controller_joints)
        self.ros_initialized = True
        

    def setup_test_without_ros(self, init_joint, final_joint, traj_num=303):
        """
        Uses a hardcoded set of weights and object position to optimize a trajectory.
        Prints information on the trajectory, including using evaluate_metrics. Returns 
        values from evaluate_metrics.
        """
        self.trajectory_solver.load_traj_file(traj_num)
        num_timesteps = self.trajectory_solver.n_pred_timesteps

        coeffs = {
            'nominal': 10.0,
            'distance': [10000.0 for _ in range(num_timesteps)],
            'velocity': [100.0 for _ in range(num_timesteps)],
            'visibility': [0.5 for _ in range(num_timesteps)],
            'regularize': [5.0 for _ in range(num_timesteps - 1)],
            'legibility': 100.0,
            'collision': dict(cost=[20], dist_pen=[0.025]),
            'smoothing': dict(cost=200, type=2)
        }



        # Test Adaptive Control Baseline
        # print("Calculating Adaptive Trajectory now!")
        # adaptive_traj = self.calculate_adaptive_trajectory(default_traj, human_poses_mean, n_human_joints, n_robot_joints)
        # print(adaptive_traj)
        
        result, _ = self.trajectory_solver.solve_traj(init_joint, final_joint, coeffs=coeffs)

        full_complete_test_traj = traj_utils.create_human_plot_traj(self.trajectory_solver.full_rightarm_test_traj)
        default_traj, _ = self.trajectory_solver.get_default_traj(init_joint, final_joint, self.trajectory_solver.n_pred_timesteps)
        return metrics.evaluate_metrics(self.scene, result.GetTraj(), 
            full_complete_test_traj, 
            len(self.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            OBJECT_POS, default_traj)

    def setup_test(self, init_joint, final_joint, traj_num=-1, execute=False):
        """
        Gets a predicted human trajectory with ROS, then solves an optimal trajectory to respond 
        and potentially executes it.

        init_joint: the starting joint configuration of the trajectory
        final_joint: the goal joint configuration
        traj_num: the number of the trajectory to load, OR a negative number 
            to read human poses from a ROS topic
        execute: whether to execute the solved trajectory
        """
        if not self.ros_initialized:
            self.setup_ros()


        if traj_num > 0:
            self.trajectory_solver.load_traj_file(traj_num)
        else:
            # Get prediction from human_traj_pred stream
            complete_pred_traj_means, complete_pred_traj_vars = get_human_obs_and_prediction()
            self.trajectory_solver.set_traj(complete_pred_traj_means, complete_pred_traj_vars)

        num_timesteps = self.trajectory_solver.n_pred_timesteps

        coeffs = {
            'nominal': 10.0,
            'distance': [10000.0 for _ in range(num_timesteps)],
            'velocity': [100.0 for _ in range(num_timesteps)],
            'visibility': [0.5 for _ in range(num_timesteps)],
            'regularize': [5.0 for _ in range(num_timesteps - 1)],
            'legibility': 100.0,
            'collision': dict(cost=[20], dist_pen=[0.025]),
            'smoothing': dict(cost=200, type=2)
        }

        if traj_num > 0 and not self.is_real:
            result, _ = self.trajectory_solver.solve_traj_save_plot_exec(init_joint, final_joint, coeffs=coeffs, 
                object_pos=OBJECT_POS, execute=execute)
        else:
            result, _ = self.trajectory_solver.solve_traj(init_joint, final_joint, 
                            coeffs=coeffs, object_pos=OBJECT_POS)
            if execute:
                self.scene.execute_trajectory(result.GetTraj())

        full_complete_test_traj = traj_utils.create_human_plot_traj(self.trajectory_solver.full_rightarm_test_traj)
        default_traj, _ = self.trajectory_solver.get_default_traj(init_joint, final_joint, self.trajectory_solver.n_pred_timesteps)
        return metrics.evaluate_metrics(self.scene, result.GetTraj(), 
            full_complete_test_traj, 
            len(self.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            OBJECT_POS, default_traj)