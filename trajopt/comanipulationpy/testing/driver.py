from collections import namedtuple
from trajopt_utils import TrajectoryPlanner
import metrics
from arm_control import FollowTrajectoryClient
import scene_utils

import json
import matlab.engine
import rospy
from std_msgs.msg import String

import os.path as path
import sys
sys.path.append(path.dirname(path.abspath(path.join(__file__, ".."))))
import comanipulationpy as comanip

import actionlib
import rospy
import openravepy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RobotInfo = namedtuple(
    "RobotInfo", "model arm_name eef_link_name all_links controller_name controller_joints")

DATA_FOLDER = "../../data/"

ROBOTS_DICT = {
    "jaco": RobotInfo(path.join(DATA_FOLDER, "jaco-test.dae"), "test_arm", "j2s7s300_ee_link",
                      ["j2s7s300_ee_link", "j2s7s300_link_6", "j2s7s300_link_4",
                       "j2s7s300_link_7", "j2s7s300_link_5", "j2s7s300_link_3", "j2s7s300_link_2"],
                      "jaco_trajectory_controller", ["j2s7s300_joint_1", "j2s7s300_joint_2",
                                                     "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6",
                                                     "j2s7s300_joint_7"]),
    "franka": RobotInfo(path.join(DATA_FOLDER, "panda_default.dae"), "panda_arm", "panda_hand",
                        ["panda_link1", "panda_link2", "panda_link3",
                         "panda_link4", "panda_link5", "panda_link6", "panda_link7"],
                        "panda_arm_controller", ["panda_joint1", "panda_joint2", "panda_joint3",
                                                 "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]),
    "iiwa": RobotInfo(path.join(DATA_FOLDER, "iiwa_env.dae"), "iiwa_arm", 'iiwa_link_ee',
                      ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
                       "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"],
                      "iiwa/PositionJointInterface_trajectory_controller", ["iiwa_joint_1",
                                                                            "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6",
                                                                            "iiwa_joint_7"])
}

OBJECT_POS = [0, 0.2, 0.83]

class TrajectoryFramework:
    def __init__(self, robot_type, plot, num_human_joints=11):
        self.num_human_joints = num_human_joints
        self.robot_type = robot_type
        self.plot = plot

        self.env = openravepy.Environment()
        self.env.StopSimulation()

        robot_info = ROBOTS_DICT[robot_type]
        self.env.Load(robot_info.model)
        self.manipulator_name = robot_info.arm_name
        self.eef_link_name = robot_info.eef_link_name
        self.all_links = robot_info.all_links

        self.robot = self.env.GetRobots()[0]
        self.manipulator = self.robot.GetManipulator(self.manipulator_name)
        self.eef_link = self.robot.GetLink(self.eef_link_name)

        self.body = openravepy.RaveCreateKinBody(self.env, '')
        self.body.SetName("Collision_Scene")
        self.ros_initialized = False
        self.trajectory_solver = TrajectoryPlanner(self.env, self.manipulator_name,
            self.all_links, self.eef_link_name, n_human_joints=self.num_human_joints)

    def setup_ros(self):
        """
        Sets up a ROS node, if applicable
        """
        rospy.init_node("comanipulation_testing")

        self.follow_joint_trajectory_client = FollowTrajectoryClient(
            robot_info.controller_name, robot_info.controller_joints)
        self.ros_initialized = True
        

    def setup_test_without_ros(self, init_joint, final_joint,
                               traj_num=303, exec_traj=False):
        """
        Uses a hardcoded set of weights and object position to optimize a trajectory.
        Potentially executes trajectory with execute_full_trajectory. Prints information
        on the trajectory, including using evaluate_metrics. Returns values from
        evaluate_metrics.
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

        full_complete_test_traj = comanip.create_human_plot_traj(
            self.trajectory_solver.full_rightarm_test_traj)
        default_traj = self.trajectory_solver.get_default_traj(init_joint, final_joint, self.trajectory_solver.n_pred_timesteps)
        return metrics.evaluate_metrics(result.GetTraj(), 
            full_complete_test_traj, 
            len(self.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            OBJECT_POS, default_traj)

    def setup_test(self, init_joint, final_joint, exec_traj=False):
        """
        Gets a predicted human trajectory with ROS, then solves an optimal trajectory to respond 
        and potentially executes it.

        init_joint: the starting joint configuration of the trajectory
        final_joint: the goal joint configuration
        exec_traj: whether to execute the solved trajectory
        """
        if not self.ros_initialized:
            self.setup_ros()

        # Get prediction from human_traj_pred stream
        complete_pred_traj_means, complete_pred_traj_vars = scene_utils.get_human_obs_and_prediction()

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

        result, _ = self.trajectory_solver.solve_traj(init_joint, final_joint, 
                        coeffs=coeffs, object_pos=OBJECT_POS)

        if exec_traj:
            scene_utils.execute_trajectory(result.GetTraj())

        return result
