import numpy as np
import scene_utils
import traj_file_utils
import trajopt_request_utils as req_util
# import metrics
import plots
import json
import trajoptpy

import time

import os.path as path
import sys
sys.path.append(path.dirname(path.abspath(path.join(__file__, ".."))))
import comanipulationpy as comanip


class TrajectoryPlanner:
    def __init__(self, env, manipulator_name, all_links, eef_link_name, n_human_joints=11):
        self.env = env
        self.manipulator_name = manipulator_name
        self.robot = self.env.GetRobots()[0]
        self.manipulator = self.robot.GetManipulator(self.manipulator_name)
        self.all_links = all_links
        self.eef_link_name = eef_link_name
        self.n_human_joints = n_human_joints

    def optimize_problem(self, request):
        """
        Returns the result of running trajopt on a problem specified in `request`

        request: A JSON-formatted solver request
        """
        s = json.dumps(request)
        # Create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.env)
        t_start = time.time()
        result = trajoptpy.OptimizeProblem(prob)  # do optimization
        t_elapsed = time.time() - t_start
        print(result)
        print("optimization took %.3f seconds" % t_elapsed)

        # from trajoptpy.check_traj import traj_is_safe
        # prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        # assert traj_is_safe(result.GetTraj(), self.robot) # Check that trajectory is collision free

        return result

    def get_default_traj(self, init_joint, final_joint, num_timesteps):
        """
        Returns a trajectory constrained only by joint velocities and the
        corresponding end effector cartesian trajectory

        init_joint: the initial set of joint angles
        final_joint: the desired set of joint angles 
        """
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = req_util.create_empty_request(
            num_timesteps, final_joint, self.manipulator_name)

        req_util.add_joint_vel_cost(request, 1)

        result = self.optimize_problem(request)
        eef_traj = scene_utils.follow_trajectory(np.array(result.GetTraj()))

        return result.GetTraj(), eef_traj

    def load_traj_file(self, traj_num):
        """
        Loads a trajectory file and stores the trajectory in instance variables for later use.
        Also stores number of timesteps in self.n_pred_timesteps.

        traj_num: the number of the trajectory to load
        """
        self.full_rightarm_test_traj, self.obs_rightarm_test_traj, self.complete_pred_traj_means, self.complete_pred_traj_vars = traj_file_utils.load_all_human_trajectories(
            traj_num)
        self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded = comanip.expand_human_pred(
            self.complete_pred_traj_means, self.complete_pred_traj_vars)
        self.n_pred_timesteps = len(
            self.complete_pred_traj_means_expanded) / (self.n_human_joints * 3)

    def set_traj(self, complete_traj_means, complete_traj_vars):
        """
        Accepts predicted human trajectories and sets the variables necessary to solve
        Also stores number of predicted timesteps in self.n_pred_timesteps

        complete_traj_means: the expected human trajectory (complete, not arm-only)
        complete_traj_vars: the covariance matrices associated with complete_traj_means
        """
        self.full_rightarm_test_traj, self.obs_rightarm_test_traj = None, None
        self.complete_pred_traj_means, self.complete_pred_traj_vars = complete_traj_means, complete_traj_vars
        self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded = comanip.expand_human_pred(
            self.complete_pred_traj_means, self.complete_pred_traj_vars)
        self.n_pred_timesteps = len(
            self.complete_pred_traj_means_expanded) / (self.n_human_joints * 3)

    def solve_traj_save_plot_exec(self, init_joint, final_joint, coeffs={}, object_pos=[0, 0.2, 0.83],
                plot='', execute=False, save=''):
        """
        NOTE: THIS IS ONLY COMPATIBLE WITH TRAJECTORIES THAT HAVE BEEN LOADED IN WITH load_traj_file
        A convenience function, which solves a trajectory and then optionally executes, plots, and saves it.

        First four arguments (init_joint, final_joint, coeffs, and object_pos): same as solve_traj
        plot: where to save the plot of the end effector trajectory, empty string to do nothing
        execute: whether to execute the calculated trajectory (boolean)
        save: where to save the trajectory as a text file, empty string to do nothing
        """
        result, eef_traj = self.solve_traj(init_joint, final_joint, coeffs=coeffs, object_pos=object_pos)
        _, default_traj = self.get_default_traj(
            init_joint, final_joint, self.n_pred_timesteps)
        if execute:
            scene_utils.execute_full_trajectory(result.GetTraj(), self.full_rightarm_test_traj, len(
                self.obs_rightarm_test_traj) / 12, len(self.full_rightarm_test_traj) / 12)
        if plot != '':
            full_complete_test_traj = comanip.create_human_plot_traj(
                self.full_rightarm_test_traj)
            plots.plot_trajectory(eef_traj, "Distance", default_traj, "Joint Space Linear",
                                  plot, full_complete_test_traj, 11)
        if save != '':
            np.savetxt(save, eef_traj, delimiter=',')

    def solve_traj(self, init_joint, final_joint, coeffs={}, object_pos=[0, 0.2, 0.83]):
        """
        Calculates an optimal trajectory from init_joint to final_joint based on the weights in coeffs.
        Returns joint trajectory and corresponding end effector trajectory

        init_joint: the initial set of joint angles
        final_joint: the desired set of joint angles
        coeffs: A dictionary containing information on weights. All keys are optional.
        Valid keys are 
            'distance': array of length num_timesteps
            'collision': a dictionary mapping 'cost' and 'dist_pen' to number arrays
            'nominal': number
            'regularize': array of length num_timesteps - 1
            'smoothing': dictionary mapping 'cost' and 'type' to a number and an int, respectively
            'velocity': array of length num_timesteps
            'visibility': array of length num_timesteps
            'legibility': number
        object_pos: The position of the object of interest to the person. Only needed for
            visiblity cost
        """
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        _, default_traj = self.get_default_traj(
            init_joint, final_joint, self.n_pred_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = req_util.create_empty_request(
            self.n_pred_timesteps, final_joint, self.manipulator_name)
        if "distance" in coeffs:
            req_util.add_distance_cost(request, self.complete_pred_traj_means_expanded,
                                       self.complete_pred_traj_vars_expanded, coeffs["distance"], self.n_human_joints, self.all_links)
        if "collision" in coeffs:
            req_util.add_collision_cost(
                request, coeffs["collision"]["cost"], coeffs["collision"]["dist_pen"])
        if "nominal" in coeffs:
            req_util.add_optimal_trajectory_cost(
                request, default_traj, self.eef_link_name, self.n_pred_timesteps, coeffs["nominal"])
        if "regularize" in coeffs:
            req_util.add_regularize_cost(
                request, coeffs["regularize"], self.eef_link_name)
        if "smoothing" in coeffs:
            req_util.add_smoothing_cost(
                request, coeffs["smoothing"]["cost"], coeffs["smoothing"]["type"])
        if 'velocity' in coeffs:
            req_util.add_velocity_cost(request, self.complete_pred_traj_means_expanded,
                                       self.complete_pred_traj_vars_expanded, coeffs['velocity'], self.n_human_joints, self.all_links)
        if 'visibility' in coeffs:
            head_pred_traj_mean, head_pred_traj_var = comanip.create_human_head_means_vars(
                self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded)
            req_util.add_visibility_cost(request, head_pred_traj_mean, head_pred_traj_var,
                                         coeffs['visibility'], object_pos, self.eef_link_name)
        if 'legibility' in coeffs:
            req_util.add_legibility_cost(
                request, coeffs['legibility'], self.eef_link_name)

        result = self.optimize_problem(request)
        eef_traj = scene_utils.follow_trajectory(np.array(result.GetTraj()))

        return result, eef_traj
