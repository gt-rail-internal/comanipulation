import numpy as np
import robot_utils
import traj_file_utils
import trajopt_request_utils as req_util
import metric_utils
import plots
import comanipulationpy as comanip
import json
import trajoptpy

import time

import os.path as path
import sys
sys.path.append(path.dirname(path.abspath(path.join(__file__, ".."))))

OBJECT_POS = [0, 0.2, 0.83]

class TrajectoryPlanner:
    def __init__(self, env, manipulator_name, all_links, eef_link_name):
        self.env = env
        self.manipulator_name = manipulator_name
        self.robot = self.env.GetRobots()[0]
        self.manipulator = self.robot.GetManipulator(self.manipulator_name)
        self.all_links = all_links
        self.eef_link_name = eef_link_name

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

    def get_default_trajectory(self, init_joint, final_joint, num_timesteps):
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
        eef_traj = robot_utils.follow_trajectory(np.array(result.GetTraj()))

        return result.GetTraj(), eef_traj

    def run_test(self, init_joint, final_joint, traj_num, coeffs={}, plot='', execute=True, save=''):
        """
        Calculates an optimal trajectory from init_joint to final_joint based on the weights in coeffs,
        then optionally executes, plots, and saves that trajectory

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
        plot: whether to plot the end effector trajectory
        execute: whether to execute the calculated trajectory
        save: where to save the trajectory
        """
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        full_rightarm_test_traj, obs_rightarm_test_traj, complete_pred_traj_means, complete_pred_traj_vars = traj_file_utils.load_all_human_trajectories(
            traj_num)
        complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded = comanip.expand_human_pred(
            complete_pred_traj_means, complete_pred_traj_vars)

        n_human_joints = 11
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)

        print("Full trajectory timesteps:", len(full_rightarm_test_traj)/12)
        print("Observed trajectory timesteps:", len(obs_rightarm_test_traj)/12)
        print("Num timesteps:", num_timesteps)

        ref_joint_traj, ref_eef_traj = self.get_default_trajectory(
            init_joint, final_joint, num_timesteps)
        self.robot.SetDOFValues(init_joint, self.manipulator.GetArmIndices())

        request = req_util.create_empty_request(
            num_timesteps, final_joint, self.manipulator_name)
        if "distance" in coeffs:
            req_util.add_distance_cost(request, complete_pred_traj_means_expanded,
                complete_pred_traj_vars_expanded, coeffs["distance"], n_human_joints, self.all_links)
        if "collision" in coeffs:
            req_util.add_collision_cost(
                request, coeffs["collision"]["cost"], coeffs["collision"]["dist_pen"])
        if "nominal" in coeffs:
            req_util.add_optimal_trajectory_cost(
                request, ref_eef_traj, self.eef_link_name, num_timesteps, coeffs["nominal"])
        if "regularize" in coeffs:
            req_util.add_regularize_cost(
                request, coeffs["regularize"], self.eef_link_name)
        if "smoothing" in coeffs:
            req_util.add_smoothing_cost(
                request, coeffs["smoothing"]["cost"], coeffs["smoothing"]["type"])
        if 'velocity' in coeffs:
            req_util.add_velocity_cost(request, complete_pred_traj_means_expanded, 
                complete_pred_traj_vars_expanded, coeffs['velocity'], n_human_joints, self.all_links)
        if 'visibility' in coeffs:
            head_pred_traj_mean, head_pred_traj_var = comanip.create_human_head_means_vars(
                complete_pred_traj_means_expanded, complete_pred_traj_vars_expanded)
            req_util.add_visibility_cost(request, head_pred_traj_mean, head_pred_traj_var, 
                coeffs['visibility'], OBJECT_POS, self.eef_link_name)
        if 'legibility' in coeffs:
            req_util.add_legibility_cost(request, coeffs['legibility'], self.eef_link_name)

        result = self.optimize_problem(request)
        eef_traj = robot_utils.follow_trajectory(np.array(result.GetTraj()))

        if execute:
            robot_utils.execute_full_trajectory(result.GetTraj(), full_rightarm_test_traj, len(
                obs_rightarm_test_traj) / 12, len(full_rightarm_test_traj) / 12)
        if plot != '':
            full_complete_test_traj = comanip.create_human_plot_traj(
                full_rightarm_test_traj)
            plots.plot_trajectory(eef_traj, "Distance", ref_eef_traj, "Joint Space Linear",
                                  plot, full_complete_test_traj, 11)
        if save != '':
            np.savetxt('trajectories/distance.txt', eef_traj, delimiter=',')

        return eef_traj

    def get_n_timesteps(self, traj_num):
        """
        Extracts the number of predicted timesteps in a trajectory with a given number

        traj_num: the number of the trajectory
        """
        _, _, complete_pred_traj_means, complete_pred_traj_vars = traj_file_utils.load_all_human_trajectories(
            traj_num)
        complete_pred_traj_means_expanded, _ = comanip.expand_human_pred(
            complete_pred_traj_means, complete_pred_traj_vars)

        n_human_joints = 11
        num_timesteps = len(complete_pred_traj_means_expanded) / (n_human_joints * 3)
        return num_timesteps

    def distance_test(self, init_joint, final_joint, plot='', traj_num=303):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the separation distance
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.get_n_timesteps(traj_num)
        coeffs = {
            "distance": [1000.0 for _ in range(num_timesteps)],
            # "regularization": [10.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            "smoothing": dict(cost=10, type=2),
            "nominal": 0.5
        }

        eef_traj = self.run_test(init_joint, final_joint, traj_num, coeffs=coeffs,
                                 plot=plot, execute=True, save='trajectories/distance.txt')
        return eef_traj

    def velocity_test(self, init_joint, final_joint, plot='', traj_num=303):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the velocity
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.get_n_timesteps(traj_num)
        coeffs = {'velocity': [50000.0 for _ in range(num_timesteps)]}

        eef_traj = self.run_test(init_joint, final_joint, traj_num, coeffs=coeffs,
                                 plot=plot, execute=True, save='trajectories/velocity.txt')

        return eef_traj

    def visibility_test(self, init_joint, final_joint, plot='', traj_num=303):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the visibility
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.get_n_timesteps(traj_num)
        #TODO: use the visibility metric
        coeffs = {
            # 'visibility': [0.01 for _ in range(num_timesteps)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            'nominal': 10,
            'smoothing': dict(cost=10, type=2),
        }
        eef_traj = self.run_test(init_joint, final_joint, traj_num, coeffs=coeffs)
        return eef_traj

    def legibility_test(self, init_joint, final_joint, plot='', traj_num=303):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the legibility
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.get_n_timesteps(traj_num)
        coeffs = {
            'legibility': 100.0,
            'regularize': [1.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
        }
        eef_traj = self.run_test(init_joint, final_joint, traj_num, coeffs=coeffs, plot=plot, save='trajectories/legibility.txt')
        return eef_traj

    def nominal_trajectory_test(self, init_joint, final_joint, traj_num=303, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the nominal trajectory
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        coeffs = {'nominal': 5}
        eef_traj = self.run_test(init_joint, final_joint, traj_num, coeffs=coeffs, plot=plot, save='trajectories/nominal.txt')

        return eef_traj

    
