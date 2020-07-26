import numpy as np
import traj_utils 
import scene_utils
import traj_file_utils
import trajopt_request_utils as req_util
import metrics
import plots
import json
import trajoptpy
# trajoptpy.SetInteractive(True)

from scipy.interpolate import CubicSpline

import time


class TrajectoryPlanner:
    def __init__(self, scene, n_human_joints=11, n_robot_joints=7):
        self.scene = scene
        self.n_human_joints = n_human_joints
        self.n_robot_joints = n_robot_joints

    def optimize_problem(self, request):
        """
        Returns the result of running trajopt on a problem specified in `request`

        request: A JSON-formatted solver request
        """
        s = json.dumps(request)
        # Create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.scene.env)
        t_start = time.time()
        result = trajoptpy.OptimizeProblem(prob)  # do optimization
        t_elapsed = time.time() - t_start
        print(result)
        print("optimization took %.3f seconds" % t_elapsed)

        # from trajoptpy.check_traj import traj_is_safe
        # prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        # assert traj_is_safe(result.GetTraj(), self.scene.robot) # Check that trajectory is collision free

        return result

    def get_default_traj(self, init_joint, final_joint, num_timesteps):
        """
        Returns a trajectory constrained only by joint velocities and the
        corresponding end effector cartesian trajectory

        init_joint: the initial set of joint angles
        final_joint: the desired set of joint angles 
        """
        self.scene.robot.SetDOFValues(init_joint, self.scene.manipulator.GetArmIndices())

        request = req_util.create_empty_request(
            num_timesteps, final_joint, self.scene.manipulator_name)

        req_util.add_joint_vel_cost(request, 1)

        result = self.optimize_problem(request)
        eef_traj = self.scene.follow_trajectory(np.array(result.GetTraj()))

        return result.GetTraj(), eef_traj

    def load_traj_file(self, traj_num):
        """
        Loads a trajectory file and stores the trajectory in instance variables for later use.
        Also stores number of timesteps in self.n_pred_timesteps.

        traj_num: the number of the trajectory to load
        """
        self.full_rightarm_test_traj, self.obs_rightarm_test_traj, self.complete_pred_traj_means, self.complete_pred_traj_vars = traj_file_utils.load_all_human_trajectories(
            traj_num)
        self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded = traj_utils.expand_human_pred(
            self.complete_pred_traj_means, self.complete_pred_traj_vars)
        self.n_pred_timesteps = len(
            self.complete_pred_traj_means_expanded) / (self.n_human_joints * 3)
        
        self.full_complete_test_traj = traj_utils.create_human_plot_traj(self.full_rightarm_test_traj)
        self.obs_complete_test_traj = traj_utils.create_human_plot_traj(self.obs_rightarm_test_traj)
        self.num_human_timesteps = len(self.full_complete_test_traj) / (self.n_human_joints * 3)
        self.final_obs_timestep_ind = len(self.obs_complete_test_traj) / (self.n_human_joints * 3)
        head_ind = 5
        torso_ind = 6
        self.head_pos = self.full_complete_test_traj[(self.final_obs_timestep_ind * self.n_human_joints + head_ind) * 3 : (self.final_obs_timestep_ind * self.n_human_joints + head_ind + 1) * 3]
        self.torso_pos = self.full_complete_test_traj[(self.final_obs_timestep_ind * self.n_human_joints + torso_ind) * 3 : (self.final_obs_timestep_ind * self.n_human_joints + torso_ind + 1) * 3]
        self.feet_pos = [self.torso_pos[0], self.torso_pos[1], self.torso_pos[2] - 0.5]

    def set_traj(self, complete_traj_means, complete_traj_vars):
        """
        Accepts predicted human trajectories and sets the variables necessary to solve
        Also stores number of predicted timesteps in self.n_pred_timesteps

        complete_traj_means: the expected human trajectory (complete, not arm-only)
        complete_traj_vars: the covariance matrices associated with complete_traj_means
        """
        self.full_rightarm_test_traj, self.obs_rightarm_test_traj = None, None
        self.complete_pred_traj_means, self.complete_pred_traj_vars = complete_traj_means, complete_traj_vars
        self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded = traj_utils.expand_human_pred(
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
            # TODO: this method for timestep calculation should leverage class-level n_joint variables
            self.scene.execute_full_trajectory(result.GetTraj(), self.full_rightarm_test_traj, len(
                self.obs_rightarm_test_traj) / 12, len(self.full_rightarm_test_traj) / 12)
        if plot != '':
            full_complete_test_traj = traj_utils.create_human_plot_traj(
                self.full_rightarm_test_traj)
            plots.plot_trajectory(eef_traj, "Distance", default_traj, "Joint Space Linear",
                                  plot, full_complete_test_traj, 11)
        if save != '':
            np.savetxt(save, eef_traj, delimiter=',')
            
        return result, eef_traj

    def solve_traj(self, init_joint, final_joint, coeffs={}, object_pos=[0, 0.2, 0.83]):
        """
        Calculates an optimal trajectory from init_joint to final_joint based on the weights in coeffs.
        Returns joint trajectory and corresponding end effector trajectory

        init_joint: the initial set of joint angles
        final_joint: the desired set of joint angles
        coeffs: A dictionary containing information on weights. All keys are optional.
        Valid keys are 
            "distance": array of length num_timesteps
            "distanceBaseline": array of length num_timesteps
            "collision": a dictionary mapping 'cost' and 'dist_pen' to number arrays
            "nominal": number
            "regularize": array of length num_timesteps - 1
            "smoothing": dictionary mapping 'cost' and 'type' to a number and an int, respectively
            "velocity": array of length num_timesteps (this is a CoMOTO cost, not the trajopt joint velocity cost)
            "visibility": array of length num_timesteps
            "visibilityBaseline": array of length num_timesteps
            "legibility": number
            "legibilityBaseline": number
            "joint_vel": number or [number] of length 1. This is the trajopt joint velocity cost.
        object_pos: The position of the object of interest to the person. Only needed for
            visiblity cost
        """
        self.scene.robot.SetDOFValues(init_joint, self.scene.manipulator.GetArmIndices())

        _, default_traj = self.get_default_traj(
            init_joint, final_joint, self.n_pred_timesteps)
        self.scene.robot.SetDOFValues(init_joint, self.scene.manipulator.GetArmIndices())

        request = req_util.create_empty_request(
            self.n_pred_timesteps, final_joint, self.scene.manipulator_name)
        if "distance" in coeffs:
            req_util.add_distance_cost(request, self.complete_pred_traj_means_expanded,
                                       self.complete_pred_traj_vars_expanded, coeffs["distance"], self.n_human_joints, self.scene.all_links)
        if "distanceBaseline" in coeffs:
            req_util.add_distance_baseline_cost(request, self.head_pos, self.torso_pos, self.feet_pos, self.scene.all_links, self.n_pred_timesteps, coeffs["distanceBaseline"])
        
        if "visibilityBaseline" in coeffs:
            req_util.add_visibility_baseline_cost(request, self.head_pos, object_pos, self.scene.eef_link_name, self.n_pred_timesteps, coeffs["visibilityBaseline"])

        if "legibilityBaseline" in coeffs:
            req_util.add_legibility_baseline_cost(
                request, coeffs["legibilityBaseline"], self.scene.eef_link_name)
        if "collision" in coeffs:
            req_util.add_collision_cost(
                request, coeffs["collision"]["cost"], coeffs["collision"]["dist_pen"])
        if "nominal" in coeffs:
            req_util.add_optimal_trajectory_cost(
                request, default_traj, self.scene.eef_link_name, self.n_pred_timesteps, coeffs["nominal"])
        if "regularize" in coeffs:
            req_util.add_regularize_cost(
                request, coeffs["regularize"], self.scene.eef_link_name)
        if "smoothing" in coeffs:
            req_util.add_smoothing_cost(
                request, coeffs["smoothing"]["cost"], coeffs["smoothing"]["type"])
        if "velocity" in coeffs:
            req_util.add_velocity_cost(request, self.complete_pred_traj_means_expanded,
                                       self.complete_pred_traj_vars_expanded, coeffs["velocity"], self.n_human_joints, self.scene.all_links)
        if "visibility" in coeffs:
            head_pred_traj_mean, head_pred_traj_var = traj_utils.create_human_head_means_vars(
                self.complete_pred_traj_means_expanded, self.complete_pred_traj_vars_expanded)
            req_util.add_visibility_cost(request, head_pred_traj_mean, head_pred_traj_var,
                                         coeffs["visibility"], object_pos, self.scene.eef_link_name)
        if "legibility" in coeffs:
            req_util.add_legibility_cost(
                request, coeffs["legibility"], self.scene.eef_link_name)
        
        if "joint_vel" in coeffs:
            req_util.add_joint_vel_cost(request, coeffs["joint_vel"])

        result = self.optimize_problem(request)
        eef_traj = self.scene.follow_trajectory(np.array(result.GetTraj()))
        return result, eef_traj

    def calculate_adaptive_trajectory(self, robot_joints, human_traj):
        '''
        Takes in a human and a robot trajectory and returns a version of that trajectory in which the
        robot follows the same path but 

        robot_joints: time-sampled JOINT space trajectory (vectorized - timesteps*robot_num_joints*3 matrix)
        human_traj: human position from vision system (vectorized - timesteps*human_num_joints*3matrix)
        the order of human_traj is - right_shoulder + right_elbow + right_wrist + right_palm + neck + head + 
            torso + left_shoulder + left_elbow + left_wrist + left_palm
        ideally, robot_joints >= human_traj_timesteps
        '''
        scaling_factor = 1  
        d_slow = 0.15 # choose threshold
        d_stop = 0.06 # choose threshold
        beta = 3.3332 # parameter
        gamma = 0.5 # parameter
        traj_interpolation = cubic_interpolation(robot_joints, self.n_robot_joints)
        num_timesteps = len(human_traj)/(self.n_human_joints*3)
        robot_total_timesteps = len(robot_joints)
        new_exec_traj = [] 
        robot_timestep = 0
        done = False
        human_timestep = 0
        while not done:
            new_robot_joints = traj_interpolation(robot_timestep).tolist()
            new_exec_traj.append(new_robot_joints)

            curr_human_pos = human_traj[human_timestep*self.n_human_joints*3:(human_timestep + 1)*self.n_human_joints*3]
            d = metrics.get_separation_dist(self.scene, curr_human_pos, new_robot_joints)
            if (d_stop <= d <= d_slow):
                scaling_factor = 1 - beta * ((d - d_stop) ** gamma)
            elif (d_slow < d):# change to new timestamp and trajectory pts
                scaling_factor = 0
            else:
                scaling_factor = 1
            
            robot_timestep += 0.1 - scaling_factor/10
            if human_timestep < num_timesteps - 1:
                human_timestep += 1

            if robot_total_timesteps <= robot_timestep:
                done = True
            elif human_timestep == (num_timesteps - 1) and scaling_factor == 1:
                done = True
            else:
                done = False
        
        return new_exec_traj

def cubic_interpolation(robot_joints, robot_num_joints):
    """
    Returns a scipy CubicSpline object that splines between trajectory waypoints in robot_joints

    robot_joints: An array representing a robot joint trajectory. 1-D or 2-D.
    robot_num_joints: The number of joints the robot has
    """
    robot_joints = np.array(robot_joints)
    robot_joints_pos = np.reshape(robot_joints, (-1, robot_num_joints))
    x = list(range(0, len(robot_joints_pos)))
    return CubicSpline(x, robot_joints_pos)