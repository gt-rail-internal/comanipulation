from trajectory_framework import TrajectoryFramework
import traj_calc
import scene_utils
import metrics
import traj_utils
import sys
import trajoptpy

class Test:
    def __init__(self, robot_type, init_joint, final_joint, plot='', traj_num=303, execute=False):
        self.robot_type, self.plot = robot_type, plot
        self.framework = TrajectoryFramework(self.robot_type, self.plot)
        self.framework.trajectory_solver.load_traj_file(traj_num)
        self.OBJECT_POS = [0, 0.2, 0.83]
        self.init_joint, self.final_joint = init_joint, final_joint
        self.execute = execute
        if execute:
            self.framework.setup_ros()

    def distance_visibility_test(self, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the separation distance
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        


        num_timesteps = self.framework.trajectory_solver.n_pred_timesteps
        coeffs = {
            "distanceBaseline": 20,
            "visibilityBaseline": 1.5,
            "regularize": [1.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            "smoothing": dict(cost=10, type=2)
        }

        result, eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(self.init_joint, 
                self.final_joint, coeffs=coeffs, plot=plot, execute=self.execute, 
                save='trajectories/distance.txt')
        default_traj, _ = self.framework.trajectory_solver.get_default_traj(self.init_joint, self.final_joint, self.framework.trajectory_solver.n_pred_timesteps)
        
        # print("Head pos = " + str(self.framework.trajectory_solver.head_pos))
        # print("Torso pos = " + str(self.framework.trajectory_solver.torso_pos))
        # print("Feet pos = " + str(self.framework.trajectory_solver.feet_pos))
        # print("Object pos = " + str(self.OBJECT_POS))
        # print("Distance viz baseline = " + str(result.GetTraj()))
        # print("Default Traj = " + str(default_traj))
        # sys.exit("Distance viz baseline traj above")
        
        return metrics.evaluate_metrics(self.framework.scene, result.GetTraj(), 
            self.framework.trajectory_solver.full_complete_test_traj, 
            len(self.framework.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            self.OBJECT_POS, default_traj)

    def legibility_test(self, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the legibility
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        
        num_timesteps = self.framework.trajectory_solver.n_pred_timesteps
        coeffs = {
            "legibilityBaseline": 10000000.0,
            # "regularize": [10.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025])
            # "smoothing": dict(cost=10, type=2)
        }

        result, eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(self.init_joint, 
                self.final_joint, coeffs=coeffs, plot=plot, execute=self.execute, 
                save='trajectories/legibility.txt')
        
        full_complete_test_traj = traj_utils.create_human_plot_traj(self.framework.trajectory_solver.full_rightarm_test_traj)
        default_traj, _ = self.framework.trajectory_solver.get_default_traj(self.init_joint, self.final_joint, self.framework.trajectory_solver.n_pred_timesteps)
        return metrics.evaluate_metrics(self.framework.scene, result.GetTraj(), 
            full_complete_test_traj, 
            len(self.framework.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            self.OBJECT_POS, default_traj)

    def nominal_trajectory_test(self, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the nominal trajectory
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        
        coeffs = {"joint_vel": 1}
        result, eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(self.init_joint, 
                self.final_joint, coeffs=coeffs, plot=plot, execute=self.execute, 
                save='trajectories/nominal.txt')
        
        full_complete_test_traj = traj_utils.create_human_plot_traj(self.framework.trajectory_solver.full_rightarm_test_traj)
        default_traj, _ = self.framework.trajectory_solver.get_default_traj(self.init_joint, self.final_joint, self.framework.trajectory_solver.n_pred_timesteps)
        return metrics.evaluate_metrics(self.framework.scene, result.GetTraj(), 
            full_complete_test_traj, 
            len(self.framework.trajectory_solver.obs_rightarm_test_traj) / 12, # assuming 4 arm joints
            self.OBJECT_POS, default_traj)


    def speed_control_baseline_test(self):
        """
        Generates a default trajectory, then calculates the speed-adapted version. This version is slowed
        by a factor of 10, then potentially executed and its metrics returned.

        init_joint: the starting robot position
        final_joint: the end robot position

        """
        
        default_traj, default_eef_traj = self.framework.trajectory_solver.get_default_traj(
                self.init_joint, self.final_joint, self.framework.trajectory_solver.n_pred_timesteps)
        
        human_joints_num = 11
        human_traj = self.framework.trajectory_solver.full_complete_test_traj[self.framework.trajectory_solver.final_obs_timestep_ind * human_joints_num * 3 : ]
        adapted_robot_traj_fast = self.framework.trajectory_solver.calculate_adaptive_trajectory(default_traj, human_traj)

        if len(adapted_robot_traj_fast) < 200:
            last_joints = adapted_robot_traj_fast[-1]
            for i in range(200 - len(adapted_robot_traj_fast)):
                adapted_robot_traj_fast.append(last_joints)

        new_num_timesteps_fast = len(adapted_robot_traj_fast)

        adapted_robot_fast_spline = traj_calc.cubic_interpolation(adapted_robot_traj_fast, 7)

        adapted_robot_traj_slow = []
        for i in range(20):
            adapted_robot_traj_slow.append(adapted_robot_fast_spline(i * 10))
        
        # print("Adapted Traj = " + str(adapted_robot_traj_slow))
        # sys.exit("Speed Adj Baseline above")

        if (self.execute):
            self.framework.scene.execute_full_trajectory(adapted_robot_traj_slow, self.framework.trajectory_solver.full_rightarm_test_traj, self.framework.trajectory_solver.final_obs_timestep_ind, self.framework.trajectory_solver.num_human_timesteps)

        return metrics.evaluate_metrics(self.framework.scene, adapted_robot_traj_slow, self.framework.trajectory_solver.full_complete_test_traj, self.framework.trajectory_solver.final_obs_timestep_ind, self.OBJECT_POS, default_traj)

    def run_all_baselines(self):
        metrics = []
        print("Distance + Visibility Baseline: ")
        # trajoptpy.SetInteractive(True)
        metrics.append(self.distance_visibility_test(plot=self.plot))
        # trajoptpy.SetInteractive(False)

        print("Legibility Baseline: ")
        metrics.append(self.legibility_test(plot=self.plot))

        print("Nominal Trajectory Baseline: ")
        metrics.append(self.nominal_trajectory_test(plot=self.plot))

        print("Speed Control Baseline")
        metrics.append(self.speed_control_baseline_test())

        return metrics
