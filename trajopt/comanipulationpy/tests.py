from trajectory_framework import TrajectoryFramework
import traj_calc
import scene_utils
import metrics

class Test:
    def __init__(self, robot_type, plot, traj_num=303):
        self.robot_type, self.plot = robot_type, plot
        self.framework = TrajectoryFramework(self.robot_type, self.plot)
        self.framework.trajectory_solver.load_traj_file(traj_num)

    def distance_test(self, init_joint, final_joint, plot='', execute=True):
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
            "distance": [1000.0 for _ in range(num_timesteps)],
            # "regularization": [10.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            "smoothing": dict(cost=10, type=2),
            "nominal": 0.5
        }

        eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(init_joint, 
                final_joint, coeffs=coeffs, plot=plot, execute=execute, 
                save='trajectories/distance.txt')
        return eef_traj

    def velocity_test(self, init_joint, final_joint, plot='', execute=True):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the velocity
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.framework.trajectory_solver.n_pred_timesteps
        coeffs = {'velocity': [50000.0 for _ in range(num_timesteps)]}

        eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(init_joint, 
                final_joint, coeffs=coeffs, plot=plot, execute=execute, 
                save='trajectories/velocity.txt')

        return eef_traj

    def visibility_test(self, init_joint, final_joint, plot='', execute=True):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the visibility
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        # TODO: use the visibility metric
        coeffs = {
            # 'visibility': [0.01 for _ in range(num_timesteps)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            'nominal': 10,
            'smoothing': dict(cost=10, type=2),
        }
        eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(init_joint, 
            final_joint, coeffs=coeffs, plot=plot, execute=execute)
        return eef_traj

    def legibility_test(self, init_joint, final_joint, plot='', execute=True):
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
            'legibility': 100.0,
            'regularize': [1.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
        }
        eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(
            init_joint, final_joint, coeffs=coeffs, plot=plot, 
            save='trajectories/legibility.txt', execute=execute)
        return eef_traj

    def nominal_trajectory_test(self, init_joint, final_joint, plot='', execute=True):
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
        eef_traj = self.framework.trajectory_solver.solve_traj_save_plot_exec(
            init_joint, final_joint, coeffs=coeffs, plot=plot, save='trajectories/nominal.txt', execute=execute)

        return eef_traj


    def speed_control_baseline_test(self, init_joint, final_joint, execute=False):
        """
        Generates a default trajectory, then calculates the speed-adapted version. This version is slowed
        by a factor of 10, then potentially executed and its metrics returned.

        init_joint: the starting robot position
        final_joint: the end robot position

        """
        object_pos = [0, 0.2, 0.83]

        default_traj, default_eef_traj = self.framework.trajectory_solver.get_default_traj(
                init_joint, final_joint, self.framework.trajectory_solver.n_pred_timesteps)
        
        human_traj = self.framework.trajectory_solver.complete_pred_traj_means_expanded
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

        rightarm_traj = self.framework.trajectory_solver.full_rightarm_test_traj
        num_obs_timesteps = len(self.framework.trajectory_solver.obs_rightarm_test_traj)/12
        if (execute):
            num_human_timesteps = len(rightarm_traj)/12
            self.framework.scene.execute_full_trajectory(adapted_robot_traj_slow, rightarm_traj, num_obs_timesteps, num_human_timesteps)

        return metrics.evaluate_metrics(self.framework.scene, adapted_robot_traj_slow, rightarm_traj, num_obs_timesteps, object_pos, default_traj)

