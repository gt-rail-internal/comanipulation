
class Test:
    def __init__(self):
        #TODO
        pass

    def distance_test(self, init_joint, final_joint, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the separation distance
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.n_pred_timesteps
        coeffs = {
            "distance": [1000.0 for _ in range(num_timesteps)],
            # "regularization": [10.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
            "smoothing": dict(cost=10, type=2),
            "nominal": 0.5
        }

        eef_traj = self.solve_traj(init_joint, final_joint, coeffs=coeffs,
                                   plot=plot, execute=True, save='trajectories/distance.txt')
        return eef_traj

    def velocity_test(self, init_joint, final_joint, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the velocity
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.n_pred_timesteps
        coeffs = {'velocity': [50000.0 for _ in range(num_timesteps)]}

        eef_traj = self.solve_traj(init_joint, final_joint, coeffs=coeffs,
                                   plot=plot, execute=True, save='trajectories/velocity.txt')

        return eef_traj

    def visibility_test(self, init_joint, final_joint, plot=''):
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
        eef_traj = self.solve_traj(init_joint, final_joint, coeffs=coeffs)
        return eef_traj

    def legibility_test(self, init_joint, final_joint, plot=''):
        """
        Computes, executes, plots, and saves an optimal trajectory from init_joint to
        final_joint. Uses weights that exaggerate the importance of the legibility
        metric.

        init_joint: a vector of joint angles representing the starting position
        final_joint: a vector of joint angles representing the goal position
        plot: the file to which to write a plot of the end effector trajectory
        traj_num: the trajectory number to examine
        """
        num_timesteps = self.n_pred_timesteps
        coeffs = {
            'legibility': 100.0,
            'regularize': [1.0 for _ in range(num_timesteps - 1)],
            "collision": dict(cost=[20], dist_pen=[0.025]),
        }
        eef_traj = self.solve_traj(
            init_joint, final_joint, coeffs=coeffs, plot=plot, save='trajectories/legibility.txt')
        return eef_traj

    def nominal_trajectory_test(self, init_joint, final_joint, plot=''):
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
        eef_traj = self.solve_traj(
            init_joint, final_joint, coeffs=coeffs, plot=plot, save='trajectories/nominal.txt')

        return eef_traj