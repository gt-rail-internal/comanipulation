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




def setup_test_without_ros(self, init_joint, final_joint, traj_num = 303, exec_traj = False):

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

def add_collision_objects():
    from octomap_trajopt_bridge.msg import octomap_custom
    import rospy

    octomap_message = rospy.wait_for_message("/octomap_decrypted", octomap_custom)

    octomap_array = np.zeros((octomap_message.num_vox, 6))
    for i in range(octomap_message.num_vox):
        octomap_array[i] = [octomap_message.x[i], octomap_message.y[i],
                            octomap_message.z[i], 0.025, 0.025, 0.025]

    self.env.Remove(self.body)

    self.body.InitFromBoxes(octomap_array)

    self.env.Add(self.body)