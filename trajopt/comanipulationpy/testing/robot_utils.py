def get_eef_position(dof_vals):
    self.robot.SetDOFValues(dof_vals, self.manipulator.GetArmIndices())
    posevec = openravepy.poseFromMatrix(self.eef_link.GetTransform())
    position = posevec[4:7]
    pos = np.array([position[0], position[1], position[2]])
    return pos


def follow_trajectory(traj):
    num_timesteps = traj.shape[0]
    eef_traj = np.zeros((num_timesteps, 3))
    for i in range(num_timesteps):
        p_eef_t = self.get_eef_position(traj[i])
        for j in range(3):
            eef_traj[i, j] = p_eef_t[j]
    return eef_traj


def execute_trajectory(traj):
    print("Executing trajectory!")
    raw_input("Ready to move to initial position")
    # self.follow_joint_trajectory_client.move_to(traj[0], duration=1)
    self.follow_joint_trajectory_client.follow_trajectory(
        [traj[0], traj[0]], duration=0.5)
    raw_input("Ready for Gazebo execution?")
    self.follow_joint_trajectory_client.follow_trajectory(traj, duration=0.5)


def execute_full_trajectory(traj, human_traj, obs_traj_len, full_human_traj_len):
    # if self.use_ros and not self.use_jaco:
    self.follow_joint_trajectory_client.follow_trajectory(
        [traj[0], traj[0]], duration=0.5)
    raw_input("Ready for Gazebo execution")
    full_human_traj = create_human_trajectory_tree(human_traj)
    self.follow_joint_trajectory_client.execute_full_trajectory(
        traj, 0.1, 0.01, obs_traj_len, full_human_traj_len - obs_traj_len, len(traj), full_human_traj)


# use openrave's FK module to get robot_joints in cartesian space rather than joint space
def performFK(robot_joints):
    self.robot.SetDOFValues(robot_joints, self.manipulator.GetArmIndices())

    robot_joint_cart_pos = np.zeros((len(self.all_links), 3))
    for i in range(len(self.all_links)):
        link = self.all_links[i]
        posevec = openravepy.poseFromMatrix(self.robot.GetLink(link).GetTransform())
        position = posevec[4:7]
        robot_joint_cart_pos[i] = np.array([position[0], position[1], position[2]])
    return robot_joint_cart_pos
