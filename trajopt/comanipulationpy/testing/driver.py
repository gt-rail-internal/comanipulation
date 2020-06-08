from trajectory_framework import TrajectoryFramework

if __name__ == "__main__":
    joint_start = [0.19870164472412005, 3.6537395701846425, 5.4752445504190534, 1.3257644675349949, 0.28113437007680186, 4.046902840861367, 4.379301031261863]
    joint_target = [0.03038485167871689, 3.398900886526633, 5.683668331018617, 0.7373113943856778, 0.007464753150518836, 3.595717812147304, 4.520514082798879]

    framework = TrajectoryFramework('iiwa', '')
    framework.trajectory_solver.load_traj_file(303)
    framework.scene.robot.SetDOFValues(joint_start, framework.scene.manipulator.GetArmIndices())

    framework.setup_test(joint_start, joint_target, execute=True)