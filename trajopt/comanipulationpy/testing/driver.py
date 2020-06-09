from trajectory_framework import TrajectoryFramework

if __name__ == "__main__":
    joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    joint_target = [-0.21084585626752084, 1.696737816218337, -2.565970219832999, 0.17682048063096367, 2.5144914879697158, 1.2588615840260928, -0.1733579520497237]

    framework = TrajectoryFramework('iiwa', '')
    framework.scene.robot.SetDOFValues(joint_start, framework.scene.manipulator.GetArmIndices())

    framework.setup_test(joint_start, joint_target, traj_num=124, execute=True)