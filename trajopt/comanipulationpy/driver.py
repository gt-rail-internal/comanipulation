from trajectory_framework import TrajectoryFramework
from tests import Test
from metrics import print_metrics
import numpy as np


def analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, execute_baseline, 
                                enable_estop, resume_safely, collision_threshold, num_baselines, num_metrics):
    comanipulationFramework = TrajectoryFramework(robot, '', enable_estop=enable_estop, resume_safely=resume_safely, collision_threshold=collision_threshold)
    comanipulationFramework.scene.robot.SetDOFValues(joint_start, comanipulationFramework.scene.manipulator.GetArmIndices())

    all_comanipulation_metrics = np.zeros((num_metrics, len(trajectories)))
    all_baseline_metrics = np.zeros((num_baselines, num_metrics, len(trajectories)))

    for trajIndex, trajectory in enumerate(trajectories):
        all_comanipulation_metrics[:, trajIndex] = comanipulationFramework.setup_test(joint_start, joint_target, traj_num=trajectory, execute=execute_comanipulation)
        baselineTest = Test(robot,joint_start, joint_target, traj_num=trajectory, execute=execute_baseline, 
                        enable_estop=enable_estop, resume_safely=resume_safely, collision_threshold=collision_threshold)
        all_baseline_metrics[:, :, trajIndex] = baselineTest.run_all_baselines()
    print_metrics(all_comanipulation_metrics, all_baseline_metrics)

    

if __name__ == "__main__":

    ################## PARAMETERS ################
    #iiwa
    joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    joint_target = [-0.21084585626752084, 1.696737816218337, -2.565970219832999, 0.17682048063096367, 2.5144914879697158, 1.2588615840260928, -0.1733579520497237]

    #jaco
    # joint_start = [3.941421366763722, 2.840465567025116, 0.0016481772134505363, 0.7576862412004652, -1.6470106708707843, 4.495901148004366, -1.2516118096169921]
    # joint_target = [4.871800476914653, 1.895875884203746, 4.728695515739245, 1.2668175273349631, 4.713923493804794, 4.641769937759059, 5.034508434241916]
    trajectories = [120, 303]
    enable_estop = False
    resume_safely = False
    execute_comanipulation = True
    execute_baseline = False
    robot = 'iiwa'
    collision_threshold = 0.25
    num_baselines = 4
    num_metrics = 4
    ###############################################
    analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, execute_baseline, enable_estop, 
                                resume_safely, collision_threshold, num_baselines, num_metrics)
