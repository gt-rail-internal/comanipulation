from trajectory_framework import TrajectoryFramework
from tests import Test
from metrics import print_metrics, metrics_to_csv, save_experiments
import numpy as np
from spider_plot_data import make_spider_plot 


def analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, execute_baseline, plot_baselines,
                                enable_estop, resume_safely, collision_threshold, num_baselines, num_metrics):
    comanipulationFramework = TrajectoryFramework(robot, '', enable_estop=enable_estop, resume_safely=resume_safely, collision_threshold=collision_threshold)
    comanipulationFramework.scene.robot.SetDOFValues(joint_start, comanipulationFramework.scene.manipulator.GetArmIndices())

    all_comanipulation_metrics = np.zeros((num_metrics, len(trajectories)))
    all_baseline_metrics = np.zeros((num_baselines, num_metrics, len(trajectories)))

    for trajIndex, trajectory in enumerate(trajectories):
        all_comanipulation_metrics[:, trajIndex] = comanipulationFramework.setup_test(joint_start, joint_target, traj_num=trajectory, execute=execute_comanipulation)
        baselineTest = Test(robot,joint_start, joint_target, traj_num=trajectory, execute=execute_baseline)
        all_baseline_metrics[:, :, trajIndex] = baselineTest.run_all_baselines()
        metrics_to_csv(trajectory, all_comanipulation_metrics[:, trajIndex], all_baseline_metrics[:, :, trajIndex])
    print_metrics(all_comanipulation_metrics, all_baseline_metrics)
    test_case = ''.join(str(test) + ", " for test in trajectories)
    save_experiments(test_case[:-2], all_comanipulation_metrics, all_baseline_metrics)
    if plot_baselines:
        make_spider_plot()

def run_single_test(start, target, name='jaco', traj=303):
    comanipulationFramework = TrajectoryFramework(name, '')
    comanipulationFramework.scene.robot.SetDOFValues(joint_start, comanipulationFramework.scene.manipulator.GetArmIndices())
    comanipulationFramework.setup_test(start, target, traj_num=traj, execute=True)
    

if __name__ == "__main__":

    ################## PARAMETERS ################
    #iiwa
    joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    joint_target = [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    #jaco
    # joint_start = [3.941421366763722, 2.840465567025116, 0.0016481772134505363, 0.7576862412004652, -1.6470106708707843, 4.495901148004366, -1.2516118096169921]
    # joint_target = [4.871800476914653, 1.895875884203746, 4.728695515739245, 1.2668175273349631, 4.713923493804794, 4.641769937759059, 5.034508434241916]
    trajectories = [120, 124, 144, 204, 240]
    # trajectories = [520, 524, 544, 604, 640] #Stationary case
    enable_estop = False
    resume_safely = False
    execute_comanipulation = False
    execute_baseline = False
    plot_baselines = True
    robot = 'iiwa'
    collision_threshold = 0.25
    num_baselines = 4
    num_metrics = 4
    ###############################################
    analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, execute_baseline, plot_baselines, enable_estop, 
                                resume_safely, collision_threshold, num_baselines, num_metrics)
