import yaml
import numpy as np
from spider_plot_data import make_spider_plot 

from tests import Test
from metrics import print_metrics, metrics_to_csv, save_experiments
from trajectory_framework import TrajectoryFramework

def analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, execute_baseline, plot_baselines,
                                num_baselines, num_metrics, potential_goal_states):
    comanipulationFramework = TrajectoryFramework(robot, '')
    comanipulationFramework.scene.robot.SetDOFValues(joint_start, comanipulationFramework.scene.manipulator.GetArmIndices())

    all_comanipulation_metrics = np.zeros((num_metrics, len(trajectories)))
    all_baseline_metrics = np.zeros((num_baselines, num_metrics, len(trajectories)))

    for trajIndex, trajectory in enumerate(trajectories):
        all_comanipulation_metrics[:, trajIndex] = comanipulationFramework.setup_test(joint_start, joint_target, potential_goal_states,  traj_num=trajectory, execute=execute_comanipulation)
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
    ''' TRAJECTORIES FOR REAL JACO
        REACHING NEAR => 3008
        REACHING FAR => 4005
        STANDING STILL => 520
        [520, 524, 544, 604, 640] --> Stationary case
    '''

    robot_name = 'jaco' #'iiwa'
    with open(r'./config/{}.yaml'.format(robot_name)) as f
        config = yaml.load(f)
    
    robot = config['robot']
    trajectories = config['trajectories']
    joint_start = config['joint_start']
    joint_target = config['joint_target']

    legibility_goal_state_1 = [2.06554427935604, 2.422036354454592, 3.027546242016567, 0.873941944634994, 4.640153398977381, 3.712183705570867, 5.029900100291643]
    legibility_goal_state_2 = [1.7155479204723005, 2.4311550180270234, 3.0275291977855905, 0.8739619849221968, 4.640151268448508, 3.7121714550298526, 5.029900100291643]
    potential_goal_states = [legibility_goal_state_1, legibility_goal_state_2]

    execute_comanipulation = True
    execute_baseline = False
    plot_baselines = False

    num_baselines = 4
    num_metrics = 4
    ###############################################
    analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, 
        execute_baseline, plot_baselines, num_baselines, num_metrics, potential_goal_states)  
