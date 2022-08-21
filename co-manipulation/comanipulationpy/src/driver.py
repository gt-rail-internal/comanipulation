from trajectory_framework import TrajectoryFramework
from tests import Test
from metrics import print_metrics, metrics_to_csv, save_experiments
import numpy as np
from spider_plot_data import make_spider_plot 

# -1.3542 , -0.8448
# 0.38719, 0.71919 
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

def run_single_test(start, target, potential_goal_states, name='iiwa', traj=104):
    comanipulationFramework = TrajectoryFramework(name, '')
    comanipulationFramework.scene.robot.SetDOFValues(joint_start, comanipulationFramework.scene.manipulator.GetArmIndices())
    comanipulationFramework.setup_test(start, target, potential_goal_states, traj_num=traj, execute=True)
    

if __name__ == "__main__":

    ################## PARAMETERS ################
    #iiwa
    joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    #joint_target = [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    #jaco
    #joint_start = [4.93928, 2.8396507, 6.28319, 0.76043996, 4.628979695, 4.494728969, 5.028974253]
    #joint_target = [5.046514812, 4.111088146,6.819123561, 0.9933942863, 4.413815505, 4.151859038, 5.491224706]

    #Alternate Start and End
    #joint_start = [3.298604109345377, 3.9015456443520793, 6.929810939862968, 0.9327826247075317, 3.1043060014837027, 4.336929872002593, 5.031395198927613] #from right side
    #joint_start =  [6.275706085574208, 3.5177979797093295, 6.284589858338494, 1.2498775591151325, 4.631694666723071, 4.492623861966915, 5.031556586489672]
    '''
    TRAJECTORIES FOR REAL JACO
    REACHING NEAR => 3008
    REACHING FAR => 4005
    STANDING STILL => 520

    '''

    legibility_goal_state_1 = [2.06554427935604, 2.422036354454592, 3.027546242016567, 0.873941944634994, 4.640153398977381, 3.712183705570867, 5.029900100291643]
    legibility_goal_state_2 = [1.7155479204723005, 2.4311550180270234, 3.0275291977855905, 0.8739619849221968, 4.640151268448508, 3.7121714550298526, 5.029900100291643]
    potential_goal_states = [legibility_goal_state_1, legibility_goal_state_2]
    trajectories = [100012]
    #trajectories = [520, 524, 544, 604, 640] #Stationary case
    execute_comanipulation = False
    execute_baseline = False
    plot_baselines = False
    robot =    'iiwa' #'iiwa' # 
    num_baselines = 4
    num_metrics = 4
    ###############################################

    #joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    run_single_test(joint_start, joint_target, potential_goal_states)
    # analyze_multiple_trajectories(trajectories, joint_start, joint_target, execute_comanipulation, 
    #     execute_baseline, plot_baselines, num_baselines, num_metrics, potential_goal_states)  
