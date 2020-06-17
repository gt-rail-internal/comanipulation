from trajectory_framework import TrajectoryFramework
from tests import Test
from metrics import print_metrics
    

if __name__ == "__main__":
    ################## PARAMETERS ################
    #iiwa
    joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    joint_target = [0.526845265542, 1.696737816218337, 2.565970219832999, 0.17682048063096367, 2.5144914879697158, 1.2588615840260928, -0.1733579520497237]
    
    #jaco
    # joint_start = [3.941421366763722, 2.840465567025116, 0.0016481772134505363, 0.7576862412004652, -1.6470106708707843, 4.495901148004366, -1.2516118096169921]
    # joint_target = [4.871800476914653, 1.895875884203746, 4.728695515739245, 1.2668175273349631, 4.713923493804794, 4.641769937759059, 5.034508434241916]
    traj_num = 303
    enable_estop = True
    resume_safely = False
    execute_comanipulation = True
    execute_baseline = False
    robot = 'iiwa'
    ###############################################

    framework = TrajectoryFramework(robot, '', enable_estop=enable_estop, resume_safely=resume_safely)
    framework.scene.robot.SetDOFValues(joint_start, framework.scene.manipulator.GetArmIndices())

    comanipulationMetrics = framework.setup_test(joint_start, joint_target, traj_num=traj_num, execute=execute_comanipulation)
    baselineTest = Test(robot,joint_start, joint_target, traj_num=traj_num, execute=execute_baseline, enable_estop=enable_estop, resume_safely=resume_safely)
    baselineMetrics = baselineTest.run_all_baselines()
    print_metrics(comanipulationMetrics, baselineMetrics)