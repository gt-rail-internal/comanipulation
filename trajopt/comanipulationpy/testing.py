from testing_framework import TestingFramework
import numpy as np


def run_test(test_framework):

    test_case_1_trajs = [120, 124, 131, 144, 165, 204, 221, 240, 269, 274, 276, 281, 303]
    
    test_case_1_start_joints = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    
    test_case_1_end_joints = [-0.21084585626752084, 1.696737816218337, -2.565970219832999, 0.17682048063096367, 2.5144914879697158, 1.2588615840260928, -0.1733579520497237]
    
    if test_framework.use_ros:
        result = test_framework.setup_test(test_case_1_start_joints, test_case_1_end_joints)
        print(result)

    else:
        num_test_case_1_tests = len(test_case_1_trajs)

        our_metrics = np.ndarray((num_test_case_1_tests, 4))
        all_baseline_metrics = np.ndarray((num_test_case_1_tests, 4, 4))

        for i in range(num_test_case_1_tests):
            metrics = test_framework.setup_test_without_ros(test_case_1_start_joints, test_case_1_end_joints, test_case_1_trajs[i], False)
            for j in range(4):
                our_metrics[i, j] = metrics[j]

            baseline_metrics = test_framework.test_baselines(test_case_1_start_joints, test_case_1_end_joints, test_case_1_trajs[i], False)
            for j in range(4):
                for k in range(4):
                    all_baseline_metrics[i, j, k] = baseline_metrics[j][k]
            # raw_input("Next traj")

        print_metrics(our_metrics, all_baseline_metrics, 1)




    # test_case_2_trajs = [520, 524, 531, 544, 565, 604, 621, 640, 669, 674, 676, 681, 703]

    # test_case_2_start_joints = [[-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067] for i in range (len(test_case_2_trajs))]

    # test_case_2_end_joints = [[-0.21084585626752084, 1.696737816218337, -2.565970219832999, 0.17682048063096367, 2.5144914879697158, 1.2588615840260928, -0.1733579520497237] for i in range (len(test_case_2_trajs))]

    # num_test_case_2_tests = len(test_case_2_trajs)

    # our_metrics = np.ndarray((num_test_case_2_tests, 4))
    # all_baseline_metrics = np.ndarray((num_test_case_2_tests, 4, 4))

    # for i in range(num_test_case_2_tests):
    #     metrics = test_framework.setup_test(test_case_2_start_joints[i], test_case_2_end_joints[i], test_case_2_trajs[i], False)
    #     for j in range(4):
    #         our_metrics[i, j] = metrics[j]

    #     baseline_metrics = test_framework.test_baselines(test_case_2_start_joints[i], test_case_2_end_joints[i], test_case_2_trajs[i], False)
    #     for j in range(4):
    #         for k in range(4):
    #             all_baseline_metrics[i, j, k] = baseline_metrics[j][k]
    #     # raw_input("Next traj")

    # print_metrics(our_metrics, all_baseline_metrics, 2)




    # test_case_3_trajs = [120, 124, 131, 144, 165, 204, 221, 240, 269, 274, 276, 281, 303]
    # test_case_3_trajs = [303]

    # test_case_3_start_joints = [[-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067] for i in range (len(test_case_3_trajs))]

    # test_case_3_end_joints = [[0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] for i in range (len(test_case_3_trajs))]

    # num_test_case_3_tests = len(test_case_3_trajs)

    # our_metrics = np.ndarray((num_test_case_3_tests, 4))
    # all_baseline_metrics = np.ndarray((num_test_case_3_tests, 4, 4))

    # for i in range(num_test_case_3_tests):
    #     metrics = test_framework.setup_test(test_case_3_start_joints[i], test_case_3_end_joints[i], test_case_3_trajs[i], False)
    #     for j in range(4):
    #         our_metrics[i, j] = metrics[j]

    #     baseline_metrics = test_framework.test_baselines(test_case_3_start_joints[i], test_case_3_end_joints[i], test_case_3_trajs[i], False)
    #     for j in range(4):
    #         for k in range(4):
    #             all_baseline_metrics[i, j, k] = baseline_metrics[j][k]
    #     # raw_input("Next traj")

    # print_metrics(our_metrics, all_baseline_metrics, 2)

    return False

def print_metrics(our_metrics, baseline_metrics, test_case):
    print("Metrics for test case " + str(test_case))

    print("\n\nOur Metrics: \n")
    print("Distance metric : \t" + str(np.mean(our_metrics[:,0])) + " +/- " + str(np.std(our_metrics[:, 0])))
    print("Visibility metric: \t" + str(np.mean(our_metrics[:,1])) + " +/- " + str(np.std(our_metrics[:, 1])))
    print("Legibility metric: \t" + str(np.mean(our_metrics[:,2])) + " +/- " + str(np.std(our_metrics[:, 2])))
    print("Nominal traj metric: \t" + str(np.mean(our_metrics[:,3])) + " +/- " + str(np.std(our_metrics[:, 3])))
    np.savetxt('metrics/our_metrics_' + str(test_case) + '.txt', our_metrics, delimiter=',')

    print("\n\nNominal Trajectory Baseline: \n")
    print("Distance metric : \t" + str(np.mean(baseline_metrics[:,0,0])) + " +/- " + str(np.std(baseline_metrics[:,0, 0])))
    print("Visibility metric: \t" + str(np.mean(baseline_metrics[:,0,1])) + " +/- " + str(np.std(baseline_metrics[:,0, 1])))
    print("Legibility metric: \t" + str(np.mean(baseline_metrics[:,0,2])) + " +/- " + str(np.std(baseline_metrics[:,0, 2])))
    print("Nominal traj metric: \t" + str(np.mean(baseline_metrics[:,0,3])) + " +/- " + str(np.std(baseline_metrics[:,0, 3])))
    np.savetxt('metrics/nominal_baseline_' + str(test_case) + '.txt', baseline_metrics[:, 0, :], delimiter=',')

    print("\n\nDistance + Visibility Baseline: \n")
    print("Distance metric : \t" + str(np.mean(baseline_metrics[:,1,0])) + " +/- " + str(np.std(baseline_metrics[:,1, 0])))
    print("Visibility metric: \t" + str(np.mean(baseline_metrics[:,1,1])) + " +/- " + str(np.std(baseline_metrics[:,1, 1])))
    print("Legibility metric: \t" + str(np.mean(baseline_metrics[:,1,2])) + " +/- " + str(np.std(baseline_metrics[:,1, 2])))
    print("Nominal traj metric: \t" + str(np.mean(baseline_metrics[:,1,3])) + " +/- " + str(np.std(baseline_metrics[:,1, 3])))
    np.savetxt('metrics/dist_vis_baseline_' + str(test_case) + '.txt', baseline_metrics[:, 1, :], delimiter=',')

    print("\n\nLegibility Baseline: \n")
    print("Distance metric : \t" + str(np.mean(baseline_metrics[:,2,0])) + " +/- " + str(np.std(baseline_metrics[:,2, 0])))
    print("Visibility metric: \t" + str(np.mean(baseline_metrics[:,2,1])) + " +/- " + str(np.std(baseline_metrics[:,2, 1])))
    print("Legibility metric: \t" + str(np.mean(baseline_metrics[:,2,2])) + " +/- " + str(np.std(baseline_metrics[:,2, 2])))
    print("Nominal traj metric: \t" + str(np.mean(baseline_metrics[:,2,3])) + " +/- " + str(np.std(baseline_metrics[:,2, 3])))
    np.savetxt('metrics/legibility_baseline_' + str(test_case) + '.txt', baseline_metrics[:, 2, :], delimiter=',')

    print("\n\nSpeed Control Baseline: \n")
    print("Distance metric : \t" + str(np.mean(baseline_metrics[:,3,0])) + " +/- " + str(np.std(baseline_metrics[:,3, 0])))
    print("Visibility metric: \t" + str(np.mean(baseline_metrics[:,3,1])) + " +/- " + str(np.std(baseline_metrics[:,3, 1])))
    print("Legibility metric: \t" + str(np.mean(baseline_metrics[:,3,2])) + " +/- " + str(np.std(baseline_metrics[:,3, 2])))
    print("Nominal traj metric: \t" + str(np.mean(baseline_metrics[:,3,3])) + " +/- " + str(np.std(baseline_metrics[:,3, 3])))
    np.savetxt('metrics/speed_control_baseline_' + str(test_case) + '.txt', baseline_metrics[:, 3, :], delimiter=',')






if __name__ == '__main__':

    test_framework = TestingFramework(True, False, True, False)


    # joint_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # joint_start =  [-0.453, 1.270, 0.889, -0.964, -0.965, 1.937, -2.073]
    # joint_start = [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    joint_start = [0.19870164472412005, 3.6537395701846425, 5.4752445504190534, 1.3257644675349949, 0.28113437007680186, 4.046902840861367, 4.379301031261863]
    test_framework.robot.SetDOFValues(joint_start, test_framework.manipulator.GetArmIndices())

    # joint_target = [1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    # joint_target =  [-0.006, 0.200, 0.076, -2.573, -0.045, 2.781, -0.676]
    # joint_target = [-0.4122322680453383, 1.2116827308773201, 0.7828531390874005, -0.9880801179338503, -0.7031814420527613, 1.631669171511069, 0.25310638721127177]
    # test_framework.robot.SetDOFValues(joint_target, test_framework.manipulator.GetArmIndices())
    joint_target = [0.03038485167871689, 3.398900886526633, 5.683668331018617, 0.7373113943856778, 0.007464753150518836, 3.595717812147304, 4.520514082798879]

    # test_framework.setup_test(joint_start, joint_target, 87)
    # test_framework.legibility_test(joint_start, joint_target)

    # test_framework.visualize_all_traj()
    # test_framework.visualize_traj_pred()

    # print("Optimizing and following distance trajectory...")
    # dist_eef_traj = test_framework.distance_test(joint_start, joint_target)

    # print("Optimizing and following velocity trajectory...")
    # vel_eef_traj = test_framework.velocity_test(joint_start, joint_target)

    # print("Optimizing and following visibility trajectory...")
    # vis_eef_traj = test_framework.visibility_test(joint_start, joint_target)

    # print("Optimizing and following legibility trajectory...")
    # leg_eef_traj = test_framework.legibility_test(joint_start, joint_target)

    # print("Optimizing and following optimal trajectory...")
    # opt_eef_traj = test_framework.optimal_trajectory_test(joint_start, xyz_target, joint_target, straight_eef_traj)
    
    # test_framework.still_trajectories_wrapper()
    # test_framework.prep_data_for_prediction()
    
    run_test(test_framework)