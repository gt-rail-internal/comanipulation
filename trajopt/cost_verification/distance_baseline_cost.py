# C++ Implementation for Reference
#
# VectorXd DistanceBaselineCostCalculator::operator()(const VectorXd& dof_vals) const {
#
#   int num_timesteps = dof_vals.size() / 7;
#   double err = 0.0;
#   // VectorXd err_vec(num_timesteps);
#
#   for (int i = 0; i < num_timesteps; i++) {
#
#     manip_->SetDOFValues(toDblVec(dof_vals.segment(i * 7, 7)));
#     double curr_head_cost = 0;
#     double curr_torso_cost = 0;
#
#     for (int i = 0; i < links_.size(); i++) {
#
#       VectorXd curr_joints = toVector3d(links_.at(i)->GetTransform().trans);
#       VectorXd vec_head_eef = head_pos_ - curr_joints;
#       curr_head_cost = max(curr_head_cost, exp(-vec_head_eef.norm()));
#
#       for (int j = 0; j < 20; j++) {
#
#         VectorXd torso_feet_sample = torso_pos_ + (j / 19) * (feet_pos_ - torso_pos_);
#         double torso_joint_sample_dist = 2 * (torso_feet_sample - curr_joints).norm();
#         curr_torso_cost = max(curr_torso_cost, exp(-torso_joint_sample_dist));
#
#       }
#     }
#
#     err += max(curr_head_cost, curr_torso_cost);
#     if (isnan(max(curr_head_cost, curr_torso_cost))) {
#       std::cout << "Error is NaN" << std::endl;
#       std::cout << curr_head_cost << std::endl;
#       std::cout << curr_torso_cost << std::endl;
#     }
#
#   }
#   VectorXd err_vec(1);
#   err_vec << err;
#   return err_vec;
#
# }

import numpy as np
from math import exp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def distanceBaselineCostCalculator(human_head, human_torso, human_feet, robot_loc):

    curr_head_cost = 0
    curr_torso_cost = 0

    vec_head_joint = human_head - robot_loc
    curr_head_cost = exp(-1 * np.linalg.norm(vec_head_joint))

    for i in range(20):
        torso_feet_sample = human_torso + (float(i)/19) * (human_feet - human_torso)
        torso_distance = np.linalg.norm(4 * (torso_feet_sample - robot_loc))
        curr_torso_cost = max(curr_torso_cost, exp(-1 * torso_distance))
    
    return max(curr_head_cost, curr_torso_cost)


def plotCostmap(human_head, human_torso, human_feet):
    
    x_range = [-0.5, 0.5, 0.1]
    y_range = [-0.5, 0.5, 0.1]
    z_range = [0, 2.0, 0.1]

    x_coord = []
    y_coord = []
    z_coord = []
    costs  = []

    for curr_x in np.arange(x_range[0], x_range[1], x_range[2]):
        for curr_y in np.arange(y_range[0], y_range[1], y_range[2]):
            for curr_z in np.arange(z_range[0], z_range[1], z_range[2]):
                robot_loc = np.array([curr_x, curr_y, curr_z])
                curr_cost = distanceBaselineCostCalculator(human_head, human_torso, human_feet, robot_loc)
                x_coord.append(curr_x)
                y_coord.append(curr_y)
                z_coord.append(curr_z)
                costs.append(-1 * curr_cost)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    cmhot = plt.get_cmap("RdYlGn")
    cax = ax.scatter(x_coord, y_coord, z_coord, s=50, c=costs, cmap=cmhot)

    plt.show()


HUMAN_HEAD = np.array([0, 0, 1.8])
HUMAN_TORSO = np.array([0, 0, 1.37])
HUMAN_FEET = np.array([0, 0, 0])

plotCostmap(HUMAN_HEAD, HUMAN_TORSO, HUMAN_FEET)