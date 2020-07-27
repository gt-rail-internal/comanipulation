'''
Visibility C++ Baseline Implementation

VectorXd VisibilityBaselineCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;
  //VectorXd p_obj = (1, 1, 1);
  //VectorXd p_head = (0, 0, 0);
  VectorXd vec_head_obj = head_pos_ - obj_pos_;

  double err_ = 0.0;

  for (int i = 0; i < num_timesteps; i++) {
    manip_->SetDOFValues(toDblVec(dof_vals.segment(i * 7, 7)));
    VectorXd p_eef_t_ = toVector3d(link_->GetTransform().trans);
    VectorXd vec_head_eef = head_pos_ - p_eef_t_;

    err_ += acos(vec_head_obj.dot(vec_head_eef)/(vec_head_obj.norm() * vec_head_eef.norm()));
  }

  VectorXd err(1);
  err << err_;
  return err;

}
'''

import numpy as np
from math import exp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visibilityBaselineCostCalculator(human_head, object_pos, end_effector_pos):

    cost = 0
    vec_head_obj = np.array(human_head - object_pos)
    vec_head_eef = np.array(human_head - end_effector_pos)
    dot_prod_gaze_eef = np.dot(vec_head_obj, vec_head_eef)
    magnitute_of_vecs = np.linalg.norm(vec_head_obj) * np.linalg.norm(vec_head_eef)
    cost = np.arccos(dot_prod_gaze_eef / magnitute_of_vecs)
    # print(end_effector_pos, cost)

    return cost

def plotCostmap(human_head, object_pos):
    
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
                end_effector_pos = np.array([curr_x, curr_y, curr_z])
                curr_cost = visibilityBaselineCostCalculator(human_head, object_pos, end_effector_pos)
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
OBJECT_POS = np.array([0, 0.2, 0.83])

plotCostmap(HUMAN_HEAD, OBJECT_POS)