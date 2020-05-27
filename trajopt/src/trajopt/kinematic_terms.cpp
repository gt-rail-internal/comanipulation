#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/eigen_slicing.hpp"
#include "utils/logging.hpp"
#include "utils/stl_to_string.hpp"
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;


namespace {
  
#if 0
Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
#endif
inline Vector3d rotVec(const OpenRAVE::Vector& q) {
  return Vector3d(q[1], q[2], q[3]);
}

#if 0
VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}
#endif

}

namespace trajopt {


// CostPtr ConstructCost(VectorOfVectorPtr err_calc, const VarVector& vars, const VectorXd& coeffs, PenaltyType type, const string& name) {
//   return CostPtr(new CostFromErrFunc(err_calc), vars, coeffs, type, name);
// }
  
  
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const {
  manip_->SetDOFValues(toDblVec(dof_vals));
  OR::Transform newpose = link_->GetTransform();

  OR::Transform pose_err = pose_inv_ * newpose;
  VectorXd err = concat(rotVec(pose_err.rot), toVector3d(pose_err.trans));
  return err;  
}

#if 0
CartPoseCost::CartPoseCost(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs) :
    CostFromErrFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)), vars, coeffs, ABS, "CartPose")
{}
CartPoseConstraint::CartPoseConstraint(const VarVector& vars, const OR::Transform& pose,
    RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs) :
    ConstraintFromFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)), vars, coeffs, EQ, "CartPose")
{}
#endif

void CartPoseErrorPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(m_calc.get());
  DblVec dof_vals = getDblVec(x, m_vars);
  calc->manip_->SetDOFValues(dof_vals);
  OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
  PlotAxes(env, cur, .05,  handles);
  PlotAxes(env, target, .05,  handles);
  handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
}


#if 0
struct CartPositionErrCalculator {
  Vector3d pt_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  CartPositionErrCalculator(const Vector3d& pt_world, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link) :
  pt_world_(pt_world),
  manip_(manip),
  link_(link)
  {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return pt_world_ - toVector3d(newpose.trans);
  }
};
#endif

MatrixXd CartVelJacCalculator::operator()(const VectorXd& dof_vals) const {
  int n_dof = manip_->GetDOF();
  MatrixXd out(6, 2*n_dof);
  manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
  OR::Transform pose0 = link_->GetTransform();
  MatrixXd jac0 = manip_->PositionJacobian(link_->GetIndex(), pose0.trans);
  manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
  OR::Transform pose1 = link_->GetTransform();
  MatrixXd jac1 = manip_->PositionJacobian(link_->GetIndex(), pose1.trans);
  out.block(0,0,3,n_dof) = -jac0;
  out.block(0,n_dof,3,n_dof) = jac1;
  out.block(3,0,3,n_dof) = jac0;
  out.block(3,n_dof,3,n_dof) = -jac1;
  return out;
}

VectorXd CartVelCalculator::operator()(const VectorXd& dof_vals) const {
  int n_dof = manip_->GetDOF();
  manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
  OR::Transform pose0 = link_->GetTransform();
  manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
  OR::Transform pose1 = link_->GetTransform();
  VectorXd out(6);
  out.topRows(3) = toVector3d(pose1.trans - pose0.trans - OR::Vector(limit_,limit_,limit_));
  out.bottomRows(3) = toVector3d( - pose1.trans + pose0.trans - OR::Vector(limit_, limit_, limit_));
  return out;
}


#if 0
CartVelConstraint::CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit) :
        ConstraintFromFunc(VectorOfVectorPtr(new CartVelCalculator(manip, link, distlimit)),
             MatrixOfVectorPtr(new CartVelJacCalculator(manip, link, distlimit)), concat(step0vars, step1vars), VectorXd::Ones(0), INEQ, "CartVel") 
{} // TODO coeffs
#endif

#if 0
struct UpErrorCalculator {
  Vector3d dir_local_;
  Vector3d goal_dir_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  MatrixXd perp_basis_; // 2x3 matrix perpendicular to goal_dir_world
  UpErrorCalculator(const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    dir_local_(dir_local),
    goal_dir_world_(goal_dir_world),
    manip_(manip),
    link_(link)
  {
    Vector3d perp0 = goal_dir_world_.cross(Vector3d::Random()).normalized();
    Vector3d perp1 = goal_dir_world_.cross(perp0);
    perp_basis_.resize(2,3);
    perp_basis_.row(0) = perp0.transpose();
    perp_basis_.row(1) = perp1.transpose();
  }
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return perp_basis_*(toRot(newpose.rot) * dir_local_ - goal_dir_world_);
  }
};
#endif




//////////////////////////////////
// Co-Manipulatin Costs
//////////////////////////////////

// Distance Cost
VectorXd DistanceCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  // MatrixXd distances_costs(num_timesteps, 7 * n_human_joints_);
  VectorXd costs_vals(num_timesteps);

  double err_ = 0.0;

  // for each timestep
  for (int t = 0; t < num_timesteps; t++) {

    costs_vals(t) = 0;
    // Set robot to pose for timestep
    manip_->SetDOFValues(toDblVec(dof_vals.segment(t * 7, 7)));

    // For each robot joint
    for (int i = 0; i < links_.size(); i++) {
      // Get 3D position of robot joint
      KinBody::LinkPtr link = links_.at(i);
      Vector3d curr_joint_pos = toVector3d(link->GetTransform().trans);

      // For each human joint
      for (int j = 0; j < n_human_joints_; j++) {

        // Get distance of robot joint with human joint's expected value
        Vector3d dist = (human_poses_mean_.at(t * n_human_joints_ + j) - curr_joint_pos).cwiseAbs();

        // Calculate distance cost as quadratic
        double dist_cost_inv = dist.transpose() * human_poses_var_.at(t * n_human_joints_ + j).inverse() * dist;
        double dist_cost_quad = 1.0 / dist_cost_inv;
        
        costs_vals(t) = dist_cost_quad;
      }
    }
  }
  return costs_vals;
}


// Velocity Cost
VectorXd VelocityCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;
  //VectorXd p_obj = (1, 1, 1);
  //VectorXd p_head = (0, 0, 0);
  // VectorXd vec_head_obj = head_pos_ - obj_pos_;

  // MatrixXd distances_costs(num_timesteps, 7 * human_skeleton_size_);
  VectorXd costs_vals(num_timesteps - 1);

  double err_ = 0.0;
  double min_dist_cost = 0.0;

  // for each timestep
  for (int t = 0; t < num_timesteps - 1; t++) {

    costs_vals(t) = 0;
    int min_robot_joint = 0;

    // Set robot to pose for timestep
    manip_->SetDOFValues(toDblVec(dof_vals.segment(t * 7, 7)));

    // For each robot joint
    for (int i = 0; i < links_.size(); i++){
    // for (int i = 6; i < 7; i++){
      // Get 3D position of robot joint
      // TODO get list of joints...
      Vector3d curr_joint_pos = toVector3d(links_.at(i)->GetTransform().trans);

      // For each human joint
      for (int j = 0; j < n_human_joints_; j++) {

        // Get distance of robot joint with human joint's expected value
        Vector3d dist = (human_poses_mean_.at(t * n_human_joints_ + j) - curr_joint_pos).cwiseAbs();

        // Calculate distance cost as quadratic
        double dist_cost_inv = dist.transpose() * human_poses_var_.at(t * n_human_joints_ + j).inverse() * dist;
        double dist_cost_quad = 1.0 / dist_cost_inv;

        if (min_dist_cost > dist_cost_quad || min_dist_cost == 0) {
          min_dist_cost = dist_cost_quad;
          min_robot_joint = i;
        }

      }
    }
    // TODO cost = min_dist_vals * p_eef_t(t + 1) - p_eef_t(t)
    // Get closest joint position at t_min
    manip_->SetDOFValues(toDblVec(dof_vals.segment((t) * 7, 7)));
    // Vector3d p_minjoint_t = toVector3d(links_.at(min_robot_joint)->GetTransform().trans);
    // Vector3d p_minjoint_t = toVector3d(links_.at(6)->GetTransform().trans);

    std::vector<Vector3d> p_j_t;
    for (int i = 0; i < links_.size(); i++){
      p_j_t.push_back(toVector3d(links_.at(i)->GetTransform().trans));
    }
    
    // Set robot to pose at t_min + 1
    manip_->SetDOFValues(toDblVec(dof_vals.segment((t + 1) * 7, 7)));
    // Get closest joint position at t_min + 1
    // Vector3d p_minjoint_t1 = toVector3d(links_.at(min_robot_joint)->GetTransform().trans);
    // Vector3d p_minjoint_t1 = toVector3d(links_.at(6)->GetTransform().trans);
    
    std::vector<Vector3d> p_j_t1;
    for (int i = 0; i < links_.size(); i++){
      p_j_t1.push_back(toVector3d(links_.at(i)->GetTransform().trans));
    }
    for (int i = 0; i < links_.size(); i++) {
      costs_vals(t) += min_dist_cost * (p_j_t.at(i) - p_j_t1.at(i)).cwiseAbs().norm();
    }
    // std::cout << "Min distance joint for timestep " << t << ": " << min_robot_joint << std::endl;

    // Vector3d diff = (p_minjoint_t - p_minjoint_t1).cwiseAbs();
    // costs_vals(t) = min_dist_cost * diff.norm();
    

  }
  return costs_vals;
}


// Visibility Cost
VectorXd VisibilityCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  VectorXd err(num_timesteps);
  // double alpha = 0.5;

  // for each timestep
  for (int t = 0; t < num_timesteps; t++) {
    // Set arm to trajectory's i-th pose and get eef position in baselink frame
    manip_->SetDOFValues(toDblVec(dof_vals.segment(t * 7, 7)));
    Vector3d p_eef_t_ = toVector3d(link_->GetTransform().trans);

    // Get vectors from head to eef and head to obj
    Vector3d vec_head_eef = head_pos_mean_.at(t) - p_eef_t_;
    Vector3d vec_head_obj = head_pos_mean_.at(t) - obj_pos_;

    // Calculate angle between above two vectors and scale by variance
    err(t) = acos(vec_head_obj.dot(vec_head_eef) / (vec_head_obj.norm() * vec_head_eef.norm())) / head_pos_var_.at(t).norm();

    // // Elliptical FoV setup
    // // Transform EEF position to gaze frame...
    // Vector3d gaze_p_eef_t = (0, 0, 0);

    // // Generate projections of head to eef line on xy and xz planes
    // Vector3d gaze_p_eef_xy_proj = (gaze_p_eef_t.x, gaze_p_eef_t.y, 0);
    // Vector3d gaze_p_eef_xz_proj = (gaze_p_eef_t.x, 0, gaze_p_eef_t.z);

    // // Calculate horizontal (xy) and vertical (xz) angles
    // double horizontal_error = acos(vec_head_obj.dot(gaze_p_eef_xy_proj) / (vec_head_obj.norm() * gaze_p_eef_xy_proj.norm()));
    // double vertical_error = acos(vec_head_obj.dot(gaze_p_eef_xz_proj) / (vec_head_obj.norm() * gaze_p_eef_xz_proj.norm()));

    // // Scale and add errors, multiply by variance
    // err(t) = head_pos_var_.at(t).norm() * (alpha * horizontal_error + vertical_error);
  }

  return err;

}

// void VisibilityCostPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
//   VisibilityCostCalculator* calc = static_cast<VisibilityCostCalculator*>(m_calc.get());
//   DblVec dof_vals_all = getDblVec(x, m_vars);
//   DblVec::const_iterator first = dof_vals_all.begin() + 5 * 7;
//   DblVec::const_iterator last = dof_vals_all.begin() + 5 * 7 + 7;
//   DblVec dof_vals(first, last);
//   calc->manip_->SetDOFValues(dof_vals);
//   OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
//   PlotAxes(env, cur, .05,  handles);
//   PlotAxes(env, target, .05,  handles);
//   handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
// }


// Legibility Cost  
VectorXd LegibilityCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  // length of segment from t to t+1
  VectorXd d_eef_q(num_timesteps - 1);
  // total cost
  double err = 0;
  // scaling function
  VectorXd f_t(num_timesteps - 1);
  // regularizer constant
  double lambda = 1.0;

  // Get cost of optimal traj from Start to Goal of optimal traj
  manip_->SetDOFValues(toDblVec(dof_vals.segment((num_timesteps - 1) * 7, 7)));
  VectorXd p_eef_g = toVector3d(link_->GetTransform().trans);
  manip_->SetDOFValues(toDblVec(dof_vals.segment(0, 7)));
  VectorXd p_eef_s = toVector3d(link_->GetTransform().trans);
  double cstar_s_g = (p_eef_g - p_eef_s).norm();

  // For every timestep (upto T - 1)
  for (int t = 0; t < num_timesteps - 1; t++) {
    // Update scaling factor
    f_t(t) = 1.0;

    // Calulcate length of segment and store
    manip_->SetDOFValues(toDblVec(dof_vals.segment(t * 7, 7)));
    VectorXd p_eef_t = toVector3d(link_->GetTransform().trans);
    manip_->SetDOFValues(toDblVec(dof_vals.segment((t + 1) * 7, 7)));
    VectorXd p_eef_t1 = toVector3d(link_->GetTransform().trans);
    d_eef_q(t) = (p_eef_t1 - p_eef_t).norm();
    
    // Calculate cost of path till (t + 1)
    double c_s_q = d_eef_q.segment(0, t).sum();
    // Calculate cost of optimal path from (t + 1) to goal
    double cstar_q_g = (p_eef_g - p_eef_t1).norm();
    // Calculate probability of goal given path till (t + 1)
    double prob_g_given_q = exp(-c_s_q - cstar_q_g)/exp(-cstar_s_g);
    // accumulate error as product of prob and scaling function
    err += prob_g_given_q * f_t(t);
  }
  // divide error by sum of scaling function and subtract regularizer
  err = (err / f_t.sum()) - (lambda * d_eef_q.sum());

  // reformat error as vector
  VectorXd err_vec(1);
  err_vec << err;

  // std::cout << "Error: " << err << std::endl;
  return err_vec;
}

// void LegibilityCostPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
//   LegibilityCostCalculator* calc = static_cast<LegibilityCostCalculator*>(m_calc.get());
//   DblVec dof_vals_all = getDblVec(x, m_vars);
//   DblVec::const_iterator first = dof_vals_all.begin() + 5 * 7;
//   DblVec::const_iterator last = dof_vals_all.begin() + 5 * 7 + 7;
//   DblVec dof_vals(first, last);
//   calc->manip_->SetDOFValues(dof_vals);
//   OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
//   PlotAxes(env, cur, .05,  handles);
//   PlotAxes(env, target, .05,  handles);
//   handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
// }


//////////////////////
// END Costs
//////////////////////





////////////////////////////////////
// Co-Manipulation additional costs
////////////////////////////////////



// Regularize Cost  
VectorXd RegularizeCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  // length of segment from t to t+1
  VectorXd d_eef_q(num_timesteps - 1);

  // For every timestep (upto T - 1)
  for (int t = 0; t < num_timesteps - 1; t++) {
    // Calulcate length of segment and store
    manip_->SetDOFValues(toDblVec(dof_vals.segment(t * 7, 7)));
    VectorXd p_eef_t = toVector3d(link_->GetTransform().trans);
    manip_->SetDOFValues(toDblVec(dof_vals.segment((t + 1) * 7, 7)));
    VectorXd p_eef_t1 = toVector3d(link_->GetTransform().trans);
    d_eef_q(t) = (p_eef_t1 - p_eef_t).norm();
    
  }
  return d_eef_q;
}


// Smoothing Cost  
VectorXd SmoothingCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  // length of segment from t to t+1
  VectorXd d_xi_t(num_timesteps - 1);
  double err = 0;

  // For every timestep (upto T - 1)
  for (int t = 0; t < num_timesteps - 1; t++) {

    d_xi_t(t) = (dof_vals.segment(t * 7, 7) - dof_vals.segment((t + 1) * 7, 7)).squaredNorm();
    err += d_xi_t(t);
    
  }
  // reformat error as vector
  VectorXd err_vec(1);
  err_vec << err;

  // std::cout << "Error: " << err << std::endl;
  return err_vec;
}


// Smoothing (acceleration) Cost  
VectorXd SmoothingAccelCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;

  // length of segment from t to t+1
  std::vector<VectorXd> d_xi_t;
  VectorXd dd_xi_t(num_timesteps -2);
  double err = 0;

  // For every timestep (upto T - 1), calculate first derivative
  for (int t = 0; t < num_timesteps - 1; t++) {
    d_xi_t.push_back(dof_vals.segment(t * 7, 7) - dof_vals.segment((t + 1) * 7, 7));
  }
  // Calculate second derivative
  for (int t = 0; t < num_timesteps - 2; t++) {
    dd_xi_t(t) = (d_xi_t.at(t + 1) - d_xi_t.at(t)).squaredNorm();
  }
  err = dd_xi_t.sum();
  // reformat error as vector
  VectorXd err_vec(1);
  err_vec << err;

  return err_vec;
}


//////////////////////
// END additional Costs
//////////////////////

////////////////////////////////////
// Co-Manipulation baseline costs
////////////////////////////////////

// Distance baseline cost
VectorXd DistanceBaselineCostCalculator::operator()(const VectorXd& dof_vals) const {

  int num_timesteps = dof_vals.size() / 7;
  double err = 0.0;
  // VectorXd err_vec(num_timesteps);

  for (int i = 0; i < num_timesteps; i++) {
    manip_->SetDOFValues(toDblVec(dof_vals.segment(i * 7, 7)));
    VectorXd p_eef_t_ = toVector3d(link_->GetTransform().trans);
    VectorXd vec_head_eef = head_pos_ - p_eef_t_;
    double head_eef_cost = exp(-vec_head_eef.norm());

    double torso_eef_cost = 0;

    for (int j = 0; j < 20; j++) {
      VectorXd torso_feet_sample = torso_pos_ + (i / 19) * (feet_pos_ - torso_pos_);

      double torso_eef_sample_dist = 2 * (torso_feet_sample - p_eef_t_).norm();
      double torso_eef_sample_cost = exp(-torso_eef_sample_dist);

      if (torso_eef_cost < torso_eef_sample_cost) {
        torso_eef_cost = torso_eef_sample_cost;
      }
    }

    err += max(head_eef_cost, torso_eef_cost);
    if (isnan(max(head_eef_cost, torso_eef_cost))) {
      std::cout << "Error is NaN" << std::endl;
      std::cout << head_eef_cost << std::endl;
      std::cout << torso_eef_cost << std::endl;
    }

  }
  VectorXd err_vec(1);
  err_vec << err;
  return err_vec;

}
// void DistanceBaselineCostPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
//   DistanceBaselineCostCalculator* calc = static_cast<DistanceBaselineCostCalculator*>(m_calc.get());
//   DblVec dof_vals_all = getDblVec(x, m_vars);
//   DblVec::const_iterator first = dof_vals_all.begin() + 5 * 7;
//   DblVec::const_iterator last = dof_vals_all.begin() + 5 * 7 + 7;
//   DblVec dof_vals(first, last);
//   calc->manip_->SetDOFValues(dof_vals);
//   // OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
//   // PlotAxes(env, cur, .05,  handles);
//   // PlotAxes(env, target, .05,  handles);
//   // handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
// }


// Visibility baseline cost
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
// void VisibilityBaselineCostPlotter::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
//   VisibilityBaselineCostCalculator* calc = static_cast<VisibilityBaselineCostCalculator*>(m_calc.get());
//   DblVec dof_vals_all = getDblVec(x, m_vars);
//   DblVec::const_iterator first = dof_vals_all.begin() + 5 * 7;
//   DblVec::const_iterator last = dof_vals_all.begin() + 5 * 7 + 7;
//   DblVec dof_vals(first, last);
//   calc->manip_->SetDOFValues(dof_vals);
//   OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
//   PlotAxes(env, cur, .05,  handles);
//   PlotAxes(env, target, .05,  handles);
//   handles.push_back(env.drawarrow(cur.trans, target.trans, .005, OR::Vector(1,0,1,1)));
// }

//////////////////////
// END Baseline Costs
//////////////////////


}
