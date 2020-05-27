#pragma once

#include "sco/modeling.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/sco_fwd.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "trajopt/common.hpp"
#include <openrave/openrave.h>
#include <math.h>
namespace trajopt {

using namespace sco;
typedef BasicArray<Var> VarArray;

#if 0
void makeTrajVariablesAndBounds(int n_steps, const RobotAndDOF& manip, OptProb& prob_out, VarArray& vars_out);

class FKFunc {
public:
  virtual OpenRAVE::Transform operator()(const VectorXd& x) const = 0;
  virtual ~FKFunc() {}
};

class FKPositionJacobian {
public:
  virtual Eigen::MatrixXd operator()(const VectorXd& x) const = 0;
  virtual ~FKPositionJacobian() {}
};
#endif


struct CartPoseErrCalculator : public VectorOfVector {
  OR::Transform pose_inv_;
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  CartPoseErrCalculator(const OR::Transform& pose, ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    pose_inv_(pose.inverse()),
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};

struct CartPoseErrorPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  CartPoseErrorPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};

struct CartVelJacCalculator : MatrixOfVector {
  ConfigurationPtr manip_;
  KinBody::LinkPtr link_;
  double limit_;
  CartVelJacCalculator(ConfigurationPtr manip, KinBody::LinkPtr link, double limit) :
    manip_(manip), link_(link), limit_(limit) {}

  MatrixXd operator()(const VectorXd& dof_vals) const;
};

struct CartVelCalculator : VectorOfVector {
  ConfigurationPtr manip_;
  KinBody::LinkPtr link_;
  double limit_;
  CartVelCalculator(ConfigurationPtr manip, KinBody::LinkPtr link, double limit) :
    manip_(manip), link_(link), limit_(limit) {}

  VectorXd operator()(const VectorXd& dof_vals) const;
};

#if 0
class CartPoseCost : public CostFromErrFunc {
public:
  CartPoseCost(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs);
};

class CartPoseConstraint : public ConstraintFromFunc {
public:
  CartPoseConstraint(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs);
};

class CartVelConstraint : public ConstraintFromFunc {
public:
  CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit);
};
#endif



//////////////////////////////////
// Co-Manipulatin Costs
//////////////////////////////////

// Distance Cost
struct DistanceCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  std::vector<Vector3d> human_poses_mean_;
  std::vector<Matrix3d> human_poses_var_;
  int n_human_joints_;
  std::vector<OR::KinBody::LinkPtr> links_;

  DistanceCostCalculator(const std::vector<Vector3d> human_poses_mean, const std::vector<Matrix3d> human_poses_var, std::vector<OR::KinBody::LinkPtr> links, int n_human_joints,  ConfigurationPtr manip) :
    human_poses_mean_(human_poses_mean),
    human_poses_var_(human_poses_var),
    n_human_joints_(n_human_joints),
    links_(links),
    manip_(manip) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct DistanceCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  DistanceCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};


// Velocity Cost
struct VelocityCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  std::vector<Vector3d> human_poses_mean_;
  std::vector<Matrix3d> human_poses_var_;
  int n_human_joints_;
  std::vector<OR::KinBody::LinkPtr> links_;

  VelocityCostCalculator(const std::vector<Vector3d> human_poses_mean, const std::vector<Matrix3d> human_poses_var, std::vector<OR::KinBody::LinkPtr> links, int n_human_joints,  ConfigurationPtr manip) :
    human_poses_mean_(human_poses_mean),
    human_poses_var_(human_poses_var),
    n_human_joints_(n_human_joints),
    links_(links),
    manip_(manip) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct VelocityCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  VelocityCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};

// struct VelocityCostCalculator : public VectorOfVector {
//   ConfigurationPtr manip_;
//   OR::KinBody::LinkPtr link_;
//   std::vector<Vector3d> human_poses_mean_;
//   // TODO check covariance matrix
//   std::vector<Matrix3d> human_poses_covar_;
//   int human_skeleton_size_;

//   VelocityCostCalculator(const std::vector<Vector3d> human_poses_mean, const std::vector<Vector3d> human_poses_covar, ConfigurationPtr manip, OR::KinBody::LinkPtr link, int human_skeleton_size) :
//     human_poses_mean_(human_poses_mean),
//     human_poses_covar_(human_poses_covar),
//     human_skeleton_size_(human_skeleton_size),
//     manip_(manip),
//     link_(link) {}
//   VectorXd operator()(const VectorXd& dof_vals) const;
// };


// Visibility Cost
struct VisibilityCostCalculator : public VectorOfVector {
  // OR::Transform pose_inv_;
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  std::vector<Vector3d> head_pos_mean_;
  std::vector<Vector3d> head_pos_var_;
  Vector3d obj_pos_;
  VisibilityCostCalculator(const std::vector<Vector3d> head_pos_mean, const std::vector<Vector3d> head_pos_var, const Vector3d obj_pos, ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    head_pos_mean_(head_pos_mean),
    head_pos_var_(head_pos_var),
    obj_pos_(obj_pos),
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct VisibilityCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  VisibilityCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};


// Legibility Cost
struct LegibilityCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  LegibilityCostCalculator(ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct LegibilityCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  LegibilityCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};


//////////////////////
// END Costs
//////////////////////


////////////////////////////////////
// Co-Manipulation additional costs
////////////////////////////////////



// Regularize Cost  
struct RegularizeCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  RegularizeCostCalculator(ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
// struct LegibilityCostPlotter : public Plotter {
//   boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
//   VarVector m_vars;
//   LegibilityCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
//   void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
// };

// Smoothing Cost  
struct SmoothingCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  SmoothingCostCalculator(ConfigurationPtr manip) :
    manip_(manip) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};

// Smoothing Cost  
struct SmoothingAccelCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  SmoothingAccelCostCalculator(ConfigurationPtr manip) :
    manip_(manip) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};


//////////////////////
// END additional Costs
//////////////////////


////////////////////////////////////
// Co-Manipulation baseline costs
////////////////////////////////////

// Distance baseline cost
struct DistanceBaselineCostCalculator : public VectorOfVector {
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  Vector3d head_pos_;
  Vector3d torso_pos_;
  Vector3d feet_pos_;
  DistanceBaselineCostCalculator(const Vector3d head_pos, const Vector3d torso_pos, const Vector3d feet_pos, ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    head_pos_(head_pos),
    torso_pos_(torso_pos),
    feet_pos_(feet_pos),
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct DistanceBaselineCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  DistanceBaselineCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};


// Visibility baseline cost
struct VisibilityBaselineCostCalculator : public VectorOfVector {
  OR::Transform pose_inv_;
  ConfigurationPtr manip_;
  OR::KinBody::LinkPtr link_;
  Vector3d head_pos_;
  Vector3d obj_pos_;
  VisibilityBaselineCostCalculator(const Vector3d head_pos, const Vector3d obj_pos, ConfigurationPtr manip, OR::KinBody::LinkPtr link) :
    head_pos_(head_pos),
    obj_pos_(obj_pos),
    manip_(manip),
    link_(link) {}
  VectorXd operator()(const VectorXd& dof_vals) const;
};
struct VisibilityBaselineCostPlotter : public Plotter {
  boost::shared_ptr<void> m_calc; //actually points to a CartPoseErrCalculator = CartPoseCost::f_
  VarVector m_vars;
  VisibilityBaselineCostPlotter(boost::shared_ptr<void> calc, const VarVector& vars) : m_calc(calc), m_vars(vars) {}
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};

//////////////////////
// END Baseline Costs
//////////////////////


}
