import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku
import time

env = openravepy.Environment()
env.StopSimulation()
env.Load("../data/jaco-test.dae")
env.Load("../data/table.xml")
env.SetDefaultViewer()

# trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]
print(robot.__dict__)
print(robot.GetManipulator('test_arm'))

joint_start = [0, 3.14, 0, 3.14, 0, 3.14, 0]
robot.SetDOFValues(joint_start, robot.GetManipulator('test_arm').GetArmIndices())

quat_target = [1,0,0,0] # wxyz
# Target that should be around the human but not inside the human
xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
manip = robot.GetManipulator("test_arm")
robot.SetActiveManipulator(manip)
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
    robot, iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
init_joint_target = ku.ik_for_link(hmat_target, manip, "j2s7s300_eef_link",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
# END ik

num_steps = 10

# Get human pose from "vision" system
# Will remain static for entire duration of trajectory execution
human_pose = [[1, 2, 3], [4, 5, 6]] # array of length skeleton size, each element is array of size 3 with x,y,z joint locations in base_link frame


request = {
  "basic_info" : {
    "n_steps" : num_steps,
    "manip" : "test_arm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
  },
  {
    "type" : "collision",
    "name" :"cont_coll", # shorten name so printed table will be prettier
    "params" : {
      "continuous" : True,
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_constraint
  {
    "type" : "pose", 
    "params" : {"xyz" : xyz_target, 
                "wxyz" : quat_target, 
                "link": "j2s7s300_eef_link",
                "timestep" : num_steps - 1
                }
                 
  }
  # END pose_constraint
  ],
  # BEGIN init
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
  }
  # END init
}

for i in range(num_steps):
  for j in range(len(human_pose)):
    dist_cost = {"type" : "pose", "params" : {"xyz" : human_pose[j], "wxyz" : [1, 0, 0, 0], "link" : "j2s7s300_eef_link", "timestep" : i, "rot_coeffs": [0, 0, 0], "pos_coeffs" : [-1, -1, -1]}}
    request["costs"].append(dist_cost)

print(request)

if args.position_only: request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]

s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print result
print result
print "optimization took %.3f seconds"%t_elapsed

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

# Now we'll check to see that the final constraint was satisfied
robot.SetActiveDOFValues(result.GetTraj()[-1])
posevec = openravepy.poseFromMatrix(robot.GetLink("test_arm").GetTransform())
quat, xyz = posevec[0:4], posevec[4:7]

quat *= np.sign(quat.dot(quat_target))
if args.position_only:
    assert (quat - quat_target).max() > 1e-3
else:
    assert (quat - quat_target).max() < 1e-3


raw_input("waiting..")