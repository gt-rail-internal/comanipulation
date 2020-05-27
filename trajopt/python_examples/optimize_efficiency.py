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

test = True

if test: print('\n\n*** entered file here ***\n\n')

env = openravepy.Environment()
env.StopSimulation()
# env.Load("../data/table.xml")
env.Load("/home/rail/Downloads/trajopt/data/jaco-test.dae")
env.SetDefaultViewer()

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
joint_start = [3.2228856458644617, 3.176139711882519, 3.0080511038888598, 0.4595325074138349, 2.7749619242213095, 2.6735964204946048, 1.9190092998546646]
robot.SetDOFValues(joint_start, robot.GetManipulator('test_arm').GetArmIndices())
# robot.SetDOFValues([0, 3.14, 0, 3.14, 0, 3.14, 0], robot.GetManipulator('test_arm').GetArmIndices())

if test: print('\n\n*** loaded robot here ***\n\n')

xyz_init = [0.1, 0.2, 0.2]
quat_target = [1,0,0,0] # w,x,y,z
xyz_target = [-0.2, 0.1, 0.2] # Initial x,y,z values in cartesian space
hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

if test: print('\n\n*** set initial pos here ***\n\n')

# BEGIN ik
manip = robot.GetManipulator("test_arm")
robot.SetActiveManipulator(manip)
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
    robot, iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
joint_target = ku.ik_for_link(hmat_target, manip, "j2s7s300_ee_link", filter_options = openravepy.IkFilterOptions.CheckEnvCollisions).tolist()
# END ik

if test: print('\n\n*** set robot here ***\n\n')

x = []
y = []
z = []
n = 10
for i in range(n):
  x.append(xyz_init[0] + (xyz_target[0] - xyz_init[0])/n * i)
  y.append(xyz_init[1] + (xyz_target[1] - xyz_init[1])/n * i)
  z.append(xyz_init[2] + (xyz_target[2] - xyz_init[2])/n * i)

# Initialize json request with position values
with open('/home/rail/Downloads/trajopt/data/optimize_efficiency.json', 'r+') as file:
  request = json.load(file)
for i in range(n):
  request['costs'][i]['params']['xyz'] = [x[i], y[i], z[i]]
request['init_info']['endpoint'] = joint_target

if test: print('\n\n*** finished request creation here ***\n\n')

if args.position_only: request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]

pretty = json.dumps(request, indent=4, separators=(',', ': ')) # Convert dictionary into pretty-printed JSON-formatted string
if test: print(pretty)

s = json.dumps(request)
prob = trajoptpy.ConstructProblem(s, env) # Create object that stores optimization problem (FAILS HERE)
if test:
  print('\n\n*** this is the optimization problem ***\n\n')
  print(prob)

result = trajoptpy.OptimizeProblem(prob) # Do optimization
print result

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

# Now we'll check to see that the final constraint was satisfied
robot.SetActiveDOFValues(result.GetTraj()[-1])
posevec = openravepy.poseFromMatrix(robot.GetLink("j2s7s300_ee_link").GetTransform())
quat, xyz = posevec[0:4], posevec[4:7]

quat *= np.sign(quat.dot(quat_target))
if args.position_only:
    assert (quat - quat_target).max() > 1e-3
else:
    assert (quat - quat_target).max() < 1e-3

if test: raw_input("waiting..")