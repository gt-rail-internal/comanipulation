import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time

env = openravepy.Environment()
env.StopSimulation()
env.Load("../data/jaco-test.dae")
env.Load("../data/table.xml")
env.SetDefaultViewer()

test = True

# trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
robot = env.GetRobots()[0]
print(robot.__dict__)
print(robot.GetManipulator('test_arm'))

joint_start = [0, 3.14, 0, 3.14, 0, 3.14, 0]
robot.SetDOFValues(joint_start, robot.GetManipulator('test_arm').GetArmIndices())

joint_target = [0.5, 1.8, 0, 3.14, 0, 3.14, 0]


request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "test_arm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "visibility_baseline_cost", #  cost
    "params": {"coeffs" : 1.0,
               "head_pos" : [0, 0, 0],
               "obj_pos" : [1, 1, 1],
               "link" : "j2s7s300_ee_link"
    }
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    },    
  },
  
  {
    "type" : "legibility_cost", 
    "params" : {"xyz" : [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01], 
                "wxyz" : [1, 0, 0, 0], 
                "link": "j2s7s300_ee_link",
                "rot_coeffs" : [0,0,0],
                "pos_coeffs" : 1.0,
                "timestep" : 5
                }
                 
  }
  ],
  "constraints" : [
  {
    "type" : "joint", # joint-space target
    "params" : {"vals" : joint_target } # length of vals = # dofs of manip
  }
  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : joint_target
  }
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
if test:
  print('\n\n*** this is the optimization problem ***\n\n')
  print(prob)
t_start = time.time()
result = trajoptpy.OptimizeProblem(prob) # do optimization
t_elapsed = time.time() - t_start
print result
print "optimization took %.3f seconds"%t_elapsed

print(len(result.GetTraj()))

# from trajoptpy.check_traj import traj_is_safe
# prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
# assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

raw_input("waiting..")