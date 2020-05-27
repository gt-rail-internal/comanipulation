#! /usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import time
import rospy
from geometry_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *
import co_manipulation_demo.msg

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class TrajoptTester(object):

  def __init__(self, name):
    print("Starting...")
    self.action_name = name
    self.action_server = actionlib.SimpleActionServer(self.action_name, co_manipulation_demo.msg.TrajoptAction,
                            execute_cb=self.executeTrajectory, auto_start=False)
    self.action_server.start()

    self.controller_client = actionlib.SimpleActionClient("/jaco_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    self.controller_client.wait_for_server()
    self.joint_names = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5",
                        "j2s7s300_joint_6", "j2s7s300_joint_7"]
    self.joint_positions = []

    print("Setting up environment")

    self.env = openravepy.Environment()
    self.env.StopSimulation()
    self.env.Load("/home/rail/Downloads/trajopt/data/jaco-test.dae")
    self.env.Load("/home/rail/Downloads/trajopt/data/table.xml")
    self.env.SetDefaultViewer()

    # trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting
    self.robot = self.env.GetRobots()[0]
    print(self.robot.__dict__)
    print(self.robot.GetManipulator('test_arm'))

    rospy.Subscriber("/j2s7s300_driver/out/joint_state", JointState, self.joint_position_cb)


  def joint_position_cb(self, data):
    self.joint_positions = data.position
    joint_start = self.joint_positions[:7]
    # # robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())
    self.robot.SetDOFValues(joint_start, self.robot.GetManipulator('test_arm').GetArmIndices())

  
  def executeTrajectory(self, data):
    print("received command")

    # joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1  , -1.926,  3.074]
    joint_start = self.joint_positions[:7]
    # # robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())
    self.robot.SetDOFValues(joint_start, self.robot.GetManipulator('test_arm').GetArmIndices())

    joint_target = [1, 3.14, 0, 3.14, 0, 3.14, 0]


    request = {
      "basic_info" : {
        "n_steps" : 10,
        "manip" : "test_arm", # see below for valid values
        "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
      },
      "costs" : [
      {
        "type" : "joint_vel", # joint-space velocity cost
        "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
        # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
      },
      {
        "type" : "collision",
        "params" : {
          "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
          "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        },    
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
    prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem
    t_start = time.time()
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    t_elapsed = time.time() - t_start
    print result
    print "optimization took %.3f seconds"%t_elapsed

    from trajoptpy.check_traj import traj_is_safe
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    assert traj_is_safe(result.GetTraj(), self.robot) # Check that trajectory is collision free

    print("executing")
    trajectory = JointTrajectory()

    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[-1].positions = result.GetTraj()[-1]
    trajectory.points[-1].velocities = [0.0 for _ in result.GetTraj()[-1]]
    trajectory.points[-1].accelerations = [0.0 for _ in result.GetTraj()[-1]]
    trajectory.points[-1].time_from_start = rospy.Duration(5.0)


    # for point in result.GetTraj():
    #   trajectory.points.append(JointTrajectoryPoint())
    #   trajectory.points[-1].positions = point
    #   trajectory.points[-1].velocities = [0.0 for _ in point]
    #   trajectory.points[-1].accelerations = [0.0 for _ in point]
    #   trajectory.points[-1].time_from_start = rospy.Duration(5.0)

    print("sending goal")
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory
    print(follow_goal.trajectory)
    self.controller_client.send_goal(follow_goal)
    self.controller_client.wait_for_result()



if __name__ == '__main__':
    rospy.init_node('demo_server')
    demo = TrajoptTester("tester")
    rospy.spin()