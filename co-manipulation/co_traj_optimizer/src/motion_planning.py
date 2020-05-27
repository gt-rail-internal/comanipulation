#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from dmp_planner import *


class ChairAssemblyMotionPlanner():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("conimbus_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        
        self.start_pose = self.move_group.get_current_pose().pose
        # print(self.start_pose)
        # print(self.move_group.get_planning_frame())

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_frame = "j2s7s300_eef_link"

        self.dmp_planner = dmp_generator()
        self.go_to_ready()

    def pickup_object(self, goal_pose):
        # traj = self.generate_dmp_traj(goal_pose)
        # print(traj)
        # self.follow_cartesian_trajectory(traj, execute=True)
        # print(curr_pose)
        print(goal_pose)
        self.go_to_goal(goal_pose)
        curr_pose = self.move_group.get_current_pose().pose
        self.follow_cartesian_trajectory([[curr_pose.position.x, curr_pose.position.y, curr_pose.position.z - 0.1]], execute=True)
        # # TODO Gripper actions
        self.follow_cartesian_trajectory([[curr_pose.position.x, curr_pose.position.y, curr_pose.position.z]], execute=True)

    def handoff_object(self, goal_pose):
        self.go_to_goal(goal_pose)
        # traj = self.generate_dmp_traj(goal_pose)
        # self.follow_cartesian_trajectory(traj, execute=True)
        # TODO wait
        # TODO Gripper actions
        rospy.sleep(2.)
        self.go_to_ready()

    def stabilize(self, goal_pose):
        self.go_to_goal(goal_pose)
        # traj = self.generate_dmp_traj(goal_pose)
        # self.follow_cartesian_trajectory(traj, execute=True)
        curr_pose = self.move_group.get_current_pose().pose
        self.follow_cartesian_trajectory([[curr_pose.position.x, curr_pose.position.y, curr_pose.position.z - 0.1]], execute=True)
        # TODO Gripper actions
        # TODO wait
        rospy.sleep(5.)
        # TODO Gripper actions
        self.follow_cartesian_trajectory([[curr_pose.position.x, curr_pose.position.y, curr_pose.position.z]], execute=True)
        # self.go_to_goal(self.start_pose)
        self.go_to_ready()

    def prealign(self, goal_pose):
        self.go_to_goal(goal_pose)
        # traj = self.generate_dmp_traj(goal_pose)
        # self.follow_cartesian_trajectory(traj, execute=True)
        # TODO wait
        # TODO Gripper actions
        # TODO wait
        rospy.sleep(2.)
        # self.go_to_goal(self.start_pose)
        self.go_to_ready()

    def generate_dmp_traj(self, goal_pose):
        curr_pose = self.move_group.get_current_pose().pose
        x_0 = [curr_pose.position.x, curr_pose.position.y, curr_pose.position.z]
        print(goal_pose.position)
        goal = [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]
        plan = self.dmp_planner.generatePlan(x_0, goal)
        new_traj = []
        for point in plan.plan.points:
            new_traj.append([point.positions[0], point.positions[1], point.positions[2]])
        print(len(new_traj))
        return new_traj

    def go_to_goal(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def go_to_ready(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 1.848390656703197
        joint_goal[1] = 3.031491981487628
        joint_goal[2] = 2.8363014483446687
        joint_goal[3] = 0.8258030442448853
        joint_goal[4] = 3.1492481764593347
        joint_goal[5] = 2.3929354605910778
        joint_goal[6] = 3.051068346028583
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def follow_cartesian_trajectory(self, traj, execute = False):

        waypoints = []
        for point in traj:
            pose = self.move_group.get_current_pose().pose
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.position.z = point[2]
            waypoints.append(pose)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.05, 0.0)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        if (execute):
            self.move_group.execute(plan)

        return plan, fraction

# if __name__=="__main__":
#     rospy.init_node('ChairAssemblyMotionPlanner', anonymous=True)
#     planner = ChairAssemblyMotionPlanner()
#     pose = geometry_msgs.msg.Pose(planner.start_pose.position, planner.start_pose.orientation)
#     pose1 = geometry_msgs.msg.Pose(planner.start_pose.position, planner.start_pose.orientation)
#     pose2 = geometry_msgs.msg.Pose(planner.start_pose.position, planner.start_pose.orientation)
#     pose3 = geometry_msgs.msg.Pose(planner.start_pose.position, planner.start_pose.orientation)
#     pose1.position.x -= 0.1
#     pose2.position.x -= 0.2
#     pose3.position.x -= 0.2
#     pose3.position.z += 0.1
#     print("Planning...")
#     (plan, fraction) = planner.move_group.compute_cartesian_path([pose1, pose2], 0.05, 0.0)
#     print("Displaying...")
#     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     display_trajectory.trajectory_start = planner.robot.get_current_state()
#     display_trajectory.trajectory.append(plan)
#     planner.display_trajectory_publisher.publish(display_trajectory)
#     print("Executing...")
#     planner.move_group.execute(plan)
#     # planner.go_to_goal(pose)
