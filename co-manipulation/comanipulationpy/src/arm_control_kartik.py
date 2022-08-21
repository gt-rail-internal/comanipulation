#!/usr/bin/env python

import copy
import actionlib
import rospy
import tf

import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import time

from geometry_msgs.msg import Point
from human_traj_display.srv import ExecuteHumanTraj


# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        rospy.loginfo("Found %s" % name)

        self.joint_names = joint_names

    def move_to(self, positions, duration=10.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        print(follow_goal)
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()
        print("Done...")

    def follow_trajectory(self, points, duration=1.0):
        if len(self.joint_names) != len(points[0]):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for t, point in enumerate(points):
            trajectory.points.append(JointTrajectoryPoint())
            trajectory.points[-1].positions = point
            trajectory.points[-1].velocities = [0.0 for _ in point]
            trajectory.points[-1].accelerations = [0.0 for _ in point]
            trajectory.points[-1].time_from_start = rospy.Duration((t + 1) * duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        print("sent, waiting...")
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()
        print("Done...")

    def execute_full_trajectory(self, points, robot_timestep_size, human_timestep_size,
                                obs_steps_human, exec_steps_human, exec_steps_robot, human_trajectory):
        # rate = rospy.Rate(1.0 / human_timestep_size)

        duration = robot_timestep_size

        #####
        # Setup robot trajectory
        #####
        if len(self.joint_names) != len(points[0]):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for t, point in enumerate(points):
            # print(point)
            trajectory.points.append(JointTrajectoryPoint())
            trajectory.points[-1].positions = point
            trajectory.points[-1].velocities = [0.0 for _ in point]
            trajectory.points[-1].accelerations = [0.0 for _ in point]
            trajectory.points[-1].time_from_start = rospy.Duration(t * duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory


        right_shoulder = []
        right_elbow = []
        right_wrist = []
        right_palm = []

        neck = []
        head = []
        torso = []

        left_shoulder = []
        left_elbow = []
        left_wrist = []
        left_palm = []

        for i in range(len(human_trajectory["right_shoulder"])):
            right_shoulder.append(Point(human_trajectory["right_shoulder"][i][0], human_trajectory["right_shoulder"][i][1], human_trajectory["right_shoulder"][i][2]))
            right_elbow.append(Point(human_trajectory["right_elbow"][i][0], human_trajectory["right_elbow"][i][1], human_trajectory["right_elbow"][i][2]))
            right_wrist.append(Point(human_trajectory["right_wrist"][i][0], human_trajectory["right_wrist"][i][1], human_trajectory["right_wrist"][i][2]))
            right_palm.append(Point(human_trajectory["right_palm"][i][0], human_trajectory["right_palm"][i][1], human_trajectory["right_palm"][i][2]))

            neck.append(Point(human_trajectory["neck"][i][0], human_trajectory["neck"][i][1], human_trajectory["neck"][i][2]))
            head.append(Point(human_trajectory["head"][i][0], human_trajectory["head"][i][1], human_trajectory["head"][i][2]))
            torso.append(Point(human_trajectory["torso"][i][0], human_trajectory["torso"][i][1], human_trajectory["torso"][i][2]))

            left_shoulder.append(Point(human_trajectory["left_shoulder"][i][0], human_trajectory["left_shoulder"][i][1], human_trajectory["left_shoulder"][i][2]))
            left_elbow.append(Point(human_trajectory["left_elbow"][i][0], human_trajectory["left_elbow"][i][1], human_trajectory["left_elbow"][i][2]))
            left_wrist.append(Point(human_trajectory["left_wrist"][i][0], human_trajectory["left_wrist"][i][1], human_trajectory["left_wrist"][i][2]))
            left_palm.append(Point(human_trajectory["left_palm"][i][0], human_trajectory["left_palm"][i][1], human_trajectory["left_palm"][i][2]))


        #####
        # Execute observed human trajectory
        #####
        rospy.wait_for_service("execute_human_traj")
        if(obs_steps_human != 0):
            try:
                execute_human_traj_service = rospy.ServiceProxy("execute_human_traj", ExecuteHumanTraj)
                # resp = execute_human_traj_service(human_trajectory["right_shoulder"][:obs_steps_human], human_trajectory["right_elbow"][:obs_steps_human], human_trajectory["right_wrist"][:obs_steps_human], human_trajectory["right_palm"][:obs_steps_human], human_trajectory["neck"][:obs_steps_human], human_trajectory["head"][:obs_steps_human], human_trajectory["torso"][:obs_steps_human], human_trajectory["left_shoulder"][:obs_steps_human], human_trajectory["left_elbow"][:obs_steps_human], human_trajectory["left_wrist"][:obs_steps_human], human_trajectory["left_palm"][:obs_steps_human], human_timestep_size, obs_steps_human)


                resp = execute_human_traj_service(right_shoulder[:obs_steps_human], right_elbow[:obs_steps_human], right_wrist[:obs_steps_human], right_palm[:obs_steps_human], neck[:obs_steps_human], head[:obs_steps_human], torso[:obs_steps_human], left_shoulder[:obs_steps_human], left_elbow[:obs_steps_human], left_wrist[:obs_steps_human], left_palm[:obs_steps_human], human_timestep_size, obs_steps_human)

                print("Received: ", resp)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        #####
        # Execute robot trajectory (not blocking)
        #####
        self.client.send_goal(follow_goal)


        #####
        # Execute remaining human trajectory
        #####
        try:
            execute_human_traj_service = rospy.ServiceProxy("execute_human_traj", ExecuteHumanTraj)
            # resp = execute_human_traj_service(human_trajectory["right_shoulder"][obs_steps_human:], human_trajectory["right_elbow"][obs_steps_human:], human_trajectory["right_wrist"][obs_steps_human:], human_trajectory["right_palm"][obs_steps_human:], human_trajectory["neck"][obs_steps_human:], human_trajectory["head"][obs_steps_human:], human_trajectory["torso"][obs_steps_human:], human_trajectory["left_shoulder"][obs_steps_human:], human_trajectory["left_elbow"][obs_steps_human:], human_trajectory["left_wrist"][obs_steps_human:], human_trajectory["left_palm"][obs_steps_human:], human_timestep_size, obs_steps_human)


            resp = execute_human_traj_service(right_shoulder[obs_steps_human:], right_elbow[obs_steps_human:], right_wrist[obs_steps_human:], right_palm[obs_steps_human:], neck[obs_steps_human:], head[obs_steps_human:], torso[obs_steps_human:], left_shoulder[obs_steps_human:], left_elbow[obs_steps_human:], left_wrist[obs_steps_human:], left_palm[obs_steps_human:], human_timestep_size, exec_steps_human)
            print("Received: ", resp)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def visualize_human_trajectory(self, human_timestep_size, human_timesteps, human_trajectory):


        right_shoulder = []
        right_elbow = []
        right_wrist = []
        right_palm = []

        neck = []
        head = []
        torso = []

        left_shoulder = []
        left_elbow = []
        left_wrist = []
        left_palm = []

        for i in range(len(human_trajectory["right_shoulder"])):
            right_shoulder.append(Point(human_trajectory["right_shoulder"][i][0], human_trajectory["right_shoulder"][i][1], human_trajectory["right_shoulder"][i][2]))
            right_elbow.append(Point(human_trajectory["right_elbow"][i][0], human_trajectory["right_elbow"][i][1], human_trajectory["right_elbow"][i][2]))
            right_wrist.append(Point(human_trajectory["right_wrist"][i][0], human_trajectory["right_wrist"][i][1], human_trajectory["right_wrist"][i][2]))
            right_palm.append(Point(human_trajectory["right_palm"][i][0], human_trajectory["right_palm"][i][1], human_trajectory["right_palm"][i][2]))

            neck.append(Point(human_trajectory["neck"][i][0], human_trajectory["neck"][i][1], human_trajectory["neck"][i][2]))
            head.append(Point(human_trajectory["head"][i][0], human_trajectory["head"][i][1], human_trajectory["head"][i][2]))
            torso.append(Point(human_trajectory["torso"][i][0], human_trajectory["torso"][i][1], human_trajectory["torso"][i][2]))

            left_shoulder.append(Point(human_trajectory["left_shoulder"][i][0], human_trajectory["left_shoulder"][i][1], human_trajectory["left_shoulder"][i][2]))
            left_elbow.append(Point(human_trajectory["left_elbow"][i][0], human_trajectory["left_elbow"][i][1], human_trajectory["left_elbow"][i][2]))
            left_wrist.append(Point(human_trajectory["left_wrist"][i][0], human_trajectory["left_wrist"][i][1], human_trajectory["left_wrist"][i][2]))
            left_palm.append(Point(human_trajectory["left_palm"][i][0], human_trajectory["left_palm"][i][1], human_trajectory["left_palm"][i][2]))


        #####
        # Execute human trajectory
        #####
        rospy.wait_for_service("execute_human_traj")
        try:
            execute_human_traj_service = rospy.ServiceProxy("execute_human_traj", ExecuteHumanTraj)


            resp = execute_human_traj_service(right_shoulder, right_elbow, right_wrist, right_palm, neck, head, torso, left_shoulder, left_elbow, left_wrist, left_palm, human_timestep_size, human_timesteps)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # pub = rospy.Publisher("human_tfs", String, queue_size=10)
    # follow_joint_trajectory_client = FollowTrajectoryClient("panda_arm_controller", ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"])

    # follow_joint_trajectory_client = FollowTrajectoryClient("iiwa/PositionJointInterface_trajectory_controller/", ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"])
    follow_joint_trajectory_client = FollowTrajectoryClient("jaco_trajectory_controller", ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"])

    points = [[1.5, 1.5, 0.0, 0, 0.0, 0, 1.0],
              [1.0, 1.0, 0.0, 0, 0.0, 0, 1.0]
              # [0.22, 0.26, 0.5, 1.72, 0.0, 1.66, 0.0],
              # [0.22, 0.26, 1.0, 1.72, 0.0, 1.66, 0.0],
              # [1.22, 0.26, 1.0, 1.72, 0.0, 1.66, 0.0],
              # [1.22, 0.26, 0.5, 1.72, 0.0, 1.66, 0.0],
              # [0.22, 0.26, 0.5, 1.72, 0.0, 1.66, 0.0],
              ]

    raw_input("waiting to publish")
    while True:
        follow_joint_trajectory_client.follow_trajectory(points, duration=5)

    # br = tf.TransformBroadcaster()
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     br.sendTransform((0.0, 3.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "test", "base_link")

    # hello_str = "test"
    # pub.publish(hello_str)
    print("Working...")

    rospy.spin()
