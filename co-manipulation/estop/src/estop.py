#!/usr/bin/env python

import rospy
import tf
import actionlib

from actionlib_msgs.msg import GoalStatusArray, GoalID
from sensor_msgs.msg import JointState

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import sys
import comanipulationpy
from scene_utils import Scene

from const import ROBOTS_DICT
from metrics import get_separation_dist

ESTOP_THRESHOLD = 0.2

joint_state_topics = {
    "jaco": "/j2s7s300/joint_states",
    "iiwa": "/iiwa/joint_states"
}

class EStop:
    def __init__(self, robot_name, human_is_fake, threshold=ESTOP_THRESHOLD, rate=10):
        """
        robot_name: the name of the robot ('jaco', 'jaco-real', 'iiwa', or 'franka')
        human_is_fake: boolean telling us if we're getting real or fake human data
        threshold: how close to let the robot get
        rate: rate, in Hz, at which to check for danger
        """
        self.human_is_fake = human_is_fake
        self.threshold = threshold
        self.rate = rospy.Rate(rate)

        self.is_stopped = False
        self.tfListener = tf.TransformListener()

        self.robot_info = ROBOTS_DICT[robot_name]
        self.robot_joint_names = self.robot_info.controller_joints

        self.scene = Scene(self.robot_info)

        self.robot_joints = []
        joint_topic = joint_state_topics[robot_name]
        rospy.Subscriber(joint_topic, JointState, self.robot_joints_callback)

    def get_human_pose_fake(self):
        """
        Returns a cartesian-space human pose in the world frame
        """
        pose = []
        for joint_name in ['human_right_shoulder', 'human_right_elbow', 'human_right_wrist', 'human_right_palm', 'human_neck', 'human_head', 'human_torso', 'human_left_shoulder', 'human_left_elbow', 'human_left_wrist', 'human_left_palm']:
            try:
                translation, _ = self.tfListener.lookupTransform("/world", joint_name, rospy.Time(0))
                pose.extend(translation)
            except Exception, e:
                # print("Error looking up transform: %s"%e)
                return []
        return pose

    def robot_joints_callback(self, data):
        """
        Rospy callback that sets robot_joints to the values given by parameter `data`
        """
        my_joints = []
        name_to_pos = dict(zip(data.name, data.position))
        for joint_name in self.robot_joint_names:
            my_joints.append(name_to_pos[joint_name])
        self.robot_joints = my_joints

    def stop_robot(self):
        """
        Stops the robot by cancelling the in-progress trajectory
        """
        print("STOPPING")
        client = actionlib.SimpleActionClient(self.robot_info.controller_name+"/follow_joint_trajectory", FollowJointTrajectoryAction)
        client.wait_for_server()
        print("Got ac")
        client.cancel_all_goals()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.robot_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.robot_joints
        trajectory.points[0].velocities = [0.0 for _ in self.robot_joints]
        trajectory.points[0].accelerations = [0.0 for _ in self.robot_joints]
        trajectory.points[0].time_from_start = rospy.Duration(0.01)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        client.send_goal(follow_goal)
        client.wait_for_result()
        print("Done")

        self.is_stopped = True
    
    def get_human_pose_real(self):
        """
        TODO: Implement this
        """
        return []

    def spin(self):
        """
        Main estop loop - spins until ros is shut down checking for collisions and stopping them
        """
        while not rospy.is_shutdown():
            if self.human_is_fake:
                human_pose = self.get_human_pose_fake()
            else:
                human_pose = self.get_human_pose_real()
            
            if len(human_pose) == 0:
                continue
            
            distance = get_separation_dist(self.scene, human_pose, self.robot_joints)
            # print("Distance = %0.3f" % distance)
            if distance < self.threshold and not self.is_stopped:
                self.stop_robot()
            self.rate.sleep()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("""Usage: estop.py [robot type] [human traj type]
        robot type: any of "jaco", "jaco-real", "franka", or "iiwa"
        human traj type: "real" or "fake"
        """)
        sys.exit(1)
    rospy.init_node("conimbus_estop")
    stopper = EStop(sys.argv[1], sys.argv[2]=='fake', rate=50)
    stopper.spin()


    

    

    

    