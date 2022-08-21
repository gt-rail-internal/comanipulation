import traj_utils
import metrics

import openravepy
import numpy as np
import json

#TODO uncomment this once matlab works
# import matlab.engine
import rospy
from std_msgs.msg import String

class Scene:
    def __init__(self, robot_info, joint_traj_client=None):
        self.follow_joint_trajectory_client = joint_traj_client

        self.env = openravepy.Environment()
        self.env.StopSimulation()

        self.env.Load(robot_info.model)
        self.manipulator_name = robot_info.arm_name
        self.eef_link_name = robot_info.eef_link_name
        self.all_links = robot_info.all_links

        self.robot = self.env.GetRobots()[0]
        self.manipulator = self.robot.GetManipulator(self.manipulator_name)
        self.eef_link = self.robot.GetLink(self.eef_link_name)

        self.body = openravepy.RaveCreateKinBody(self.env, '')
        self.body.SetName("Collision_Scene")

    def get_eef_position(self, dof_vals):
        """
        uses OpenRAVE FK to calculate eef pose from joint angles, then returns only the eef position

        dof_vals: the joint angles of the robot (python list only, no numpy)
        """
        self.robot.SetDOFValues(dof_vals, self.manipulator.GetArmIndices())
        posevec = openravepy.poseFromMatrix(self.eef_link.GetTransform())
        position = posevec[4:7]
        pos = np.array([position[0], position[1], position[2]])
        return pos

    def follow_trajectory(self, traj):
        """
        Takes in a joint space trajectory and returns a cartesian space trajectory for the 
        end effector using get_eef_position. Output trajectory has the same number of 
        timesteps as input (no interpolation).

        traj: joint space trajectory
        """
        num_timesteps = traj.shape[0]
        eef_traj = np.zeros((num_timesteps, 3))
        for i in range(num_timesteps):
            p_eef_t = self.get_eef_position(traj[i])
            for j in range(3):
                eef_traj[i, j] = p_eef_t[j]
        return eef_traj


    def execute_trajectory(self, traj):
        """
        Moves the robot to the starting point of a joint space trajectory, 
        then dispatches the remainder of the trajectory to the robot

        traj: a joint-space trajectory, vectorized
        """
        print("Executing trajectory!")
        raw_input("Ready to move to initial position")
        # self.follow_joint_trajectory_client.move_to(traj[0], duration=1)
        self.follow_joint_trajectory_client.move_to(traj[0])
        raw_input("Ready for Gazebo execution?")
        self.follow_joint_trajectory_client.follow_trajectory(traj, duration=0.05)

    def execute_full_trajectory(self, traj, human_traj, obs_traj_len, full_human_traj_len):
        """
        Moves the robot to the starting point of a joint space trajectory, then dispatches both 
        a human trajectory and a robot trajectory via ros

        traj: a joint-space trajectory, vectorized
        human_traj: a human arm trajectory (arm only, 4 joints vectorized)
        obs_traj_len: number of timesteps in observed human trajectory
        full_human_traj_len: total number of human timesteps
        """
        # if self.use_ros and not self.use_jaco:
        # self.follow_joint_trajectory_client.follow_trajectory(
        #     [traj[0], traj[0]], duration=4)

        raw_input("Ready to move to initial position")
        self.follow_joint_trajectory_client.move_to(traj[0])
        full_human_traj = traj_utils.create_human_trajectory_tree(human_traj)
        
        raw_input("Ready for Gazebo execution")
        
        self.follow_joint_trajectory_client.execute_full_trajectory(
            traj, 0.1, 0.01, obs_traj_len, full_human_traj_len - obs_traj_len, len(traj), full_human_traj)
            

    # use openrave's FK module to get robot_joints in cartesian space rather than joint space
    def performFK(self, robot_joints):
        """
        Takes in the robot's joint angles and returns the cartesian 
        positions of each joint in a 2D numpy vector

        robot_joints: a set of robot joint angles
        """
        self.robot.SetDOFValues(robot_joints, self.manipulator.GetArmIndices())

        robot_joint_cart_pos = np.zeros((len(self.all_links), 3))
        for i in range(len(self.all_links)):
            link = self.all_links[i]
            posevec = openravepy.poseFromMatrix(self.robot.GetLink(link).GetTransform())
            position = posevec[4:7]
            robot_joint_cart_pos[i] = np.array([position[0], position[1], position[2]])
        return robot_joint_cart_pos

    def add_collision_objects(self):
        """
        Reads from the octomap ros topic to detect collision objects, then adds them 
        to self.body, the OpenRAVE collision scene
        """
        from octomap_trajopt_bridge.msg import octomap_custom

        octomap_message = rospy.wait_for_message("/octomap_decrypted", octomap_custom)

        octomap_array = np.zeros((octomap_message.num_vox, 6))
        for i in range(octomap_message.num_vox):
            octomap_array[i] = [octomap_message.x[i], octomap_message.y[i],
                                octomap_message.z[i], 0.025, 0.025, 0.025]

        self.env.Remove(self.body)
        self.body.InitFromBoxes(octomap_array)
        self.env.Add(self.body)

def get_human_obs_and_prediction():
    """
    Waits for a predicted human trajectory, then returns an array containing the trajectory
    and an array containing the covariance matrices of each prediction. Dead code after first return.
    """

    #TODO: is this right arm only? or full body?
    pred_msg_data = rospy.wait_for_message("/human_traj_pred", String)

    expData, expSigma = pred_msg_data.data.split("splitTag")
    expData, expSigma = matlab.double(json.loads(str(expData))[:100]), matlab.double(json.loads(str(expSigma))[:100])
    predictedMeans, variances = [],[]

    for row in expData:
        for col in row:
            predictedMeans.append(col)
    
    for timestep in range(len(expSigma)):
        for row in range(len(expSigma[timestep])):
            colStart = row%3 * 3
            for col in range(len(expSigma[timestep][row])):
                if col >= colStart and col < (colStart + 3):
                    variances.append(expSigma[timestep][row][col])
    return predictedMeans, variances