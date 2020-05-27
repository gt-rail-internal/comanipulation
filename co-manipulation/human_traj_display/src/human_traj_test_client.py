#! /usr/bin/env python

from human_traj_display.srv import ExecuteHumanTraj
from geometry_msgs.msg import Point
import rospy


if __name__ == "__main__":
    rospy.wait_for_service("execute_human_traj")


    right_shoulder = [Point(1, 1, 1), Point(1, 1, 1)]
    right_elbow = [Point(0, 0, -0.2), Point(0, 0, -0.2)]
    right_wrist = [Point(0, 0.2, 0), Point(0, 0, -0.2)]
    right_palm = [Point(0, 0, -0.1), Point(0, 0, -0.1)]

    neck = [Point(0.2, 0, 0), Point(0.2, 0, 0)]
    head = [Point(0, 0, 0.1), Point(0, 0, 0.1)]
    torso = [Point(0, 0, -0.4), Point(0, 0, -0.4)]

    left_shoulder = [Point(0.2, 0, 0), Point(0.2, 0, 0)]
    left_elbow = [Point(0, 0, -0.2), Point(0, 0, -0.2)]
    left_wrist = [Point(0, 0, -0.2), Point(0, 0, -0.2)]
    left_palm = [Point(0, 0, -0.1), Point(0, 0, -0.1)]

    timestep_size = 2
    num_timesteps = 2


    
    try:
        execute_human_traj_service = rospy.ServiceProxy("execute_human_traj", ExecuteHumanTraj) 
        resp = execute_human_traj_service(right_shoulder, right_elbow, right_wrist, right_palm, neck, head, torso, left_shoulder, left_elbow, left_wrist, left_palm, timestep_size, num_timesteps)
        print("Received: ", resp)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e