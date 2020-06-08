#! /usr/bin/env python

from human_traj_display.srv import ExecuteHumanTraj
import rospy
import tf

def execute_human_traj(req):
    print("Request received: ", req)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1 / req.timestep_size)
    traj_length = len(req.head)
    print("Trajectory of length ", traj_length)
    orientation_quart = tf.transformations.quaternion_from_euler(0, 0, 0)
    for i in range(traj_length):
        # br.sendTransform((i/20.0, i/20.0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "human_pose", "world")
        br.sendTransform((req.right_shoulder[i].x, req.right_shoulder[i].y, req.right_shoulder[i].z), orientation_quart, rospy.Time.now(), "human_right_shoulder", "world")
        br.sendTransform((req.right_elbow[i].x, req.right_elbow[i].y, req.right_elbow[i].z), orientation_quart, rospy.Time.now(), "human_right_elbow", "human_right_shoulder")
        br.sendTransform((req.right_wrist[i].x, req.right_wrist[i].y, req.right_wrist[i].z), orientation_quart, rospy.Time.now(), "human_right_wrist", "human_right_elbow")
        br.sendTransform((req.right_palm[i].x, req.right_palm[i].y, req.right_palm[i].z), orientation_quart, rospy.Time.now(), "human_right_palm", "human_right_wrist")


        br.sendTransform((req.neck[i].x, req.neck[i].y, req.neck[i].z), orientation_quart, rospy.Time.now(), "human_neck", "human_right_shoulder")
        br.sendTransform((req.head[i].x, req.head[i].y, req.head[i].z), orientation_quart, rospy.Time.now(), "human_head", "human_neck")
        br.sendTransform((req.torso[i].x, req.torso[i].y, req.torso[i].z), orientation_quart, rospy.Time.now(), "human_torso", "human_neck")


        br.sendTransform((req.left_shoulder[i].x, req.left_shoulder[i].y, req.left_shoulder[i].z), orientation_quart, rospy.Time.now(), "human_left_shoulder", "human_neck")
        br.sendTransform((req.left_elbow[i].x, req.left_elbow[i].y, req.left_elbow[i].z), orientation_quart, rospy.Time.now(), "human_left_elbow", "human_left_shoulder")
        br.sendTransform((req.left_wrist[i].x, req.left_wrist[i].y, req.left_wrist[i].z), orientation_quart, rospy.Time.now(), "human_left_wrist", "human_left_elbow")
        br.sendTransform((req.left_palm[i].x, req.left_palm[i].y, req.left_palm[i].z), orientation_quart, rospy.Time.now(), "human_left_palm", "human_left_wrist")
        
        rate.sleep()
        print("Time: ", rospy.Time.now())
    return True


if __name__ == "__main__":
    rospy.init_node("human_traj_server")
    s = rospy.Service("execute_human_traj", ExecuteHumanTraj, execute_human_traj)
    print("Ready to receive request")
    rospy.spin()