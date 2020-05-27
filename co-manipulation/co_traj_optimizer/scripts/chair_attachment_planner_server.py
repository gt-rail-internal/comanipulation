#! /usr/bin/env python

import rospy
import actionlib
import co_traj_optimizer.msg 

def execute_action(goal):
	result = co_traj_optimizer.msg.DemoResult()
	result.success = 1
	server.set_succeeded(result)

if __name__=="__main__":
	rospy.init_node('task_planner_server')
	server = actionlib.SimpleActionServer('task', co_traj_optimizer.msg.DemoAction, execute_action, False)
	server.start()
	rospy.spin()
