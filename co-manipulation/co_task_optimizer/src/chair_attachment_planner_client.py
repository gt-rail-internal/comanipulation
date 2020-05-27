#! /usr/bin/env python
"""
Author: Nithin Shrivatsav, Kevin Chen
This file should work correctly in both Python 2.7 
"""
# primitive, part; result; status
import actionlib
import co_traj_optimizer.msg
import copy
from pyhop import *
import rospy
import table_attachment_4


def co_manipulation_client(plan):
	client = actionlib.SimpleActionClient('task_september_demo', co_traj_optimizer.msg.DemoAction)
	client.wait_for_server()
	for sub_plan in plan: 
		goal = co_traj_optimizer.msg.DemoGoal()
		result = co_traj_optimizer.msg.DemoResult()
		goal.primitiveID = sub_plan[0]
		goal.partID = sub_plan[1]
		raw_input("Press Enter to continue...")
		client.send_goal(goal)
		client.wait_for_result()
		print("primitive", sub_plan[0])
		print("part", sub_plan[1])
		result = client.get_result()
		if result.success != 1:
			return False
	return True


if __name__=="__main__":
	## Create the states
	## robot state ==> empty, r_leg/l_leg, chair_press
	## human state ==> idle, reach, stabilize, manipulate_part
	## ==> (idle, idle), (idle, reach), (idle, manipulate_part)
	## ==> (stabilize, idle), (stabilize, reach), (stabilize, manipulate_part)
	## ==> (manipulate_part, idle), (manipulate_part, reach), (manipulate_part, stabilize), (manipulate_part, manipulate_part)
	## right leg positions ==> table, robot_hand, human_hand, chair_bottom
	## left leg positions ==> table, robot_hand, human_hand, chair_bottom
	world_state = State('world_state')
	world_state.pos = {'r_leg':'table', 'l_leg':'table'}
	world_state.robot_state = 'empty'
	world_state.human_state = {'l_hand':'idle','r_hand':'reach'}

	## Create the goal
	goal_state = Goal('goal_state')
	goal_state.pos = {'r_leg':'chair_bottom', 'l_leg':'chair_bottom'}

	plan_string = pyhop(world_state,[('build_chair_', goal_state)], verbose=1)
	plan = []
	for sub_plan in plan_string:
		primitive_plan = []
		if sub_plan[0] != 'status_check':
			if sub_plan[0] == 'pickup' and sub_plan[1] == 'l_leg':
				primitive_plan.append(0)
				primitive_plan.append(1)
			elif sub_plan[0] == 'pickup' and sub_plan[1] == 'r_leg':
				primitive_plan.append(0)
				primitive_plan.append(0)
			elif sub_plan[0] == 'handover' and sub_plan[1] == 'l_leg':
				primitive_plan.append(1)
				primitive_plan.append(1)
			elif sub_plan[0] == 'handover' and sub_plan[1] == 'r_leg':
				primitive_plan.append(1)
				primitive_plan.append(0)
			elif sub_plan[0] == 'stabilize':
				primitive_plan.append(2)
				primitive_plan.append(2)
			elif sub_plan[0] == 'prealign' and sub_plan[1] == 'l_leg':
				primitive_plan.append(3)
				primitive_plan.append(2)
			elif sub_plan[0] == 'prealign' and sub_plan[1] == 'r_leg':
				primitive_plan.append(3)
				primitive_plan.append(2)


			# if sub_plan[1] == 'l_leg':
			# 	primitive_plan.append(1)
			# elif sub_plan[1] == 'r_leg':
			# 	primitive_plan.append(0)
			# elif sub_plan[1] == 'chair_bottom':
			# 	primitive_plan.append(2)
			plan.append(primitive_plan)

	try:
		print("Working")
		rospy.init_node('task_planner_client')
		result = co_manipulation_client(copy.deepcopy(plan))
		print(result)
	except rospy.ROSInterruptException:
		print("program interrupted before completion")
