"""
The build a table
Author: Nithin Shrivatsav, Kevin Chen
This file should work correctly in both Python 2.7 
"""

from pyhop import *
import copy

## Operators
def pickup(state, leg_name):
    if state.pos[leg_name] == 'table' and state.robot_state == 'empty':
        ## need to call the server of the action server
        state.pos[leg_name] = 'robot_hand'
        state.robot_state = leg_name
        return state
    else:
        return False

def handover(state, leg_name):
    if state.pos[leg_name] == 'robot_hand' and state.robot_state != 'empty':
        ## need to call the server of the action server
        state.pos[leg_name] = 'human_hand'
        state.robot_state = 'empty'
        return state 
    else:
        return False        

def stabilize(state, part):
    if state.robot_state == 'empty':
        ## need to call the server of the action server
        state.robot_state = part
        return state
    else:
        return False


def prealign(state, leg_name):
    if state.pos[leg_name] == 'robot_hand' and state.robot_state != 'empty':
        ## call the action server
        state.pos[leg_name] = 'chair_bottom'
        state.robot_state = leg_name  
        return state
    else:
        return False

def status_check(state, leg_name):
    if state.pos[leg_name] == 'table' and state.robot_state == 'empty':
        human_state_left = input("Enter the human state left hand: ")
        human_state_right = input("Enter the human state right hand: ")
        state.human_state['l_hand'] = human_state_left
        state.human_state['r_hand'] = human_state_right
        # state.human_state['l_hand'] = 'idle'
        # state.human_state['r_hand'] = 'idle'
        return state
    elif state.pos[leg_name] == 'table' and state.robot_state != 'empty':
        state.robot_state = 'empty'
        human_state_left = input("Enter the human state left hand: ")
        human_state_right = input("Enter the human state right hand: ")
        state.human_state['l_hand'] = human_state_left
        state.human_state['r_hand'] = human_state_right
        # state.human_state['l_hand'] = 'idle'
        # state.human_state['r_hand'] = 'idle'
        return state
    elif state.pos[leg_name] == 'chair_bottom' and state.robot_state != 'empty':
        state.robot_state = 'empty'
        human_state_left = input("Enter the human state left hand: ")
        human_state_right = input("Enter the human state right hand: ")
        state.human_state['l_hand'] = human_state_left
        state.human_state['r_hand'] = human_state_right
        # state.human_state['l_hand'] = 'idle'
        # state.human_state['r_hand'] = 'idle'
        return state
    elif state.pos[leg_name] == 'human_hand' and state.robot_state == 'empty':
        state.pos[leg_name] = 'chair_bottom'
        human_state_left = input("Enter the human state left hand: ")
        human_state_right = input("Enter the human state right hand: ")
        state.human_state['l_hand'] = human_state_left
        state.human_state['r_hand'] = human_state_right
        # state.human_state['l_hand'] = 'idle'
        # state.human_state['r_hand'] = 'idle'
        return state
    elif state.pos[leg_name] == 'human_hand' and state.robot_state == 'chair_bottom':
        state.pos[leg_name] = 'chair_bottom'
        state.robot_state = 'empty'
        human_state_left = input("Enter the human state left hand: ")
        human_state_right = input("Enter the human state right hand: ")
        state.human_state['l_hand'] = human_state_left
        state.human_state['r_hand'] = human_state_right
        # state.human_state['l_hand'] = 'idle'
        # state.human_state['r_hand'] = 'idle'
        return state
    else:
        return state

## Declare the operators
declare_operators(pickup, handover, stabilize, prealign, status_check)

## Helper functions for main task

def status(state, goal, leg_name):
    if state.pos[leg_name] == goal.pos[leg_name] and state.robot_state == 'empty' and state.human_state['l_hand'] == 'idle' and state.human_state['r_hand'] == 'idle':
        return 'done'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'idle' and state.human_state['r_hand'] == 'idle':
        return 'prealign_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'idle' and state.human_state['r_hand'] == 'manipulate_part':
        return 'stabilize_chair'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'idle' and state.human_state['r_hand'] == 'reach':
        return 'handover_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'stabilize' and state.human_state['r_hand'] == 'idle':
        return 'prealign_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'stabilize' and state.human_state['r_hand'] == 'reach':
        return 'handover_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'stabilize' and state.human_state['r_hand'] == 'manipulate_part':
        return 'prealign_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'idle' and state.human_state['r_hand'] == 'manipulate_part':
        return 'stabilize_chair'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'manipulate_part' and state.human_state['r_hand'] == 'idle':
        return 'stabilize_chair'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'manipulate_part' and state.human_state['r_hand'] == 'stabilize':
        return 'prealign_leg'
    elif state.pos[leg_name] != goal.pos[leg_name] and state.human_state['l_hand'] == 'manipulate_part' and state.human_state['r_hand'] == 'manipulate_part':
        return 'stabilize_chair'


## Method
def build_chair(state, goal):
    part = 'chair_bottom'
    for leg_name in ['l_leg','r_leg']:
        s = status(state, goal, leg_name)
        # print("status ...")
        # print("leg_name:", leg_name)
        # print("part_name", part)
        # print("status: ", s)
        # print("location of leg:", state.pos[leg_name])
        # print("human state left: ", state.human_state['l_hand'])
        # print("human state right: ", state.human_state['r_hand'])
        # print("robot state: ", state.robot_state)
        if s == 'prealign_leg' or s == 'handover_leg':
            return [('attach_legs', s, leg_name),('status_check', leg_name),  ('build_chair_', goal)]
        elif s == 'stabilize_chair':
            return [('stabilize', part), ('status_check', leg_name), ('build_chair_', goal)]
        else:
            continue
    return []
declare_methods('build_chair_',build_chair)

## Attach Subtasks
def attach_handover_subtask(state, s, leg_name):
    if s == 'handover_leg':
        return [('pickup', leg_name), ('handover', leg_name)]
    return False

def attach_prealignment_subtask(state, s, leg_name):
    if s == 'prealign_leg':
        return [('pickup', leg_name), ('prealign', leg_name)]
    return False
declare_methods('attach_legs', attach_prealignment_subtask, attach_handover_subtask)
