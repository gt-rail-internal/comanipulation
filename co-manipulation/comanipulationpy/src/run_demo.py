#!/usr/bin/env python

# For simple data rostopic echo -b file.bag -p /topic > data.csv
# http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

from codecs import raw_unicode_escape_decode
from pickle import TRUE
import sys
import re

sys.path.insert(1, '/home/hritiksapra/Documents/comoto.jl/src')
from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import Tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from message_filters import Subscriber
from std_msgs.msg import Bool, String
from learning_tf2.msg import Num, MAstamp
import json
from pygame import mixer
import time
import sys
import os
import subprocess
import csv

import tf2_ros
import geometry_msgs.msg 
from geometry_msgs.msg import TwistStamped

from threading import Thread                #importing threads

mixer.init()
 
def play_music():
    time.sleep(3)
    mixer.music.load("/home/hritiksapra/Documents/comoto.jl/src/beep.mp3")
    mixer.music.play()

def get_problem():
    operators = [' + ', ' - ']
    first_num = random.randint(100000, 999999)
    second_num = random.randint(100000, 999999)
    third_num = random.randint(100000, 999999)
    return str(first_num) + random.choice(operators) + str(second_num) + random.choice(operators) + str(third_num)

def run1(stop, case):
    msg1 = Num()            #For start flag
    msg2 = Num()            #For stop flag
    
    if case == "true":
        while not rospy.is_shutdown():
            rate = rospy.Rate(15) #15 hz
            
            #publishes start/stop conditions + time
            msg1.header.stamp = rospy.get_rostime()
            msg2.header.stamp = rospy.get_rostime()
            msg1.statement = bool(True)
            msg2.statement = bool(False)

            pub_start.publish(msg1)
            pub_stop.publish(msg2)

            rate.sleep()
            if stop():
                    break
    elif case == "false":
        while not rospy.is_shutdown():
            rate = rospy.Rate(15) #15 hz

            msg1.header.stamp = rospy.get_rostime()
            msg2.header.stamp = rospy.get_rostime()
            msg1.statement = bool(False)
            msg2.statement = bool(True)

            pub_start.publish(msg1)
            pub_stop.publish(msg2)

            rate.sleep()

            if stop():
                    break

def tf_cor(stop, case):
    print("inside")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(15.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('frontdepth_camera_link', 'j2s7s300_ee_link', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print("TF not working")
            continue
        
        msg = geometry_msgs.msg.TwistStamped()

        msg.header.stamp = rospy.get_rostime()
        msg.twist.linear.x = trans.transform.translation.x
        msg.twist.linear.y = trans.transform.translation.y
        msg.twist.linear.z = trans.transform.translation.z

        # print(msg)
        tf_pub.publish(msg)

        rate.sleep()
        if stop():
            break

def mycallback(data):
    msged = MAstamp()

    for marker in data.markers:
        msged.header.stamp = rospy.get_rostime()
        msged.markers.markers.append(marker)
    
    mastamp_pub.publish(msged)
        
def mastamp(stop, case):
    print("here")       #ck if here
    while not rospy.is_shutdown():
        skeleton_subscriber = Subscriber('/front/body_tracking_data', MarkerArray)
        skeleton_subscriber.registerCallback(mycallback)

        rospy.spin()
    
def save2csv(tag, skeleton_trajectory, dump_folder):

    with open('{}/{}.csv'.format(dump_folder, tag), 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        for t in skeleton_trajectory:
            sklen = len(t)
            if sklen >0:
                writer.writerow([t])
            else:
                writer.writerow(['NONE'])

client = None

def run_experiment(user_id, path, pub_start, pub_stop, methods):

    # Zhi -- additional safe guards for data
    result_pub = rospy.Publisher("/study/result", String, queue_size=5)


    global client
    msg1 = Num()            #For start flag
    msg2 = Num()            #For stop flag

    #-------------------------------------------------------------------
    stop_thread3 = False
    t3 = Thread(target= mastamp, args=( lambda: stop_thread3, "case"))
    t3.start()
    # mastamp()# --------------------------------------------------------
    #-------------------------------------------------------------------------
    stop_thread2 = False
    t2 = Thread(target= tf_cor, args=( lambda: stop_thread2, "true"))
    t2.start()
    #-------------------------------------------------------------------------

    # path = "/home/hritiksapra/Documents/comoto.jl/src"
    map_type_to_traj = {
        "WHITE_1_A_D_62":"traj_1_A_D_62.txt",
        "PINK_2_A_B":"traj_2_A_B.txt",
        "PINK_2_D_E":"traj_2_D_E.txt",
        "PINK_2_E_B":"traj_2_E_B.txt",
        "PINK_2_E_C":"traj_2_E_C.txt",
        "BLACK_3_A_C":"traj_3_A_C_52.txt",
        "BLACK_3_C_B":"traj_3_C_B.txt",
        "BLACK_3_D_B":"traj_3_D_B.txt",
    }

    #raw_input("state 1")

    start_traj = ["PINK_2_E_C", "WHITE_1_A_D_62"] # [, 
    end_traj = ["BLACK_3_A_C","PINK_2_D_E"] # , 
    middle_traj = ["BLACK_3_D_B", "BLACK_3_C_B", "PINK_2_A_B", "PINK_2_E_B"] # 
    
    #first 12 in front ["PINK_2_E_C", "BLACK_3_A_C", "BLACK_3_D_B", "PINK_2_E_B"] 


    # methods =  ["COMOTO", "NOMINAL", "DIST"] #["COMOTO", "NOMINAL", "DIST"]
    print(methods)

    log_data = []
    log_data_prime = []
    file_array = []

    # random.shuffle(methods)       #Comment or uncomment?
    # random.shuffle(start_traj)
    # random.shuffle(end_traj)
    # random.shuffle(middle_traj)

    start_ros_node()

    window = tk.Tk()
    width= window.winfo_screenwidth() 
    height= window.winfo_screenheight()
    window.geometry("%dx%d" % (width, height))
    window.configure(bg='#8f918e')
    frame = tk.Frame(window, bg="#8f918e", width = width, height = height)
    my_string_var = tk.StringVar() 
    my_string_var.set("Please listen to the instructions before starting the experiment")
    text_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=my_string_var, font=("Arial", 40))
    second_string_var = tk.StringVar()
    second_string_var.set("")
    calculate_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=second_string_var, font=("Arial", 40))
    third_string_var = tk.StringVar()
    third_string_var.set("")
    clear_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=third_string_var, font=("Arial", 40))
    frame.pack()
    text_label.place(x = 300, y = 500)
    calculate_label.place(x = 500, y = 600)
    clear_label.place(x = 455, y = 300)

    counter = 1

    #------------------------------------------------------------------------
    next_types = [start_traj[0], end_traj[0], middle_traj[0], middle_traj[3]]
    #------------------------------------------------------------------------

    random.shuffle(next_types)
    
    
    log_data_prime.append("Part:, Method:, Trajectories:, Problem:, Time:, Answer:, Solution:, Accuracy:")
    robot_type = 1
    
    #--------------------------------------------------------------------------------------------------------
    part = 1

    #Fully Randomized

    # print(next_types)
    # methods = methods[1:2]
    # print(methods)

    for type_name in next_types:
        for method in methods:
            # print(method)
            traj_file_name = map_type_to_traj[type_name]

            if method == "NOMINAL":
                traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]
            
            if method == "DIST":
                traj_file_name = traj_file_name[:len(traj_file_name) - 4] + "_dist.txt"

            file_array.append(traj_file_name)

    random.shuffle(file_array)
    
    # print(len(file_array))
    print(file_array)

    """
    Fully Randomized Trajectories
    """

    for idx, traj_file_name in enumerate(file_array):
        break
        # print(traj_file_name)
        
        # random.shuffle(methods)
        # robot_type = 0

        #----------------------------------------------------------------------------------------------------
        # my_string_var.set("         The Robot Algorithm is different for every ")
        # second_string_var.set("experiment and not always the same")
        # third_string_var.set("")
        
        # frame.configure(bg='#8f918e')
        # text_label.configurepub_start(bg = "#8f918e", fg = "black")
        # calculate_label.configure(bg = "#8f918e", fg = "black")
        # clear_label.configure(bg = "#8f918e", fg = "black")
        #----------------------------------------------------------------------------------------------------

        # for method in methods:
        robot_type += 1

        #----------------------------------------------------------------------------------------------------
        # traj_file_name = map_type_to_traj[type_name]

        # if method == "NOMINAL":
        #     traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]
        
        # if method == "DIST":
        #     traj_file_name = traj_file_name[:len(traj_file_name) - 4] + "_dist.txt"
        #----------------------------------------------------------------------------------------------------

        """
        Does its job given the context but not beatiful
        """
        # print(traj_file_name[5])
        if traj_file_name[5] == "1":
            # print("one")
            type_name = "WHITE"
            if traj_file_name[11] == "d":
                method = "DIST"
            else:
                method = "COMOTO"
        elif traj_file_name[5] == "2":
            # print("two")
            type_name = "PINK"
            if traj_file_name[11] == "d":
                method = "DIST"
            else:
                method = "COMOTO"
        elif traj_file_name[5] == "3":
            # print("three")
            type_name = "BLACK"
            if traj_file_name[11] == "d":
                method = "DIST"
            else:
                method = "COMOTO"
        elif traj_file_name[5] == "n":
            # print(traj_file_name[13])
            if traj_file_name[13] == "1":
                # print("n one")
                type_name = "WHITE"
                method = "NOMINAL"
            elif traj_file_name[13] == "2":
                # print("n two")
                type_name = "PINK"
                method = "NOMINAL"
            elif traj_file_name[13] == "3":
                # print("n three")
                type_name = "BLACK"
                method = "NOMINAL"
        # print(type_name)

        #for end file
        traj_file = traj_file_name

        traj_file_name = "/home/hritiksapra/Documents/comoto.jl/src/"+traj_file_name

        # print(traj_file_name)

        traj, dt, t_0 = parse_traj_file(traj_file_name)

        print("Running {} | {} out of {}".format(method, idx+1, len(file_array)))

        raw_input("Display Start Instructions?")

        my_string_var.set("         The arm is now moving to the start position")
        second_string_var.set("")
        third_string_var.set("")
        
        frame.configure(bg='#8f918e')
        text_label.configure(bg = "#8f918e", fg = "black")
        calculate_label.configure(bg = "#8f918e", fg = "black")
        clear_label.configure(bg = "#8f918e", fg = "black")

        raw_input("Go to start?")
        print("[SAY] Robot is moving to starting position.")

        my_string_var.set("         The arm is now moving to the start position")
        # second_string_var.set("                 This is Robot {}".format(robot_type))
        third_string_var.set("")

        follow_trajectory([traj[0]], 4.0,  4.0)

        raw_input("Display move instructions?")
        print("[ASK] The calculator is cleared and on?")

        if type_name.startswith("WHITE"):
            frame.configure(bg='white')
            my_string_var.set("Reach for the WHITE calculator (Number 1) after the BEEP")
            third_string_var.set("Make sure the calculator is on and cleared")
            text_label.configure(bg = "white")
            calculate_label.configure(bg = "white")
            clear_label.configure(bg = "white")
        
        if type_name.startswith("PINK"):
            frame.configure(bg='#e75480')
            my_string_var.set("Reach for the PINK calculator (Number 2) after the BEEP")
            third_string_var.set("Make sure the calculator is on and cleared")
            text_label.configure(bg = "#e75480", fg = "white")
            calculate_label.configure(bg = "#e75480", fg = "white")
            clear_label.configure(bg = "#e75480", fg = "white")
        
        if type_name.startswith("BLACK"):
            frame.configure(bg='black')
            my_string_var.set("Reach for the BLACK calculator (Number 3) after the BEEP")
            third_string_var.set("Make sure the calculator is on and cleared")
            text_label.configure(bg = "black", fg = "white")
            calculate_label.configure(bg = "black", fg = "white")
            clear_label.configure(bg = "black", fg = "white")
        
        calculator_problem = get_problem()
        second_string_var.set("Calculate: " + calculator_problem)
        
        # print(len(calculator_problem))
        fhalf = int(calculator_problem[0:6])
        shalf = int(calculator_problem[9:15])
        thalf = int(calculator_problem[18:24])
        # print(calculator_problem[16])
        # print(thalf)
        if calculator_problem[7] == '+':
            if calculator_problem[16] == '+':
                solution = fhalf + shalf + thalf
            else:
                solution = fhalf + shalf - thalf
        else:
            if calculator_problem[16] == '+':
                solution = fhalf - shalf + thalf
            else:
                solution = fhalf - shalf - thalf

        raw_input("Start countdown?")
        
        play_music()

        time.sleep(1.0)

        begin = time.time()

        #---------------------------------------------------------------------
        stop_thread1 = False
        # stop_thread2 = False
        t1 = Thread(target= run1, args=( lambda: stop_thread1, "true"))
        # t2 = Thread(target= tf_cor, args=( lambda: stop_thread2, "true"))
        t1.start()
        # t2.start()
        #---------------------------------------------------------------------

        end = follow_trajectory(traj, dt, 1.0)

        # raw_input("Calculated Problem?")

        # end = time.time()
        run_time = str(end - begin)
        print(run_time)

        #---------------------------------------------------------------------
        stop_thread1 = True
        t1.join()
        #---------------------------------------------------------------------

        #---------------------------------------------------------------------
        stop_thread1 = False
        t1 = Thread(target= run1, args=( lambda: stop_thread1, "false"))
        t1.start()
        time.sleep(1.0)
        stop_thread1 = True
        t1.join()
        #---------------------------------------------------------------------

        # time.sleep(1.0)

        # msg1.header.stamp = rospy.get_rostime()
        # msg2.header.stamp = rospy.get_rostime()
        # msg1.statement = bool(True)
        # msg2.statement = bool(True)

        # pub_start.publish(msg1)
        # pub_stop.publish(msg2)


        #---------------------------------------------------------------------
        # stop_thread2 = True
        # t2.join()
        #---------------------------------------------------------------------

        answer = ""

        while len(answer) == 0:
            answer = raw_input("Answer: ")
            answer.strip()
            answer = re.sub("[^0-9]","", answer)

        print(answer)

        if solution == int(answer):
            cond = bool(True)
        else:
            cond = bool(False)

        log_result_data = "{},{},{},{},{},{},{},{}".format(part,method,traj_file,calculator_problem,run_time,answer,solution,cond)
        log_data_prime.append(log_result_data)
        result_pub.publish(data=log_result_data)
        log_data = []
        # print(log_data_prime)

        traj_file_name = ""

        # break #------------------------

        #break #----------------------
        
        # my_string_var.set("Please step back from the table and fill out the survey")
        # second_string_var.set("")
        # third_string_var.set("")

        # frame.configure(bg='#8f918e')
        # text_label.configure(bg = "#8f918e", fg = "black")
        # calculate_label.configure(bg = "#8f918e", fg = "black")
        # clear_label.configure(bg = "#8f918e", fg = "black")

        # raw_input("----Stop for survey!----")

        # break #-----------------------------
    print(log_data_prime)

    my_string_var.set("Please step back from the table and fill out the survey")
    second_string_var.set("")
    third_string_var.set("")

    frame.configure(bg='#8f918e')
    text_label.configure(bg = "#8f918e", fg = "black")
    calculate_label.configure(bg = "#8f918e", fg = "black")
    clear_label.configure(bg = "#8f918e", fg = "black")

    raw_input("\n----Stop for survey!----\n")

    print("First part done")
    #--------------------------------------------------------------------------------------------------------
    
    #------------------------------------------------------------------------
    types = [start_traj[1], end_traj[1], middle_traj[1], middle_traj[2]]
    #------------------------------------------------------------------------

    part = 2

    """
    Preset Methods and random applied trajectories
    """

    for method in methods:

        print(method)
        random.shuffle(types)

        for i in range(len(types)):

            print("Running {} out of {} in {}".format(i+1, len(types), method))

            traj_file_name = map_type_to_traj[types[i]]

            if method == "NOMINAL":
                traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]

            if method == "DIST":
                traj_file_name = traj_file_name[:len(traj_file_name) - 4] + "_dist.txt"

            # log_data.append(traj_file_name)

            type_name = types[i]

            traj_file = traj_file_name
            traj_file_name = "/home/hritiksapra/Documents/comoto.jl/src/"+traj_file_name
            traj, dt, t_0 = parse_traj_file(traj_file_name)

            raw_input("Display Start Instructions?")

            frame.configure(bg='#8f918e')
            text_label.configure(bg = "#8f918e", fg = "black")
            calculate_label.configure(bg = "#8f918e", fg = "black")
            clear_label.configure(bg = "#8f918e", fg = "black")
            
            my_string_var.set("         The arm is now moving to the start position")
            second_string_var.set("")
            # second_string_var.set("                 This is Robot {}".format(robot_type))
            third_string_var.set("")

            raw_input("Go to start?")
            print("[SAY] Robot is moving to starting position.")

            follow_trajectory([traj[0]], 4.0,  4.0)

            raw_input("Display move instructions?")
            print("[ASK] The calculator is cleared and on?")

            if type_name.startswith("WHITE"):
                frame.configure(bg='white')
                my_string_var.set("Reach for the WHITE calculator (Number 1) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "white")
                calculate_label.configure(bg = "white")
                clear_label.configure(bg = "white")
            
            if type_name.startswith("PINK"):
                frame.configure(bg='#e75480')
                my_string_var.set("Reach for the PINK calculator (Number 2) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "#e75480", fg = "white")
                calculate_label.configure(bg = "#e75480", fg = "white")
                clear_label.configure(bg = "#e75480", fg = "white")
            
            if type_name.startswith("BLACK"):
                frame.configure(bg='black')
                my_string_var.set("Reach for the BLACK calculator (Number 3) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "black", fg = "white")
                calculate_label.configure(bg = "black", fg = "white")
                clear_label.configure(bg = "black", fg = "white")
            
        
            calculator_problem = get_problem()
            second_string_var.set("Calculate: " + calculator_problem)

            # print(len(calculator_problem))
            fhalf = int(calculator_problem[0:6])
            shalf = int(calculator_problem[9:15])
            thalf = int(calculator_problem[18:24])
            # print(calculator_problem[16])
            # print(thalf)
            if calculator_problem[7] == '+':
                if calculator_problem[16] == '+':
                    solution = fhalf + shalf + thalf
                else:
                    solution = fhalf + shalf - thalf
            else:
                if calculator_problem[16] == '+':
                    solution = fhalf - shalf + thalf
                else:
                    solution = fhalf - shalf - thalf

            # print(solution)    
            # print(shalf)

            # Copy into Part 2
            #------------------------------------------------------------------------
            raw_input("Start countdown?")
            
            play_music()

            time.sleep(1.0)

            begin = time.time()

            #---------------------------------------------------------------------
            stop_thread1 = False
            # stop_thread2 = False
            t1 = Thread(target= run1, args=( lambda: stop_thread1, "true"))
            # t2 = Thread(target= tf_cor, args=( lambda: stop_thread2, "true"))
            t1.start()
            # t2.start()
            #---------------------------------------------------------------------

            end = follow_trajectory(traj, dt, 1.0)

            # raw_input("Calculated Problem?")

            time.time()
            run_time = str(end - begin)
            print(run_time)

            #---------------------------------------------------------------------
            stop_thread1 = True
            t1.join()
            #---------------------------------------------------------------------

            #---------------------------------------------------------------------
            stop_thread1 = False
            t1 = Thread(target= run1, args=( lambda: stop_thread1, "false"))
            t1.start()
            time.sleep(1.0)
            stop_thread1 = True
            t1.join()
            #---------------------------------------------------------------------

            # time.sleep(1.0)

            # msg1.header.stamp = rospy.get_rostime()
            # msg2.header.stamp = rospy.get_rostime()
            # msg1.statement = bool(True)
            # msg2.statement = bool(True)

            # pub_start.publish(msg1)
            # pub_stop.publish(msg2)


            #---------------------------------------------------------------------
            # stop_thread2 = True
            # t2.join()
            #---------------------------------------------------------------------

            answer = ""

            while len(answer) == 0:
                answer = raw_input("Answer: ")
                answer.strip()
                answer = re.sub("[^0-9]","", answer)

            if solution == int(answer):
                cond = bool(True)
            else:
                cond = bool(False)

            log_result_data = "{},{},{},{},{},{},{},{}".format(part,method,traj_file,calculator_problem,run_time,answer,solution,cond)
            log_data_prime.append(log_result_data)
            result_pub.publish(data=log_result_data)
            log_data = []
            # print(log_data_prime)

            #------------------------------------------------------------------------
            
            # break #------------------------

        

        robot_type += 1
        
        my_string_var.set("Please step back from the table and fill out the survey")
        second_string_var.set("")
        third_string_var.set("")

        frame.configure(bg='#8f918e')
        text_label.configure(bg = "#8f918e", fg = "black")
        calculate_label.configure(bg = "#8f918e", fg = "black")
        clear_label.configure(bg = "#8f918e", fg = "black")

        raw_input("----Stop for survey!----")

        # break #----------------------
    
    print(log_data_prime)


    #stops publishing tf cordinates
    #----------------------------------------------------------------------
    stop_thread3 = True
    # t3.join()
    #----------------------------------------------------------------------
    #---------------------------------------------------------------------
    stop_thread2 = True
    t2.join()
    #---------------------------------------------------------------------


    print(path)
    raw_input("End experiment?")
    
    tag = "experiment_run"
    save2csv(tag, log_data_prime, path)

    end_ros_node()


def main(user_id, pub_start, pub_stop, methods):
    logdir_path = '/home/hritiksapra/Documents/comoto.jl/src/expt_logs/user_{}'.format(user_id)
    if not os.path.exists(logdir_path):
        os.mkdir(logdir_path)
        print('Created directory for User {}'.format(user_id))
    
    run_experiment(user_id, logdir_path, pub_start, pub_stop, methods)

if __name__ == '__main__':

    # wait 1 seconds for eveything to start up
    time.sleep(1)

    rospy.init_node('flag_topic', anonymous=False)
    pub_start = rospy.Publisher('/flag_topic/start', Num, queue_size=10)
    pub_stop = rospy.Publisher('/flag_topic/stop', Num, queue_size=10)
    tf_pub = rospy.Publisher('/tf2_listener/publisher', TwistStamped, queue_size = 10)
    mastamp_pub = rospy.Publisher('/MAstamp/publisher', MAstamp, queue_size = 10)

    # print(sys.argv[2])
    # print(sys.argv[3])
    # print(sys.argv[4])

    valid_names = ["COMOTO", "DIST", "NOMINAL"]
    methods = [str(sys.argv[2]).strip(), str(sys.argv[3]).strip(), str(sys.argv[4]).strip()]
    for method in methods:
        if method not in valid_names:
            print("Incorrect Method Name {}".format(method))
            time.sleep(10)
            sys.exit()

    # print(methods)

    main(sys.argv[1], pub_start, pub_stop, methods)



