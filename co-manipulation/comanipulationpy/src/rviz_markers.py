#! /usr/bin/env python

from audioop import avg
from cProfile import run
from cmath import e, sqrt
from code import interact
from re import X


import rospy
import csv
import numpy as np
import sys
import itertools as IT
import os

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

#calculates distance in 3D space
def distan(base,coord):
        dist_x = abs(base[0]- coord[0])**2.0
        dist_y = abs(base[1]- coord[1])**2.0
        dist_z = abs(base[2]- coord[2])**2.0
        # dist = sqrt(dist_y+ dist_z)
        
        #only for 
        if coord[0] == 0.0:
            dist_x = 0
        if coord[2] == 0.0:
            dist_z = 0

        dist = sqrt(dist_x + dist_y + dist_z)
        # print("Distance: {}".format(dist))

        return dist

#saves data as csv
def save2csv(tag, skeleton_trajectory, dump_folder, name):

    with open('{}/{}_{}.csv'.format(dump_folder, tag, name), 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        
        for t in skeleton_trajectory:
            #raw_input(t)
            sklen = len(t)
            if sklen >0:
                writer.writerow([t])
                # writer.writerow([(sklen*'{},').format(*t.ravel())])
            else:
                writer.writerow(['NONE'])

#transfroms the data
def transform(x,y,z):

    translation = np.array([-0.954, 0.296, 0.976])
    
    transformation = np.array([[ -0.78328082, -0.31397157, 0.53655663, translation[0]],
                            [ -0.62003833, 0.45701115, -0.63772508, translation[1]],
                            [ -0.04498481, -0.8322035, -0.55264248, translation[2]],
                            [0.0, 0.0, 0.0, 1.0]])

    old_coord = np.array([[x],[y],[z],[1.0]])
    
    return np.matmul(transformation, old_coord)



class MarkerPub:
    
    done = 0
    gate = 0

    def __init__(self, dump_folder, dump_foldered, extra_folder, user_id):

        rospy.init_node('rviz_marker')

        # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        markerArray_pub = rospy.Publisher("/visualization_markerArray", MarkerArray, queue_size = 100)     #markerarray
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
        
        #Manipulator Base Coordinates estimate 
        # base = [float(-0.371), float(0.113), float(1.216)]      #will need to change based on TF 
        # base_new = transform(base[0],base[1],base[2])    
        base = [float(0),float(0),float(0)]                         #at TF:world
        calc = [[-0.315,-0.205,0],[-0.16,-0.625,0],[0.4,-0.615,0]] #calculator loc at TF:world
        prev_time = 0
        prev_coord = [0,0,0]

        #print(type(base[1]))

        #3,21
        #--------------------------------------------------------------------------------
        start_file = 5 
        num_file = 6 #number of files
        #--------------------------------------------------------------------------------

        for i in range(start_file,(num_file+1)):
            print(i)
        
            file_name = "run_" + str(i)                #set to a random file in the directory
            #print(file_name)
            tag = 'User-{}'.format(user_id)
            #print(user_id)

            # print(extra_folder[3])
            # raw_input(extra_folder[1])

            with open('{}/{}_{}.csv'.format(extra_folder[3], tag, "skeletal_"+file_name)) as sk_csvfile, open('{}/{}_{}.csv'.format(extra_folder[1], tag, "point_"+file_name)) as pt_csvfile, open('{}/{}_{}.csv'.format(extra_folder[0], tag, "orientation_"+file_name)) as or_csvfile:
                sk = csv.reader(sk_csvfile)
                pt = csv.reader(pt_csvfile)
                ori = csv.reader(or_csvfile)
                # reader = csv.reader(csvfile, delimiter=',')           #backup
                
                interupt = 0
                lift = 0
                runtime_prime = 0
                prev_lift = 0

                point = []
                ori_pt = []
                robot_pt = []
                ins_init = []
                avg_loc_x = np.array([])
                avg_loc_y = np.array([])
                avg_loc_z = np.array([])

                metric_1 = []
                metric_2 = []
                metric_3 = []
                metric_4 = []
                metric_5 = []
                metric_6 = []
                counter = 1

                metric_1.append("Distance: ,Run Time: ")
                metric_2.append("Lift State: ,Interupt: ,Run Time: ")
                metric_3.append("Distance: ,Run Time: ")
                metric_4.append("Distance: ,Run Time: ")
                metric_5.append("Distance: ,Run Time: ")
                metric_6.append("Distance: ,Run Time: ")

                for sk_A, pt_A, or_A in IT.izip(sk, pt, ori):
                # for row in sk: <---
                    counter += 1

                    markerarray = MarkerArray()
                    marker_rob = Marker()

                    marker_rob.header.frame_id = "world"
                    # marker_rob.header.frame_id = "frontdepth_camera_link"
                    marker_rob.header.stamp = rospy.Time.now()
                    marker_rob.type = 2
                    marker_rob.id = 0

                    # print(len(sk_A))

                    point = np.append(point, sk_A[0:len(sk_A)-1])
                    ori_pt = np.append(ori_pt, or_A[0:len(ori_pt)-1])
                    robot_pt = np.append(robot_pt, pt_A[0:3])

                    robot_pt_new = transform(float(robot_pt[0]), float(robot_pt[1]), float(robot_pt[2]))
                    robot_pt = np.array([float(robot_pt_new[0]),float(robot_pt_new[1]),float(robot_pt_new[2])])
                    
                    # print(counter)
                    # print(len(point)-1)

                    #Time
                    run_time = float(point[(len(point)-1)])
                    # raw_input(run_time)

                    count = 0                
                    
                    ins_aft = []

                    time_dif = run_time - prev_time

                    for c in range(0,((len(point)-1)),3):
                        marker = Marker()

                        marker.header.frame_id = "world"
                        # marker.header.frame_id = "frontdepth_camera_link"
                        marker.header.stamp = rospy.Time.now()

                        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; Line: 4 ; Line list: 5 
                        marker.type = 2

                        # Set the scale of the marker
                        marker.scale.x = 0.05
                        marker.scale.y = 0.05
                        marker.scale.z = 0.05


                        # Set the pose of the marker
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = 0.0
                        marker.pose.orientation.w = 1.0



                        # print(c)
                        x = float(point[c])
                        y = float(point[c+1])
                        z = float(point[c+2])
                        # print("x: {}, y: {}, z: {}".format(x,y,z))
                        new_coord = transform(x,y,z)
                        # raw_input("x: {}, y: {}, z: {}".format(new_coord[0],new_coord[1],new_coord[2]))
                        
                        #transformed coords at TF:world
                        x = float(new_coord[0])
                        y = float(new_coord[1])
                        z = float(new_coord[2])
                        
                        coord = [x,y,z]
                        #raw_input(coord)

                        # print("x: {}".format(x))
                        # print("y: {}".format(y))
                        # print("z: {}".format(z))
                        # print("Marker.id: {}".format(marker.id))
                        # print("-------------------")
                        
                        # marker.pose.position.x = -0.335870683193
                        # marker.pose.position.y = 0.0144129609689
                        # marker.pose.position.z = 1.87311255932

                        #Hip joint

                        
                        

                        if count == 0:

                            # print(time_dif)     #---------------------

                            # Set the color
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0

                            #Calcaulate the distance between base coordinate estimate and hip point
                            # coord[0] = float(0.0)
                            coords = coord
                            coords[0] = float(0.0)
                            # raw_input(coord)
                            dists1 = distan(base,coords)
                            metric_1.append("{},{}".format(dists1.real, run_time))
                            # raw_input(metric_1)
                            # print("Distance: {}".format(dists1))

                        elif count == 14:
                            # print(lift)

                            marker.color.r = 0.0
                            marker.color.g = 0.0
                            marker.color.b = 1.0
                            marker.color.a = 1.0

                            cond = [distan(calc[0],coord),distan(calc[1],coord),distan(calc[2],coord)]
                            

                            state_cond = 0.28

                            # print(runtime_prime)
                            # print(MarkerPub.gate)
                            # print("------------")

                            
                            if (cond[0].real <= state_cond or cond[1].real <= state_cond or cond[2].real <= state_cond) and MarkerPub.gate == 0:
                                # print("Start State Condition")
                                runtime_prime = run_time
                                prev_coord = coord
                                MarkerPub.gate = 1

                            if MarkerPub.gate == 1:

                                #Calculate average location of the hand
                                if run_time <= 1.0 + runtime_prime:
                                    avg_loc_x = np.append(avg_loc_x, [x], axis = 0)
                                    avg_loc_y = np.append(avg_loc_y, [y], axis = 0)
                                    avg_loc_z = np.append(avg_loc_z, [z], axis = 0)
                                    # print(avg_loc_x)

                                    avg_loc_x_ = round(np.mean(avg_loc_x),9)
                                    avg_loc_y_ = round(np.mean(avg_loc_y),9)
                                    avg_loc_z_ = round(np.mean(avg_loc_z),9)

                                    std_x = np.std(avg_loc_x)
                                    std_y = np.std(avg_loc_y)
                                    std_z = np.std(avg_loc_z)

                                    ins_init = [std_x, std_y, std_z]

                                else:
                                    # raw_input(avg_loc)
                                    # raw_input(ins_init)

                                    if MarkerPub.done != 2:
                                        # print("done")
                                        MarkerPub.done = 1

                                # if MarkerPub.done == 1:
                                #     # avg_loc_x = round(np.mean(avg_loc_x),9)
                                #     # avg_loc_y = round(np.mean(avg_loc_y),9)
                                #     # avg_loc_z = round(np.mean(avg_loc_z),9)

                                #     # print("average-x: {}".format(avg_loc_x))
                                #     # print("average-y: {}".format(avg_loc_y))
                                #     # raw_input("average-z: {}".format(avg_loc_z))

                                #     # std_x = np.std(np.append(avg_loc_x, [x], axis = 0))
                                #     # std_y = np.std(np.append(avg_loc_y, [y], axis = 0))
                                #     # std_z = np.std(np.append(avg_loc_z, [z], axis = 0))

                                #     # print("--------------------------------------")
                                #     # print("standard deviation-x: {}".format(std_x))
                                #     # print("standard deviation-y: {}".format(std_y))
                                #     # print("standard deviation-z: {}".format(std_z))

                                #     # ins_init = [std_x, std_y, std_z]
                                #     # raw_input(ins_init)

                                #     MarkerPub.done = 2
                                #     MarkerPub.gate = 1

                                if MarkerPub.done == 1:
                                #elif MarkerPub.gate == 1:
                                    # print("done")

                                    
                                    std_x = np.std(np.append(avg_loc_x, [x], axis = 0))
                                    std_y = np.std(np.append(avg_loc_y, [y], axis = 0))
                                    std_z = np.std(np.append(avg_loc_z, [z], axis = 0))

                                    ins_aft = [std_x, std_y, std_z]
                                    # print("--------------------------------------")
                                    # print(ins_aft)

                                    #------------------------------------------------------------------------------

                                    #The Scale (2) may need to change
                                    scale = 2
                                    dif_x = abs(ins_init[0] - ins_aft[0])*scale
                                    dif_y = abs(ins_init[1] - ins_aft[1])*scale
                                    dif_z = abs(ins_init[2] - ins_aft[2])*scale


                                    lim_y = 0.0041 #0.107/1.1/0.105
                                    lim_x = lim_y
                                    lim_z = 0.0112

                                    limi = 0.23 #0.2, 0.23 #0.29(big)
                                    limi2 = 0.23 #0.02, 0.11
                                    jump = 5 #[m/s]

                                    dif = [dif_x, dif_y, dif_z]

                                    #distance-------------------
                                    dif_avg = [avg_loc_x_, avg_loc_y_, avg_loc_z_,]
                                    
                                    # coord[0] = float(0)
                                    # print(coord)
                                    # print(dif_avg)

                                    # between current wrist vs calculator wrist
                                    dif_dist = distan(dif_avg, coord) 

                                    change_dist = distan(prev_coord, coord)

                                    # print(cond)

                                    print("Dist: {}".format(dif_dist.real))
                                    # print("Lift: {}".format(lift))
                                    # print("Interupt: {}".format(interupt))

                                    # print("Dist: {}".format(change_dist.real))
                                    # print("Run Time: {}".format(run_time))
                                    # # print("Time: {}".format(time_dif))
                                    # print("velocity: {}".format(change_dist.real / time_dif))
                                    print("------------------------------------------")

                                    #----------------------
                                    
                                    #[[-0.315,-0.205,0],[-0.16,-0.625,0],[0.4,-0.615,0]]

                                    scale_factor = 0.1
                                    cx = [0.235,0.855,0.15] # [0.12,0.17,0.15]
                                    cy = [0.34,0.235,0.235]
                                    cz = [0.165,0.1,0.065]

                                    print(coord)
                                    #New code
                                    #------------------------------------------------------------------------------
                                    # print("velocity: {}".format(change_dist.real / time_dif))
                                    # print("Calc 1:")
                                    # print("X: {} - {}".format((calc[0])[0] - cx[0], (calc[0])[0] + cx[0]))
                                    # print("Y: {} - {}".format((calc[0])[1] - cy[0], (calc[0])[1]))
                                    # print("Z: - {} ".format((calc[0])[2] + cz[0]))
                                    # # print("Calc 2:")
                                    # # print("X: {} - {}".format((calc[1])[0] - cx[1], (calc[1])[0] + cx[1]))
                                    # # print("Y: {} - {}".format((calc[1])[1] - cy[1], (calc[1])[1]))
                                    # # print("Z: - {} ".format((calc[1])[2] + cz[1]))
                                    # # print("Calc 3:")
                                    # # print("X: {} - {}".format((calc[2])[0] - cx[2], (calc[2])[0] + cx[2]))
                                    # # print("Y: {} - {}".format((calc[2])[1] - cy[2], (calc[2])[1]))
                                    # # print("Z: - {} ".format((calc[2])[2] + cz[2]))
                                    # print("------------------------------------------")
                                    # if (coord[0] >= (calc[0])[0] - cx[0] and  coord[0] <= (calc[0])[0] + cx[0]):
                                    #     print("1")
                                    # if (coord[1] >= (calc[0])[1] - cy[0] and  coord[1] <= (calc[0])[1]):
                                    #     print("2")
                                    # if (coord[2] <= (calc[0])[2] + cz[0]):
                                    #     print("3")
                                    
                                    # #c1 works
                                    # if (coord[0] >= (calc[0])[0] - cx[0] and  coord[0] <= (calc[0])[0] + cx[0]) and (coord[1] >= (calc[0])[1] - cy[0] and  coord[1] <= (calc[0])[1]) and (coord[2] <= (calc[0])[2] + cz[0]):
                                    #     print("in c1")

                                    # #c2 works
                                    # if (coord[0] >= (calc[1])[0] - cx[1] and  coord[0] <= (calc[1])[0] + cx[1]) and (coord[1] >= (calc[1])[1] - cy[1] and  coord[1] <= (calc[1])[1]) and (coord[2] <= (calc[1])[2] + cz[1]):
                                    #     print("in c2")

                                    # if (coord[0] >= (calc[2])[0] - cx[2] and  coord[0] <= (calc[2])[0] + cx[2]) and (coord[1] >= (calc[2])[1] - cy[2] and  coord[1] <= (calc[2])[1]) and (coord[2] <= (calc[2])[2] + cz[2]):
                                    #     print("in c3")
                                    #------------------------------------------------------------------------------

                                    #Old code
                                    #------------------------------------------------------------------------------
                                    if dif_dist.real >= limi:
                                        lift = 1
                                        if (change_dist.real / time_dif) >= jump:
                                            lift = prev_lift
                                        # print(lift)
                                        
                                    # elif dif_dist.real <= (limi * (1-scale_factor)) and lift == 1:
                                    elif dif_dist.real <= limi2 and lift == 1:
                                        interupt += 1
                                        lift = 0

                                    # if dif_x >= lim or dif_y >= lim or dif_z >= lim:
                                    #     lift = 1
                                    #     # print(lift)
                                        
                                    # elif dif_x <= lim and dif_y <= lim and dif_z <= lim and lift == 1:
                                    #     interupt += 1
                                    #     lift = 0
                                    #------------------------------------------------------------------------------
                                    

                                    #------------------------------------------------------------------------------

                                    # print("lift state: {}, Interupt: {}".format(lift, interupt))
                                    
                                    metric_2.append("{},{},{}".format(lift, interupt, run_time))
                                    # print("{}, Int: {}, lift: {}".format(dif_dist, interupt, lift))

                                    prev_lift = lift

                                    # print("--------------------------------------")
                                    # print("standard deviation-x: {}".format(std_x))
                                    # print("standard deviation-y: {}".format(std_y))
                                    # print("standard deviation-z: {}".format(std_z))
                                
                                #Calculate distance between wrist and robot point

                                # pt_A = [float(pt_A[0]), float(pt_A[1]), float(pt_A[2])]
                                # dists2 = distan(pt_A,coord)

                            #distance between the end effector and the right wrist
                            dists2 = distan(robot_pt,coord)
                            metric_3.append("{} ,{}".format(dists2.real, run_time))

                            #distance between the manipulator base and the right wrist
                            coords = coord
                            coords[0] = float(0.0)
                            dist_wrist_base = distan(base,coords)
                            metric_5.append("{} ,{}".format(dist_wrist_base.real, run_time))

                            # print("Distance: {}".format(dists2))

                            prev_coord = coord

                        elif count == 26:

                            # Set the color
                            marker.color.r = 0.0
                            marker.color.g = 0.5
                            marker.color.b = 0.5
                            marker.color.a = 1.0

                            #Calcaulate the distance between head point and the base
                            coords = coord
                            coords[0] = float(0.0)
                            dist_head_base = distan(base,coords)
                            metric_6.append("{},{}".format(dist_head_base.real, run_time))

                        else:
                            # Set the color
                            marker.color.r = 0.0
                            marker.color.g = 1.0
                            marker.color.b = 0.0
                            marker.color.a = 1.0

                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        # marker.pose.position.x = new_coord[0]
                        # marker.pose.position.y = new_coord[1]
                        # marker.pose.position.z = new_coord[2]


                        # raw_input("end?")
                        markerarray.markers.append(marker)  #markerarray
                        id = 0
                        for m in markerarray.markers:
                            m.id = id
                            id += 1

                        
                        markerArray_pub.publish(markerarray)
                        count += 1
                        
                        if count == ((len(point)-1)/3):
                            markerarray.markers.pop(0)  #markerarray
                            point = np.array([])
                            
                            # raw_input("end?")


                        rospy.sleep(0.005)
                        # rospy.sleep(0.005)
                        # rospy.sleep(0.008)

                    prev_time = run_time

                    # Set the scale of the marker
                    marker_rob.scale.x = 0.05
                    marker_rob.scale.y = 0.05
                    marker_rob.scale.z = 0.05

                    # Set the pose of the marker
                    marker_rob.pose.orientation.x = 0.0
                    marker_rob.pose.orientation.y = 0.0
                    marker_rob.pose.orientation.z = 0.0
                    marker_rob.pose.orientation.w = 1.0

                    # Set the color
                    marker_rob.color.r = 0.5
                    marker_rob.color.g = 0.0
                    marker_rob.color.b = 0.5
                    marker_rob.color.a = 1.0

                    # Set Position
                    # raw_input(robot_pt)
                    marker_rob.pose.position.x = float(robot_pt[0])
                    marker_rob.pose.position.y = float(robot_pt[1])
                    marker_rob.pose.position.z = float(robot_pt[2])

                    marker_pub.publish(marker_rob)

                    #distance between the end effector and the base of the manipulator
                    dist_arm_base = distan(base, robot_pt)
                    metric_4.append("{},{}".format(dist_arm_base.real, run_time))

                    robot_pt = np.array([])

                    # print(avg_loc_x)
                    # print(avg_loc_y)
                    # print(avg_loc_z)
                    # print(coord)

                runtime_prime = 0

            # print(metric_1)
            # print("--------------------------------------")
            # print(metric_2)
            # print("--------------------------------------")
            # raw_input(metric_3)

            save2csv(tag, metric_1, dump_foldered, "metric1_"+file_name)
            save2csv(tag, metric_2, dump_foldered, "metric2_"+file_name)
            save2csv(tag, metric_3, dump_foldered, "metric3_"+file_name)
            save2csv(tag, metric_4, dump_foldered, "metric4_"+file_name)
            save2csv(tag, metric_5, dump_foldered, "metric5_"+file_name)
            save2csv(tag, metric_6, dump_foldered, "metric6_"+file_name)


def main(user_id):
    logdir_path = '/home/hritiksapra/Documents/comoto.jl/src/expt_logs/user_{}'.format(user_id)
    logdir_path1 = '{}/orientation'.format(logdir_path)
    logdir_path2 = '{}/point'.format(logdir_path)
    logdir_path3 = '{}/robot'.format(logdir_path)
    logdir_path4 = '{}/skeleton'.format(logdir_path)

    extra_folder = [logdir_path1,logdir_path2,logdir_path3,logdir_path4]

    logdir_pathed = '/home/hritiksapra/Documents/comoto.jl/src/expt_logs/user_{}/metrics'.format(user_id)
    if not os.path.exists(logdir_pathed):
        os.mkdir(logdir_pathed)

    MarkerPub(logdir_path, logdir_pathed, extra_folder, user_id)


if __name__ == '__main__':
    main(sys.argv[1])