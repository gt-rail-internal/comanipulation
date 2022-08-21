
from cmath import pi
from re import X
import rospy
import csv
import numpy as np
import sys
import tf.transformations
import subprocess, shlex, psutil

# import rviz_tools_py as rviz_tools
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)

marker = Marker()

marker.header.frame_id = "world"
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3 ; Line: 4 ; Line list: 5
marker.type = 0
marker.id = 0

# Set the scale of the marker
marker.scale.x = 0.02
marker.scale.y = 0.02
marker.scale.z = 0.02

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set marker orientation
x = -0.27861598134
y = 0.193611130118
z = -0.480773985386
w = 0.808544516563

# x = 0
# y = 0
# z = 0.99810947
# w = 0.06146124

# x = 0
# y = 0
# z = 0
# w = 1

marker.pose.orientation.x = x
marker.pose.orientation.y = y
marker.pose.orientation.z = z
marker.pose.orientation.w = w

count = 0

sol = euler_from_quaternion([w,x,y,z]) #sol = psi(z), theta(y), phi(x)
# print(sol) 
sol = [sol[0],sol[1],sol[2]]
# print([sol[0]*pi+180,sol[1]*pi+180,sol[2]*pi+180]) 
# raw_input(sol)

sol2 = quaternion_from_euler(sol[0],sol[1],sol[2],'sxyz')   #sol2 = w,x,y,z
# raw_input(sol2)                                            #negative signs are flipped

calc = [[-0.315,-0.205,0],[-0.16,-0.625,0],[0.4,-0.615,0]]

#print((calc[0])[0])

while not rospy.is_shutdown():

  #Set the pose of the marker
  marker.pose.position.x = (calc[2])[0] + 0.15
  marker.pose.position.y = (calc[2])[1] - 0.235
  marker.pose.position.z = (calc[2])[2] + 0.06
  # # Set the pose of the marker
  # start_point = Point()        #start point
  # start_point.x = float(count)
  # start_point.y = 0.0
  # start_point.z = float(count)
  # marker.points.append(start_point)

  # end_point = Point()        #end point
  # end_point.x = float((count + 1))
  # end_point.y = 0.0
  # end_point.z = float((count + 1))
  # marker.points.append(end_point)

  # marker.id += 1
  count += 1
  
  marker.action = 0
  marker_pub.publish(marker)

  #clears the array
  for i in range(len(marker.points)):
    marker.points.pop(0)

  #marker.points.pop(0)
  # raw_input("test")
  rospy.rostime.wallsleep(0.1)
        



