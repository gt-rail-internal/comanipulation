#! /usr/bin/env python

import rospy
import csv
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker


def get_future(frame):
    future_rows = []
    with open('../trajectories/temp.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        rows = [r for r in csv_reader]
        for i in range(150):
            curr_row = rows[(frame + i) % len(rows)]
            curr_row = [float(i) for i in curr_row]
            future_rows.append(curr_row)
    return str(future_rows)

def get_messages(frame):
    right_shoulder = 12
    right_elbow = 13
    right_wrist = 14
    right_hand = 15
    marker_array = [Marker() for i in range(32)]
    with open('../trajectories/temp.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        rows = [r for r in csv_reader]
        curr_row = rows[frame % len(rows)]

        marker_array[right_shoulder].pose.position.x = float(curr_row[0])
        marker_array[right_shoulder].pose.position.y = float(curr_row[1])
        marker_array[right_shoulder].pose.position.z = float(curr_row[2])

        marker_array[right_elbow].pose.position.x = float(curr_row[3])
        marker_array[right_elbow].pose.position.y = float(curr_row[4])
        marker_array[right_elbow].pose.position.z = float(curr_row[5])

        marker_array[right_wrist].pose.position.x = float(curr_row[6])
        marker_array[right_wrist].pose.position.y = float(curr_row[7])
        marker_array[right_wrist].pose.position.z = float(curr_row[8])

        marker_array[right_hand].pose.position.x = float(curr_row[9])
        marker_array[right_hand].pose.position.y = float(curr_row[10])
        marker_array[right_hand].pose.position.z = float(curr_row[11])
        
    return MarkerArray(markers=marker_array)

def publish_stream():
    rospy.init_node('phanton_stream', anonymous=False)
    pub = rospy.Publisher('human_traj_stream', MarkerArray, queue_size=10)
    pub_frames = rospy.Publisher('human_traj_future', String, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    frame = 1
    while not rospy.is_shutdown():
        pub.publish(get_messages(frame))
        pub_frames.publish(get_future(frame))
        frame += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_stream()
    except rospy.ROSInterruptException:
        pass


#MARKER DOCUMENTATION - 
# marker.header.frame_id = "/neck"
# marker.type = marker.SPHERE
# marker.action = marker.ADD
# marker.scale.x = 0.2
# marker.scale.y = 0.2
# marker.scale.z = 0.2
# marker.color.a = 1.0
# marker.color.r = 1.0
# marker.color.g = 1.0
# marker.color.b = 0.0
# marker.pose.orientation.w = 1.0
# marker.pose.position.x = float(i) / 50.0
# marker.pose.position.y = float(i) / 40.0 
# marker.pose.position.z = float(i) / 30.0