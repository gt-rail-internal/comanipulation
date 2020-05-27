import actionlib
import rospy
import tf

import numpy as np
from std_msgs.msg import String


def callback(data):
    rospy.sleep(0.5)
    i = 0
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # rospy.loginfo("I heared %s", data.data)
        rospy.loginfo("Time is :%d", i)
        rate.sleep()
        i = i + 1


rospy.init_node("tf_subscriber")
rospy.Subscriber("human_tfs", String, callback)
rospy.spin()




# if __name__ == "__main__":
