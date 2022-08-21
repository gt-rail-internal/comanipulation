
import rospy
from learning_tf2.msg import Num

def talker():
    rospy.init_node('custom_talker', anonymous=True)
    pub1 = rospy.Publisher('custom_chatter1', Num, queue_size=10)
    pub2 = rospy.Publisher('custom_chatter2', Num, queue_size=10)
    
    r = rospy.Rate(10) #10hz
    msg1 = Num()
    msg2 = Num()

    while not rospy.is_shutdown():
        msg1.head.stamp = rospy.get_rostime()
        msg2.head.stamp = rospy.get_rostime()
        msg1.statement = bool(True)
        msg2.statement = bool(False)
        #rospy.loginfo(msg)
        pub1.publish(msg1)
        pub2.publish(msg2)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass