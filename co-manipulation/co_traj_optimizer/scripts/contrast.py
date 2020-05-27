#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError




def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    try:
        cv_image = cv_image * 256
#        print("Working")
    except:
        print("image_callback error")
        return
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image))


def camera_info_callback(msg):
    info_pub.publish(msg)

if __name__ == '__main__':
    try:
        # talker()
        rospy.init_node('talker', anonymous=True)

        image_pub = rospy.Publisher('/camera/ir_augmented/image', Image, queue_size=3)
        # rect_pub = rospy.Publisher('/camera/ir_augmented/image_rect', Image, queue_size=3)
        info_pub = rospy.Publisher('/camera/ir_augmented/camera_info', CameraInfo, queue_size=3)

        rospy.Subscriber('/camera/ir/image', Image, image_callback)
        # rospy.Subscriber('/camera/ir/image', Image, image_callback)
        rospy.Subscriber('/camera/ir/camera_info', CameraInfo, camera_info_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
