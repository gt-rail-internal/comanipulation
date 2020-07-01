import rospy
import tf

from actionlib_msgs.msg import GoalStatusArray, GoalID
from sensor_msgs.msg import JointState
import sys
from scene_utils import Scene

from const import ROBOTS_DICT
from metrics import get_separation_dist

ESTOP_THRESHOLD = 0.1

joint_state_topics = {
    "jaco": "/j2s7s300/joint_states"
}

global robot_joints
robot_joints = []

def get_human_pose_fake(listener):
    """
    Returns a cartesian-space human pose in the world frame
    """
    pose = []
    for joint_name in ['/right_shoulder', '/right_elbow', '/right_wrist', '/right_palm', '/neck', '/head', '/torso', '/left_shoulder', '/left_elbow', '/left_wrist', '/left_palm']:
        try:
            translation, _ = listener.lookupTransform("/world", joint_name, rospy.Time(0))
            pose.extend(translation)
        except:
            return []
    return pose

def get_human_pose_real():
    """
    TODO: Implement this
    """
    return []

def set_robot_joints(data):
    """
    Rospy callback that sets robot_joints to the values given by parameter `data`
    """
    global robot_joints
    pass

def stop_robot(controller_topic):
    """
    Stops the robot by cancelling the in-progress trajectory
    """
    msg = rospy.wait_for_message(controller_topic+"/follow_joint_trajectory/status", GoalStatusArray)
    stopper = rospy.Publisher(controller_topic+"/follow_joint_trajectory/cancel", GoalID, queue_size=1)
    stopper.publish(msg.status_list[-1].goal_id)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("""Usage: estop.py [robot type] [human traj type]
        robot type: any of "jaco", "jaco-real", "franka", or "iiwa"
        human traj type: "real" or "fake"
        """)
        sys.exit(1)
    rospy.init_node("conimbus_estop")

    listener = tf.TransformListener()
    rate = rospy.Rate(10)

    robot_info = ROBOTS_DICT[sys.argv[1]]
    joint_topic = joint_state_topics[sys.argv[1]]
    scene = Scene(robot_info)
    rospy.Subscriber(joint_topic, JointState, set_robot_joints)

    while not rospy.is_shutdown():
        if sys.argv[2] == "fake":
            human_pose = get_human_pose_fake(listener)
        elif sys.argv[2] == "real":
            human_pose = get_human_pose_real()
        
        distance = get_separation_dist(scene, human_pose, robot_joints)
        if distance < ESTOP_THRESHOLD:
            stop_robot(robot_info.controller_name)
        pass