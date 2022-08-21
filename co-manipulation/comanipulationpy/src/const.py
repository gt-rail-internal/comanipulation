import os.path as path
from collections import namedtuple

BASE_LINK_WORLD_OFFSET = [0, 0, 0] #0.5, -0.2, -1.0
RIGHT_SHOULDER_BASE_LINK_OFFSET = [0.]*3
RIGHT_SHOULDER_OFFSETS =   {120 : [-1.5, -0.4, 0.1], 
                            102 : [0.]*3,
                            103 : [0.]*3,
                            104 : [0.]*3,
                            165 : [0.]*3,
                            5 : [0.]*3,#
                            2 : [0.]*3,#
                            240 : [-1.5, -0.4, 0.1],#
                            1 : [0.]*3,#
                            3 : [0.]*3,#
                            4 : [0.]*3,#
                            6 : [0.]*3,#
                            7 : [0.]*3,#
                            99 : [0.]*3, 
                            201 : [0., 0., -0.13],
                            202 : [0.]*3,
                            203 : [0., 0., 0.17],
                            204 : [0.]*3,
                            604 : [0.]*3,#
                            621 : [0.]*3,#
                            640 : [0.]*3,#
                            669 : [0.]*3,#
                            674 : [0.]*3,#
                            101 : [0.]*3,#
                            681 : [0.]*3,#
                            703 : [0.]*3,#
                            999 : [0.]*3,#
                            3014: [0.]*3,#
                            3008: [0.]*3,
                            3012: [0.]*3,
                            4006: [0.]*3,
                            4008: [0.]*3,
                            4010: [0.]*3,
                            6012: [0, 0, 0],
                            5002: [0, 0.2, 0],
                            6008: [0,0,0],
                            # 6010: [0]*3,
                            100002: [0]*3,
                            }

# RIGHT_SHOULDER_BASE_LINK_OFFSET = [-2.0, -0.5, 0]
# RIGHT_SHOULDER_OFFSETS = {120 : [-1.5, -0.4, 0.1], 
#                         124 : [-1.5, -0.5, 0.1],
#                         131 : [-1.9, -0.5, 0.1],
#                         144 : [-1.5, -0.5, 0],
#                         165 : [-1.9, -0.5, 0],
#                         204 : [-1.5, -0.4, 0],#
#                         221 : [-1.9, -0.5, 0],#
#                         240 : [-1.5, -0.5, 0],#
#                         269 : [-1.9, -0.5, 0],#
#                         274 : [-1.9, -0.5, 0],#
#                         276 : [-1.9, -0.5, 0],#
#                         281 : [-1.9, -0.5, 0],#
#                         303 : [-1.9, -0.5, 0],#
#                         520 : [-1.5, -0.4, 0.1], 
#                         524 : [-1.5, -0.5, 0.1],
#                         531 : [-1.9, -0.5, 0.1],
#                         544 : [-1.5, -0.5, 0],
#                         565 : [-1.9, -0.5, 0],
#                         604 : [-1.5, -0.4, 0],#
#                         621 : [-1.9, -0.5, 0],#
#                         640 : [-1.5, -0.5, 0],#
#                         669 : [-1.9, -0.5, 0],#
#                         674 : [-1.9, -0.5, 0],#
#                         676 : [-1.9, -0.5, 0],#
#                         681 : [-1.9, -0.5, 0],#
#                         703 : [-1.9, -0.5, 0],#
#                         999 : [-1.9, -0.5, 0],#
#                         3014: [-1.5, -0.5, 0.1],#
#                         3008: [-1.5, -0.5, 0.1],
#                         3012: [-1.5, -0.5, 0.1],
#                         4006: [-1.5, -0.5, 0.1],
#                         4005: [-1.5, -0.5, 0.1],
#                         }

RobotInfo = namedtuple(
    "RobotInfo", "model arm_name eef_link_name all_links controller_name controller_joints")

PARENT_DIR = path.abspath(path.join(path.dirname(path.abspath(__file__)), path.pardir))
DATA_FOLDER = path.join(PARENT_DIR, 'dae')

ROBOTS_DICT = {
    "jaco": RobotInfo(path.join(DATA_FOLDER, "jaco-test.dae"), "test_arm", "j2s7s300_ee_link",
                      ["j2s7s300_ee_link", "j2s7s300_link_6", "j2s7s300_link_4",
                       "j2s7s300_link_7", "j2s7s300_link_5", "j2s7s300_link_3", "j2s7s300_link_2"],
                      "/j2s7s300/effort_joint_trajectory_controller", ["j2s7s300_joint_1", "j2s7s300_joint_2",
                                                     "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6",
                                                     "j2s7s300_joint_7"]),
    "jaco-real": RobotInfo(path.join(DATA_FOLDER, "jaco-test.dae"), "test_arm", "j2s7s300_ee_link",
                      ["j2s7s300_ee_link", "j2s7s300_link_6", "j2s7s300_link_4",
                       "j2s7s300_link_7", "j2s7s300_link_5", "j2s7s300_link_3", "j2s7s300_link_2"],
                      "/jaco_trajectory_controller", ["j2s7s300_joint_1", "j2s7s300_joint_2",
                                                     "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6",
                                                     "j2s7s300_joint_7"]),
    "franka": RobotInfo(path.join(DATA_FOLDER, "panda_default.dae"), "panda_arm", "panda_hand",
                        ["panda_link1", "panda_link2", "panda_link3",
                         "panda_link4", "panda_link5", "panda_link6", "panda_link7"],
                        "panda_arm_controller", ["panda_joint1", "panda_joint2", "panda_joint3",
                                                 "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]),
    "iiwa": RobotInfo(path.join(DATA_FOLDER, "iiwa_env.dae"), "iiwa_arm", 'iiwa_link_ee',
                      ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
                       "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"],
                      "iiwa/PositionJointInterface_trajectory_controller", ["iiwa_joint_1",
                                                                            "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6",
                                                                            "iiwa_joint_7"])
}
