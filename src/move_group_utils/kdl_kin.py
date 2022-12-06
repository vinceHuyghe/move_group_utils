import numpy as np
import PyKDL
import rospy

from urdf_parser_py.urdf import URDF

# set constants for joints
SHOULDER_PAN_JOINT = 'shoulder_pan_joint'
SHOULDER_LIFT_JOINT = 'shoulder_lift_joint'
ELBOW_JOINT = 'elbow_joint'
WRIST_1_JOINT = 'wrist_1_joint'
WRIST_2_JOINT = 'wrist_2_joint'
WRIST_3_JOINT = 'wrist_3_joint'

# set constants for links
BASE_LINK = 'base_link'
BASE_LINK = 'base_link'
SHOULDER_LINK = 'shoulder_link'
UPPER_ARM_LINK = 'upper_arm_link'
FOREARM_LINK = 'forearm_link'
WRIST_1_LINK = 'wrist_1_link'
WRIST_2_LINK = 'wrist_2_link'
WRIST_3_LINK = 'wrist_3_link'
EE_LINK = 'ee_link'

class KdlKin():
    
    def __init__(self) -> None:
        
        self.urdf = URDF.from_parameter_server()
        self.kdl_tree = kdl_