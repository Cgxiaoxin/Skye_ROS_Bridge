from loguru import logger
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from common.device_mapping.device_mapping_server import DeviceToTopic

def get_topic_and_type(device_to_topic: DeviceToTopic, use_arm = ['A']):
    # 'A' for left arm, 'B' for right arm
    topicName_and_messageType = []
    # Add camera topics
    for camera_name, info in device_to_topic.realsense.items():
        topicName_and_messageType.append((f'/{camera_name}/color/image_raw', Image))
    # Add vitai camera topics
    for sensor_name, info in device_to_topic.tactile_sensor.items():
        topicName_and_messageType.append((f'/{sensor_name}/color/image_raw', Image))
        #topicName_and_messageType.append((f'/{sensor_name}/marker_offset/information', PointCloud2))
    # Add robot topics
    if 'A' in use_arm:
        topicName_and_messageType.extend([
            ('/left_joints', JointState),
            ('/left_tcp_pose', PoseStamped),
            ('/left_gripper_state', JointState),
            #('/left_action_cmd', PoseStamped),
            #('/left_gripper_cmd', PoseStamped),
        ])

    if 'B' in use_arm:
        topicName_and_messageType.extend([
            ('/right_joints', JointState),
            ('/right_tcp_pose', PoseStamped),
            ('/right_gripper_state', JointState),
            #('/right_action_cmd', PoseStamped),
            #('/right_gripper_cmd', PoseStamped),
        ])
    return topicName_and_messageType


