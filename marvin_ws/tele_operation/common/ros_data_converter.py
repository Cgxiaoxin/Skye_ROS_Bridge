import copy
import numpy as np
from typing import Dict, List
from common.space_utils import ros_pose_to_6d_pose
from common.data_models import Vitai_SensorMessage_Double

class ROS2DataConverter:
    """
    Data converter class that converts ROS2 topic data into Pydantic data models
    """
    def __init__(self,
                 use_arm: List[str] = ['A'],
                 tactile_camera_marker_dimension: int = 2,
                 ):
        self.use_arm = use_arm
        self.tactile_camera_marker_dimension = tactile_camera_marker_dimension

    def convert_all_data(self, topic_dict: Dict) -> Vitai_SensorMessage_Double: # changed to Vitai
        sensor_msg_args = dict()
        # calculate the lastest timestamp in the topic_dict
        latest_timestamp = max([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                                for msg in topic_dict.values()])
        sensor_msg_args['timestamp'] = latest_timestamp
        # convert external camera rgb image
        if '/external_camera/color/image_raw' in topic_dict:
            sensor_msg_args['externalCameraRGB'] = np.frombuffer(topic_dict['/external_camera/color/image_raw'].data, np.uint8)
        # left robot arm
        if 'A' in self.use_arm:
            # convert robot states
            ## ee pose
            if '/left_tcp_pose' in topic_dict:
                left_tcp_pose_array = ros_pose_to_6d_pose(topic_dict['/left_tcp_pose'].pose)
                sensor_msg_args['leftRobotTCP'] = left_tcp_pose_array
            ## gripper state
            if '/left_gripper_state' in topic_dict:
                left_gripper_state_array = np.array([topic_dict['/left_gripper_state'].position[0],
                                                    topic_dict['/left_gripper_state'].effort[0]])
                sensor_msg_args['leftRobotGripperState'] = left_gripper_state_array
            ## joints
            if '/left_joints' in topic_dict:
                left_joints_array = np.array(topic_dict['/left_joints'].position)
                sensor_msg_args['leftRobotJoints'] = left_joints_array
            ## left action
            if '/left_action_cmd' in topic_dict:
                left_action_array = ros_pose_to_6d_pose(topic_dict['/left_action_cmd'].pose)
                sensor_msg_args['leftRobotAction'] = left_action_array
            ## left gripper action
            if '/left_gripper_cmd' in topic_dict:
                left_gripper_action_array = np.array([topic_dict['/left_gripper_cmd'].pose.position.x])
                sensor_msg_args['leftRobotGripperAction'] = left_gripper_action_array
            # convert wrist images and tactile sensors
            ## wrist images and tactile sensor images
            if '/left_wrist_camera/color/image_raw' in topic_dict:
                sensor_msg_args['leftWristCameraRGB'] = np.frombuffer(topic_dict['/left_wrist_camera/color/image_raw'].data, np.uint8)
            if '/left_gripper_camera_1/color/image_raw' in topic_dict:
                sensor_msg_args['leftGripperCameraRGB1'] = np.frombuffer(topic_dict['/left_gripper_camera_1/color/image_raw'].data, np.uint8)
            if '/left_gripper_camera_2/color/image_raw' in topic_dict:
                sensor_msg_args['leftGripperCameraRGB2'] = np.frombuffer(topic_dict['/left_gripper_camera_2/color/image_raw'].data, np.uint8)
            ## tactile marker info
            if '/left_gripper_camera_1/marker_offset/information' in topic_dict:
                left_tactile_data1 = np.frombuffer(topic_dict['/left_gripper_camera_1/marker_offset/information'].data, dtype=np.float32).reshape(-1, 2*self.tactile_camera_marker_dimension)
                sensor_msg_args['leftGripperCameraMarkerOffset1'] = left_tactile_data1[:, self.tactile_camera_marker_dimension:]
            if '/left_gripper_camera_2/marker_offset/information' in topic_dict:
                left_tactile_data2 = np.frombuffer(topic_dict['/left_gripper_camera_2/marker_offset/information'].data, dtype=np.float32).reshape(-1, 2*self.tactile_camera_marker_dimension)
                sensor_msg_args['leftGripperCameraMarkerOffset2'] = left_tactile_data2[:, self.tactile_camera_marker_dimension:]

        # right robot arm
        if 'B' in self.use_arm:
            ## ee pose
            if '/right_tcp_pose' in topic_dict:
                right_tcp_pose_array = ros_pose_to_6d_pose(topic_dict['/right_tcp_pose'].pose)
                sensor_msg_args['rightRobotTCP'] = right_tcp_pose_array
            ## gripper state
            if '/right_gripper_state' in topic_dict:
                right_gripper_state_array = np.array([topic_dict['/right_gripper_state'].position[0],
                                                    topic_dict['/right_gripper_state'].effort[0]])
                sensor_msg_args['rightRobotGripperState'] = right_gripper_state_array
            ## joints
            if '/right_joints' in topic_dict:
                right_joints_array = np.array(topic_dict['/right_joints'].position)
                sensor_msg_args['rightRobotJoints'] = right_joints_array
            ## right action
            if '/right_action_cmd' in topic_dict:
                right_action_array = ros_pose_to_6d_pose(topic_dict['/right_action_cmd'].pose)
                sensor_msg_args['rightRobotAction'] = right_action_array
            ## right gripper action
            if 'right_gripper_cmd' in topic_dict:
                right_gripper_action_array = np.array([topic_dict['/right_gripper_cmd'].pose.position.x])
                sensor_msg_args['rightRobotGripperAction'] = right_gripper_action_array
            # convert wrist images and tactile sensors
            ## wrist images and tactile sensor images
            if '/right_wrist_camera/color/image_raw' in topic_dict:
                sensor_msg_args['rightWristCameraRGB'] = np.frombuffer(topic_dict['/right_wrist_camera/color/image_raw'].data, np.uint8)
            if '/right_gripper_camera_1/color/image_raw' in topic_dict:
                sensor_msg_args['rightGripperCameraRGB1'] = np.frombuffer(topic_dict['/right_gripper_camera_1/color/image_raw'].data, np.uint8)
            if '/right_gripper_camera_2/color/image_raw' in topic_dict:
                sensor_msg_args['rightGripperCameraRGB2'] = np.frombuffer(topic_dict['/right_gripper_camera_2/color/image_raw'].data, np.uint8)
            ## tactile marker info
            if '/right_gripper_camera_1/marker_offset/information' in topic_dict:
                right_tactile_data1 = np.frombuffer(topic_dict['/right_gripper_camera_1/marker_offset/information'].data, dtype=np.float32).reshape(-1, 2*self.tactile_camera_marker_dimension)
                sensor_msg_args['rightGripperCameraMarkerOffset1'] = right_tactile_data1[:, self.tactile_camera_marker_dimension:]
            if '/right_gripper_camera_2/marker_offset/information' in topic_dict:
                right_tactile_data2 = np.frombuffer(topic_dict['/right_gripper_camera_2/marker_offset/information'].data, dtype=np.float32).reshape(-1, 2*self.tactile_camera_marker_dimension)
                sensor_msg_args['rightGripperCameraMarkerOffset2'] = right_tactile_data2[:, self.tactile_camera_marker_dimension:]
        
        sensor_msg = Vitai_SensorMessage_Double(**sensor_msg_args)
        
        return sensor_msg
        
    
 

