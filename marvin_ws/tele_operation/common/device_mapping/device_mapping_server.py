'''
This file initiate the DeviceMappingServer
The server then dynamic maintain the mapping between
cameras and the topics
'''
import uvicorn
import subprocess
from typing import Dict
from loguru import logger
import pyrealsense2 as rs
from fastapi import FastAPI
from pydantic import BaseModel
from omegaconf import DictConfig

class RealsenseCameraInfo(BaseModel):
    topic_image: str
    device_id: str
    type: str

class TactileSensorInfo(BaseModel):
    topic_image: str
    device_id: str
    type: str

class DeviceToTopic(BaseModel):
    realsense: Dict[str, RealsenseCameraInfo] = {}
    tactile_sensor: Dict[str, TactileSensorInfo] = {}

class DeviceMappingServer:
    """Server class that defines the device mapping (device to ROS topic name)"""
    def __init__(self, publisher_cfg: DictConfig, host_ip: str = '127.0.0.1', port: int = 8062):
        self.host_ip = host_ip
        self.port = port

        self.app = FastAPI()
        self.device_to_topic_mapping = DeviceToTopic()
        self.init_mapping(publisher_cfg)
        self.setup_routes()

    def setup_routes(self):
        @self.app.get("/get_mapping", response_model=DeviceToTopic)
        def get_mapping() -> DeviceToTopic:
            return self.device_to_topic_mapping

    @staticmethod
    def get_tactile_sensor_ids():
        result = subprocess.run(['v4l2-ctl', '--list-devices'], stdout=subprocess.PIPE, text=True)
        output = result.stdout

        camera_ids = []
        lines = output.split('\n')
        current_camera_name = None
        found_video_path = False

        for line in lines:
            if line.strip() == '':
                current_camera_name = None
                found_video_path = False
                continue

            if line.startswith('\t'):
                '''obtain the device id of the tactile sensor, which is the number after "video" in the path like "/dev/videoX"
                '''
                if (current_camera_name and 'ViTai' in current_camera_name and '/dev/video' in line and not found_video_path):
                    device_id = line.split('/')[-1]
                    camera_ids.append(int(device_id.replace('video', '')))
                    found_video_path = True
            else:
                '''
                obtain the name of the tactile sensor
                '''
                current_camera_name = line.strip()
        return camera_ids

    def init_mapping(self, publisher_cfg: DictConfig):
        '''
        get the device ids of the cameras in sequence
        '''
        tactile_sensor_ids = self.get_tactile_sensor_ids()

        # realsense camera
        ## 1. Query all connected realsense cameras
        ## 2. Match the realsense cameras using the serial number defined in the config
        ## 3. Map the camera names defined in the config to Info class containing: ros topic, serial number and type.
        if publisher_cfg.realsense_camera_publisher is not None:
            for rs_cam in publisher_cfg.realsense_camera_publisher:
                context = rs.context()
                for device in context.query_devices():
                    if device.get_info(rs.camera_info.serial_number) == rs_cam.camera_serial_number:
                        self.device_to_topic_mapping.realsense[rs_cam.camera_name] = RealsenseCameraInfo(
                            topic_image=f"/{rs_cam.camera_name}/color/image_raw",
                            device_id=rs_cam.camera_serial_number,
                            type="realsense"
                        )
                        break

        # tactile sensor
        if publisher_cfg.tactile_sensor_publisher is not None:
            for tactile_sensor in publisher_cfg.tactile_sensor_publisher:
                self.device_to_topic_mapping.tactile_sensor[tactile_sensor.sensor_name] = TactileSensorInfo(
                    topic_image=f'/{tactile_sensor.sensor_name}/color/image_raw',
                    device_id=tactile_sensor.sensor_sn,
                    type='tactile_sensor'
                )

    def run(self):
        logger.info(f"Device mapping server is running on {self.host_ip}:{self.port}")
        uvicorn.run(self.app, host=self.host_ip, port=self.port)
