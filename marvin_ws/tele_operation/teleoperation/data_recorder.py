import os
import time
import pickle
import uvicorn
import requests
import threading
import os.path as osp
from typing import List
from loguru import logger
from fastapi import FastAPI
from rclpy.node import Node

from common.ros_data_converter import ROS2DataConverter
from common.time_utils import check_sync, check_timestamp
from message_filters import ApproximateTimeSynchronizer, Subscriber
from common.device_mapping.device_mapping_server import DeviceToTopic
from common.device_mapping.device_mapping_utils import get_topic_and_type
from common.data_models import SensorMessageList, Vitai_SensorMessage_Double 

class DataRecorder(Node):
    last_sensor_msg: Vitai_SensorMessage_Double = None
    def __init__(self,
                 base_dir: str = 'data',
                 use_arm: List[str] = ['A'],
                 data_recorder_ip: str = '192.168.1.165',
                 data_recorder_port: int = 8092,
                 device_mapping_server_ip: str = '127.0.0.1',
                 device_mapping_server_port: int = 8062,
                 tactile_camera_marker_dimension: int = 3
                 ):
        super().__init__('sync_listener')
        print("Initializing DataRecorder...")

        # init configs
        self.save_dir = base_dir
        self.time_check = False
        self.subscribers = []
        self.sensor_msg_list: SensorMessageList = SensorMessageList(sensorMessages=[])
        self.IS_SAVING_FLAG = True # save data when False
        self.use_arm = use_arm

        # Get device to topic mapping
        response = requests.get(f"http://{device_mapping_server_ip}:{device_mapping_server_port}/get_mapping")
        self.device_to_topic_mapping = DeviceToTopic.model_validate(response.json())
        topicName_and_messageType = get_topic_and_type(self.device_to_topic_mapping,  use_arm=self.use_arm)
        print(topicName_and_messageType )
        
        # setup subscribers
        for name, msg_type in topicName_and_messageType:
            self.subscribers.append(Subscriber(self, msg_type, name))
            logger.debug(f"Subscribed to topic: {name} with type: {msg_type}")
        self.ts = ApproximateTimeSynchronizer(self.subscribers, queue_size=10, slop=0.1,
                                              allow_headerless=False)
        self.ts.registerCallback(self.callback)

        # calculating FPS
        self.timestamps = dict()
        self.prev_time = None
        self.frame_count = 0
        
        # extract data from ros message
        self.data_converter = ROS2DataConverter(
            use_arm = use_arm,
            tactile_camera_marker_dimension = tactile_camera_marker_dimension
            )
        
        # setting up FastAPI server for external control
        self.app = FastAPI()
        self.setup_routes()
        self.data_recorder_ip = data_recorder_ip
        self.data_recorder_port = data_recorder_port
        self.start_fastapi_server()

    def start_fastapi_server(self):
        """启动FastAPI服务器"""
        def run_server():
            uvicorn.run(self.app, host=self.data_recorder_ip, port=self.data_recorder_port, log_level="info")
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        logger.info(f"DataRecorder API server started on http://{self.data_recorder_ip}:{self.data_recorder_port}")

    def setup_routes(self):
        @self.app.post('/start_episodes')
        async def start_episodes():
            self.IS_SAVING_FLAG = False
            logger.info("开始记录episode数据")
            return {'status': 'started', 'message': '开始记录数据'}
        
        @self.app.post('/save_episodes')
        async def save_episodes():
            self.IS_SAVING_FLAG = True
            logger.info("保存episode数据")
            self.save()
            logger.info("episode数据保存完成")
            return {'status': 'saved', 'message': '数据保存完成'}

    def callback(self, *msgs):
        if self.IS_SAVING_FLAG:
            return
        # data collection
        topic_dict = dict()
        for i, msg in enumerate(msgs):
            topic_name = self.subscribers[i].topic
            topic_dict[topic_name] = msg       
        sensor_msg = self.data_converter.convert_all_data(topic_dict)
        self.sensor_msg_list.append(sensor_msg)
        
        # time check
        if self.time_check:
            # check the time differences across topics and interval between time stamps
            for i, msg in enumerate(msgs):
                topic_name = self.subscribers[i].topic
                if topic_name not in self.timestamps:
                    self.timestamps[topic_name] = []
                self.timestamps[topic_name].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        # calculate fps
        self.frame_count += 1
        current_time = time.time()
        if self.prev_time == None:
            self.prev_time = current_time
        elapsed_time = current_time - self.prev_time
        if elapsed_time >= 1.0:
            frame_rate = self.frame_count / elapsed_time
            logger.debug(f"Frame rate: {frame_rate:.2f} FPS")
            self.prev_time = current_time
            self.frame_count = 0
            if self.time_check:
                check_sync()
                check_timestamp()

    def save(self):
        # save sensor_msg_list to pickle file
        logger.debug('Trying to save sensor messages...')
        if not osp.exists(osp.dirname(self.save_dir)):
            os.makedirs(osp.dirname(self.save_dir))
        
        # get save_file_path
        pkl_files = [f for f in os.listdir(self.save_dir) if f.endswith('.pkl')]
        file_count = len(pkl_files)
        save_file_path = osp.join(self.save_dir, f'episode_{file_count:04d}.pkl')

        with open(save_file_path, 'wb') as f:
            pickle.dump(self.sensor_msg_list.get_all(), f)
        self.sensor_msg_list.clear()
        logger.info(f"Saved sensor messages to {save_file_path}")
        



