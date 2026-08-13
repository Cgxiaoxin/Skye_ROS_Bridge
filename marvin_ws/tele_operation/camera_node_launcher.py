'''
This file reads mapping from the api server
and launches all the nodes
'''
import os
import pdb
import time
import hydra
import rclpy
import psutil
import signal
import requests
import threading
import multiprocessing
from loguru import logger
from omegaconf import DictConfig, OmegaConf


# from common.publisher.usb_camera_publisher_vitai import UsbCameraPublisher
from publisher.tactile_sensor_publisher import TactileSensorPublisher
from publisher.realsense_camera_publisher import RealsenseCameraPublisher
from common.device_mapping.device_mapping_server import DeviceToTopic, DeviceMappingServer

# add this to prevent assigning too may threads when using numpy
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

import cv2
# add this to prevent assigning too may threads when using open-cv
cv2.setNumThreads(12)

import ctypes
libc = ctypes.CDLL("libc.so.6")
SCHED_RR = 2  # real-time scheduling policy
class SchedParam(ctypes.Structure):
    _fields_ = [("sched_priority", ctypes.c_int)]
param = SchedParam()
param.sched_priority = 99  # highest priority

pid = os.getpid()
if libc.sched_setscheduler(pid, SCHED_RR, ctypes.byref(param)) != 0:
    raise OSError("Failed to set scheduler")

def to_plain_dict(cfg):
    return OmegaConf.to_container(cfg, resolve=True)


def prepare_realsense_config(camera_config, camera_info):
    cfg = to_plain_dict(camera_config)
    cfg["publisher_type"] = "realsense"
    cfg["camera_serial_number"] = camera_info.device_id
    return OmegaConf.create(cfg)


def prepare_tactile_config(sensor_config, sensor_info, sensor_index):
    cfg = to_plain_dict(sensor_config)
    cfg["publisher_type"] = "tactile_sensor"
    cfg["camera_index"] = sensor_index
    cfg["camera_name"] = cfg.pop("sensor_name", cfg.get("camera_name"))
    cfg["camera_sn"] = cfg.pop("sensor_sn", sensor_info.device_id)
    if "marker_dimension" in cfg:
        cfg["dimension"] = cfg.pop("marker_dimension")
    cfg.pop("sensor_type", None)
    if not cfg.get("camera_name") or not cfg.get("camera_sn"):
        raise ValueError(f"Invalid tactile sensor config: {cfg}")
    return OmegaConf.create(cfg)


class CameraWorker:
    def __init__(self, camera_config):
        self.camera_config = camera_config
        publisher_type = camera_config.publisher_type
        publisher_kwargs = to_plain_dict(camera_config)
        publisher_kwargs.pop("publisher_type", None)
        if publisher_type == 'realsense':
            self.camera_publisher = RealsenseCameraPublisher(**publisher_kwargs)
        elif publisher_type == 'tactile_sensor':
            self.camera_publisher = TactileSensorPublisher(**publisher_kwargs)
        else:
            raise NotImplementedError(f"Unsupported publisher_type: {publisher_type}")

    def handle_signal(self, signum, frame):
        self.camera_publisher.stop()
        logger.info(f"Stopped {self.camera_config.camera_name} camera publisher")
        self.camera_publisher.destroy_node()

def start_camera_publisher(camera_config):
    # bind the process to the specific cpu core to prevent jitter
    cpu_core_id = set(camera_config.cpu_core_id)
    total_cores = psutil.cpu_count()
    for id in cpu_core_id:
        if id >= total_cores:
            raise ValueError(f"Invalid cpu_id: {id}, total cores: {total_cores}")
    os.sched_setaffinity(0, cpu_core_id)
    camera_config = to_plain_dict(camera_config)
    camera_config.pop("cpu_core_id", None)
    camera_config = OmegaConf.create(camera_config)
    rclpy.init(args=None)
    worker = CameraWorker(camera_config)
    signal.signal(signal.SIGUSR1, worker.handle_signal)
    logger.info(f"Starting {camera_config.camera_name} camera publisher")
    rclpy.spin(worker.camera_publisher)

@hydra.main(
    config_path="config", config_name="real_world_env", version_base="1.3"
)
def main(cfg: DictConfig):

    try:
        device_mapper_server = DeviceMappingServer(publisher_cfg=cfg.publisher,
                                                   **cfg.device_mapping_server)
        device_mapping_thread = threading.Thread(target=device_mapper_server.run, daemon=True)
        device_mapping_thread.start()
        time.sleep(1)

        # require the latest mapping of name and topic from fastAPI
        response = requests.get(f"http://{cfg.device_mapping_server.host_ip}:{cfg.device_mapping_server.port}/get_mapping")
        device_to_topic = DeviceToTopic.model_validate(response.json())

        # launch the subprocesses based on the mapping from fastapi server
        processes = []
        # Handle realsense cameras
        for camera_name, camera_info in device_to_topic.realsense.items(): # get realsense 
            camera_config = None
            for cam in cfg.publisher.realsense_camera_publisher:
                if cam.camera_name == camera_name:
                    camera_config = cam
                    break
            if camera_config:
                camera_config = prepare_realsense_config(camera_config, camera_info)

                p = multiprocessing.Process(target=start_camera_publisher, args=(camera_config,))
                processes.append(p)
                p.start()

        # Handle tactile sensors cameras
        for sensor_index, (sensor_name, sensor_info) in enumerate(device_to_topic.tactile_sensor.items()):  
            sensor_config = None
            for sensor in cfg.publisher.tactile_sensor_publisher:
                if sensor.sensor_name == sensor_name:
                    sensor_config = sensor
                    break
            if sensor_config:
                sensor_config = prepare_tactile_config(sensor_config, sensor_info, sensor_index)

                p = multiprocessing.Process(target=start_camera_publisher, args=(sensor_config,))
                processes.append(p)
                p.start()
                time.sleep(2)

        device_mapping_thread.join()
    except KeyboardInterrupt:
        for p in processes:
            os.kill(p.pid, signal.SIGUSR1)
        time.sleep(2)
    finally:
        for p in processes:
            p.terminate()
        for p in processes:
            p.join()
        rclpy.shutdown()
        logger.info("All Camera publishers shutdown")

if __name__ == "__main__":
    main()
