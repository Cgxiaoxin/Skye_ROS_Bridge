import cv2
import copy
import time
import rclpy
import numpy as np
import pyrealsense2 as rs
from loguru import logger
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from common.time_utils import convert_float_to_ros_time

class RealsenseCameraPublisher(Node):
    """
    Realsense Camera publisher class
    """
    def __init__(self,
                 camera_serial_number: str = '036422060422',
                 camera_type: str = 'D400',
                 camera_name: str = 'camera_base',
                 rgb_resolution: tuple = (640, 480),
                 exposure: int = 120,
                 white_balance: int = 5900,  # 2800-6500
                 depth_resolution: tuple = (640, 480),
                 fps: int = 30,
                 decimate: int = 2,  # (0-4) decimation_filter magnitude for point cloud
                 random_sample_point_num: int = 10000,
                 debug: bool = False
                 ):
        node_name = f'{camera_name}_publisher'
        super().__init__(node_name)
        self.fps = fps
        self.debug = debug
        self.exposure = exposure
        self.camera_type = camera_type
        self.camera_name = camera_name
        self.white_balance = white_balance
        self.rgb_resolution = rgb_resolution
        self.depth_resolution = depth_resolution
        self.camera_serial_number = camera_serial_number
        self.random_sample_point_num = random_sample_point_num
        
        self.pipeline = None
        self.depth_scale = None
        self.timestamp_offset = None
        self.timer = self.create_timer(1 / fps, self.timer_callback)
        self.color_publisher_ = self.create_publisher(Image, f'/{camera_name}/color/image_raw', 10)

        self.fps_list = []
        self.frame_count = 0
        self.frame_intervals = []
        self.last_frame_time = None
        self.prev_time = time.time()
        self.last_print_time = time.time()

        # Create a decimation filter
        self.decimate_filter = rs.decimation_filter()
        self.decimate_filter.set_option(rs.option.filter_magnitude, 2 ** decimate)

        # Start the camera
        self.start()

    def set_exposure(self, exposure=None, gain=None):
        """
        exposure: (1, 10000) 100us unit. (0.1 ms, 1/10000s)
        gain: (0, 128)
        """

        if exposure is None and gain is None:
            # auto exposure
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 1.0)
        else:
            # manual exposure
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            if exposure is not None:
                self.color_sensor.set_option(rs.option.exposure, exposure)
            if gain is not None:
                self.color_sensor.set_option(rs.option.gain, gain)

    def set_white_balance(self, white_balance=None):
        if white_balance is None:
            self.color_sensor.set_option(rs.option.enable_auto_white_balance, 1.0)
        else:
            self.color_sensor.set_option(rs.option.enable_auto_white_balance, 0.0)
            self.color_sensor.set_option(rs.option.white_balance, white_balance)

    def start(self):
        # get the context of the connected devices
        context = rs.context()
        devices = context.query_devices()

        # check if there are connected devices
        if len(devices) == 0:
            logger.error("No connected devices found")
            raise Exception("No connected devices found")

        config = rs.config()
        is_camera_valid = False
        for device in devices:
            # check if the device serial number matches the provided serial number
            serial_number = device.get_info(rs.camera_info.serial_number)
            if serial_number == self.camera_serial_number:
                is_camera_valid = True
                break

        # if the provided camera is not found, raise an exception
        if not is_camera_valid:
            logger.error("Camera with serial number {} not found".format(self.camera_serial_number))
            raise Exception("Camera with serial number {} not found".format(self.camera_serial_number))

        # Start the camera
        config.enable_device(self.camera_serial_number)
        self.pipeline = rs.pipeline()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        assert device_product_line == self.camera_type, f'With {self.camera_name}, Camera type does not match the camera product line.'
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # report global time
        # https://github.com/IntelRealSense/librealsense/pull/3909
        self.color_sensor = device.first_color_sensor()
        self.color_sensor.set_option(rs.option.global_time_enabled, 1)
        # realsense exposure
        # self.set_exposure(exposure=self.exposure, gain=0)
        self.set_exposure(exposure=None, gain=None) # use auto exposure
        # realsense white balance
        self.set_white_balance(white_balance=None) #self.white_balance

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # set the resolution and format of the camera
        config.enable_stream(rs.stream.color, self.rgb_resolution[0], self.rgb_resolution[1], rs.format.bgr8, int(self.fps))
        self.pipeline.start(config)
        logger.debug("Camera started!")

        # capture some frames for the camera to stabilize
        logger.debug("Capturing some frames for the camera to stabilize...")
        for _ in range(self.fps):
            self.pipeline.wait_for_frames()

        # Capture initial frames to get initial timestamps
        frames = self.pipeline.wait_for_frames()

        initial_frame = frames.get_color_frame()
        if not initial_frame:
            logger.error("Failed to get initial frame")
            raise ValueError("Failed to get initial frame")

        # convert the camera timestamp to system timestamp
        initial_camera_timestamp = convert_float_to_ros_time(initial_frame.get_timestamp() / 1000)  # convert to time class in ROS
        # we assume that the internal clock of realsense is synchronized with the system clock
        initial_system_timestamp = self.get_clock().now()

        # Calculate timestamp offset
        self.timestamp_offset = initial_system_timestamp - initial_camera_timestamp
        logger.debug(f"Timestamp offset: {self.timestamp_offset.nanoseconds / 1e6} ms")
        logger.debug("Camera is ready! Start publishing images...")

    def stop(self):
        # Stop the camera
        self.pipeline.stop()
        logger.info("Camera stopped!")

    def convert_to_system_timestamp(self, camera_timestamp: Time) -> Time:
        """
        Convert camera timestamp to system timestamp
        """
        return camera_timestamp + self.timestamp_offset

    def publish_color_image(self, color_frame: rs.composite_frame, camera_timestamp: Time):
        """
        Publish color image
        """
        color_image = copy.deepcopy(np.asanyarray(color_frame.get_data()))
        success, encoded_image = cv2.imencode('.jpg', color_image)
        # Fill the message
        msg = Image()
        msg.header.stamp = self.convert_to_system_timestamp(camera_timestamp).to_msg()
        msg.header.frame_id = "camera_color_frame"
        msg.height, msg.width, _ = color_image.shape
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        if success:
            image_bytes = encoded_image.tobytes()
            msg.data = image_bytes
        else:
            logger.debug('fail to encode the colorimage!')
            msg.data = color_image.tobytes()
        self.color_publisher_.publish(msg)
    
    def timer_callback(self):
        """
        Publish the color and depth frames
        """

        while True:
            # capture frames
            frames = self.pipeline.wait_for_frames()

            camera_timestamp = convert_float_to_ros_time(frames.get_timestamp() / 1000)

            # we only record the raw color frame
            raw_color_frame = frames.get_color_frame()

            color_frame = raw_color_frame
            depth_frame = None
            if not color_frame:
                continue

            # publish the color image
            self.publish_color_image(raw_color_frame, camera_timestamp)

            # calculate fps
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            if elapsed_time >= 1.0:
                frame_rate = self.frame_count / elapsed_time
                self.fps_list.append(frame_rate)
                logger.debug(f"Frame rate: {frame_rate:.2f} FPS")
                self.prev_time = current_time
                self.frame_count = 0

            # calculate interval between frames
            if self.last_frame_time is not None:
                frame_interval = (current_time - self.last_frame_time) * 1000
                self.frame_intervals.append(frame_interval)
            self.last_frame_time = current_time

            # Print info and make plots every 5 seconds
            if current_time - self.last_print_time >= 5:
                logger.info(f"Publishing image from {self.camera_name} at timestamp (s): {camera_timestamp.nanoseconds / 1e9}")
                self.last_print_time = current_time
            break

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCameraPublisher(camera_name='external_camera')
    try:
        rclpy.spin(node)
    except IndentationError as e:
        logger.exception(e)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()