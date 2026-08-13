import cv2
import time
import rclpy
import struct
import numpy as np
from loguru import logger
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from pyvitaisdk import VTSensor, VTSDeviceFinder, VTSDataType

class TactileSensorPublisher(Node):
    '''
    Tactile Sensor publisher Class
    '''

    def __init__(self,
                 camera_index: int = 0,
                 fps: int = 30,
                 camera_name: str = 'left_gripper_camera_1',
                 camera_sn: str = 'GF22510462C00',
                 dimension=3,
                 marker_reset_interval: float = 60.0,
                 debug=False,
                 sensor_name=None,
                 sensor_sn=None,
                 marker_dimension=None,
                 sensor_type=None,
                 **kwargs,
                 ):
        if sensor_name is not None:
            camera_name = sensor_name
        if sensor_sn is not None:
            camera_sn = sensor_sn
        if marker_dimension is not None:
            dimension = marker_dimension
        if kwargs:
            logger.warning(f"Ignoring unsupported tactile sensor args: {sorted(kwargs)}")

        node_name = f'{camera_name}_publisher_{camera_index}'
        super().__init__(node_name)
        self.fps = fps
        self.debug = debug
        self.dimension = dimension
        self.camera_sn = camera_sn
        self.camera_name = camera_name
        self.camera_index = camera_index
        self.marker_reset_interval = marker_reset_interval
        
        # suit vitai 240x240
        self.width = 240
        self.height = 240

        # init vitai sdk
        self.init_vt()
        self.color_publisher_ = self.create_publisher(Image, f'/{camera_name}/color/image_raw', 10)
        self.marker_publisher = self.create_publisher(PointCloud2, f'/{camera_name}/marker_offset/information', 10)
        
        self.timer = self.create_timer(1 / fps, self.timer_callback)
        self.timestamp_offset = None

        # 数据存储（from vitai sdk）
        self.datatypes = [
            VTSDataType.WARPED_IMG,
            VTSDataType.MARKER_ORIGIN_VECTOR,
            VTSDataType.MARKER_OFFSET_VECTOR
        ]
        self.latest_initial_markers = None  # 原始标记点位置
        self.latest_marker_offsets = None   # 标记点偏移向量
        
        self.fps_list = []
        self.frame_count = 0
        self.frame_intervals = []
        self.last_frame_time = None
        self.prev_time = time.time()
        
    def init_vt(self):
        self.finder = VTSDeviceFinder()
        device_config = self.finder.get_device_by_sn(self.camera_sn) 
        self.gf225 = VTSensor(config=device_config)
        self.gf225.calibrate()

    def marker_normalization(self, marker_loc, marker_offset):
        """归一化标记点数据"""
        if marker_loc is None or marker_offset is None:
            return None, None
            
        marker_loc = marker_loc.copy()
        marker_offset = marker_offset.copy()
        
        marker_loc[:, 0] /= self.width
        marker_loc[:, 1] /= self.height
        marker_offset[:, 0] /= self.width
        marker_offset[:, 1] /= self.height
        
        # if self.dimension == 3 and marker_offset.shape[1] > 2:
        #     marker_offset[:, 2] /= (self.width * self.height)

        return marker_loc, marker_offset

    def publish_marker_offset(self, initial_markers, marker_offsets, camera_timestamp: Time):
        """发布marker数据（转换为PointCloud2格式）"""
        if initial_markers is None or marker_offsets is None:
            return
            
        if len(initial_markers) != len(marker_offsets):
            logger.warning("Markers and offsets length mismatch!")
            return
            
        if self.dimension == 2:
            # x,y
            cur_marker = initial_markers[:, :2]
            marker_information = np.hstack((cur_marker, marker_offsets[:, :2])).astype(np.float32)
        else:
            # x,y,z
            cur_marker = np.concatenate([initial_markers[:, :3],0.1*np.ones([len(initial_markers),1])],axis=1)
            marker_information = np.hstack((cur_marker, marker_offsets[:, :3])).astype(np.float32)
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = camera_timestamp.to_msg()
        msg.header.frame_id = f'camera_marker_offset_{self.camera_name}'
        msg.height = 1
        msg.width = len(marker_information)
        
        msg.is_bigendian = False
        if self.dimension == 2:
            msg.point_step = 16  # 4 fields * 4 bytes
            msg.row_step = msg.point_step * msg.width
        else:
            msg.point_step = 24  # 6 fields * 4 bytes
            msg.row_step = msg.point_step * msg.width
            
        msg.is_dense = True
        
        # Define the field
        if self.dimension == 2:
            msg.fields = [
                PointField(name='marker_location_x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_location_y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_offset_x', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_offset_y', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            pointcloud_data = b''.join(
                map(lambda row: struct.pack('ffff', row[0], row[1], row[2], row[3]), marker_information)
            )
        else:
            msg.fields = [
                PointField(name='marker_location_x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_location_y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_location_z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_offset_x', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_offset_y', offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name='marker_offset_z', offset=20, datatype=PointField.FLOAT32, count=1),
            ]
            
            pointcloud_data = b''.join(
                map(lambda row: struct.pack('ffffff', row[0], row[1], row[2], row[3], row[4], row[5]),
                    marker_information)
            )
        
        msg.data = pointcloud_data
        self.marker_publisher.publish(msg)

    def publish_color_image(self, color_image, camera_timestamp: Time):
        """发布RGB图像"""
        if color_image is None:
            return
        success, encoded_image = cv2.imencode('.jpg', color_image)

        # Fill the message
        msg = Image()
        msg.header.stamp = camera_timestamp.to_msg()
        msg.header.frame_id = f"camera_color_frame_{self.camera_index}"
        msg.height, msg.width, _ = color_image.shape
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        if success:
            image_bytes = encoded_image.tobytes()
            msg.data = image_bytes
        else:
            logger.warning('fail to image encoding!')
            msg.data = color_image.tobytes()
        self.color_publisher_.publish(msg)

    def get_vitai_data(self):
        try:
            data = self.gf225.collect_sensor_data(*self.datatypes)
            warped_frame = data[VTSDataType.WARPED_IMG]
            marker_offsets = data[VTSDataType.MARKER_OFFSET_VECTOR]
            marker_offsets = np.concatenate(
                [marker_offsets, np.zeros((marker_offsets.shape[0], marker_offsets.shape[1], 1))],
                axis=2  # 指定沿第3维拼接
            ) # covert to (N,M,3)
            marker_offsets = marker_offsets.reshape(-1,3)
            origin_markers = data[VTSDataType.MARKER_ORIGIN_VECTOR].reshape(-1,2)

        except Exception as e:
            return None, None, None
        return warped_frame, marker_offsets, origin_markers
    
    def stop(self):
        # 先停止 timer 避免回调继续执行
        self.timer.cancel()
        # 停止传感器后台线程
        self.gf225.release()
        # 销毁节点并关闭 ROS2
        self.destroy_node()

    def timer_callback(self):
        '''
        Publish the color frames, markers and marker offsets
        '''

        camera_timestamp = self.get_clock().now()

        warped_frame, marker_offsets, initial_markers = self.get_vitai_data()
        if warped_frame is None or marker_offsets is None:
            return
        # normalization
        initial_markers, marker_motion = self.marker_normalization(initial_markers, marker_offsets)

        # publish image and marker offset
        self.publish_color_image(warped_frame, camera_timestamp)
        self.publish_marker_offset(initial_markers, marker_motion, camera_timestamp)
        # FPS
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.prev_time
        
        if elapsed_time >= 1.0:
            frame_rate = self.frame_count / elapsed_time
            self.fps_list.append(frame_rate)
            logger.info(f"Publishing from {self.camera_name} - Frame rate: {frame_rate:.2f} FPS")
            self.prev_time = current_time
            self.frame_count = 0

        if self.last_frame_time is not None:
            frame_interval = (current_time - self.last_frame_time) * 1000
            self.frame_intervals.append(frame_interval)
        self.last_frame_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorPublisher(
        camera_index=0,
        camera_name='right_gripper_sensor_1',
        camera_sn='GF2251386F6E6',
        debug=False,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.exception(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
