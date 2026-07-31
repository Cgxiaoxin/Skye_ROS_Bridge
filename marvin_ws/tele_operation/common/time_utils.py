from loguru import logger
from rclpy.time import Time
from rclpy.clock import ClockType



def convert_float_to_ros_time(timestamp: float):
    """
    Convert a float timestamp (in seconds) to a Time object
    """
    seconds = int(timestamp)
    nanoseconds = int((timestamp - seconds) * 1e9)
    return Time(seconds=seconds, nanoseconds=nanoseconds, clock_type=ClockType.ROS_TIME)

def convert_ros_time_to_float(time: Time) -> float:
    """
    Convert a Time object to a float timestamp (in seconds)
    """
    return time.nanoseconds * 1e-9

def check_sync(timestamps: dict):
    # Check and log timestamp differences across topics
    all_times = list(timestamps.values())
    if not all(all_times):
        return
    # Calculate time differences for each frame across topics
    for i in range(len(all_times[0])):
        max_diff = 0
        for j in range(len(all_times)):
            for k in range(j + 1, len(all_times)):
                if i < len(all_times[j]) and i < len(all_times[k]):
                    time_diff = abs(all_times[j][i] - all_times[k][i])
                    max_diff = max(max_diff, time_diff)
        logger.info(f"Frame {i}: Maximum time difference across topics: {max_diff:.6f} seconds")

def check_timestamp(timestamps: dict):
    # check the interval between different time stampss
    all_times = list(timestamps.values())
    if not all(all_times):
        return
    time_stamps = []
    for i in range(len(all_times[0])):
        timestamps_for_frame = []
        for j in range(len(all_times)):
            if i < len(all_times[j]):
                timestamp = all_times[j][i]
                timestamps_for_frame.append(timestamp)

        if timestamps_for_frame:
            mean_time_stamp = sum(timestamps_for_frame) / len(timestamps_for_frame)
            time_stamps.append(mean_time_stamp)
            logger.info(f"Frame {i}: Mean timestamp: {mean_time_stamp:.6f} seconds")

