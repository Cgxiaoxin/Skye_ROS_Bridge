import os
import sys
import rclpy
import psutil
from loguru import logger
from hydra import initialize, compose
from common.space_utils import print_green
from teleoperation.data_recorder import DataRecorder

# add this to prevent assigning too may threads when using numpy
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

# Get the total number of CPU cores
total_cores = psutil.cpu_count()
# Define the number of cores you want to bind to
num_cores_to_bind = 8
# Calculate the indices of the first ten cores
# Ensure the number of cores to bind does not exceed the total number of cores
cores_to_bind = set(range(24, 24+num_cores_to_bind))
# Set CPU affinity for the current process to the first ten cores
os.sched_setaffinity(0, cores_to_bind)

def main(args=None):
    import argparse
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Data Recorder')
    parser.add_argument('--save_file_dir', type=str, default='/home/vitai/wyz/shanghai_code/vr_data/dewu/pkl/')
    args = parser.parse_args()
        
    with initialize(config_path='./config', version_base="1.1"):
        cfg = compose(config_name="real_world_env")
    rclpy_args = sys.argv
    rclpy.init(args=rclpy_args)

    base_dir = args.save_file_dir
    print_green(f"data save in {base_dir}")

    node = DataRecorder(base_dir=base_dir,
                        use_arm=cfg.robot_server.use_arm,
                        data_recorder_ip=cfg.teleop_server.data_recorder_ip,
                        data_recorder_port=cfg.teleop_server.data_recorder_port,
                        device_mapping_server_ip=cfg.device_mapping_server.host_ip,
                        device_mapping_server_port=cfg.device_mapping_server.port,
                        )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Data recordder get interrupted, quitting program now...")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()