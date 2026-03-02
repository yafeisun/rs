"""
ROS Bag 格式转换工具 - 模块化版本
"""

from .bag_reader import BagReader, ROS_AVAILABLE, find_bag_file
from .utils import (
    CAMERA_MAP,
    LIDAR_PCD_MAP,
    CALIBRATION_MAP,
    create_directory,
    read_yaml_file,
    write_yaml_file,
)
from .extractors import (
    extract_camera_images,
    extract_lidar_pcd,
    extract_lidar_concat,
    extract_calibration,
)
from .converters import (
    extract_imu_from_bag,
    extract_gnss_from_bag,
    extract_wheel_from_bag,
    convert_pose_online,
    convert_pose_offline,
)
from .calibration import (
    generate_calib_anno,
    generate_calib_anno_vc,
    generate_node_output,
    generate_camera_poses,
    generate_lidar_main_pose,
)
from .pose import (
    euler_from_quaternion,
    euler_to_rotation_vector,
    transform_poses_to_first_frame,
)

__all__ = [
    "BagReader",
    "ROS_AVAILABLE",
    "find_bag_file",
    "CAMERA_MAP",
    "LIDAR_PCD_MAP",
    "CALIBRATION_MAP",
    "create_directory",
    "read_yaml_file",
    "write_yaml_file",
    "extract_camera_images",
    "extract_lidar_pcd",
    "extract_lidar_concat",
    "extract_calibration",
    "extract_imu_from_bag",
    "extract_gnss_from_bag",
    "extract_wheel_from_bag",
    "convert_pose_online",
    "convert_pose_offline",
    "generate_calib_anno",
    "generate_calib_anno_vc",
    "generate_node_output",
    "generate_camera_poses",
    "generate_lidar_main_pose",
    "euler_from_quaternion",
    "euler_to_rotation_vector",
    "transform_poses_to_first_frame",
]

# 全局变量
SRC_BAG_DIR = None
TARGET_DIR = None
TARGET_ROOT = None
