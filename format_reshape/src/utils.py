"""
工具函数和配置
"""

import os
import yaml
import glob
from typing import Dict, List, Optional, Any
import glob
import numpy as np


# 数据映射配置
CAMERA_MAP = {
    "cam_around_back": "sensor_data/camera/camera_rear_fisheye",
    "cam_around_front": "sensor_data/camera/camera_front_fisheye",
    "cam_around_left": "sensor_data/camera/camera_left_fisheye",
    "cam_around_right": "sensor_data/camera/camera_right_fisheye",
    "cam_back": "sensor_data/camera/camera_rear_mid",
    "cam_front_left": "sensor_data/camera/camera_front_far",
    "cam_front_right": "sensor_data/camera/camera_front_wide",
    "cam_side_left_back": "sensor_data/camera/camera_left_rear",
    "cam_side_left_front": "sensor_data/camera/camera_left_front",
    "cam_side_right_back": "sensor_data/camera/camera_right_rear",
    "cam_side_right_front": "sensor_data/camera/camera_right_front",
}

CALIBRATION_MAP = {
    "cam_around_back/compressed": "calibration/camera/camera_rear_fisheye.yaml",
    "cam_around_front/compressed": "calibration/camera/camera_front_fisheye.yaml",
    "cam_around_left/compressed": "calibration/camera/camera_left_fisheye.yaml",
    "cam_around_right/compressed": "calibration/camera/camera_right_fisheye.yaml",
    "cam_back/compressed": "calibration/camera/camera_rear_mid.yaml",
    "cam_front_left/compressed": "calibration/camera/camera_front_far.yaml",
    "cam_front_right/compressed": "calibration/camera/camera_front_wide.yaml",
    "cam_side_left_back/compressed": "calibration/camera/camera_left_rear.yaml",
    "cam_side_left_front/compressed": "calibration/camera/camera_left_front.yaml",
    "cam_side_right_back/compressed": "calibration/camera/camera_right_rear.yaml",
    "cam_side_right_front/compressed": "calibration/camera/camera_right_front.yaml",
}

LIDAR_PCD_MAP = {
    "raw/pcd/middle": "sensor_data/lidar/lidar",
    "raw/pcd/front": "sensor_data/lidar/lidar_front_up",
    "raw/pcd/back": "sensor_data/lidar/lidar_rear_up",
    "raw/pcd/right": "sensor_data/lidar/lidar_fr",
    "raw/pcd/left": "sensor_data/lidar/lidar_fl",
}

LIDAR_CONCAT_SRC = "result/test_calibration/middle"
LIDAR_CONCAT_DST = "sensor_data/lidar/lidar_concat"
SYNC_SENSORS_FILE = "result/test_calibration/sync_sensors.txt"

LIDAR_MAP_SRC = "result/test_calibration/map_intensity_bev.pcd"
LIDAR_MAP_DST = "sensor_data/lidar/lidar_map"

LIDAR_MAP = {
    "/middle/rslidar_packets_unique": "calibration/lidar/lidar.yaml",
    "/left/rslidar_packets_unique": "calibration/lidar/lidar_fl.yaml",
    "/right/rslidar_packets_unique": "calibration/lidar/lidar_fr.yaml",
    "/front/rslidar_packets_unique": "calibration/lidar/lidar_front_up.yaml",
    "/back/rslidar_packets_unique": "calibration/lidar/lidar_rear_up.yaml",
}

LIDAR_HEIGHT_MAP = {"RSP128": 128, "RSBPV4": 32, "RSBPV2": 16, "RSL16": 16}
LIDAR_WIDTH = 1800

POSE_ONLINE_SRC = "result/bev/BEV_pose.json"
POSE_OFFLINE_SRC = "result/mapping/mapping_pose_quaterniond.txt"


def create_directory(path: str) -> None:
    """创建目录结构"""
    os.makedirs(path, exist_ok=True)


def read_yaml_file(yaml_path: str) -> Optional[Dict]:
    """读取 yaml 文件内容"""
    try:
        with open(yaml_path, "r") as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"    错误: 无法读取YAML文件 {yaml_path}: {e}")
        return None


def convert_numpy(obj: Any) -> Any:
    """将 numpy 数组转换为 Python 原生类型"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(item) for item in obj]
    elif isinstance(obj, (np.integer, np.floating)):
        return float(obj) if isinstance(obj, np.floating) else int(obj)
    return obj


def find_calibration_yaml(src_dir: str) -> Optional[str]:
    """查找以 'car_' 开头的 yaml 标定文件"""
    yaml_files = glob.glob(os.path.join(src_dir, "car_*.yaml"))
    return yaml_files[0] if yaml_files else None


def write_yaml_file(data: Dict, yaml_path: str) -> None:
    """写入相机标定 yaml 文件"""
    converted = convert_numpy(data)
    with open(yaml_path, "w") as f:
        f.write("%YAML:1.0\n---\n")
        f.write(
            f'CLOCK_calib_version: "{converted.get("CLOCK_calib_version", "N/A")}"\n'
        )
        f.write(
            f'CLOCK_calib_details: "{converted.get("CLOCK_calib_details", "output by default value")}"\n'
        )
        f.write(f'CLOCK_calib_date: "{converted.get("CLOCK_calib_date", "N/A")}"\n')
        f.write(f'vehicle_type: "{converted.get("vehicle_type", "alpine")}"\n')
        f.write(f'serial_number: "{converted.get("serial_number", "N/A")}"\n')
        f.write(f'sensor_name: "{converted.get("sensor_name", "")}"\n')
        f.write(f'sensor_type: "{converted.get("sensor_type", "camera")}"\n')
        f.write(f"timestamp_shift: {converted.get('timestamp_shift', 0)}\n")
        f.write(f'vehicle_xyz: "{converted.get("vehicle_xyz", "front_left_up")}"\n')
        r_s2b = converted.get("r_s2b", [0, 0, 0])
        f.write(f"r_s2b:  [{r_s2b[0]}, {r_s2b[1]}, {r_s2b[2]}]\n")
        t_s2b = converted.get("t_s2b", [0, 0, 0])
        f.write(f"t_s2b:  [{t_s2b[0]}, {t_s2b[1]}, {t_s2b[2]}]\n")
        f.write(f'camera_model: "{converted.get("camera_model", "polyn")}"\n')
        f.write(f"fx: {converted.get('fx', 0)}\n")
        f.write(f"fy: {converted.get('fy', 0)}\n")
        f.write(f"cx: {converted.get('cx', 0)}\n")
        f.write(f"cy: {converted.get('cy', 0)}\n")
        f.write(f"kc2: {converted.get('kc2', 0)}\n")
        f.write(f"kc3: {converted.get('kc3', 0)}\n")
        f.write(f"kc4: {converted.get('kc4', 0)}\n")
        f.write(f"kc5: {converted.get('kc5', 0)}\n")
        f.write(f"is_fisheye: {str(converted.get('is_fisheye', False)).lower()}\n")
        f.write(f"line_exposure_delay: {converted.get('line_exposure_delay', 0)}\n")
        f.write(f"width: {converted.get('width', 0)}\n")
        f.write(f"height: {converted.get('height', 0)}\n")
        roi = converted.get("suggested_rect_region_within_ROI", [0, 0, 0, 0])
        f.write(
            f"suggested_rect_region_within_ROI: [{roi[0]}, {roi[1]}, {roi[2]}, {roi[3]}]\n"
        )
        f.write(
            f'suggested_diagonal_FOV_within_ROI: "{converted.get("suggested_diagonal_FOV_within_ROI", "N/A")}"\n'
        )


def write_lidar_yaml_file(data: Dict, yaml_path: str) -> None:
    """写入 lidar 标定 yaml 文件"""
    converted = convert_numpy(data)
    with open(yaml_path, "w") as f:
        f.write("%YAML:1.0\n---\n")
        f.write(f'calib_version: "{converted.get("calib_version", "")}"\n')
        f.write(
            f'calib_detailes: "{converted.get("calib_detailes", "output by factory calibration")}"\n'
        )
        f.write(f'calib_date: "{converted.get("calib_date", "")}"\n')
        f.write(f'sensor_name: "{converted.get("sensor_name", "lidar")}"\n')
        f.write(f'sensor_type: "{converted.get("sensor_type", "LiDAR")}"\n')
        f.write(f"timestamp_shift: {converted.get('timestamp_shift', 0)}\n")
        f.write(f'vehicle_xyz: "{converted.get("vehicle_xyz", "front_left_up")}"\n')
        r_s2b = converted.get("r_s2b", [0, 0, 0])
        f.write(f"r_s2b:  [{r_s2b[0]}, {r_s2b[1]}, {r_s2b[2]}]\n")
        t_s2b = converted.get("t_s2b", [0, 0, 0])
        f.write(f"t_s2b:  [{t_s2b[0]}, {t_s2b[1]}, {t_s2b[2]}]\n")
        f.write(f"width: {converted.get('width', LIDAR_WIDTH)}\n")
        f.write(f"height: {converted.get('height', 128)}\n")
