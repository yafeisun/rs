#!/usr/bin/env python3
"""
ROS Bag 格式转换工具 (改进版)

功能：
1. 相机图像拷贝
2. 点云数据拷贝
3. 标定参数转换
4. Pose数据转换（在线/离线优化）
5. IMU/GNSS/轮速数据提取

改进：
- 增强异常处理和字段存在性检查
- 改进时间戳转换精度
- 更详细的错误提示
- 支持批量处理：自动查找符合时间戳格式的数据目录

使用：
    python3 reshape.py <数据根目录> [--target-root <目标根目录>]

示例：
    python3 reshape.py /path/to/data_root
    python3 reshape.py /path/to/data_root --target-root /path/to/output

说明：
    程序会递归查找符合 YYYY-MM-DD-HH-MM-SS 格式且包含 .bag 文件的目录，
    然后批量进行格式转换。
"""

import os
import shutil
import yaml
import glob
import json
import re
import argparse
import numpy as np
from typing import Dict, List, Optional, Any, Iterator, Tuple

# 尝试导入 ROS 1 和 ROS 2 的 bag 库
ROS_AVAILABLE = None  # 'ros1', 'ros2', or None

try:
    import rosbag

    ROS_AVAILABLE = "ros1"
except ImportError:
    pass

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    ROS_AVAILABLE = "ros2"
except ImportError:
    pass


# =============================================================================
# 第一部分：全局变量（由命令行参数初始化）
# =============================================================================

# 源 bag 目录路径
SRC_BAG_DIR: Optional[str] = None

# 目标根目录
TARGET_ROOT: Optional[str] = None

# 目标目录路径
TARGET_DIR: Optional[str] = None

# Bag文件路径（自动查找）
BAG_PATH: Optional[str] = None

# =============================================================================
# ROS Bag 读取器（支持 ROS 1 和 ROS 2）
# =============================================================================


class BagReader:
    """统一的 Bag 读取器，支持 ROS 1 和 ROS 2"""

    def __init__(self, bag_path: str):
        self.bag_path = bag_path

        # 根据 bag 文件扩展名自动选择 API
        if bag_path.endswith(".bag"):
            # ROS 1 格式
            if ROS_AVAILABLE != "ros1":
                raise RuntimeError(
                    f"ROS 1 bag 文件需要 ROS 1 的 rosbag 库，但当前环境只支持 ROS 2"
                )
            self.ros_version = "ros1"
            self._bag = rosbag.Bag(bag_path)
        else:
            # ROS 2 格式 (.db3, .mcap 或目录)
            if ROS_AVAILABLE != "ros2":
                raise RuntimeError(
                    f"ROS 2 bag 文件需要 ROS 2 的 rosbag2_py 库，但当前环境只支持 ROS 1"
                )
            self.ros_version = "ros2"
            storage_options = StorageOptions(uri=bag_path)
            converter_options = ConverterOptions("", "")
            self._bag = SequentialReader()
            self._bag.open(storage_options, converter_options)

    def read_messages(
        self, topics: Optional[List[str]] = None
    ) -> Iterator[Tuple[Any, Any, Any]]:
        """
        读取 bag 中的消息
        返回: (topic, message, timestamp)
        """
        if self.ros_version == "ros1":
            for topic, msg, timestamp in self._bag.read_messages(topics=topics):
                yield (topic, msg, timestamp)
        elif self.ros_version == "ros2":
            topic_types = self._bag.get_all_topics_and_types()
            type_map = {topic.type: topic.name for topic, _ in topic_types}

            for msg in self._bag:
                topic = msg.topic
                if topics and topic not in topics:
                    continue

                # 反序列化消息
                msg_type = get_message(type_map[msg.type_id])
                deserialized_msg = deserialize_message(msg.data, msg_type)

                # ROS 2 的 timestamp 需要从消息 header 中获取
                timestamp = 0
                if hasattr(deserialized_msg, "header"):
                    timestamp = deserialized_msg.header.stamp

                yield (topic, deserialized_msg, timestamp)

    def close(self):
        """关闭 bag 文件"""
        if self._bag:
            if self.ros_version == "ros1":
                self._bag.close()
            elif self.ros_version == "ros2":
                self._bag = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def find_bag_file() -> Optional[str]:
    """自动查找目录中的 bag 文件 (ROS 1: .bag, ROS 2: .db3, .mcap)"""
    bag_files = glob.glob(os.path.join(SRC_BAG_DIR, "*.bag"))
    if bag_files:
        # 找到了 ROS 1 格式的 bag 文件
        if ROS_AVAILABLE != "ros1":
            print(f"  注意: 检测到 ROS 1 格式的 .bag 文件，但当前环境只支持 ROS 2")
            print(f"  提示: 要处理 ROS 1 bag 文件，请安装 ROS Noetic 或使用 Docker")
        return bag_files[0]

    # 查找 ROS 2 格式的 bag 文件
    bag_files = glob.glob(os.path.join(SRC_BAG_DIR, "*.db3"))
    if not bag_files:
        bag_files = glob.glob(os.path.join(SRC_BAG_DIR, "*.mcap"))

    # 查找包含 db3/mcap 文件的目录
    if not bag_files:
        db3_dirs = glob.glob(os.path.join(SRC_BAG_DIR, "*/*.db3"))
        if db3_dirs:
            bag_files = [os.path.dirname(db3_dirs[0])]

    if not bag_files:
        return None

    if len(bag_files) > 1:
        print(
            f"  警告: 发现多个 bag 文件，使用第一个: {os.path.basename(bag_files[0])}"
        )
    return bag_files[0]


# =============================================================================
# 第二部分：数据映射配置
# =============================================================================

# -----------------------------------------------------------------------------
# 2.1 相机图像映射 (源文件夹 -> 目标路径)
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# 2.2 相机标定文件映射 (源话题 -> 目标文件)
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# 2.3 点云数据映射 (源路径 -> 目标路径)
# -----------------------------------------------------------------------------
LIDAR_PCD_MAP = {
    "raw/pcd/middle": "sensor_data/lidar/lidar",
    "raw/pcd/front": "sensor_data/lidar/lidar_front_up",
    "raw/pcd/back": "sensor_data/lidar/lidar_rear_up",
    "raw/pcd/right": "sensor_data/lidar/lidar_fr",
    "raw/pcd/left": "sensor_data/lidar/lidar_fl",
}

# -----------------------------------------------------------------------------
# 2.4 拼接点云映射 (用middle时间戳命名)
# -----------------------------------------------------------------------------
LIDAR_CONCAT_SRC = "result/test_calibration/middle"
LIDAR_CONCAT_DST = "sensor_data/lidar/lidar_concat"
SYNC_SENSORS_FILE = "result/test_calibration/sync_sensors.txt"

# -----------------------------------------------------------------------------
# 2.4.1 Lidar地图映射 (用第一个PCD时间戳命名)
# -----------------------------------------------------------------------------
LIDAR_MAP_SRC = "result/test_calibration/map_intensity_bev.pcd"
LIDAR_MAP_DST = "sensor_data/lidar/lidar_map"

# -----------------------------------------------------------------------------
# 2.5 Lidar标定映射 (源话题 -> 目标文件)
# -----------------------------------------------------------------------------
LIDAR_MAP = {
    "/middle/rslidar_packets_unique": "calibration/lidar/lidar.yaml",
    "/left/rslidar_packets_unique": "calibration/lidar/lidar_fl.yaml",
    "/right/rslidar_packets_unique": "calibration/lidar/lidar_fr.yaml",
    "/front/rslidar_packets_unique": "calibration/lidar/lidar_front_up.yaml",
    "/back/rslidar_packets_unique": "calibration/lidar/lidar_rear_up.yaml",
}

# Lidar参数
LIDAR_HEIGHT_MAP = {"RSP128": 128, "RSBPV4": 32, "RSBPV2": 16, "RSL16": 16}
LIDAR_WIDTH = 1800

# -----------------------------------------------------------------------------
# 2.6 Pose数据源路径
# -----------------------------------------------------------------------------
POSE_ONLINE_SRC = "result/bev/BEV_pose.json"
POSE_OFFLINE_SRC = "result/mapping/mapping_pose_quaterniond.txt"

# =============================================================================
# 第三部分：工具函数
# ==============================================================================


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


# =============================================================================
# 第四部分：相机/点云/标定提取函数
# ==============================================================================


def extract_camera_images() -> None:
    """从源目录拷贝相机图像"""
    total_images = 0
    missing_dirs = []

    print(f"  [Camera] Src: {SRC_BAG_DIR}/cam_*")
    print(f"           Dst: {TARGET_DIR}/sensor_data/camera/")

    for camera_name, target_subdir in CAMERA_MAP.items():
        src_camera_dir = os.path.join(SRC_BAG_DIR, camera_name)
        target_path = os.path.join(TARGET_DIR, target_subdir)

        if not os.path.exists(src_camera_dir):
            missing_dirs.append(camera_name)
            continue

        create_directory(target_path)

        image_files = (
            glob.glob(os.path.join(src_camera_dir, "*.jpeg"))
            + glob.glob(os.path.join(src_camera_dir, "*.jpg"))
            + glob.glob(os.path.join(src_camera_dir, "*.png"))
        )

        for img_file in image_files:
            filename = os.path.basename(img_file)
            if not filename.endswith(".jpeg"):
                filename = os.path.splitext(filename)[0] + ".jpeg"
            shutil.copy2(img_file, os.path.join(target_path, filename))

        total_images += len(image_files)

    print(f"           Done: {total_images} images", end="")
    if missing_dirs:
        print(f" (missing: {len(missing_dirs)} dirs)")
    else:
        print()


def extract_lidar_pcd() -> None:
    """从源目录拷贝 pcd 点云文件"""
    total_files = 0
    missing_dirs = []

    print(f"  [LiDAR] Src: {SRC_BAG_DIR}/raw/pcd/*")
    print(f"          Dst: {TARGET_DIR}/sensor_data/lidar/")

    for src_subdir, target_subdir in LIDAR_PCD_MAP.items():
        src_pcd_dir = os.path.join(SRC_BAG_DIR, src_subdir)
        target_path = os.path.join(TARGET_DIR, target_subdir)

        if not os.path.exists(src_pcd_dir):
            missing_dirs.append(src_subdir)
            continue

        create_directory(target_path)
        pcd_files = glob.glob(os.path.join(src_pcd_dir, "*.pcd"))

        for pcd_file in pcd_files:
            shutil.copy2(
                pcd_file, os.path.join(target_path, os.path.basename(pcd_file))
            )

        total_files += len(pcd_files)

    print(f"          Done: {total_files} files", end="")
    if missing_dirs:
        print(f" (missing: {missing_dirs})")
    else:
        print()


def extract_lidar_concat() -> None:
    """拷贝拼接点云，用middle时间戳命名"""
    src_dir = os.path.join(SRC_BAG_DIR, LIDAR_CONCAT_SRC)
    sync_file = os.path.join(SRC_BAG_DIR, SYNC_SENSORS_FILE)
    target_path = os.path.join(TARGET_DIR, LIDAR_CONCAT_DST)

    if not os.path.exists(src_dir) or not os.path.exists(sync_file):
        return

    print(f"  [LiDAR Concat] Src: {src_dir}")
    print(f"                 Dst: {target_path}")

    # 解析 sync_sensors.txt 建立 frame -> middle_timestamp 映射
    frame_to_timestamp = {}
    try:
        with open(sync_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 2:
                    frame = int(parts[0])
                    middle_ts = parts[1]
                    frame_to_timestamp[frame] = middle_ts
    except Exception:
        return

    # 拷贝文件，用时间戳命名
    target_path = os.path.join(TARGET_DIR, LIDAR_CONCAT_DST)
    create_directory(target_path)

    pcd_files = glob.glob(os.path.join(src_dir, "*.pcd"))
    copied = 0

    for pcd_file in pcd_files:
        basename = os.path.basename(pcd_file)
        try:
            frame_num = int(basename.replace(".pcd", ""))
            if frame_num in frame_to_timestamp:
                new_name = frame_to_timestamp[frame_num] + ".pcd"
                shutil.copy2(pcd_file, os.path.join(target_path, new_name))
                copied += 1
        except ValueError:
            pass

    if copied > 0:
        print(f"                 Done: {copied} files")


def extract_lidar_map() -> None:
    """拷贝 lidar 地图，使用第一个在 egopose_optpose 中有对应位姿的 PCD 时间戳命名"""
    src_path = os.path.join(SRC_BAG_DIR, LIDAR_MAP_SRC)
    target_dir = os.path.join(TARGET_DIR, LIDAR_MAP_DST)

    if not os.path.exists(src_path):
        return

    # 从 egopose_optpose 中获取第一个时间戳（这些时间戳对应有位姿数据的 PCD）
    optpose_dir = os.path.join(TARGET_DIR, "sensor_data/egopose_opt/egopose_optpose")
    if not os.path.exists(optpose_dir):
        return

    json_files = sorted([f for f in os.listdir(optpose_dir) if f.endswith(".json")])
    if not json_files:
        return

    # 使用第一个有位姿数据的 PCD 时间戳
    first_timestamp = json_files[0].replace(".json", "")

    print(f"  [LiDAR Map] Src: {src_path}")
    print(f"              Dst: {target_dir}/")
    print(f"              Using timestamp: {first_timestamp} (from egopose_optpose)")

    # 创建目标目录并拷贝文件
    create_directory(target_dir)
    target_path = os.path.join(target_dir, f"{first_timestamp}.pcd")
    shutil.copy2(src_path, target_path)

    print(f"              Done: 1 file")


def extract_calibration() -> None:
    """提取相机和Lidar标定参数"""
    src_yaml = find_calibration_yaml(SRC_BAG_DIR)
    if not src_yaml:
        return

    calib_data = read_yaml_file(src_yaml)
    if not calib_data:
        return

    print(f"  [Calibration] Src: {src_yaml}")
    print(f"                Dst: {TARGET_DIR}/calibration/")

    # 构建相机索引
    camera_index = {}
    if "sensors" in calib_data and "camera" in calib_data["sensors"]:
        for cam_data in calib_data["sensors"]["camera"]:
            topic = cam_data.get("topic", "")
            camera_index[topic.split("/")[1]] = cam_data

    # 构建Lidar索引
    lidar_index = {}
    if "sensors" in calib_data and "lidar" in calib_data["sensors"]:
        for lidar_data in calib_data["sensors"]["lidar"]:
            topic = lidar_data.get("topic", "")
            lidar_index[topic] = lidar_data

    # 提取相机标定
    cam_count = 0
    for src_topic, target_yaml_path in CALIBRATION_MAP.items():
        camera_name = src_topic.split("/")[0]
        if camera_name not in camera_index:
            continue

        cam_data = camera_index[camera_name]
        calib = cam_data.get("calibration", {})

        # 提取参数
        cam_ext = calib.get("CameraExt", {})
        roll, pitch, yaw = (
            cam_ext.get("roll", 0),
            cam_ext.get("pitch", 0),
            cam_ext.get("yaw", 0),
        )
        cam_int = calib.get("CameraIntMat", [])
        fx, fy, cx, cy = (
            (cam_int[0], cam_int[4], cam_int[2], cam_int[5])
            if len(cam_int) >= 6
            else (0, 0, 0, 0)
        )
        dist_coeff = calib.get("DistCoeff", [])
        k1, k2, k3, k4 = (
            (dist_coeff[0], dist_coeff[1], dist_coeff[2], dist_coeff[3])
            if len(dist_coeff) > 3
            else (0, 0, 0, 0)
        )
        image_size = calib.get("ImageSize", [0, 0])
        width, height = image_size[0], image_size[1]

        sensor_name = target_yaml_path.split("/")[-1].replace(".yaml", "")
        r_s2b = euler_to_rotation_vector(roll, pitch, yaw)

        target_calib = {
            "CLOCK_calib_version": "N/A",
            "CLOCK_calib_details": "output by default value",
            "CLOCK_calib_date": "N/A",
            "vehicle_type": "alpine",
            "serial_number": "N/A",
            "sensor_name": sensor_name,
            "sensor_type": "camera",
            "timestamp_shift": 0,
            "vehicle_xyz": "front_left_up",
            "r_s2b": r_s2b,
            "t_s2b": [cam_ext.get("x", 0), cam_ext.get("y", 0), cam_ext.get("z", 0)],
            "camera_model": "polyn",
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "kc2": k1,
            "kc3": k2,
            "kc4": k3,
            "kc5": k4,
            "is_fisheye": "fisheye" in sensor_name,
            "line_exposure_delay": 0,
            "width": width,
            "height": height,
            "suggested_rect_region_within_ROI": [0, 0, width, height],
            "suggested_diagonal_FOV_within_ROI": "N/A",
        }

        target_full_path = os.path.join(TARGET_DIR, target_yaml_path)
        os.makedirs(os.path.dirname(target_full_path), exist_ok=True)
        write_yaml_file(target_calib, target_full_path)
        cam_count += 1

        # 生成虚拟相机标定 (去畸变后)
        vc_yaml_path = target_yaml_path.replace(
            "calibration/camera/", "calibration/virtual_camera/"
        )
        vc_target_calib = target_calib.copy()

        # 虚拟相机内参：去畸变后
        vc_target_calib["fx"] = fx  # 保持原焦距
        vc_target_calib["fy"] = fx  # fx = fy (去畸变后)
        vc_target_calib["kc2"] = 0.0
        vc_target_calib["kc3"] = 0.0
        vc_target_calib["kc4"] = 0.0
        vc_target_calib["kc5"] = 0.0
        vc_target_calib["is_fisheye"] = False
        vc_target_calib["sensor_name"] = sensor_name + "_virtual_camera"

        vc_target_full_path = os.path.join(TARGET_DIR, vc_yaml_path)
        os.makedirs(os.path.dirname(vc_target_full_path), exist_ok=True)
        write_yaml_file(vc_target_calib, vc_target_full_path)

    # 提取Lidar标定
    lidar_count = 0
    for src_topic, target_yaml_path in LIDAR_MAP.items():
        if src_topic not in lidar_index:
            continue

        lidar_data = lidar_index[src_topic]
        calib = lidar_data.get("calibration", {})

        roll, pitch, yaw = (
            calib.get("roll", 0),
            calib.get("pitch", 0),
            calib.get("yaw", 0),
        )
        lidar_type = lidar_data.get("lidar_type", "RSP128")
        r_s2b = euler_to_rotation_vector(roll, pitch, yaw)

        target_calib = {
            "calib_version": "",
            "calib_detailes": "output by factory calibration",
            "calib_date": "",
            "sensor_name": "lidar",
            "sensor_type": "LiDAR",
            "timestamp_shift": 0,
            "vehicle_xyz": "front_left_up",
            "r_s2b": r_s2b,
            "t_s2b": [calib.get("x", 0), calib.get("y", 0), calib.get("z", 0)],
            "width": LIDAR_WIDTH,
            "height": LIDAR_HEIGHT_MAP.get(lidar_type, 128),
        }

        target_full_path = os.path.join(TARGET_DIR, target_yaml_path)
        os.makedirs(os.path.dirname(target_full_path), exist_ok=True)
        write_lidar_yaml_file(target_calib, target_full_path)
        lidar_count += 1

    print(f"                Done: {cam_count} cameras, {lidar_count} lidars")


# =============================================================================
# 第五部分：Pose转换函数
# ==============================================================================


def euler_from_quaternion(q: List[float]) -> List[float]:
    """从四元数提取欧拉角 [roll, pitch, yaw]"""
    import math

    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def euler_to_rotation_vector(roll: float, pitch: float, yaw: float) -> List[float]:
    """将欧拉角转换为旋转向量 (Rodrigues公式)"""
    import math

    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    R = np.array(
        [
            [
                cos_y * cos_p,
                cos_y * sin_p * sin_r - sin_y * cos_r,
                cos_y * sin_p * cos_r + sin_y * sin_r,
            ],
            [
                sin_y * cos_p,
                sin_y * sin_p * sin_r + cos_y * cos_r,
                sin_y * sin_p * cos_r - cos_y * sin_r,
            ],
            [-sin_p, cos_p * sin_r, cos_p * cos_r],
        ]
    )

    # 计算旋转角度
    trace = np.trace(R)
    # 钳位 trace 到 [-3, 3] 避免数值问题
    trace = max(-3.0, min(3.0, trace))
    angle = math.acos((trace - 1) / 2)

    if angle < 1e-6:
        return [0.0, 0.0, 0.0]

    rx = (R[2, 1] - R[1, 2]) / (2 * math.sin(angle))
    ry = (R[0, 2] - R[2, 0]) / (2 * math.sin(angle))
    rz = (R[1, 0] - R[0, 1]) / (2 * math.sin(angle))
    return [rx * angle, ry * angle, rz * angle]


def rotation_vector_to_matrix(r_s2b: List[float]) -> np.ndarray:
    """将旋转向量转换为旋转矩阵 (Rodrigues公式)"""
    import math

    rx, ry, rz = r_s2b
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)

    if angle < 1e-10:
        return np.eye(3)

    # 单位旋转轴
    kx, ky, kz = rx / angle, ry / angle, rz / angle

    # Rodrigues 公式: R = I + sin(θ) * K + (1 - cos(θ)) * K²
    K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])

    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    return R


def quat_to_rotation_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """四元数转3x3旋转矩阵"""
    R = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]
    )
    return R


def rotation_matrix_to_quat(R: np.ndarray) -> List[float]:
    """3x3旋转矩阵转四元数 [qw, qx, qy, qz]"""
    import math

    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [w, x, y, z]


def transform_poses_to_first_frame(
    pose_dict: Dict[str, List[float]],
) -> Dict[str, List[float]]:
    """
    将世界坐标系下的 pose 序列转换到第一帧车体坐标系。

    输入 pose_dict: {timestamp_ns_str: [x, y, z, qx, qy, qz, qw], ...}
    输出同格式，但所有 pose 变换为相对第一帧的相对位姿：
        T_rel_i = T_world_body0^{-1} * T_world_bodyi

    第一帧结果为 identity (x=0, y=0, z=0, qw=1, qx=qy=qz=0)。
    """
    sorted_keys = sorted(pose_dict.keys(), key=lambda k: int(k))
    if not sorted_keys:
        return pose_dict

    # 第一帧位姿
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_dict[sorted_keys[0]]
    R0 = quat_to_rotation_matrix(qw0, qx0, qy0, qz0)
    t0 = np.array([x0, y0, z0])

    # T0^{-1}: R0^T, -R0^T * t0
    R0_inv = R0.T
    t0_inv = -R0_inv @ t0

    result = {}
    for key in sorted_keys:
        xi, yi, zi, qxi, qyi, qzi, qwi = pose_dict[key]
        Ri = quat_to_rotation_matrix(qwi, qxi, qyi, qzi)
        ti = np.array([xi, yi, zi])

        # T_rel = T0^{-1} * Ti
        R_rel = R0_inv @ Ri
        t_rel = R0_inv @ ti + t0_inv

        qw_r, qx_r, qy_r, qz_r = rotation_matrix_to_quat(R_rel)
        result[key] = [
            float(t_rel[0]),
            float(t_rel[1]),
            float(t_rel[2]),
            float(qx_r),
            float(qy_r),
            float(qz_r),
            float(qw_r),
        ]

    return result


def load_bev_pose(bev_pose_path: str) -> Optional[Dict[str, List[float]]]:
    """加载 BEV_pose.json 文件"""
    try:
        with open(bev_pose_path, "r") as f:
            data = json.load(f)

        # 如果数据是字典格式，提取pose字段
        if isinstance(data, dict):
            pose_data = data.get("pose", data)
            # 如果pose数据是嵌套字典，根据实际情况提取
            if isinstance(pose_data, dict):
                # 假设pose数据格式为 {timestamp_ns: [x,y,z,qx,qy,qz,qw], ...}
                return pose_data
            return pose_data
        return data
    except Exception as e:
        print(f"    错误: 读取BEV pose文件失败: {e}")
        return None


def load_mapping_pose(pose_path: str) -> Optional[Dict[str, List[float]]]:
    """加载 mapping_pose_quaterniond.txt 文件"""
    pose_dict = {}
    try:
        with open(pose_path, "r") as f:
            lines = f.readlines()
            for line in lines[1:]:  # 跳过表头
                parts = line.strip().split()
                if len(parts) >= 9:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        ox, oy, oz, ow = (
                            float(parts[4]),
                            float(parts[5]),
                            float(parts[6]),
                            float(parts[7]),
                        )

                        # 改进的时间戳转换：避免浮点精度问题
                        # 假设格式为 "1764310903.399620" (秒.微秒)
                        time_str = parts[8]
                        if "." in time_str:
                            sec_part, usec_part = time_str.split(".")
                            usec_part = usec_part.ljust(6, "0")[:6]  # 确保微秒部分是6位
                            timestamp_ns = (
                                int(sec_part) * 1_000_000_000 + int(usec_part) * 1000
                            )
                        else:
                            timestamp_ns = int(float(time_str) * 1_000_000_000)

                        pose_dict[str(timestamp_ns)] = [x, y, z, ox, oy, oz, ow]
                    except (ValueError, IndexError) as e:
                        print(f"    警告: 跳过无效行: {line.strip()}")
                        continue
    except Exception as e:
        print(f"    错误: 读取mapping pose文件失败: {e}")
        return None

    return pose_dict


def create_pose_message(
    timestamp_ns_str: str, pose_data: List[float]
) -> Dict[str, Any]:
    """创建完整的pose消息结构"""
    timestamp_ns = int(timestamp_ns_str)
    secs, nsecs = timestamp_ns // 1_000_000_000, timestamp_ns % 1_000_000_000
    x, y, z, qx, qy, qz, qw = pose_data
    roll, pitch, yaw = euler_from_quaternion([qw, qx, qy, qz])

    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": secs, "nsecs": nsecs},
            "frame_id": '{"coordinate_system":"GCJ02","version":"LTS_6188e01"}',
        },
        "meta": {"timestamp_us": timestamp_ns // 1000, "seq": 0, "type": 0},
        "position": {
            "available": 2,
            "position_global": {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0},
            "position_local": {"x": x, "y": y, "z": z},
        },
        "velocity": {
            "available": 2,
            "velocity_global": {"ve": 0.0, "vn": 0.0, "vu": 0.0},
            "velocity_local": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
        },
        "angular_velocity": {
            "available": 1,
            "angvelocity_local": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
        },
        "orientation": {
            "available": 12,
            "euler_global": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "quaternion_global": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "euler_local": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "quaternion_local": {"w": qw, "x": qx, "y": qy, "z": qz},
        },
        "acceleration": {
            "available": 2,
            "acceleration_global": {"ae": 0.0, "an": 0.0, "au": 0.0},
            "acceleration_local": {"ax": 0.0, "ay": 0.0, "az": 0.0},
        },
        "position_std": {
            "available": 2,
            "pos_std_global": {"std_pe": 0.0, "std_pn": 0.0, "std_pu": 0.0},
            "pos_std_local": {"std_px": 0.0, "std_py": 0.0, "std_pz": 0.0},
        },
        "velocity_std": {
            "available": 2,
            "vel_std_global": {"std_ve": 0.0, "std_vn": 0.0, "std_vu": 0.0},
            "vel_std_local": {"std_vx": 0.0, "std_vy": 0.0, "std_vz": 0.0},
        },
        "angular_velocity_std": {
            "available": 0,
            "angvel_std_local": {"std_vx": 0.0, "std_vy": 0.0, "std_vz": 0.0},
        },
        "orientation_std": {
            "available": 12,
            "ori_std_xyz": {"std_faix": 0.0, "std_faiy": 0.0, "std_faiz": 0.0},
        },
        "acceleration_std": {
            "available": 0,
            "acc_std_global": {"std_ae": 0.0, "std_an": 0.0, "std_au": 0.0},
            "acc_std_local": {"std_ax": 0.0, "std_ay": 0.0, "std_az": 0.0},
        },
    }


def convert_pose_from_dict(
    pose_dict: Dict[str, List[float]], output_dir: str, output_json: str
) -> None:
    """通用pose转换函数"""
    abs_output_dir = os.path.join(TARGET_DIR, output_dir)
    create_directory(abs_output_dir)

    pose_dict = transform_poses_to_first_frame(pose_dict)
    sorted_timestamps = sorted(pose_dict.keys(), key=lambda x: int(x))
    pose_messages = []

    for seq, timestamp_ns in enumerate(sorted_timestamps):
        msg = create_pose_message(timestamp_ns, pose_dict[timestamp_ns])
        msg["header"]["seq"] = seq
        msg["meta"]["seq"] = seq
        pose_messages.append(msg)

        filepath = os.path.join(abs_output_dir, f"{timestamp_ns}.json")
        with open(filepath, "w") as f:
            json.dump(msg, f, indent=2)

    with open(os.path.join(TARGET_DIR, output_json), "w") as f:
        json.dump(pose_messages, f, indent=2)


def convert_pose_online() -> int:
    """转换在线定位结果 (egopose)，返回记录数"""
    src_path = os.path.join(SRC_BAG_DIR, POSE_ONLINE_SRC)
    if not os.path.exists(src_path):
        return 0

    pose_dict = load_bev_pose(src_path)
    if not pose_dict:
        return 0

    target_dir = os.path.join(TARGET_DIR, "sensor_data/egopose")
    target_json = os.path.join(TARGET_DIR, "sensor_data/egopose.json")
    print(f"  [Pose Online] Src: {src_path}")
    print(f"                Dst: {target_json}")

    convert_pose_from_dict(pose_dict, "sensor_data/egopose", "sensor_data/egopose.json")
    return len(pose_dict)


def convert_pose_offline() -> int:
    """转换离线优化结果 (egopose_opt)，返回记录数"""
    src_path = os.path.join(SRC_BAG_DIR, POSE_OFFLINE_SRC)
    if not os.path.exists(src_path):
        return 0

    pose_dict = load_mapping_pose(src_path)
    if not pose_dict:
        return 0

    target_json = os.path.join(TARGET_DIR, "sensor_data/egopose_opt.json")
    print(f"  [Pose Offline] Src: {src_path}")
    print(f"                 Dst: {target_json}")

    # 只生成 JSON 文件，不生成单独的 JSON 文件
    sorted_timestamps = sorted(pose_dict.keys(), key=lambda x: int(x))
    pose_messages = []

    for seq, timestamp_ns in enumerate(sorted_timestamps):
        msg = create_pose_message(timestamp_ns, pose_dict[timestamp_ns])
        msg["header"]["seq"] = seq
        msg["meta"]["seq"] = seq
        pose_messages.append(msg)

    with open(target_json, "w") as f:
        json.dump(pose_messages, f, indent=2)
    return len(pose_dict)


# =============================================================================
# 第六部分：IMU/GNSS/Wheel数据提取函数
# ==============================================================================


def extract_imu_from_bag() -> int:
    """从bag提取IMU数据，返回记录数"""
    if not BAG_PATH or not os.path.exists(BAG_PATH):
        return 0

    output_path = os.path.join(TARGET_DIR, "sensor_data/imu.json")
    print(f"  [IMU] Src: {BAG_PATH} (topic: /rs/imu)")
    print(f"        Dst: {output_path}")

    try:
        with BagReader(BAG_PATH) as bag:
            imu_messages = []

            for __, msg, ___ in bag.read_messages(topics=["/rs/imu"]):
                # ROS 1 和 ROS 2 的时间戳处理方式不同
                if ROS_AVAILABLE == "ros1":
                    timestamp_us = int(msg.header.stamp.to_nsec() // 1000)
                    secs = msg.header.stamp.secs
                    nsecs = msg.header.stamp.nsecs
                else:  # ROS 2
                    timestamp_us = int(
                        msg.header.stamp.sec * 1000000
                        + msg.header.stamp.nanosec // 1000
                    )
                    secs = msg.header.stamp.sec
                    nsecs = msg.header.stamp.nanosec

                imu_messages.append(
                    {
                        "header": {
                            "seq": msg.header.seq,
                            "stamp": {"secs": secs, "nsecs": nsecs},
                            "frame_id": msg.header.frame_id,
                        },
                        "meta": {"timestamp_us": timestamp_us, "seq": msg.header.seq},
                        "info": {
                            "available": 1,
                            "imu_data_basic": {
                                "accx": msg.linear_acceleration.x,
                                "accy": msg.linear_acceleration.y,
                                "accz": msg.linear_acceleration.z,
                                "gyrox": msg.angular_velocity.x,
                                "gyroy": msg.angular_velocity.y,
                                "gyroz": msg.angular_velocity.z,
                                "temperature": 25.0,
                            },
                        },
                    }
                )
    except Exception:
        return 0

    with open(output_path, "w") as f:
        json.dump(imu_messages, f, indent=2)
    print(f"        Done: {len(imu_messages)} records")
    return len(imu_messages)


def extract_gnss_from_bag() -> int:
    """从bag提取GNSS数据，返回记录数"""
    if not BAG_PATH or not os.path.exists(BAG_PATH):
        return 0

    output_path = os.path.join(TARGET_DIR, "sensor_data/gnss_rtk.json")
    print(f"  [GNSS] Src: {BAG_PATH} (topic: /rs/gps)")
    print(f"         Dst: {output_path}")

    try:
        with BagReader(BAG_PATH) as bag:
            gnss_messages = []

            for __, msg, ___ in bag.read_messages(topics=["/rs/gps"]):
                # ROS 1 和 ROS 2 的时间戳处理方式不同
                if ROS_AVAILABLE == "ros1":
                    timestamp_us = int(msg.header.stamp.to_nsec() // 1000)
                    secs = msg.header.stamp.secs
                    nsecs = msg.header.stamp.nsecs
                else:  # ROS 2
                    timestamp_us = int(
                        msg.header.stamp.sec * 1000000
                        + msg.header.stamp.nanosec // 1000
                    )
                    secs = msg.header.stamp.sec
                    nsecs = msg.header.stamp.nanosec

                # 安全获取status字段
                status_value = 0
                if hasattr(msg, "status") and hasattr(msg.status, "status"):
                    status_value = msg.status.status

                gnss_messages.append(
                    {
                        "header": {
                            "seq": msg.header.seq,
                            "stamp": {"secs": secs, "nsecs": nsecs},
                            "frame_id": msg.header.frame_id,
                        },
                        "meta": {"timestamp_us": timestamp_us, "seq": msg.header.seq},
                        "info": {
                            "available": 3,
                            "gnss_data_basic": {
                                "lat": msg.latitude,
                                "lon": msg.longitude,
                                "alt": msg.altitude,
                                "vel_e": 0.0,
                                "vel_n": 0.0,
                                "vel_u": 0.0,
                                "azi_track": 0.0,
                                "speed": 0.0,
                                "status": status_value,
                                "msg_type": 0,
                                "sat_num": 0,
                                "hdop": 0.0,
                                "rtk_age": 0.0,
                                "sec_in_gps_week": 0.0,
                                "gps_week": 0,
                                "utc_time": float(secs),
                            },
                            "gnss_data_extend": {
                                "sv_num": 0,
                                "sv_id": [],
                                "sv_elv": [],
                                "sv_az": [],
                                "sv_cno": [],
                            },
                        },
                    }
                )
    except Exception:
        return 0

    with open(output_path, "w") as f:
        json.dump(gnss_messages, f, indent=2)
    print(f"         Done: {len(gnss_messages)} records")
    return len(gnss_messages)


def extract_wheel_from_bag() -> int:
    """从bag提取轮速数据，返回记录数"""
    if not BAG_PATH or not os.path.exists(BAG_PATH):
        return 0

    output_path = os.path.join(TARGET_DIR, "sensor_data/wheel_report.json")
    print(f"  [Wheel] Src: {BAG_PATH} (topic: /ft_vehicle_data_v3)")
    print(f"          Dst: {output_path}")

    try:
        with BagReader(BAG_PATH) as bag:
            wheel_messages = []

            for __, msg, ___ in bag.read_messages(topics=["/ft_vehicle_data_v3"]):
                timestamp_us = msg.timeStamp

                # 安全获取字段，使用getattr提供默认值
                wheel_speed_front_left = getattr(msg, "wheelSpeedFrontLeft", 0.0)
                wheel_speed_front_right = getattr(msg, "wheelSpeedFrontRight", 0.0)
                wheel_speed_rear_left = getattr(msg, "wheelSpeedRearLeft", 0.0)
                wheel_speed_rear_right = getattr(msg, "wheelSpeedRearRight", 0.0)
                lateral_acceleration = getattr(msg, "lateralAcceleration", 0.0)
                acceleration = getattr(msg, "acceleration", 0.0)
                yaw_rate = getattr(msg, "yawRate", 0.0)
                frame_num = getattr(msg, "frameNum", 0)
                speed_timestamp = getattr(msg, "speedTimeStamp", timestamp_us * 1000)

                wheel_messages.append(
                    {
                        "header": {
                            "seq": frame_num,
                            "stamp": {
                                "secs": speed_timestamp // 1000000,
                                "nsecs": (speed_timestamp % 1000000) * 1000,
                            },
                            "frame_id": "",
                        },
                        "meta": {"timestamp_us": timestamp_us},
                        "wheel_position_report": {
                            "available": 1,
                            "wheel_position_report_data": {
                                "timestamp_us": timestamp_us,
                                "front_left": 0,
                                "front_right": 0,
                                "rear_left": 0,
                                "rear_right": 0,
                                "valid": True,
                            },
                        },
                        "wheel_speed_report": {
                            "available": 1,
                            "wheel_speed_report_data": {
                                "timestamp_us": timestamp_us,
                                "front_left": wheel_speed_front_left,
                                "front_right": wheel_speed_front_right,
                                "rear_left": wheel_speed_rear_left,
                                "rear_right": wheel_speed_rear_right,
                                "front_left_mps": wheel_speed_front_left,
                                "front_right_mps": wheel_speed_front_right,
                                "rear_left_mps": wheel_speed_rear_left,
                                "rear_right_mps": wheel_speed_rear_right,
                                "valid": True,
                            },
                        },
                        "wheel_angle_report": {
                            "available": 1,
                            "wheel_angle_report_data": {
                                "timestamp_us": timestamp_us,
                                "rws_state": {"valid": False, "value": 0},
                                "front_left": 0.0,
                                "front_right": 0.0,
                                "rear_left": 0.0,
                                "rear_right": 0.0,
                                "valid": False,
                            },
                        },
                        "vehicle_imu_report": {
                            "available": 1,
                            "vehicle_imu_report_data": {
                                "timestamp_us": timestamp_us,
                                "linear_acceleration_lat": lateral_acceleration,
                                "linear_acceleration_lon": acceleration,
                                "linear_acceleration_vert": 0.0,
                                "angular_velocity_roll": 0.0,
                                "angular_velocity_pitch": 0.0,
                                "angular_velocity_yaw": yaw_rate,
                                "valid": True,
                            },
                        },
                        "wheel_cylinder_press_report": {
                            "available": 0,
                            "wheel_cylinder_press_report_data": {
                                "timestamp_us": 0,
                                "front_left": 0.0,
                                "front_right": 0.0,
                                "rear_left": 0.0,
                                "rear_right": 0.0,
                                "valid": False,
                            },
                        },
                        "wheel_frct_torque_report": {
                            "available": 0,
                            "wheel_frct_torque_report_data": {
                                "timestamp_us": 0,
                                "front_left": 0.0,
                                "front_right": 0.0,
                                "rear_left": 0.0,
                                "rear_right": 0.0,
                                "valid": False,
                            },
                        },
                    }
                )
    except Exception:
        return 0

    with open(output_path, "w") as f:
        json.dump(wheel_messages, f, indent=2)
    print(f"          Done: {len(wheel_messages)} records")
    return len(wheel_messages)


# =============================================================================
# 第七部分：YAML写入函数
# ==============================================================================


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


# =============================================================================
# 第八部分：主函数
# =============================================================================

# 时间戳目录格式: YYYY-MM-DD-HH-MM-SS
TIMESTAMP_DIR_PATTERN = re.compile(r"^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$")


# =============================================================================
# 第七部分：calib_anno 生成函数
# =============================================================================


def read_camera_calibration_yaml(yaml_path: str) -> Optional[Dict]:
    """读取相机标定 YAML 文件"""
    try:
        with open(yaml_path, "r") as f:
            content = f.read()
        # 手动解析 YAML (因为格式不是标准 YAML)
        result = {}
        for line in content.split("\n"):
            line = line.strip()
            if (
                ":" in line
                and not line.startswith("#")
                and not line.startswith("r_s2b")
                and not line.startswith("t_s2b")
            ):
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                # 处理数值
                if key in ["is_fisheye", "is_fisheye:"]:
                    result[key.replace(":", "")] = value.lower() == "true"
                elif key.replace(":", "") in [
                    "fx",
                    "fy",
                    "cx",
                    "cy",
                    "kc2",
                    "kc3",
                    "kc4",
                    "kc5",
                    "width",
                    "height",
                    "timestamp_shift",
                    "line_exposure_delay",
                ]:
                    try:
                        result[key.replace(":", "")] = float(value)
                    except:
                        pass
        return result
    except Exception as e:
        return None


def generate_calib_anno() -> None:
    """从 calibration/camera/*.yaml 生成 calib_anno/*.json"""
    calib_camera_dir = os.path.join(TARGET_DIR, "calibration/camera")
    calib_anno_dir = os.path.join(TARGET_DIR, "calib_anno")

    if not os.path.exists(calib_camera_dir):
        return

    print(f"  [CalibAnno] Src: {calib_camera_dir}")
    print(f"              Dst: {calib_anno_dir}")

    create_directory(calib_anno_dir)
    yaml_files = glob.glob(os.path.join(calib_camera_dir, "*.yaml"))
    count = 0

    for yaml_file in yaml_files:
        sensor_name = os.path.basename(yaml_file).replace(".yaml", "")

        # 读取 YAML 文件 (处理 ROS YAML 格式: 跳过 %YAML:1.0 头部)
        try:
            with open(yaml_file, "r") as f:
                content = f.read()
                # 跳过 ROS YAML 头部 (如 %YAML:1.0)
                lines = content.split("\n")
                yaml_content = "\n".join(
                    [line for line in lines if not line.startswith("%")]
                )
                calib = yaml.safe_load(yaml_content)
        except:
            continue

        if not calib:
            continue

        # 构建外参矩阵 (4x4)
        r_s2b = calib.get("r_s2b", [0, 0, 0])
        t_s2b = calib.get("t_s2b", [0, 0, 0])

        R = rotation_vector_to_matrix(r_s2b)
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        extrinsic[:3, 3] = t_s2b

        # 构建内参矩阵和畸变系数
        is_fisheye = calib.get("is_fisheye", False)

        if is_fisheye:
            # 鱼眼相机 (ocam 模型)
            affine = calib.get("affine_parameters", {})
            ac = affine.get("ac", 1.0)
            ad = affine.get("ad", 0.0)
            ae = affine.get("ae", 0.0)
            cx = affine.get("cx", 0.0)
            cy = affine.get("cy", 0.0)

            intrinsic = [ac, ad, cx, ae, 1.0, cy, 0.0, 0.0, 1.0]

            result = {
                "extrinsic": extrinsic.flatten().tolist(),
                "intrinsic": intrinsic,
                "distcoeff": [0.0, 0.0, 0.0, 0.0],
            }

            # 添加鱼眼参数
            poly = calib.get("poly_parameters", {})
            inv_poly = calib.get("inv_poly_parameters", {})

            if poly:
                result["poly_parameters"] = [
                    poly.get("p0", 0),
                    poly.get("p1", 0),
                    poly.get("p2", 0),
                    poly.get("p3", 0),
                    poly.get("p4", 0),
                ]

            if inv_poly:
                result["inv_poly_parameters"] = [
                    inv_poly.get(f"p{i}", 0) for i in range(16)
                ]
        else:
            # 普通相机 (polyn 模型)
            fx = calib.get("fx", 0)
            fy = calib.get("fy", 0)
            cx = calib.get("cx", 0)
            cy = calib.get("cy", 0)
            kc2 = calib.get("kc2", 0)
            kc3 = calib.get("kc3", 0)
            kc4 = calib.get("kc4", 0)
            kc5 = calib.get("kc5", 0)

            intrinsic = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

            result = {
                "extrinsic": extrinsic.flatten().tolist(),
                "intrinsic": intrinsic,
                "distcoeff": [kc2, kc3, kc4, kc5],
            }

        # 写入 JSON
        json_path = os.path.join(calib_anno_dir, f"{sensor_name}.json")
        with open(json_path, "w") as f:
            json.dump(result, f, indent=4)
        count += 1

    print(f"              Done: {count} cameras")


def generate_calib_anno_vc() -> None:
    """从 calibration/virtual_camera/*.yaml 生成 calib_anno_vc/*.json"""
    calib_vc_dir = os.path.join(TARGET_DIR, "calibration/virtual_camera")
    calib_anno_vc_dir = os.path.join(TARGET_DIR, "calib_anno_vc")

    if not os.path.exists(calib_vc_dir):
        return

    print(f"  [CalibAnnoVC] Src: {calib_vc_dir}")
    print(f"                Dst: {calib_anno_vc_dir}")

    create_directory(calib_anno_vc_dir)
    yaml_files = glob.glob(os.path.join(calib_vc_dir, "*.yaml"))
    count = 0

    for yaml_file in yaml_files:
        sensor_name = os.path.basename(yaml_file).replace(".yaml", "")
        # 移除 _virtual_camera 后缀
        if sensor_name.endswith("_virtual_camera"):
            sensor_name = sensor_name[:-15]

        # 读取 YAML 文件 (处理 ROS YAML 格式: 跳过 %YAML:1.0 头部)
        try:
            with open(yaml_file, "r") as f:
                content = f.read()
                # 跳过 ROS YAML 头部 (如 %YAML:1.0)
                lines = content.split("\n")
                yaml_content = "\n".join(
                    [line for line in lines if not line.startswith("%")]
                )
                calib = yaml.safe_load(yaml_content)
        except:
            continue

        if not calib:
            continue

        # 构建外参矩阵 (4x4)
        r_s2b = calib.get("r_s2b", [0, 0, 0])
        t_s2b = calib.get("t_s2b", [0, 0, 0])

        R = rotation_vector_to_matrix(r_s2b)
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        extrinsic[:3, 3] = t_s2b

        # 内参 (虚拟相机已去畸变)
        fx = calib.get("fx", 0)
        fy = calib.get("fy", 0)
        cx = calib.get("cx", 0)
        cy = calib.get("cy", 0)

        intrinsic = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        result = {
            "extrinsic": extrinsic.flatten().tolist(),
            "intrinsic": intrinsic,
            "distcoeff": [0.0, 0.0, 0.0, 0.0],
        }

        # 写入 JSON
        json_path = os.path.join(calib_anno_vc_dir, f"{sensor_name}.json")
        with open(json_path, "w") as f:
            json.dump(result, f, indent=4)
        count += 1

    print(f"                Done: {count} virtual cameras")


# =============================================================================
# 第八部分：node_output 生成函数
# =============================================================================


def find_nearest_timestamp(
    target_ts: int, timestamps: List[int], max_diff: int = 100000000
) -> Optional[int]:
    """
    在时间戳列表中找到最接近的目标时间戳

    Args:
        target_ts: 目标时间戳 (ns)
        timestamps: 时间戳列表 (ns)
        max_diff: 最大允许时间差 (ns), 默认100ms

    Returns:
        最接近的时间戳，如果超出范围则返回 None
    """
    if not timestamps:
        return None

    # 二分查找
    left, right = 0, len(timestamps) - 1
    while left < right:
        mid = (left + right) // 2
        if timestamps[mid] < target_ts:
            left = mid + 1
        else:
            right = mid

    # 检查左右两个候选
    candidates = []
    if left > 0:
        candidates.append(timestamps[left - 1])
    if left < len(timestamps):
        candidates.append(timestamps[left])

    # 选择时间差最小的
    best_ts = None
    best_diff = max_diff
    for ts in candidates:
        diff = abs(ts - target_ts)
        if diff < best_diff:
            best_diff = diff
            best_ts = ts

    return best_ts


def generate_node_output() -> None:
    """
    生成 node_output/peral-dataproc/result.json 和 result_full11v.json
    建立点云帧到相机图像和 lidar result 的时间戳映射

    数据源: result/test_calibration/sync_sensors.txt (PCD→相机确定对应)
    format: #frame /middle /cam_front_right /cam_back /cam_side_left_front /cam_side_right_front /cam_side_left_back /cam_side_right_back /cam_front_left /cam_around_front /cam_around_left /cam_around_right /cam_around_back
    """
    # 路径配置
    lidar_concat_dir = os.path.join(TARGET_DIR, "sensor_data/lidar/lidar_concat")
    sync_sensors_file = os.path.join(
        SRC_BAG_DIR, "result/test_calibration/sync_sensors.txt"
    )
    output_dir = os.path.join(TARGET_DIR, "node_output/peral-dataproc")

    if not os.path.exists(lidar_concat_dir):
        print(f"  [NodeOutput] Skip: {lidar_concat_dir} not found")
        return

    if not os.path.exists(sync_sensors_file):
        print(f"  [NodeOutput] Skip: {sync_sensors_file} not found")
        return

    print(f"  [NodeOutput] Src: {sync_sensors_file}")
    print(f"               Dst: {output_dir}/result.json, result_full11v.json")

    # 相机名称映射 (sync_sensors.txt 列名 -> 输出 JSON 字段名)
    camera_col_mapping = {
        "/middle": "camera_front_right",
        "/cam_back": "camera_rear_mid",
        "/cam_side_left_front": "camera_left_front",
        "/cam_side_right_front": "camera_right_front",
        "/cam_side_left_back": "camera_left_rear",
        "/cam_side_right_back": "camera_right_rear",
        "/cam_front_left": "camera_front_far",
        "/cam_around_front": "camera_front_fisheye",
        "/cam_around_left": "camera_left_fisheye",
        "/cam_around_right": "camera_right_fisheye",
        "/cam_around_back": "camera_rear_fisheye",
    }

    # 收集 lidar_concat 中的 PCD 文件列表
    pcd_files_set = {
        f.replace(".pcd", "")
        for f in os.listdir(lidar_concat_dir)
        if f.endswith(".pcd")
    }

    # 读取 sync_sensors.txt 并构建映射
    result = {}
    result_full11v = {}
    matched_count = 0

    with open(sync_sensors_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 13:
                continue

            pcd_ts = parts[1]
            pcd_name = f"{pcd_ts}.pcd"

            # 检查 PCD 是否存在于 lidar_concat
            if pcd_ts not in pcd_files_set:
                continue

            frame_mapping = {}

            for col_idx, (col_name, out_name) in enumerate(
                camera_col_mapping.items(), start=2
            ):
                cam_ts = parts[col_idx]
                frame_mapping[out_name] = f"{cam_ts}.jpeg"

            result[pcd_name] = frame_mapping.copy()
            result_full11v[pcd_name] = frame_mapping.copy()
            matched_count += 1

    # 写入结果
    create_directory(output_dir)

    output_path = os.path.join(output_dir, "result.json")
    with open(output_path, "w") as f:
        json.dump(result, f, indent=2)

    output_path_full = os.path.join(output_dir, "result_full11v.json")
    with open(output_path_full, "w") as f:
        json.dump(result_full11v, f, indent=2)

    print(f"               Done: {matched_count} frames mapped")


def generate_camera_poses() -> None:
    """
    从 result/test_calibration/cam_xxx/sync_sensors.txt 生成 sensor_data/egopose_opt/camera_xxx/*.json

    每个相机文件夹的 sync_sensors.txt 格式:
    #index timestamp x y z q_x q_y q_z q_w
    1 1764310903495000000.jpeg -0.229947 1.151212 -1.999342 0.009042 0.008805 0.753082 0.657805

    重要：sync_sensors.txt 中的时间戳是相机时间戳，直接使用该时间戳命名 JSON 文件
    """
    test_calibration_dir = os.path.join(SRC_BAG_DIR, "result/test_calibration")

    if not os.path.exists(test_calibration_dir):
        return

    # 相机名称映射 (cam_xxx -> camera_xxx)
    camera_name_map = {
        "cam_around_back": "camera_rear_fisheye",
        "cam_around_front": "camera_front_fisheye",
        "cam_around_left": "camera_left_fisheye",
        "cam_around_right": "camera_right_fisheye",
        "cam_back": "camera_rear_mid",
        "cam_front_left": "camera_front_far",
        "cam_front_right": "camera_front_wide",
        "cam_side_left_back": "camera_left_rear",
        "cam_side_left_front": "camera_left_front",
        "cam_side_right_back": "camera_right_rear",
        "cam_side_right_front": "camera_right_front",
    }

    total_files = 0
    total_poses = 0

    for cam_src_name, cam_target_name in camera_name_map.items():
        sync_sensors_file = os.path.join(
            test_calibration_dir, cam_src_name, "sync_sensors.txt"
        )

        if not os.path.exists(sync_sensors_file):
            continue

        # 读取 sync_sensors.txt
        poses = {}
        try:
            with open(sync_sensors_file, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue

                    parts = line.split()
                    if len(parts) >= 9:
                        try:
                            # 格式: index timestamp x y z q_x q_y q_z q_w
                            timestamp_str = parts[1].replace(".jpeg", "")
                            timestamp_ns = int(timestamp_str)
                            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                            qx, qy, qz, qw = (
                                float(parts[5]),
                                float(parts[6]),
                                float(parts[7]),
                                float(parts[8]),
                            )

                            poses[timestamp_ns] = [x, y, z, qx, qy, qz, qw]
                        except (ValueError, IndexError):
                            continue
        except Exception:
            continue

        if not poses:
            continue

        # 生成 JSON 文件
        target_dir = os.path.join(
            TARGET_DIR, "sensor_data/egopose_opt", cam_target_name
        )
        create_directory(target_dir)

        for timestamp_ns, pose_data in poses.items():
            msg = create_pose_message(str(timestamp_ns), pose_data)

            filepath = os.path.join(target_dir, f"{timestamp_ns}.json")
            with open(filepath, "w") as f:
                json.dump(msg, f, indent=2)
            total_poses += 1

        total_files += 1

    if total_files > 0:
        print(f"  [CameraPoses] Src: {test_calibration_dir}/cam_*/sync_sensors.txt")
        print(f"               Dst: {TARGET_DIR}/sensor_data/egopose_opt/camera_*/")
        print(f"               Done: {total_files} cameras, {total_poses} poses")


def generate_lidar_main_pose() -> None:
    """
    从 result/test_calibration/middle/sync_sensors.txt 生成 sensor_data/egopose_opt/egopose_optpose/*.json
    主雷达位姿

    重要：sync_sensors.txt 中的时间戳是相机时间戳（如 1764310903495000000.jpeg）
          需要通过 result.json 找到对应的 PCD 时间戳来命名 JSON 文件
          例如：相机时间戳 1764310903495000000 -> PCD 时间戳 1764310903499619840
    """
    middle_sync_file = os.path.join(
        SRC_BAG_DIR, "result/test_calibration/middle/sync_sensors.txt"
    )
    result_json_file = os.path.join(
        TARGET_DIR, "node_output/peral-dataproc/result.json"
    )

    if not os.path.exists(middle_sync_file):
        return

    if not os.path.exists(result_json_file):
        print(
            f"  [LidarMainPose] Warning: result.json not found, cannot map camera timestamps to PCD timestamps"
        )
        return

    # 读取 sync_sensors.txt (相机时间戳 -> 位姿)
    camera_poses = {}
    try:
        with open(middle_sync_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue

                parts = line.split()
                if len(parts) >= 9:
                    try:
                        # 格式: index timestamp x y z q_x q_y q_z q_w
                        timestamp_str = parts[1].replace(".jpeg", "")
                        # 保持为字符串类型，避免类型不匹配
                        timestamp_ns = timestamp_str  # 不转换为 int
                        x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                        qx, qy, qz, qw = (
                            float(parts[5]),
                            float(parts[6]),
                            float(parts[7]),
                            float(parts[8]),
                        )

                        camera_poses[timestamp_ns] = [x, y, z, qx, qy, qz, qw]
                    except (ValueError, IndexError):
                        continue
    except Exception:
        return

    if not camera_poses:
        return

    # 读取 result.json (PCD -> 相机映射)
    try:
        with open(result_json_file, "r") as f:
            result_data = json.load(f)

        # 调试信息
        print(
            f"  [LidarMainPose] Debug: Loaded result.json with {len(result_data)} PCD entries"
        )
        print(f"  [LidarMainPose] Debug: camera_poses count: {len(camera_poses)}")

    except Exception as e:
        print(f"  [LidarMainPose] Error reading result.json: {e}")
        return

    # 构建相机时间戳 -> PCD时间戳 的映射
    camera_to_pcd = {}
    for pcd_key, cameras in result_data.items():
        pcd_ts = pcd_key.replace(".pcd", "")
        for cam_name, img_name in cameras.items():
            cam_ts = img_name.replace(".jpeg", "")
            camera_to_pcd[cam_ts] = pcd_ts

    # 生成 JSON 文件 (使用 PCD 时间戳命名)
    target_dir = os.path.join(TARGET_DIR, "sensor_data/egopose_opt/egopose_optpose")
    create_directory(target_dir)

    camera_poses = transform_poses_to_first_frame(camera_poses)

    matched_count = 0
    debug_count = 0

    for camera_ts, pose_data in camera_poses.items():
        # 查找对应的 PCD 时间戳
        pcd_ts = camera_to_pcd.get(camera_ts)

        if pcd_ts:
            # 使用 PCD 时间戳作为文件名
            msg = create_pose_message(str(camera_ts), pose_data)
            filepath = os.path.join(target_dir, f"{pcd_ts}.json")

            with open(filepath, "w") as f:
                json.dump(msg, f, indent=2)
            matched_count += 1
        else:
            print(f"  [LidarMainPose] Src: {middle_sync_file}")
    print(f"                 Dst: {target_dir}/")
    print(f"                 Done: {matched_count} poses (using PCD timestamps)")
    if debug_count > 0:
        print(
            f"                 Warning: {debug_count} camera timestamps not found in result.json"
        )


def find_valid_bag_dirs(root_path: str) -> List[str]:
    """
    递归查找符合时间戳格式且包含 bag 文件的有效目录

    Args:
        root_path: 搜索根目录

    Returns:
        有效目录路径列表
    """
    valid_dirs = []

    for dirpath, dirnames, filenames in os.walk(root_path):
        # 检查目录名是否符合时间戳格式
        dir_name = os.path.basename(dirpath)
        if not TIMESTAMP_DIR_PATTERN.match(dir_name):
            continue

        # 检查是否有 bag 文件
        bag_files = [f for f in filenames if f.endswith(".bag")]
        if bag_files:
            valid_dirs.append(dirpath)
    valid_dirs.sort()
    return valid_dirs


def process_single_bag(src_dir: str, target_root: str) -> bool:
    """
    处理单个 bag 目录

    Args:
        src_dir: 源 bag 目录路径
        target_root: 目标根目录

    Returns:
        处理是否成功
    """
    global SRC_BAG_DIR, TARGET_ROOT, TARGET_DIR, BAG_PATH

    # 初始化路径
    SRC_BAG_DIR = src_dir
    TARGET_ROOT = target_root

    # 从源路径提取最后两级目录作为目标子目录
    src_path_parts = SRC_BAG_DIR.rstrip("/").split("/")
    if len(src_path_parts) >= 2:
        TARGET_DIR = os.path.join(TARGET_ROOT, src_path_parts[-2], src_path_parts[-1])
    else:
        TARGET_DIR = os.path.join(TARGET_ROOT, src_path_parts[-1])

    # 自动查找 bag 文件
    BAG_PATH = find_bag_file()

    try:
        create_directory(TARGET_DIR)

        # 第一部分：相机、点云、标定
        extract_camera_images()
        extract_lidar_pcd()
        extract_lidar_concat()
        extract_calibration()

        # 生成 calib_anno 和 calib_anno_vc
        generate_calib_anno()
        generate_calib_anno_vc()

        # 生成 node_output
        generate_node_output()

        # 生成相机位姿
        generate_camera_poses()

        # 生成主雷达位姿
        generate_lidar_main_pose()

        # 拷贝 lidar 地图（需要在 generate_lidar_main_pose 之后，因为需要使用 egopose_optpose 中的时间戳）
        extract_lidar_map()

        # 第二部分：Pose数据
        convert_pose_online()
        convert_pose_offline()

        # 第三部分：IMU/GNSS/Wheel
        extract_imu_from_bag()
        extract_gnss_from_bag()
        extract_wheel_from_bag()

        return True

    except Exception as e:
        print(f"  Error: {e}")
        return False


def main():
    """主函数 - 批量处理符合条件的数据目录"""
    import argparse

    parser = argparse.ArgumentParser(
        description="ROS Bag Format Converter (Batch Processing)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python3 reshape.py /path/to/data_root
  python3 reshape.py /path/to/data_root --target-root /path/to/output
  
Description:
  Recursively find directories matching YYYY-MM-DD-HH-MM-SS format with .bag files,
  then perform batch format conversion.
        """,
    )
    parser.add_argument(
        "data_root",
        type=str,
        help="Data root directory, will recursively search for timestamp-format bag directories",
    )
    parser.add_argument(
        "--target-root",
        "-t",
        type=str,
        default=None,
        help="Target root directory (optional, default: current directory)",
    )

    args = parser.parse_args()

    # 检查数据根目录
    data_root = args.data_root
    if not os.path.isdir(data_root):
        print(f"Error: Data root directory not found: {data_root}")
        return

    # 设置目标根目录
    target_root = args.target_root if args.target_root else os.getcwd()

    print(f"Data root: {data_root}")
    print(f"Target root: {target_root}")

    # 查找有效的 bag 目录
    valid_dirs = find_valid_bag_dirs(data_root)

    if not valid_dirs:
        print("No valid data directories found")
        return

    print(f"Found {len(valid_dirs)} data directories")

    # 批量处理
    success_count = 0
    fail_count = 0

    for i, src_dir in enumerate(valid_dirs, 1):
        print(f"\n[{i}/{len(valid_dirs)}] {src_dir}")
        if process_single_bag(src_dir, target_root):
            success_count += 1
        else:
            fail_count += 1

    # 汇总
    print(f"\nDone! Success: {success_count}, Failed: {fail_count}")


if __name__ == "__main__":
    # 显示检测到的 ROS 版本
    if ROS_AVAILABLE == "ros1":
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print(f"检测到 ROS 版本: ROS 1 (Noetic)")
        print("=" * 60)
    elif ROS_AVAILABLE == "ros2":
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print(f"检测到 ROS 版本: ROS 2 (支持 .db3/.mcap 格式)")
        print("=" * 60)
    else:
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print("错误: 未检测到 ROS 1 或 ROS 2 的 bag 库")
        print("  - ROS 1: 请安装 ros-noetic-rosbag")
        print("  - ROS 2: 请安装 rosbag2-py")
        print("=" * 60)
        exit(1)

    # 调用 main 函数处理命令行参数
    main()
