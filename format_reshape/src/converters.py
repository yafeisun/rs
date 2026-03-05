"""
数据转换函数
"""

import numpy as np

import json
import math
import os
from typing import Dict, List, Any
from .bag_reader import BagReader, ROS_AVAILABLE
from .pose import (
    load_bev_pose,
    load_mapping_pose,
    create_pose_message,
    transform_poses_to_first_frame,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
)
from .utils import create_directory, POSE_ONLINE_SRC, POSE_OFFLINE_SRC


def transform_poses_rfu_to_flu(
    pose_dict: Dict[str, List[float]],
) -> Dict[str, List[float]]:
    """将位姿从 RFU (Y前进) 转换到 FLU (X前进) 轴的坐标系
    
    RFU: Right-Forward-Up (Y前进，X右，Z上)
    FLU: Front-Left-Up   (X前进，Y左，Z上)
    
    转换: x_flu =  y_rfu  (前方)
          y_flu = -x_rfu  (左方)
          z_flu =  z_rfu  (上方)
    
    旋转变换: R_flu = R_rfu2flu @ R_rfu (正确的公式)
    """
    # RFU -> FLU 旋转矩阵: [[0,1,0],[-1,0,0],[0,0,1]]
    # 这个矩阵表示：[FLU_X, FLU_Y, FLU_Z] = [RFU_Y, -RFU_X, RFU_Z]
    R_rfu2flu = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64)
    
    result = {}
    for key, pose in pose_dict.items():
        x, y, z, qx, qy, qz, qw = pose
        
        # 转换位置
        p_rfu = np.array([x, y, z])
        p_flu = R_rfu2flu @ p_rfu
        
        # 转换方向: R_flu = R_rfu2flu @ R_rfu
        R_rfu = quat_to_rotation_matrix(qw, qx, qy, qz)
        R_flu = R_rfu2flu @ R_rfu
        qw_flu, qx_flu, qy_flu, qz_flu = rotation_matrix_to_quat(R_flu)
        
        result[key] = [
            float(p_flu[0]),
            float(p_flu[1]),
            float(p_flu[2]),
            qx_flu,
            qy_flu,
            qz_flu,
            qw_flu,
        ]
    
    return result
def extract_imu_from_bag(bag_path: str, target_dir: str) -> int:
    """从bag提取IMU数据，返回记录数"""
    if not bag_path:
        return 0

    output_path = f"{target_dir}/sensor_data/imu.json"
    print(f"  [IMU] Src: {bag_path} (topic: /rs/imu)")
    print(f"        Dst: {output_path}")

    try:
        with BagReader(bag_path) as bag:
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


def extract_gnss_from_bag(bag_path: str, target_dir: str) -> int:
    """从bag提取GNSS数据，返回记录数"""
    if not bag_path:
        return 0

    output_path = f"{target_dir}/sensor_data/gnss_rtk.json"
    print(f"  [GNSS] Src: {bag_path} (topic: /rs/gps)")
    print(f"         Dst: {output_path}")

    try:
        with BagReader(bag_path) as bag:
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


def extract_wheel_from_bag(bag_path: str, target_dir: str) -> int:
    """从bag提取轮速数据，返回记录数"""
    if not bag_path:
        return 0

    output_path = f"{target_dir}/sensor_data/wheel_report.json"
    print(f"  [Wheel] Src: {bag_path} (topic: /ft_vehicle_data_v3)")
    print(f"          Dst: {output_path}")

    try:
        with BagReader(bag_path) as bag:
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


def convert_pose_online(
    src_bag_dir: str, target_dir: str
) -> int:
    """转换在线定位结果 (egopose)，返回记录数"""
    src_path = f"{src_bag_dir}/{POSE_ONLINE_SRC}"
    if not os.path.exists(src_path):
        return 0

    pose_dict = load_bev_pose(src_path)
    if not pose_dict:
        return 0

    target_dir_local = f"{target_dir}/sensor_data/egopose"
    target_json = f"{target_dir}/sensor_data/egopose.json"
    print(f"  [Pose Online] Src: {src_path}")
    print(f"                Dst: {target_json}")

    convert_pose_from_dict(
        pose_dict, target_dir_local, f"{target_dir}/sensor_data/egopose.json"
    )
    return len(pose_dict)


def convert_pose_offline(src_bag_dir: str, target_dir: str) -> int:
    """转换离线优化结果 (egopose_opt)，返回记录数"""
    src_path = f"{src_bag_dir}/{POSE_OFFLINE_SRC}"
    if not os.path.exists(src_path):
        return 0

    pose_dict = load_mapping_pose(src_path)
    if not pose_dict:
        return 0

    target_json = f"{target_dir}/sensor_data/egopose_opt.json"
    print(f"  [Pose Offline] Src: {src_path}")
    print(f"                 Dst: {target_json}")

    # 源 pose 数据 (mapping_pose_quaterniond.txt) 是全局绝对坐标，转换到第一帧相对坐标
    pose_dict = transform_poses_to_first_frame(pose_dict)

    # 修正车体朝向: 源数据 body X=右(RFU)，目标需要 X=前(FLU)
    # 只调整旋转，不改变位置: R_flu = R @ R_rfu2flu (后乘，重定义body轴方向)
    R_rfu2flu = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64)
    fixed = {}
    for key, pose in pose_dict.items():
        x, y, z, qx, qy, qz, qw = pose
        R = quat_to_rotation_matrix(qw, qx, qy, qz)
        R_fixed = R @ R_rfu2flu
        qw_f, qx_f, qy_f, qz_f = rotation_matrix_to_quat(R_fixed)
        fixed[key] = [x, y, z, qx_f, qy_f, qz_f, qw_f]
    pose_dict = fixed

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


def convert_pose_from_dict(
    pose_dict: Dict[str, List[float]], target_dir: str, output_json_path: str
) -> None:
    """通用pose转换函数"""
    abs_output_dir = target_dir
    create_directory(abs_output_dir)
    pose_dict = transform_poses_to_first_frame(pose_dict)
    sorted_timestamps = sorted(pose_dict.keys(), key=lambda x: int(x))
    pose_messages = []

    for seq, timestamp_ns in enumerate(sorted_timestamps):
        msg = create_pose_message(timestamp_ns, pose_dict[timestamp_ns])
        msg["header"]["seq"] = seq
        msg["meta"]["seq"] = seq
        pose_messages.append(msg)

        filepath = f"{abs_output_dir}/{timestamp_ns}.json"
        with open(filepath, "w") as f:
            json.dump(msg, f, indent=2)

    with open(output_json_path, "w") as f:
        json.dump(pose_messages, f, indent=2)
