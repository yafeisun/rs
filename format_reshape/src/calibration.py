"""
标定相关函数 - 最终稳健版 (同步 Robosense 原生逻辑)
"""

import os
import glob
import json
import yaml
import math
import numpy as np
import cv2
from typing import Optional, Dict, List
from .pose import (
    rotation_vector_to_matrix,
    create_pose_message,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
    euler_to_rotation_matrix,
    build_extrinsics_matrix,
    euler_to_rotation_vector,
)
from .utils import find_calibration_yaml, read_yaml_file


def generate_calib_anno(target_dir: str) -> None:
    """从 calibration/camera/*.yaml 生成 calib_anno/*.json

    extrinsic 存储 body_FLU -> camera (T_b2c = inv(T_c2b))，
    与参考格式 PLEX1H8791_* 一致。
    """
    calib_dir = f"{target_dir}/calibration/camera"
    calib_anno_dir = f"{target_dir}/calib_anno"
    if not os.path.exists(calib_dir):
        return
    os.makedirs(calib_anno_dir, exist_ok=True)
    for yaml_file in glob.glob(os.path.join(calib_dir, "*.yaml")):
        sensor_name = os.path.basename(yaml_file).replace(".yaml", "")
        with open(yaml_file, "r") as f:
            content = f.read()
            if "%YAML" in content:
                content = content.split("---", 1)[-1]
            calib = yaml.safe_load(content)
        if not calib:
            continue
        R = rotation_vector_to_matrix(calib.get("r_s2b", [0, 0, 0]))
        T_c2b = np.eye(4)
        T_c2b[:3, :3] = R
        T_c2b[:3, 3] = calib.get("t_s2b", [0, 0, 0])
        # 存 T_b2c = inv(T_c2b)，与参考格式一致
        T_b2c = np.linalg.inv(T_c2b)
        anno = {
            "extrinsic": T_b2c.flatten().tolist(),
            "intrinsic": [
                calib.get("fx", 0),
                0,
                calib.get("cx", 0),
                0,
                calib.get("fy", 0),
                calib.get("cy", 0),
                0,
                0,
                1,
            ],
            "distcoeff": [
                calib.get("kc2", 0),
                calib.get("kc3", 0),
                calib.get("kc4", 0),
                calib.get("kc5", 0),
            ],
        }
        with open(f"{calib_anno_dir}/{sensor_name}.json", "w") as f:
            json.dump(anno, f, indent=4)
    print(f"  [CalibAnno] Done")


def generate_calib_anno_vc(target_dir: str) -> None:
    """从 calibration/virtual_camera/*.yaml 生成 calib_anno_vc/*.json

    extrinsic 存储 body_FLU -> camera (T_b2c = inv(T_c2b))，
    与参考格式 PLEX1H8791_* 一致。
    """
    calib_dir = f"{target_dir}/calibration/virtual_camera"
    calib_anno_dir = f"{target_dir}/calib_anno_vc"
    if not os.path.exists(calib_dir):
        return
    os.makedirs(calib_anno_dir, exist_ok=True)
    for yaml_file in glob.glob(os.path.join(calib_dir, "*.yaml")):
        sensor_name = (
            os.path.basename(yaml_file).replace(".yaml", "").replace("_virtual", "")
        )
        with open(yaml_file, "r") as f:
            content = f.read()
            if "%YAML" in content:
                content = content.split("---", 1)[-1]
            calib = yaml.safe_load(content)
        if not calib:
            continue
        R = rotation_vector_to_matrix(calib.get("r_s2b", [0, 0, 0]))
        T_c2b = np.eye(4)
        T_c2b[:3, :3] = R
        T_c2b[:3, 3] = calib.get("t_s2b", [0, 0, 0])
        # 存 T_b2c = inv(T_c2b)，与参考格式一致
        T_b2c = np.linalg.inv(T_c2b)
        anno = {
            "extrinsic": T_b2c.flatten().tolist(),
            "intrinsic": [
                calib.get("fx", 0),
                0,
                calib.get("cx", 0),
                0,
                calib.get("fy", 0),
                calib.get("cy", 0),
                0,
                0,
                1,
            ],
            "distcoeff": [0.0, 0.0, 0.0, 0.0],
        }
        with open(f"{calib_anno_dir}/{sensor_name}.json", "w") as f:
            json.dump(anno, f, indent=4)
    print(f"  [CalibAnnoVC] Done")


def generate_node_output(src_bag_dir: str, target_dir: str) -> None:
    sync_file = os.path.join(src_bag_dir, "result/test_calibration/sync_sensors.txt")
    if not os.path.exists(sync_file):
        return
    target_path = f"{target_dir}/node_output/peral-dataproc"
    os.makedirs(target_path, exist_ok=True)
    mapping = {}
    with open(sync_file, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 13:
                continue
            mapping[f"{parts[1]}.pcd"] = {
                "camera_front_far": parts[8] + ".jpeg",
                "camera_front_fisheye": parts[9] + ".jpeg",
                "camera_front_wide": parts[2] + ".jpeg",
                "camera_left_fisheye": parts[10] + ".jpeg",
                "camera_left_front": parts[4] + ".jpeg",
                "camera_left_rear": parts[6] + ".jpeg",
                "camera_rear_fisheye": parts[12] + ".jpeg",
                "camera_rear_mid": parts[3] + ".jpeg",
                "camera_right_fisheye": parts[11] + ".jpeg",
                "camera_right_front": parts[5] + ".jpeg",
                "camera_right_rear": parts[7] + ".jpeg",
                "camera_front_right": parts[2] + ".jpeg",
            }
    with open(f"{target_path}/result.json", "w") as f:
        json.dump(mapping, f, indent=2)
    with open(f"{target_path}/result_full11v.json", "w") as f:
        json.dump(mapping, f, indent=2)
    print(f"  [NodeOutput] Done")


def generate_camera_poses(src_bag_dir: str, target_dir: str) -> None:
    """生成相机位姿并使用精确的 LiDAR2Body 变换对齐。"""
    test_calib_dir = f"{src_bag_dir}/result/test_calibration"
    src_yaml = find_calibration_yaml(src_bag_dir)
    data = read_yaml_file(src_yaml)
    T_l2b = np.eye(4)
    if data:
        for l in data.get("sensors", {}).get("lidar", []):
            if l.get("topic") == "/middle/rslidar_packets_unique":
                c = l.get("calibration", {})
                R = euler_to_rotation_matrix(
                    c.get("roll", 0), c.get("pitch", 0), c.get("yaw", 0)
                )
                T_l2b[:3, :3] = R
                T_l2b[:3, 3] = [c.get("x", 0), c.get("y", 0), c.get("z", 0)]
                break
    T_b2l = np.linalg.inv(T_l2b)

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

    for cam_src, cam_dst in camera_name_map.items():
        sync_file = f"{test_calib_dir}/{cam_src}/sync_sensors.txt"
        if not os.path.exists(sync_file):
            continue
        dst_path = f"{target_dir}/sensor_data/egopose_opt/{cam_dst}"
        os.makedirs(dst_path, exist_ok=True)
        with open(sync_file, "r") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                p = line.split()
                ts = p[1].replace(".jpeg", "")
                Ri = quat_to_rotation_matrix(
                    float(p[8]), float(p[5]), float(p[6]), float(p[7])
                )
                Ti = np.eye(4)
                Ti[:3, :3] = Ri
                Ti[:3, 3] = [float(p[2]), float(p[3]), float(p[4])]
                T_w2b = Ti @ T_b2l
                R_f = T_w2b[:3, :3]
                t_f = T_w2b[:3, 3]
                qw_f, qx_f, qy_f, qz_f = rotation_matrix_to_quat(R_f)
                msg = create_pose_message(
                    ts, [t_f[0], t_f[1], t_f[2], qx_f, qy_f, qz_f, qw_f]
                )
                with open(f"{dst_path}/{ts}.json", "w") as jf:
                    json.dump(msg, jf, indent=2)
    print(f"  [CameraPoses] Done")


def generate_lidar_main_pose(src_bag_dir: str, target_dir: str) -> None:
    sync_file = f"{src_bag_dir}/result/test_calibration/middle/sync_sensors.txt"
    if not os.path.exists(sync_file):
        return
    with open(f"{target_dir}/node_output/peral-dataproc/result.json") as f:
        mapping = json.load(f)
    cam_to_pcd = {
        v["camera_front_right"].replace(".jpeg", ""): k.replace(".pcd", "")
        for k, v in mapping.items()
    }
    src_yaml = find_calibration_yaml(src_bag_dir)
    data = read_yaml_file(src_yaml)
    T_l2b = np.eye(4)
    if data:
        for l in data.get("sensors", {}).get("lidar", []):
            if l.get("topic") == "/middle/rslidar_packets_unique":
                c = l.get("calibration", {})
                R = euler_to_rotation_matrix(
                    c.get("roll", 0), c.get("pitch", 0), c.get("yaw", 0)
                )
                T_l2b[:3, :3] = R
                T_l2b[:3, 3] = [c.get("x", 0), c.get("y", 0), c.get("z", 0)]
                break
    T_b2l = np.linalg.inv(T_l2b)
    dst_path = f"{target_dir}/sensor_data/egopose_opt/egopose_optpose"
    os.makedirs(dst_path, exist_ok=True)
    with open(sync_file, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            p = line.split()
            cam_ts = p[1].replace(".jpeg", "")
            if cam_ts not in cam_to_pcd:
                continue
            pcd_ts = cam_to_pcd[cam_ts]
            Ti = np.eye(4)
            Ti[:3, :3] = quat_to_rotation_matrix(
                float(p[8]), float(p[5]), float(p[6]), float(p[7])
            )
            Ti[:3, 3] = [float(p[2]), float(p[3]), float(p[4])]
            T_w2b = Ti @ T_b2l
            R_f = T_w2b[:3, :3]
            t_f = T_w2b[:3, 3]
            qw_f, qx_f, qy_f, qz_f = rotation_matrix_to_quat(R_f)
            msg = create_pose_message(
                pcd_ts, [t_f[0], t_f[1], t_f[2], qx_f, qy_f, qz_f, qw_f]
            )
            with open(f"{dst_path}/{pcd_ts}.json", "w") as jf:
                json.dump(msg, jf, indent=2)
    print(f"  [LidarMainPose] Done")
