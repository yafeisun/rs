"""
标定相关函数
"""

import os
import glob
import json
import yaml
import numpy as np
from typing import Optional, Dict, List
from .pose import (
    rotation_vector_to_matrix,
    create_pose_message,
    transform_poses_to_first_frame,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
)
from .utils import find_calibration_yaml, read_yaml_file


def generate_calib_anno(target_dir: str) -> None:
    """从 calibration/camera/*.yaml 生成 calib_anno/*.json"""
    calib_dir = f"{target_dir}/calibration/camera"
    calib_anno_dir = f"{target_dir}/calib_anno"

    if not os.path.exists(calib_dir):
        return
    os.makedirs(calib_anno_dir, exist_ok=True)

    for yaml_file in glob.glob(os.path.join(calib_dir, "*.yaml")):
        sensor_name = os.path.basename(yaml_file).replace(".yaml", "")
        try:
            with open(yaml_file, "r") as f:
                content = f.read()
                if "%YAML" in content:
                    content = content.split("---", 1)[-1]
                calib = yaml.safe_load(content)
        except:
            continue
        if not calib:
            continue

        # 构建外参 (已经是 FLU)
        R = rotation_vector_to_matrix(calib.get("r_s2b", [0, 0, 0]))
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        extrinsic[:3, 3] = calib.get("t_s2b", [0, 0, 0])

        anno = {
            "extrinsic": extrinsic.flatten().tolist(),
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
    """从 calibration/virtual_camera/*.yaml 生成 calib_anno_vc/*.json"""
    calib_dir = f"{target_dir}/calibration/virtual_camera"
    calib_anno_dir = f"{target_dir}/calib_anno_vc"

    if not os.path.exists(calib_dir):
        return
    os.makedirs(calib_anno_dir, exist_ok=True)

    for yaml_file in glob.glob(os.path.join(calib_dir, "*.yaml")):
        sensor_name = (
            os.path.basename(yaml_file)
            .replace(".yaml", "")
            .replace("_virtual_camera", "")
        )
        try:
            with open(yaml_file, "r") as f:
                content = f.read()
                if "%YAML" in content:
                    content = content.split("---", 1)[-1]
                calib = yaml.safe_load(content)
        except:
            continue
        if not calib:
            continue

        R = rotation_vector_to_matrix(calib.get("r_s2b", [0, 0, 0]))
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        extrinsic[:3, 3] = calib.get("t_s2b", [0, 0, 0])

        anno = {
            "extrinsic": extrinsic.flatten().tolist(),
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
    """生成 result.json 和 result_full11v.json"""
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
            if len(parts) < 3:
                continue

            pcd_name = f"{parts[1]}.pcd"
            mapping[pcd_name] = {
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
        json.dump(mapping, f, indent=4)
    with open(f"{target_path}/result_full11v.json", "w") as f:
        json.dump(mapping, f, indent=4)
    print(f"  [NodeOutput] Done: {len(mapping)} frames")


def generate_camera_poses(src_bag_dir: str, target_dir: str) -> None:
    """生成相机位姿并对齐到 Body FLU"""
    test_calib_dir = f"{src_bag_dir}/result/test_calibration"
    if not os.path.exists(test_calib_dir):
        return

    # R_align_inv (RFU -> FLU)
    R_align_inv = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

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

    # 读取原始轨迹并预处理
    for cam_src, cam_dst in camera_name_map.items():
        sync_file = f"{test_calib_dir}/{cam_src}/sync_sensors.txt"
        if not os.path.exists(sync_file):
            continue

        poses = {}
        with open(sync_file, "r") as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                p = line.split()
                if len(p) < 9:
                    continue
                ts = p[1].replace(".jpeg", "")
                poses[ts] = [
                    float(p[2]),
                    float(p[3]),
                    float(p[4]),
                    float(p[5]),
                    float(p[6]),
                    float(p[7]),
                    float(p[8]),
                ]

        # 对齐到 FLU 并转换到第一帧
        aligned_poses = transform_poses_to_first_frame(poses)

        dst_path = f"{target_dir}/sensor_data/egopose_opt/{cam_dst}"
        os.makedirs(dst_path, exist_ok=True)
        for ts, data in aligned_poses.items():
            msg = create_pose_message(ts, data)
            with open(f"{dst_path}/{ts}.json", "w") as f:
                json.dump(msg, f, indent=2)
    print(f"  [CameraPoses] Done")


def generate_lidar_main_pose(src_bag_dir: str, target_dir: str) -> None:
    """生成主雷达位姿并对齐到 Body FLU"""
    sync_file = f"{src_bag_dir}/result/test_calibration/middle/sync_sensors.txt"
    if not os.path.exists(sync_file):
        return

    with open(f"{target_dir}/node_output/peral-dataproc/result.json") as f:
        mapping = json.load(f)
    cam_to_pcd = {
        v["camera_front_right"].replace(".jpeg", ""): k.replace(".pcd", "")
        for k, v in mapping.items()
    }

    poses = {}
    with open(sync_file, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            p = line.split()
            if len(p) < 9:
                continue
            ts = p[1].replace(".jpeg", "")
            poses[ts] = [
                float(p[2]),
                float(p[3]),
                float(p[4]),
                float(p[5]),
                float(p[6]),
                float(p[7]),
                float(p[8]),
            ]

    aligned_poses = transform_poses_to_first_frame(poses)

    dst_path = f"{target_dir}/sensor_data/egopose_opt/egopose_optpose"
    os.makedirs(dst_path, exist_ok=True)
    for cam_ts, data in aligned_poses.items():
        if cam_ts in cam_to_pcd:
            pcd_ts = cam_to_pcd[cam_ts]
            msg = create_pose_message(pcd_ts, data)
            with open(f"{dst_path}/{pcd_ts}.json", "w") as f:
                json.dump(msg, f, indent=2)
    print(f"  [LidarMainPose] Done")
