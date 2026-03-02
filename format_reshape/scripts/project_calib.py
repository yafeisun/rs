#!/usr/bin/env python3
"""
点云到图像投影验证脚本 (修复版)
用于验证从速腾 (Robosense) 转换到目标格式后的数据投影。

6. 4路鱼眼相机保持原图投影 (使用 fisheye 模型)。
7. 剩下的7路普通相机在投影前进行实时去畸变处理 (参考 EA-LSS 逻辑)，并使用新内参投影。
"""


import os
import sys
import json
import argparse
from pathlib import Path

import numpy as np
import cv2
import yaml


def load_pcd(pcd_path: str) -> np.ndarray:
    """加载 PCD 文件"""
    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if line.startswith(b"DATA"):
                break
            header_lines.append(line.decode("utf-8"))

        point_num = 0
        for line in header_lines:
            if "POINTS" in line:
                point_num = int(line.split()[1])
                break

        data = np.fromfile(f, dtype=np.float32)[: point_num * 4]
        points = data.reshape(-1, 4)[:, :3]

    return points


def load_camera_calib_yaml(yaml_path: str) -> dict:
    """加载相机标定 YAML"""
    with open(yaml_path, "r") as f:
        content = f.read()

    if content.startswith("%YAML"):
        content = content.split("---", 1)[-1]

    calib = yaml.safe_load(content)

    return {
        "sensor_name": calib.get("sensor_name", ""),
        "r_s2b": np.array(calib.get("r_s2b", [0, 0, 0])),  # Rodrigues vector
        "t_s2b": np.array(calib.get("t_s2b", [0, 0, 0])),
        "fx": calib.get("fx", 0),
        "fy": calib.get("fy", 0),
        "cx": calib.get("cx", 0),
        "cy": calib.get("cy", 0),
        "kc2": calib.get("kc2", 0),
        "kc3": calib.get("kc3", 0),
        "kc4": calib.get("kc4", 0),
        "kc5": calib.get("kc5", 0),
        "is_fisheye": calib.get("is_fisheye", False),
        "width": calib.get("width", 1920),
        "height": calib.get("height", 1080),
    }


def build_extrinsics_matrix(r_s2b: np.ndarray, t_s2b: np.ndarray) -> np.ndarray:
    """构建 4x4 外参矩阵 (sensor -> body)"""
    R, _ = cv2.Rodrigues(r_s2b)
    t = np.array(t_s2b).flatten()

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


def project_points_to_camera(
    points: np.ndarray,
    T_body2cam: np.ndarray,
    camera_calib: dict,
    max_depth: float = 150,
) -> tuple:
    """
    将点云投影到相机图像
    points: Nx3 点云 (假设已经是 Body FLU 坐标系)
    """
    if len(points) == 0:
        return np.array([]), np.array([]), np.array([])

    # 1. 坐标变换: Body -> Camera
    ones = np.ones((len(points), 1))
    coords_hom = np.hstack([points, ones])
    coords_cam = (T_body2cam @ coords_hom.T).T

    # 2. 过滤相机前方点
    valid_mask = coords_cam[:, 2] > 0
    coords_cam = coords_cam[valid_mask]
    if len(coords_cam) == 0:
        return np.array([]), np.array([]), np.array([])

    depths = coords_cam[:, 2].copy()

    # 3. 过滤深度
    depth_mask = depths < max_depth
    coords_cam = coords_cam[depth_mask]
    depths = depths[depth_mask]
    if len(coords_cam) == 0:
        return np.array([]), np.array([]), np.array([])

    # 4. 投影到像素平面
    K = np.array(
        [
            [camera_calib["fx"], 0, camera_calib["cx"]],
            [0, camera_calib["fy"], camera_calib["cy"]],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )

    if camera_calib.get("is_fisheye", False):
        # 鱼眼模型 (等距投影)
        rvec_identity = np.zeros(3)
        tvec_zero = np.zeros(3)
        D = np.array(
            [
                camera_calib["kc2"],
                camera_calib["kc3"],
                camera_calib["kc4"],
                camera_calib["kc5"],
            ],
            dtype=np.float64,
        )

        try:
            pts_cam_3d = coords_cam[:, :3].astype(np.float64).reshape(-1, 1, 3)
            pts_2d, _ = cv2.fisheye.projectPoints(
                pts_cam_3d, rvec_identity, tvec_zero, K, D
            )
            pts_2d = pts_2d.reshape(-1, 2)
            u, v = pts_2d[:, 0], pts_2d[:, 1]
        except:
            return np.array([]), np.array([]), np.array([])
    else:
        # 针孔模型
        # 如果 kc2..kc5 全为 0，则是简单的线性投影
        D = np.array(
            [
                camera_calib["kc2"],
                camera_calib["kc3"],
                camera_calib["kc4"],
                camera_calib["kc5"],
            ],
            dtype=np.float64,
        )
        
        if np.any(D != 0):
            rvec_identity = np.zeros(3)
            tvec_zero = np.zeros(3)
            try:
                pts_cam_3d = coords_cam[:, :3].astype(np.float64).reshape(-1, 1, 3)
                pts_2d, _ = cv2.projectPoints(
                    pts_cam_3d, rvec_identity, tvec_zero, K, D
                )
                pts_2d = pts_2d.reshape(-1, 2)
                u, v = pts_2d[:, 0], pts_2d[:, 1]
            except:
                return np.array([]), np.array([]), np.array([])
        else:
            # 纯线性投影
            x_norm = coords_cam[:, 0] / coords_cam[:, 2]
            y_norm = coords_cam[:, 1] / coords_cam[:, 2]
            u = K[0, 0] * x_norm + K[0, 2]
            v = K[1, 1] * y_norm + K[1, 2]

    # 5. 过滤图像边界
    width, height = camera_calib["width"], camera_calib["height"]
    valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    return u[valid], v[valid], depths[valid]


def get_undistort_params(calib_data: dict) -> dict:
    """计算去畸变参数 (参考 EA-LSS 逻辑)"""
    intrinsic = np.array([
        [calib_data["fx"], 0, calib_data["cx"]],
        [0, calib_data["fy"], calib_data["cy"]],
        [0, 0, 1]
    ], dtype=np.float64)
    D = np.array([
        calib_data["kc2"], calib_data["kc3"], calib_data["kc4"], calib_data["kc5"]
    ], dtype=np.float64)
    img_width, img_height = calib_data["width"], calib_data["height"]
    
    alpha = 0.3
    new_intrinsic, _ = cv2.getOptimalNewCameraMatrix(
        intrinsic, D, (img_width, img_height), alpha
    )
    
    # 使用 fisheye.initUndistortRectifyMap (即使是针孔相机，EA-LSS 也统一使用了这个)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        intrinsic, D, None, new_intrinsic, (img_width, img_height), cv2.CV_16SC2
    )
    
    return {
        "new_intrinsic": new_intrinsic,
        "map1": map1,
        "map2": map2
    }

def get_color_pseudo(depth_val: float) -> tuple:
    """参考 process_data.py 的伪彩色映射 (depth 0-255)"""
    if depth_val < 32:
        c = int(128 + 4 * depth_val)
        return (c, 0, 0)
    elif depth_val < 96:
        c = int(4 + 4 * (depth_val - 32))
        return (255, c, 0)
    elif depth_val < 159:
        return (254, 255, 2)
    elif depth_val < 223:
        c = int(252 - 4 * (depth_val - 159))
        return (0, c, 255)
    else:
        c = int(252 - 4 * (depth_val - 223))
        return (0, 0, max(0, c))


def main():
    parser = argparse.ArgumentParser(description="点云投影验证 - 转换后格式")
    parser.add_argument(
        "--data-dir",
        type=str,
        default="/home/geely/Documents/sunyafei/0203select_output/605/2025-11-21-16-15-45",
    )
    parser.add_argument("--output-dir", type=str, default="visualize")
    parser.add_argument("--subsample", type=int, default=100000)
    parser.add_argument("--max-frames", type=int, default=-1)
    args = parser.parse_args()

    data_dir = args.data_dir
    output_root = os.path.join(data_dir, args.output_dir)
    os.makedirs(output_root, exist_ok=True)

    # 映射文件
    result_path = os.path.join(data_dir, "node_output/peral-dataproc/result.json")
    print(f"Loading mapping: {result_path}")
    with open(result_path) as f:
        result = json.load(f)

    CAMERA_MAPPING = {"camera_front_right": "camera_front_wide"}

    pcd_names = sorted(result.keys())
    if args.max_frames > 0:
        pcd_names = pcd_names[: args.max_frames]

    camera_calibs = {}
    camera_extrs_inv = {}

    print(f"Processing {len(pcd_names)} frames...")

    for pcd_name in pcd_names:
        # 使用 cam_front_right 的时间戳作为文件夹名
        cam_front_right_image = result[pcd_name].get("camera_front_right")
        if cam_front_right_image:
            timestamp = cam_front_right_image.replace(".jpeg", "")
        else:
            # 如果没有 cam_front_right，使用 PCD 时间戳
            timestamp = pcd_name.replace(".pcd", "")

        pcd_path = os.path.join(data_dir, "sensor_data/lidar/lidar_concat", pcd_name)
        if not os.path.exists(pcd_path):
            continue

        frame_output_dir = os.path.join(output_root, timestamp)
        os.makedirs(frame_output_dir, exist_ok=True)

        points = load_pcd(pcd_path)
        points = points[~np.any(np.isnan(points), axis=1)]
        if len(points) > args.subsample:
            points = points[
                np.random.choice(len(points), args.subsample, replace=False)
            ]

        for camera_name in result[pcd_name].keys():
            actual_camera_name = CAMERA_MAPPING.get(camera_name, camera_name)
            image_name = result[pcd_name][camera_name]
            if not image_name:
                continue

            image_path = os.path.join(
                data_dir, "sensor_data/camera", actual_camera_name, image_name
            )
            if not os.path.exists(image_path):
                continue

            img = cv2.imread(image_path)
            if img is None:
                continue

            if actual_camera_name not in camera_calibs:
                calib_path = os.path.join(
                    data_dir, "calibration/camera", f"{actual_camera_name}.yaml"
                )
                if not os.path.exists(calib_path):
                    continue

                calib = load_camera_calib_yaml(calib_path)
                camera_calibs[actual_camera_name] = calib
                T_cam2b = build_extrinsics_matrix(calib["r_s2b"], calib["t_s2b"])
                camera_extrs_inv[actual_camera_name] = np.linalg.inv(T_cam2b)

            calib = camera_calibs[actual_camera_name]
            T_body2cam = camera_extrs_inv[actual_camera_name]

            # 7V 相机：投影前进行实时去畸变处理
            if not calib["is_fisheye"]:
                params = get_undistort_params(calib)
                img = cv2.remap(img, params["map1"], params["map2"], interpolation=cv2.INTER_LINEAR)
                
                # 使用去畸变后的新内参进行投影，且畸变系数设为 0
                proj_calib = calib.copy()
                new_intr = params["new_intrinsic"]
                proj_calib["fx"], proj_calib["fy"] = float(new_intr[0, 0]), float(new_intr[1, 1])
                proj_calib["cx"], proj_calib["cy"] = float(new_intr[0, 2]), float(new_intr[1, 2])
                proj_calib["kc2"], proj_calib["kc3"] = 0.0, 0.0
                proj_calib["kc4"], proj_calib["kc5"] = 0.0, 0.0
            else:
                # 鱼眼相机：保持原始投影逻辑
                proj_calib = calib

            max_depth = 50 if calib["is_fisheye"] else 150
            u, v, depths = project_points_to_camera(
                points, T_body2cam, proj_calib, max_depth=max_depth
            )

            if len(u) > 0:
                for i in range(len(u)):
                    depth_val = (depths[i] / max_depth) * 255
                    color = get_color_pseudo(depth_val)
                    cv2.drawMarker(img, (int(u[i]), int(v[i])), color, cv2.MARKER_CROSS, 3, 1)

            cv2.imwrite(os.path.join(frame_output_dir, f"{camera_name}.jpeg"), img)

        print(f"  Frame {timestamp} done.")

    print(f"\nDone. Outputs in: {output_root}")


if __name__ == "__main__":
    main()
