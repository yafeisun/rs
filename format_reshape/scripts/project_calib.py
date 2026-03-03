#!/usr/bin/env python3
"""
点云到图像投影验证脚本

投影逻辑:
- 7V 普通相机 (不含 fisheye): EA-LSS 去畸变 (alpha=0.3) 后用针孔投影
  参考: EA-LSS/tools/datautil/convert_and_save.py undistort_and_save_img
- 4V 鱼眼相机 (含 fisheye): 使用 point_project_distort (cv2.fisheye.projectPoints)

坐标系: 点云是 Body FLU 坐标系，extrinsic 是 T_b2c (body_FLU -> camera)
"""

import os
import sys
import argparse
import json
import copy
import numpy as np
import cv2
import yaml


# 深度限制 - 与速腾 process_data.py 一致
SHOW_DEPTH = 150  # 全局默认值，实际使用时会根据相机类型调整


def load_pcd(pcd_path: str) -> np.ndarray:
    """Load point cloud from PCD file (supports structured binary)"""
    with open(pcd_path, "rb") as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                break
            try:
                line = line.decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            if line.startswith("DATA"):
                data_start = f.tell()
                break
            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = " ".join(parts[1:])

        fields = header.get("FIELDS", "x y z intensity").split()
        sizes = list(map(int, header.get("SIZE", "4 4 4 4").split()))
        types = header.get("TYPE", "F F F F").split()

        type_map = {"F": np.float32, "I": np.int32, "U": np.uint8, "L": np.uint64}
        dtype_parts = []
        for i, field in enumerate(fields):
            np_type = type_map.get(types[i], np.float32)
            dtype_parts.append((field, np_type))

        dtype = np.dtype(dtype_parts)
        num_points = int(header.get("POINTS", "0"))

        f.seek(data_start)
        data = np.fromfile(f, dtype=dtype, count=num_points)

    points = np.zeros((len(data), 3), dtype=np.float32)
    for i, field in enumerate(fields[:3]):
        points[:, i] = data[field]

    return points


def load_json_params(calib_json: dict) -> dict:
    """
    从 JSON (calib_anno 或 calib_anno_vc) 加载投影参数

    JSON 中的 extrinsic 是 4x4 body_FLU -> camera 矩阵 (T_b2c)，
    可直接用于将 Body FLU 点云变换到相机坐标系，无需取逆。
    """
    # 外参: JSON 中的 extrinsic 已经是 T_b2c (body_FLU -> camera)
    extrinsic = np.array(calib_json["extrinsic"], dtype=np.float64).reshape(4, 4)

    # 内参
    intrinsic = np.array(calib_json["intrinsic"], dtype=np.float64).reshape(3, 3)

    # 畸变系数
    D = np.array(calib_json["distcoeff"], dtype=np.float64)

    # 判断是否是鱼眼 (通过畸变系数判断)
    is_fisheye = not np.allclose(D, 0.0)

    return {
        "intr": intrinsic,
        "extr": extrinsic,  # T_b2c，直接使用，无需取逆
        "D": D,
        "is_fisheye": is_fisheye,
    }


def get_color(depth: int) -> tuple:
    """伪彩色 (参考 process_data.py)"""
    if depth < 32:
        return (128 + 4 * depth, 0, 0)
    elif depth == 32:
        return (255, 0, 0)
    elif depth < 95:
        return (255, 4 + 4 * depth, 0)
    elif depth == 96:
        return (254, 255, 2)
    elif depth < 158:
        return (254, 255, 2)
    elif depth == 159:
        return (1, 255, 254)
    elif depth < 223:
        return (0, 252 - 4 * (depth - 159), 255)
    elif depth < 255:
        return (0, 0, 252 - 4 * (depth - 223))
    return (0, 0, 0)


def point_project(coords, image, M1, M2):
    """
    标准针孔投影 (不带畸变) - 完全参考 process_data.py point_project
    
    Args:
        coords: Nx3 点云坐标 (body坐标系)
        image: 图像 (用于获取分辨率)
        M1: 4x4 内参矩阵
        M2: 4x4 外参矩阵 (body_to_camera)
        
    Returns:
        coords: Nx2 像素坐标
        zzz: N 深度值 (未归一化)
    """
    resolution = image.shape
    
    # 转换到齐次坐标
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)
    
    # 变换到相机坐标系: cam_coords = extr @ body_coords (列向量形式)
    transform = copy.deepcopy(M2).reshape(4, 4)
    coords = (transform @ coords.T).T
    
    # 过滤深度 (使用全局 SHOW_DEPTH)
    coords = coords[np.where((coords[:, 2] > 0) * (coords[:, 2] < SHOW_DEPTH))]
    zzz = coords[:, 2].copy()
    
    if len(coords) == 0:
        return None, None
    
    # 投影到像素坐标
    coords = (M1 @ coords.T).T
    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]
    coords[:, 2] = 1
    
    # 边界过滤
    valid = (coords[:, 0] >= 0) & (coords[:, 0] < resolution[1]) & \
            (coords[:, 1] >= 0) & (coords[:, 1] < resolution[0]) & (zzz > 0)
    
    return coords[valid], zzz[valid]


def point_project_distort(coords, image, intr, extr, D):
    """
    鱼眼投影 (带畸变) - 完全参考 process_data.py point_project_distort
    
    Args:
        coords: Nx3 点云坐标 (body坐标系)
        image: 图像 (用于获取分辨率)
        intr: 4x4 内参矩阵
        extr: 4x4 外参矩阵 (body_to_camera)
        D: 畸变系数
        
    Returns:
        points: Nx2 像素坐标
        zzz: N 深度值 (未归一化)
    """
    resolution = image.shape
    points = copy.deepcopy(coords)
    
    # 转换到齐次坐标
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)
    
    # 变换到相机坐标系 (行向量形式: coords = points @ extr.T)
    transform = copy.deepcopy(extr).reshape(4, 4)
    coords = coords @ transform.T
    
    # 过滤深度 (使用全局 SHOW_DEPTH)
    flag = np.where((coords[:, 2] > 0) * (coords[:, 2] < SHOW_DEPTH))
    coords = coords[flag]
    points = points[flag]
    zzz = coords[:, 2].copy()
    
    if len(coords) == 0:
        return None, None
    
    # 鱼眼投影
    rvec = cv2.Rodrigues(extr[:3, :3])[0]
    tvec = extr[:3, 3]
    points_proj = cv2.fisheye.projectPoints(
        points.reshape(-1, 1, 3), rvec, tvec, intr[:3, :3], D
    )[0].reshape(-1, 2)
    
    # 边界过滤
    valid = (zzz > 0) & (points_proj[:, 0] >= 0) & (points_proj[:, 0] < resolution[1]) & \
            (points_proj[:, 1] >= 0) & (points_proj[:, 1] < resolution[0])
    
    return points_proj[valid], zzz[valid]


def undistort_image(img: np.ndarray, intr: np.ndarray, D: np.ndarray,
                    width: int, height: int):
    """
    EA-LSS 去畸变 (参考 EA-LSS/tools/datautil/convert_and_save.py)
    适用于 7V 普通相机 (非鱼眼)

    Returns:
        undistorted_img: 去畸变后的图像
        new_intr: 去畸变后的新内参矩阵 (3x3)
    """
    alpha = 0.3
    new_intr, _ = cv2.getOptimalNewCameraMatrix(
        intr, D[:4], (width, height), alpha
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        intr, D[:4], None, new_intr, (width, height), cv2.CV_16SC2
    )
    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted, new_intr


def process_frame(data_dir: str, output_dir: str, ts: str,
                  cam_mapping: dict, camera_params: dict, points: np.ndarray):
    """处理单帧数据"""
    frame_out = os.path.join(output_dir, ts)
    os.makedirs(frame_out, exist_ok=True)

    for cam_name, params in camera_params.items():
        if cam_name not in cam_mapping:
            continue

        img_filename = cam_mapping[cam_name]
        img_path = os.path.join(data_dir, "sensor_data/camera", cam_name, img_filename)

        if not os.path.exists(img_path):
            continue

        img = cv2.imread(img_path)
        if img is None:
            continue

        # 判断是否是鱼眼相机
        is_fisheye = params["is_fisheye"] or "fisheye" in cam_name

        extr = params["extr"]  # T_b2c
        D = params["D"]

        if is_fisheye:
            # 鱼眼: 直接用 cv2.fisheye.projectPoints
            intr_4x4 = np.identity(4)
            intr_4x4[:3, :3] = params["intr"]
            coords, zzz = point_project_distort(
                copy.deepcopy(points), img, intr_4x4, extr, D
            )
        else:
            # 7V 普通相机: EA-LSS 去畸变后再用针孔投影
            width = params.get("width", img.shape[1])
            height = params.get("height", img.shape[0])
            img, new_intr = undistort_image(img, params["intr"], D, width, height)
            intr_4x4 = np.identity(4)
            intr_4x4[:3, :3] = new_intr
            coords, zzz = point_project(
                copy.deepcopy(points), img, intr_4x4, extr
            )
        
        # 深度归一化 (与速腾逻辑完全一致)
        # 速腾对所有相机都用 SHOW_DEPTH=150 过滤，但颜色映射范围不同
        if is_fisheye:
            show_depth = 50
        else:
            show_depth = 150
        
        if coords is not None and len(coords) > 0:
            zzz = zzz / show_depth * 255
            circle_thickness = 1
            for index in range(coords.shape[0]):
                p = (int(coords[index, 0]), int(coords[index, 1]))
                cv2.drawMarker(
                    img, position=p, color=get_color(int(zzz[index])),
                    markerSize=circle_thickness * 2,
                    markerType=cv2.MARKER_CROSS,
                    thickness=circle_thickness
                )
        
        # 保存
        out_file = os.path.join(frame_out, f"{cam_name}.jpeg")
        cv2.imwrite(out_file, img)


def main():
    parser = argparse.ArgumentParser(description="点云投影可视化")
    parser.add_argument("--data-dir", required=True, help="数据目录")
    parser.add_argument("--output-dir", default="visualize", help="输出子目录名")
    parser.add_argument("--max-frames", type=int, default=None, help="最大处理帧数")
    args = parser.parse_args()

    data_dir = args.data_dir
    output_root = os.path.join(data_dir, args.output_dir)
    os.makedirs(output_root, exist_ok=True)

    # 加载映射文件
    mapping_path = None
    for p in [
        os.path.join(data_dir, "node_output/peral-dataproc/result.json"),
        os.path.join(data_dir, "node_output/result.json"),
    ]:
        if os.path.exists(p):
            mapping_path = p
            break

    if not mapping_path:
        print("    Error: mapping file not found")
        return

    with open(mapping_path, "r") as f:
        mapping = json.load(f)

    # 加载相机参数 - 根据相机类型从不同来源加载
    camera_params = {}

    # 从 YAML 获取相机基本信息 (用于判断类型和分辨率)
    calib_yaml_dir = os.path.join(data_dir, "calibration/camera")

    # 所有相机均从 calib_anno 加载:
    # - 鱼眼: 含原始畸变系数，直接用于 cv2.fisheye.projectPoints
    # - 普通: 含原始畸变系数，用于 EA-LSS 去畸变 (alpha=0.3) 后再投影
    calib_anno_dir = os.path.join(data_dir, "calib_anno")

    fisheye_cams = []
    normal_cams = []
    
    if os.path.exists(calib_yaml_dir):
        for f_name in sorted(os.listdir(calib_yaml_dir)):
            if f_name.endswith(".yaml") and f_name.startswith("camera_"):
                cam_name = f_name.replace(".yaml", "")
                yaml_path = os.path.join(calib_yaml_dir, f_name)
                with open(yaml_path, 'r') as y_file:
                    lines = y_file.readlines()
                    content = "".join([line for line in lines if not line.startswith('%')])
                    calib_yaml = yaml.safe_load(content)
                
                # 判断相机类型
                is_fisheye = calib_yaml.get("is_fisheye", False) or "fisheye" in cam_name
                
                if is_fisheye:
                    # 鱼眼相机: 从 calib_anno/*.json 加载
                    json_path = os.path.join(calib_anno_dir, f"{cam_name}.json")
                    if os.path.exists(json_path):
                        with open(json_path, 'r') as j_file:
                            calib_json = json.load(j_file)
                        params = load_json_params(calib_json)
                        params["width"] = int(calib_yaml.get("width", 1920))
                        params["height"] = int(calib_yaml.get("height", 1280))
                        camera_params[cam_name] = params
                        fisheye_cams.append(cam_name)
                else:
                    # 普通相机: 从 calib_anno/*.json 加载 (含原始畸变系数，用于 EA-LSS 去畸变)
                    json_path = os.path.join(calib_anno_dir, f"{cam_name}.json")
                    if os.path.exists(json_path):
                        with open(json_path, 'r') as j_file:
                            calib_json = json.load(j_file)
                        params = load_json_params(calib_json)
                        params["width"] = int(calib_yaml.get("width", 1920))
                        params["height"] = int(calib_yaml.get("height", 1280))
                        params["is_fisheye"] = False
                        camera_params[cam_name] = params
                        normal_cams.append(cam_name)
    
    print(f"    Cameras: {len(normal_cams)} normal + {len(fisheye_cams)} fisheye")
    
    # 处理帧
    frames = sorted(mapping.keys())
    if args.max_frames:
        frames = frames[:args.max_frames]

    for i, ts_key in enumerate(frames):
        pcd_ts = ts_key.replace(".pcd", "")
        # 使用 camera_front_wide 的时间戳作为文件夹名
        cam_mapping = mapping[ts_key]
        wide_img = cam_mapping.get("camera_front_wide", "")
        frame_ts = wide_img.replace(".jpeg", "") if wide_img else pcd_ts
        
        print(f"    [{i+1}/{len(frames)}] Frame: {frame_ts}")
        
        pcd_path = os.path.join(data_dir, "sensor_data/lidar/lidar_concat", f"{pcd_ts}.pcd")
        if not os.path.exists(pcd_path):
            continue

        points = load_pcd(pcd_path)

        # 过滤无效点 (参考 process_data.py)
        flag = (np.abs(points[:, 0]) <= 500) & \
               (np.abs(points[:, 1]) <= 500) & \
               (np.abs(points[:, 2]) <= 50)
        points = points[flag]

        # lidar_concat PCD 已是 Body FLU 坐标系，calib_anno extrinsic 已是 T_b2c，
        # 无需额外坐标变换，直接投影。
        process_frame(data_dir, output_root, frame_ts, cam_mapping, camera_params, points)

    print(f"    Output: {output_root}")


if __name__ == "__main__":
    main()
