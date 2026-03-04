#!/usr/bin/env python3
"""
点云到图像投影验证模块

投影逻辑:
- 鱼眼相机 (4个): 使用 calib_anno 参数，cv2.fisheye.projectPoints
- 普通相机 (7个): 使用 calib_anno_vc 参数，针孔投影

坐标系: 点云是 Body FLU 坐标系，extrinsic 是 T_b2c (body_FLU -> camera)
"""

import os
import copy
import json
import yaml
import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional

# 深度限制
SHOW_DEPTH = 150


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


def load_camera_params_from_dir(data_dir: str) -> Tuple[Dict, List, List]:
    """
    从标定目录加载所有相机参数
    
    相机参数来源:
    - 鱼眼 (4个): calib_anno/*.json - 含原始畸变系数
    - 普通 (7个): calib_anno_vc/*.json - 虚拟相机参数（已去畸变）

    Returns:
        camera_params: 相机名 -> {intr, extr, D, width, height, is_fisheye}
        fisheye_cams: 鱼眼相机名称列表
        normal_cams: 普通相机名称列表
    """
    camera_params = {}

    calib_yaml_dir = os.path.join(data_dir, "calibration/camera")
    calib_anno_dir = os.path.join(data_dir, "calib_anno")
    calib_anno_vc_dir = os.path.join(data_dir, "calib_anno_vc")

    fisheye_cams = []
    normal_cams = []

    if os.path.exists(calib_yaml_dir):
        for f_name in sorted(os.listdir(calib_yaml_dir)):
            if f_name.endswith(".yaml") and f_name.startswith("camera_"):
                cam_name = f_name.replace(".yaml", "")
                yaml_path = os.path.join(calib_yaml_dir, f_name)
                with open(yaml_path, "r") as y_file:
                    lines = y_file.readlines()
                    content = "".join([line for line in lines if not line.startswith("%")])
                    calib_yaml = yaml.safe_load(content)

                # 判断相机类型
                is_fisheye = calib_yaml.get("is_fisheye", False) or "fisheye" in cam_name

                # 选择标定参数目录
                if is_fisheye:
                    calib_dir = calib_anno_dir
                else:
                    calib_dir = calib_anno_vc_dir

                # 从对应目录加载 JSON 参数
                json_path = os.path.join(calib_dir, f"{cam_name}.json")
                if os.path.exists(json_path):
                    with open(json_path, "r") as j_file:
                        calib_json = json.load(j_file)

                    # 外参: extrinsic 已经是 T_b2c (body_FLU -> camera)
                    extrinsic = np.array(calib_json["extrinsic"], dtype=np.float64).reshape(4, 4)

                    # 内参
                    intrinsic = np.array(calib_json["intrinsic"], dtype=np.float64).reshape(3, 3)

                    # 畸变系数
                    D = np.array(calib_json["distcoeff"], dtype=np.float64)

                    params = {
                        "intr": intrinsic,
                        "extr": extrinsic,
                        "D": D,
                        "width": int(calib_yaml.get("width", 1920)),
                        "height": int(calib_yaml.get("height", 1280)),
                        "is_fisheye": is_fisheye,
                    }

                    camera_params[cam_name] = params
                    if is_fisheye:
                        fisheye_cams.append(cam_name)
                    else:
                        normal_cams.append(cam_name)

    return camera_params, fisheye_cams, normal_cams


def get_color(depth: int) -> tuple:
    """伪彩色"""
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


def point_project(coords: np.ndarray, image: np.ndarray, M1: np.ndarray, M2: np.ndarray):
    """标准针孔投影"""
    resolution = image.shape
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)
    transform = copy.deepcopy(M2).reshape(4, 4)
    coords = (transform @ coords.T).T
    coords = coords[np.where((coords[:, 2] > 0) * (coords[:, 2] < SHOW_DEPTH))]
    zzz = coords[:, 2].copy()

    if len(coords) == 0:
        return None, None

    coords = (M1 @ coords.T).T
    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]
    coords[:, 2] = 1

    valid = (
        (coords[:, 0] >= 0)
        & (coords[:, 0] < resolution[1])
        & (coords[:, 1] >= 0)
        & (coords[:, 1] < resolution[0])
        & (zzz > 0)
    )

    return coords[valid], zzz[valid]


def point_project_distort(coords: np.ndarray, image: np.ndarray, intr: np.ndarray, extr: np.ndarray, D: np.ndarray):
    """鱼眼投影"""
    resolution = image.shape
    points = copy.deepcopy(coords)

    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)

    transform = copy.deepcopy(extr).reshape(4, 4)
    coords = coords @ transform.T

    flag = np.where((coords[:, 2] > 0) * (coords[:, 2] < SHOW_DEPTH))
    coords = coords[flag]
    points = points[flag]
    zzz = coords[:, 2].copy()

    if len(coords) == 0:
        return None, None

    rvec = cv2.Rodrigues(extr[:3, :3])[0]
    tvec = extr[:3, 3]
    points_proj = cv2.fisheye.projectPoints(
        points.reshape(-1, 1, 3), rvec, tvec, intr[:3, :3], D
    )[0].reshape(-1, 2)

    valid = (
        (zzz > 0)
        & (points_proj[:, 0] >= 0)
        & (points_proj[:, 0] < resolution[1])
        & (points_proj[:, 1] >= 0)
        & (points_proj[:, 1] < resolution[0])
    )

    return points_proj[valid], zzz[valid]


def undistort_image(img: np.ndarray, intr: np.ndarray, D: np.ndarray, width: int, height: int):
    """EA-LSS 去畸变"""
    alpha = 0.3
    new_intr, _ = cv2.getOptimalNewCameraMatrix(intr, D[:4], (width, height), alpha)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        intr, D[:4], None, new_intr, (width, height), cv2.CV_16SC2
    )
    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted, new_intr


def process_frame(data_dir: str, output_dir: str, ts: str, cam_mapping: Dict, camera_params: Dict, points: np.ndarray):
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

        is_fisheye = params["is_fisheye"] or "fisheye" in cam_name
        extr = params["extr"]
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
            coords, zzz = point_project(copy.deepcopy(points), img, intr_4x4, extr)

        # 深度归一化
        if is_fisheye:
            show_depth = 50
        else:
            show_depth = 150

        if coords is not None and len(coords) > 0:
            zzz = zzz / show_depth * 255
            for index in range(coords.shape[0]):
                p = (int(coords[index, 0]), int(coords[index, 1]))
                cv2.drawMarker(
                    img,
                    position=p,
                    color=get_color(int(zzz[index])),
                    markerSize=2,
                    markerType=cv2.MARKER_CROSS,
                    thickness=1,
                )

        out_file = os.path.join(frame_out, f"{cam_name}.jpeg")
        cv2.imwrite(out_file, img)


def run_projection_viz(data_dir: str, output_dir: str = "visualize", max_frames: Optional[int] = 10, frame_interval: int = 10) -> bool:
    """
    运行点云投影可视化验证

    Args:
        data_dir: 数据目录
        output_dir: 输出子目录名
        max_frames: 最大处理帧数，默认 10
        frame_interval: 帧间隔，每隔N帧处理一帧，默认 10

    Returns:
        是否成功
    """
    output_root = os.path.join(data_dir, output_dir)
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
        print("    Warning: result.json not found, skipping visualization")
        return False

    with open(mapping_path, "r") as f:
        mapping = json.load(f)

    # 加载相机参数
    camera_params, fisheye_cams, normal_cams = load_camera_params_from_dir(data_dir)

    print(f"    Cameras: {len(normal_cams)} normal + {len(fisheye_cams)} fisheye")

    # 处理帧
    frames = sorted(mapping.keys())
    if frame_interval > 1:
        frames = frames[::frame_interval]
    if max_frames:
        frames = frames[:max_frames]

    for i, ts_key in enumerate(frames):
        pcd_ts = ts_key.replace(".pcd", "")
        cam_mapping = mapping[ts_key]
        wide_img = cam_mapping.get("camera_front_wide", "")
        frame_ts = wide_img.replace(".jpeg", "") if wide_img else pcd_ts

        print(f"    [{i + 1}/{len(frames)}] Frame: {frame_ts}")

        pcd_path = os.path.join(data_dir, "sensor_data/lidar/lidar_undist", f"{pcd_ts}.pcd")
        if not os.path.exists(pcd_path):
            continue

        points = load_pcd(pcd_path)

        # 过滤无效点
        flag = (
            (np.abs(points[:, 0]) <= 500)
            & (np.abs(points[:, 1]) <= 500)
            & (np.abs(points[:, 2]) <= 50)
        )
        points = points[flag]

        process_frame(data_dir, output_root, frame_ts, cam_mapping, camera_params, points)

    print(f"    Output: {output_root}")
    return True


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="点云投影可视化")
    parser.add_argument("--data-dir", required=True)
    parser.add_argument("--output-dir", default="visualize")
    parser.add_argument("--max-frames", type=int, default=10)
    parser.add_argument("--frame-interval", type=int, default=1)
    args = parser.parse_args()
    run_projection_viz(args.data_dir, args.output_dir, args.max_frames, args.frame_interval)
