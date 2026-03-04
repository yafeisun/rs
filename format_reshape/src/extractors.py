"""
数据提取函数 - 坐标系对齐版 (Body FLU)
"""

import os
import shutil
import glob
import cv2
import numpy as np
from typing import List, Optional, Dict
from .utils import (
    CAMERA_MAP,
    LIDAR_PCD_MAP,
    LIDAR_CONCAT_SRC,
    LIDAR_CONCAT_DST,
    SYNC_SENSORS_FILE,
    LIDAR_MAP_SRC,
    LIDAR_MAP_DST,
    create_directory,
    read_yaml_file,
    find_calibration_yaml,
    write_yaml_file,
    write_lidar_yaml_file,
    CALIBRATION_MAP,
    LIDAR_MAP,
    LIDAR_HEIGHT_MAP,
)
from .pose import (
    read_pcd_binary,
    transform_point_cloud,
    write_pcd_binary,
    euler_to_rotation_vector,
    build_extrinsics_matrix,
)


def get_alignment_rotation() -> np.ndarray:
    """
    获取从原始车体坐标系 (RFU: Y前, X右, Z上) 到目标车体坐标系 (FLU: X前, Y左, Z上) 的旋转矩阵。
    即: P_flu = R_align_inv * P_rfu
    R_align_inv = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    """
    return np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])


def get_undistort_params(calib_data: Dict) -> Dict:
    """计算去畸变后的新内参 (参考 EA-LSS 逻辑)"""
    # 兼容处理
    cam_int = calib_data.get("CameraIntMat")
    dist_coeff = calib_data.get("DistCoeff")
    img_size = calib_data.get("ImageSize")

    intrinsic = np.array(cam_int).reshape(3, 3)
    D = np.array(dist_coeff)
    img_width, img_height = int(img_size[0]), int(img_size[1])

    alpha = 0.3
    new_intrinsic, _ = cv2.getOptimalNewCameraMatrix(
        intrinsic, D[:4], (img_width, img_height), alpha
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        intrinsic, D[:4], None, new_intrinsic, (img_width, img_height), cv2.CV_16SC2
    )

    return {"new_intrinsic": new_intrinsic, "map1": map1, "map2": map2}


def extract_camera_images(src_bag_dir: str, target_dir: str) -> None:
    """从源目录拷贝相机图像 (原图拷贝)"""
    total_images = 0
    print(f"  [Camera] Src: {src_bag_dir}/cam_*")
    print(f"           Dst: {target_dir}/sensor_data/camera/")

    for camera_name, target_subdir in CAMERA_MAP.items():
        src_camera_dir = os.path.join(src_bag_dir, camera_name)
        target_path = os.path.join(target_dir, target_subdir)

        if not os.path.exists(src_camera_dir):
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

    print(f"           Done: {total_images} images")


def extract_lidar_pcd(src_bag_dir: str, target_dir: str) -> None:
    """从源目录拷贝 pcd 点云文件"""
    total_files = 0
    for src_subdir, target_subdir in LIDAR_PCD_MAP.items():
        src_pcd_dir = os.path.join(src_bag_dir, src_subdir)
        target_path = os.path.join(target_dir, target_subdir)
        if not os.path.exists(src_pcd_dir):
            continue
        create_directory(target_path)
        for pcd_file in glob.glob(os.path.join(src_pcd_dir, "*.pcd")):
            shutil.copy2(
                pcd_file, os.path.join(target_path, os.path.basename(pcd_file))
            )
            total_files += 1
    print(f"  [LiDAR] Done: {total_files} files")


def extract_lidar_concat(src_bag_dir: str, target_dir: str) -> None:
    """
    从 result/test_calibration/middle/*.pcd 读取并转换坐标系后写入
    sensor_data/lidar/lidar_undist/*.pcd

    源 PCD 是 Body RFU 坐标系（速腾车体系：Y前、X右、Z上）。
    目标格式要求 Body FLU（X前、Y左、Z上），即 lidar.yaml 中
    r_s2b=[0,0,0]、t_s2b=[0,0,0] 所定义的与车体系重合的坐标系。

    变换: R_rfu2flu = [[0,1,0],[-1,0,0],[0,0,1]]
      x_flu =  y_rfu  (前方)
      y_flu = -x_rfu  (左方)
      z_flu =  z_rfu  (上方，不变)
    """
    src_dir = os.path.join(src_bag_dir, LIDAR_CONCAT_SRC)
    main_sync_file = os.path.join(
        src_bag_dir, "result/test_calibration/sync_sensors.txt"
    )
    if not os.path.exists(src_dir) or not os.path.exists(main_sync_file):
        return

    # Read timestamps from sync_sensors.txt (column 2 = /middle timestamp)
    timestamps = []
    with open(main_sync_file, "r") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            if len(parts) >= 2:
                timestamps.append(parts[1])  # Column 2 = /middle timestamp

    # Get all PCD files and sort them numerically
    pcd_files = sorted(glob.glob(os.path.join(src_dir, "*.pcd")),
                       key=lambda x: int(os.path.basename(x).replace(".pcd", "")))

    target_path = os.path.join(target_dir, LIDAR_CONCAT_DST)
    create_directory(target_path)

    # RFU -> FLU rotation matrix: [[0,1,0],[-1,0,0],[0,0,1]]
    R_rfu2flu = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64)
    T_rfu2flu = np.eye(4)
    T_rfu2flu[:3, :3] = R_rfu2flu

    copied = 0
    for i, pcd_file in enumerate(pcd_files):
        if i >= len(timestamps):
            break
        try:
            timestamp = timestamps[i]
            new_path = os.path.join(target_path, timestamp + ".pcd")
            # Read, transform RFU->FLU, write
            points, header = read_pcd_binary(pcd_file)
            points = transform_point_cloud(points, T_rfu2flu)
            write_pcd_binary(new_path, points, header)
            copied += 1
        except Exception as e:
            print(f"  [LiDAR Concat] Error {pcd_file}: {e}")
    print(f"  [LiDAR Concat] Done: {copied} files (Body RFU -> Body FLU)")
def extract_lidar_map(src_bag_dir: str, target_dir: str) -> None:
    """拷贝 lidar 地图"""
    src_path = os.path.join(src_bag_dir, LIDAR_MAP_SRC)
    if not os.path.exists(src_path):
        return
    optpose_dir = os.path.join(target_dir, "sensor_data/egopose_opt/egopose_optpose")
    if not os.path.exists(optpose_dir):
        return
    json_files = sorted([f for f in os.listdir(optpose_dir) if f.endswith(".json")])
    if not json_files:
        return
    first_ts = json_files[0].replace(".json", "")
    target_dir_local = os.path.join(target_dir, LIDAR_MAP_DST)
    create_directory(target_dir_local)
    shutil.copy2(src_path, os.path.join(target_dir_local, f"{first_ts}.pcd"))
    print(f"  [LiDAR Map] Done: 1 file")


def extract_calibration(src_bag_dir: str, target_dir: str) -> None:
    """提取标定参数并统一到 Body FLU 坐标系"""
    src_yaml = find_calibration_yaml(src_bag_dir)
    if not src_yaml:
        return
    calib_data = read_yaml_file(src_yaml)
    if not calib_data:
        return

    camera_index = {
        c.get("topic", "").split("/")[1]: c
        for c in calib_data.get("sensors", {}).get("camera", [])
    }
    lidar_index = {
        l.get("topic", ""): l for l in calib_data.get("sensors", {}).get("lidar", [])
    }
    R_align_inv = get_alignment_rotation()

    cam_count = 0
    for src_topic, target_yaml_path in CALIBRATION_MAP.items():
        camera_name = src_topic.split("/")[0]
        if camera_name not in camera_index:
            continue
        cam_data = camera_index[camera_name]
        cal = cam_data.get("calibration", {})
        cam_ext = cal.get("CameraExt", {})

        # 旋转外参 R和t 到 FLU
        rvec_old = euler_to_rotation_vector(
            cam_ext.get("roll", 0), cam_ext.get("pitch", 0), cam_ext.get("yaw", 0)
        )
        T_c2rfu = build_extrinsics_matrix(
            np.array(rvec_old),
            np.array([cam_ext.get("x", 0), cam_ext.get("y", 0), cam_ext.get("z", 0)]),
        )
        R_flu = R_align_inv @ T_c2rfu[:3, :3]
        t_flu = R_align_inv @ T_c2rfu[:3, 3]
        rvec_flu, _ = cv2.Rodrigues(R_flu)

        target_calib = {
            "CLOCK_calib_version": "N/A",
            "CLOCK_calib_details": "factory",
            "CLOCK_calib_date": "N/A",
            "vehicle_type": "alpine",
            "serial_number": "N/A",
            "sensor_name": target_yaml_path.split("/")[-1].replace(".yaml", ""),
            "sensor_type": "camera",
            "timestamp_shift": 0,
            "vehicle_xyz": "front_left_up",
            "r_s2b": rvec_flu.flatten().tolist(),
            "t_s2b": [float(t_flu[0]), float(t_flu[1]), float(t_flu[2])],
            "camera_model": "polyn",
            "fx": cal.get("CameraIntMat", [0] * 6)[0],
            "fy": cal.get("CameraIntMat", [0] * 6)[4],
            "cx": cal.get("CameraIntMat", [0] * 6)[2],
            "cy": cal.get("CameraIntMat", [0] * 6)[5],
            "kc2": cal.get("DistCoeff", [0] * 4)[0],
            "kc3": cal.get("DistCoeff", [0] * 4)[1],
            "kc4": cal.get("DistCoeff", [0] * 4)[2],
            "kc5": cal.get("DistCoeff", [0] * 4)[3],
            "is_fisheye": "around" in camera_name,
            "line_exposure_delay": 0,
            "width": cal.get("ImageSize", [0, 0])[0],
            "height": cal.get("ImageSize", [0, 0])[1],
            "suggested_rect_region_within_ROI": [
                0,
                0,
                cal.get("ImageSize", [0, 0])[0],
                cal.get("ImageSize", [0, 0])[1],
            ],
            "suggested_diagonal_FOV_within_ROI": "N/A",
        }

        target_full_path = os.path.join(target_dir, target_yaml_path)
        os.makedirs(os.path.dirname(target_full_path), exist_ok=True)
        write_yaml_file(target_calib, target_full_path)
        cam_count += 1

        # 生成虚拟标定
        vc_yaml_path = target_yaml_path.replace(
            "calibration/camera/", "calibration/virtual_camera/"
        )
        vc_target_full_path = os.path.join(target_dir, vc_yaml_path)
        os.makedirs(os.path.dirname(vc_target_full_path), exist_ok=True)

        vc_target_calib = target_calib.copy()
        if not target_calib["is_fisheye"]:
            params = get_undistort_params(cal)
            new_intr = params["new_intrinsic"]
            vc_target_calib.update(
                {
                    "fx": float(new_intr[0, 0]),
                    "fy": float(new_intr[1, 1]),
                    "cx": float(new_intr[0, 2]),
                    "cy": float(new_intr[1, 2]),
                    "kc2": 0.0,
                    "kc3": 0.0,
                    "kc4": 0.0,
                    "kc5": 0.0,
                    "is_fisheye": False,
                }
            )
        vc_target_calib["sensor_name"] += "_virtual_camera"
        write_yaml_file(vc_target_calib, vc_target_full_path)

    # Lidar 标定
    lidar_count = 0
    for src_topic, target_yaml_path in LIDAR_MAP.items():
        if src_topic not in lidar_index:
            continue
        cal = lidar_index[src_topic].get("calibration", {})
        rvec_old = euler_to_rotation_vector(
            cal.get("roll", 0), cal.get("pitch", 0), cal.get("yaw", 0)
        )
        T_old = build_extrinsics_matrix(
            np.array(rvec_old),
            np.array([cal.get("x", 0), cal.get("y", 0), cal.get("z", 0)]),
        )
        R_flu = R_align_inv @ T_old[:3, :3]
        t_flu = R_align_inv @ T_old[:3, 3]
        target_calib = {
            "calib_version": "",
            "calib_detailes": "factory",
            "calib_date": "",
            "sensor_name": "lidar",
            "sensor_type": "LiDAR",
            "timestamp_shift": 0,
            "vehicle_xyz": "front_left_up",
            "r_s2b": cv2.Rodrigues(R_flu)[0].flatten().tolist(),
            "t_s2b": [float(t_flu[0]), float(t_flu[1]), float(t_flu[2])],
            "width": 1800,
            "height": LIDAR_HEIGHT_MAP.get(
                lidar_index[src_topic].get("lidar_type"), 128
            ),
        }
        target_full_path = os.path.join(target_dir, target_yaml_path)
        os.makedirs(os.path.dirname(target_full_path), exist_ok=True)
        write_lidar_yaml_file(target_calib, target_full_path)
        lidar_count += 1

    print(
        f"  [Calibration] Done: {cam_count} cameras, {lidar_count} lidars (Aligned to Body FLU)"
    )
