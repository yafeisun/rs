"""
位姿处理函数
"""

import json
from typing import Dict, List, Optional, Any, Tuple
import math
import numpy as np


def euler_from_quaternion(q: List[float]) -> List[float]:
    """从四元数提取欧拉角 [roll, pitch, yaw]"""
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

def build_extrinsics_matrix(r_s2b: np.ndarray, t_s2b: np.ndarray) -> np.ndarray:
    """构建 4x4 外参矩阵 (sensor -> body)"""
    import cv2
    R, _ = cv2.Rodrigues(r_s2b)
    t = np.array(t_s2b).flatten()

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def rotation_matrix_to_quat(R: np.ndarray) -> List[float]:
    """3x3旋转矩阵转四元数 [qw, qx, qy, qz]"""
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
    将世界坐标系下的 pose 序列转换到第一帧车体坐标系，并统一到 FLU 坐标系。

    输入 pose_dict: {timestamp_ns_str: [x, y, z, qx, qy, qz, qw], ...}
    原始数据坐标系 (RFU): X=右, Y=前, Z=上
    目标坐标系 (FLU): X=前, Y=左, Z=上

    变换步骤:
    1. 计算相对第一帧的位姿 T_rel = T0^-1 * Ti
    2. 将相对位姿从 RFU 转换到 FLU: T_final = R_align^-1 * T_rel * R_align
       其中 R_align 是将 FLU 向量转回 RFU 的矩阵 (即 New X 是 Old Y, New Y 是 -Old X)
    """
    sorted_keys = sorted(pose_dict.keys(), key=lambda k: int(k))
    if not sorted_keys:
        return pose_dict

    # R_align (FLU -> RFU)
    # New X (1,0,0) -> Old Y (0,1,0)
    # New Y (0,1,0) -> -Old X (-1,0,0)
    # New Z (0,0,1) -> Old Z (0,0,1)
    R_align = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    R_align_inv = R_align.T

    # 第一帧位姿 (原始坐标系 RFU)
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_dict[sorted_keys[0]]
    R0 = quat_to_rotation_matrix(qw0, qx0, qy0, qz0)
    t0 = np.array([x0, y0, z0])

    # T0^{-1}
    R0_inv = R0.T
    t0_inv = -R0_inv @ t0

    result = {}
    for key in sorted_keys:
        xi, yi, zi, qxi, qyi, qzi, qwi = pose_dict[key]
        Ri = quat_to_rotation_matrix(qwi, qxi, qyi, qzi)
        ti = np.array([xi, yi, zi])

        # 1. 计算相对位姿 (仍在 RFU 系)
        # T_rel = T0^-1 * Ti
        R_rel = R0_inv @ Ri
        t_rel = R0_inv @ ti + t0_inv

        # 2. 转换到 FLU 系
        # P_flu = R_align_inv * P_rfu
        # T_flu = R_align_inv * T_rfu * R_align
        R_final = R_align_inv @ R_rel @ R_align
        t_final = R_align_inv @ t_rel

        qw_f, qx_f, qy_f, qz_f = rotation_matrix_to_quat(R_final)

        result[key] = [
            float(t_final[0]),
            float(t_final[1]),
            float(t_final[2]),
            qx_f,
            qy_f,
            qz_f,
            qw_f,
        ]

    return result


def read_pcd_binary(pcd_path: str) -> Tuple[np.ndarray, dict]:
    header = {}
    data_offset = 0

    with open(pcd_path, "rb") as f:
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("#"):
                continue
            elif line.startswith("DATA"):
                data_offset = f.tell()
                break

            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = " ".join(parts[1:]) if len(parts) > 2 else parts[1]

        width = int(header.get("WIDTH", 1))
        height = int(header.get("HEIGHT", 1))
        num_points = width * height

        data_bytes = f.read()

        fields_str = header.get("FIELDS", "x y z intensity")
        fields = fields_str.split()
        num_fields = len(fields)
        point_size = num_fields * 4

        dtype_list = [(field, np.float32) for field in fields]
        points = np.frombuffer(data_bytes, dtype=dtype_list, count=num_points)

        return points, header


def transform_point_cloud(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """
    使用 4x4 变换矩阵 T 变换点云
    """
    if len(points) == 0:
        return points

    if hasattr(points, "dtype") and points.dtype.names:
        # 处理结构化数组
        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1)
        ones = np.ones((len(xyz), 1))
        xyz_hom = np.hstack([xyz, ones])
        xyz_transformed = (T @ xyz_hom.T).T
        
        new_points = points.copy()
        new_points["x"] = xyz_transformed[:, 0]
        new_points["y"] = xyz_transformed[:, 1]
        new_points["z"] = xyz_transformed[:, 2]
        return new_points
    else:
        # 处理普通数组
        xyz = points[:, :3]
        ones = np.ones((len(xyz), 1))
        xyz_hom = np.hstack([xyz, ones])
        xyz_transformed = (T @ xyz_hom.T).T
        
        new_points = points.copy()
        new_points[:, :3] = xyz_transformed[:, :3]
        return new_points


def transform_point_cloud_rotation(points_transform: np.ndarray) -> np.ndarray:
    """
    [Legacy] 将点云从原始坐标系转换到目标 FLU 坐标系 (X=前, Y=左, Z=上)
    保留此函数用于兼容性，但建议使用 transform_point_cloud
    """
    # 按照 Y-forward (RFU) -> X-forward (FLU) 的硬编码逻辑
    # X_new = Y_old, Y_new = -X_old
    if hasattr(points_transform, "dtype") and points_transform.dtype.names:
        new_points = points_transform.copy()
        new_points["x"] = points_transform["y"]
        new_points["y"] = -points_transform["x"]
    else:
        new_points = points_transform.copy()
        new_points[:, 0] = points_transform[:, 1]
        new_points[:, 1] = -points_transform[:, 0]
    return new_points


def write_pcd_binary(pcd_path: str, points_transform: np.ndarray, header: dict):
    import os

    os.makedirs(os.path.dirname(pcd_path), exist_ok=True)

    with open(pcd_path, "wb") as f:
        f.write(b"# .PCD v0.7 - Point Cloud Data file format\n")
        f.write(b"VERSION 0.7\n")

        # Handle both structured and regular arrays
        if hasattr(points_transform, "dtype") and points_transform.dtype.names:
            fields = list(points_transform.dtype.names)
        else:
            fields = ["x", "y", "z", "intensity"]

        f.write(f"FIELDS {' '.join(fields)}\n".encode())

        sizes = [str(4) for _ in fields]
        f.write(f"SIZE {' '.join(sizes)}\n".encode())

        types = ["F" for _ in fields]
        f.write(f"TYPE {' '.join(types)}\n".encode())

        counts = ["1" for _ in fields]
        f.write(f"COUNT {' '.join(counts)}\n".encode())

        f.write(f"WIDTH {len(points_transform)}\n".encode())
        f.write(b"HEIGHT 1\n")
        f.write(b"VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points_transform)}\n".encode())

        f.write(b"DATA binary\n")

        points_transform.tofile(f)


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
