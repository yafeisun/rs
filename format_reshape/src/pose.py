"""
位姿处理函数 - 最终精度修复版
"""

import json
import math
import os
import numpy as np
import cv2
import math
import numpy as np
import cv2
from typing import Dict, List, Optional, Any, Tuple


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


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """根据 Robosense 约定的 R = Rz * Ry * Rx 构建旋转矩阵。输入是弧度。"""
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
    Ry = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
    Rz = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def euler_to_rotation_vector(roll: float, pitch: float, yaw: float) -> List[float]:
    """将欧拉角转换为旋转向量 (Rodrigues)"""
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten().tolist()


def rotation_vector_to_matrix(r_s2b: List[float]) -> np.ndarray:
    """将旋转向量转换为旋转矩阵 (Rodrigues)"""
    R, _ = cv2.Rodrigues(np.array(r_s2b))
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
    R, _ = cv2.Rodrigues(r_s2b)
    t = np.array(t_s2b).flatten()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rotation_matrix_to_quat(R: np.ndarray) -> List[float]:
    """3x3旋转矩阵转四元数 [qw, qx, qy, qz]"""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return [qw, qx, qy, qz]


def transform_poses_to_first_frame(
    pose_dict: Dict[str, List[float]],
) -> Dict[str, List[float]]:
    """标准相对位姿转换 (不包含轴旋转)"""
    sorted_keys = sorted(pose_dict.keys(), key=lambda k: int(k))
    if not sorted_keys:
        return pose_dict

    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_dict[sorted_keys[0]]
    R0 = quat_to_rotation_matrix(qw0, qx0, qy0, qz0)
    R0_inv = R0.T
    t0 = np.array([x0, y0, z0])
    t0_inv = -R0_inv @ t0

    result = {}
    for key in sorted_keys:
        xi, yi, zi, qxi, qyi, qzi, qwi = pose_dict[key]
        Ri = quat_to_rotation_matrix(qwi, qxi, qyi, qzi)
        ti = np.array([xi, yi, zi])
        R_f = R0_inv @ Ri
        t_f = R0_inv @ ti + t0_inv
        qw_f, qx_f, qy_f, qz_f = rotation_matrix_to_quat(R_f)
        result[key] = [
            float(t_f[0]),
            float(t_f[1]),
            float(t_f[2]),
            qx_f,
            qy_f,
            qz_f,
            qw_f,
        ]
    return result


def transform_point_cloud(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """使用 4x4 变换矩阵 T 变换点云"""
    if len(points) == 0:
        return points
    if hasattr(points, "dtype") and points.dtype.names:
        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1)
        ones = np.ones((len(xyz), 1))
        xyz_h = np.hstack([xyz, ones])
        transformed = (T @ xyz_h.T).T
        res = points.copy()
        res["x"], res["y"], res["z"] = (
            transformed[:, 0],
            transformed[:, 1],
            transformed[:, 2],
        )
        return res
    else:
        xyz = points[:, :3].astype(np.float64)  # Ensure float64 for accurate transformation
        ones = np.ones((len(xyz), 1))
        xyz_h = np.hstack([xyz, ones])
        transformed = (T @ xyz_h.T).T
        res = points.copy().astype(np.float64)
        res[:, :3] = transformed[:, :3]
        return res
        xyz = points[:, :3]
        ones = np.ones((len(xyz), 1))
        xyz_h = np.hstack([xyz, ones])
        transformed = (T @ xyz_h.T).T
        res = points.copy()
        res[:, :3] = transformed[:, :3]
        return res


def read_pcd_binary(pcd_path: str) -> Tuple[np.ndarray, dict]:
    """
    Read binary PCD file (supports DATA binary format).
    Returns points as numpy array with structured dtype and header dict.
    """
    header = {}
    with open(pcd_path, "rb") as f:
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("DATA"):
                # Mark the position after DATA line
                data_start = f.tell()
                break
            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = " ".join(parts[1:])

        # Parse SIZE, TYPE, COUNT fields to determine dtype
        fields = header.get("FIELDS", "x y z intensity").split()
        sizes = list(map(int, header.get("SIZE", "4 4 4 4").split()))
        types = header.get("TYPE", "F F F F").split()
        counts = list(map(int, header.get("COUNT", "1 1 1 1").split()))
        
        # Build numpy dtype from header specifications
        type_map = {'I': np.int32, 'F': np.float32, 'U': np.uint8, 'L': np.uint64}
        dtype_parts = []
        for field, size, type_code, count in zip(fields, sizes, types, counts):
            np_type = type_map.get(type_code, np.float32)
            dtype_parts.append((field, np_type))
        dtype = np.dtype(dtype_parts)
        
        # Read binary data and convert to structured array
        num_points = int(header.get("POINTS", "0"))
        
        # Seek to start of data section and read
        f.seek(data_start)
        data = np.fromfile(f, dtype=dtype, count=num_points)
        
        return data, header

def write_pcd_binary(pcd_path: str, points: np.ndarray, header: dict):
    os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
    with open(pcd_path, "wb") as f:
        f.write(b"# .PCD v0.7\nVERSION 0.7\n")
        fields = list(points.dtype.names)
        f.write(f"FIELDS {' '.join(fields)}\n".encode())
        f.write(f"SIZE {' '.join(['4'] * len(fields))}\n".encode())
        f.write(f"TYPE {' '.join(['F'] * len(fields))}\n".encode())
        f.write(f"COUNT {' '.join(['1'] * len(fields))}\n".encode())
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n".encode())
        f.write(f"POINTS {len(points)}\nDATA binary\n".encode())
        points.tofile(f)


def create_pose_message(ts_ns_str: str, data: List[float]) -> Dict[str, Any]:
    ts = int(ts_ns_str)
    x, y, z, qx, qy, qz, qw = data
    r, p, yaw = euler_from_quaternion([qw, qx, qy, qz])
    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": ts // 10**9, "nsecs": ts % 10**9},
            "frame_id": "GCJ02",
        },
        "meta": {"timestamp_us": ts // 1000, "seq": 0, "type": 0},
        "position": {"available": 3, "position_local": {"x": x, "y": y, "z": z}},
        "orientation": {
            "available": 15,
            "euler_local": {"roll": r, "pitch": p, "yaw": yaw},
            "quaternion_local": {"w": qw, "x": qx, "y": qy, "z": qz},
        },
    }


def load_bev_pose(bev_pose_path: str) -> Optional[Dict[str, List[float]]]:
    """加载 BEV_pose.json 文件"""
    try:
        with open(bev_pose_path, "r") as f:
            data = json.load(f)
        if isinstance(data, dict):
            pose_data = data.get("pose", data)
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
                        ox, oy, oz, ow = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                        time_str = parts[8]
                        if "." in time_str:
                            sec_part, usec_part = time_str.split(".")
                            usec_part = usec_part.ljust(6, "0")[:6]
                            ts_ns = int(sec_part) * 1_000_000_000 + int(usec_part) * 1000
                        else:
                            ts_ns = int(float(time_str) * 1_000_000_000)
                        pose_dict[str(ts_ns)] = [x, y, z, ox, oy, oz, ow]
                    except: continue
    except Exception as e:
        print(f"    错误: 读取mapping pose文件失败: {e}")
        return None
    return pose_dict
