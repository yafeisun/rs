"""
BEV4D格式数据的IPM生成模块

严格参考速腾IPM实现：所有相机全投影到同一张BEV图，后投影覆盖先投影。
投影顺序决定重叠区域的优先级。

BEV范围: X[-20m, 30m], Y[-10m, 10m], 分辨率0.01m
"""

import os
import math
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


# BEV4D格式的鱼眼相机列表
# 投影顺序：后覆盖先（与速腾around一致：back->front->left->right）
# 速腾around: cam_around_back -> cam_around_front -> cam_around_left -> cam_around_right -> cam_front_left
# 对应BEV4D: camera_rear_fisheye -> camera_front_fisheye -> camera_left_fisheye -> camera_right_fisheye
FISHEYE_CAM_NAMES = [
    "camera_rear_fisheye",
    "camera_front_fisheye",
    "camera_left_fisheye",
    "camera_right_fisheye",
]


# BEV范围（与Robosense一致）
BEV_MIN_X, BEV_MAX_X = -20.0, 30.0
BEV_MIN_Y, BEV_MAX_Y = -10.0, 10.0
BEV_RESOLUTION = 0.01  # 米/像素（与Robosense一致，生成5000x2000图像）


class AntiDistortion:
    """鱼眼畸变校正（参考Robosense实现）"""

    def __init__(self, intrinsic, dist_coeff, shape):
        K, D = intrinsic, dist_coeff
        w, h = shape
        alpha = 0.3
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), self.new_K, (w, h), cv2.CV_16SC2
        )

    def anti(self, img):
        """校正畸变"""
        return cv2.remap(
            img,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )


class PointsTrans:
    """BEV与像素坐标转换（参考Robosense实现）"""

    def __init__(self, intrinsic, extrinsic, shape):
        self.intrin = intrinsic
        self.extrin = extrinsic
        self.extrin_inv = np.linalg.inv(extrinsic)
        self.intrin_inv = np.linalg.inv(intrinsic)
        self.shape = shape

    def pixel_to_bev(self, pix_points, Z=0):
        """像素坐标转BEV坐标"""
        if not pix_points.size:
            return np.array([]), pix_points

        hom_points = np.hstack((pix_points[:, :2], np.ones((pix_points.shape[0], 1))))
        pc = np.dot(self.intrin_inv, hom_points.T)  # pc/z

        z = np.array([Z]) - self.extrin[2, 3] / np.dot(self.extrin[2, :3], pc)
        pc *= z
        idxs = pc[2, :] > 0
        pc = pc[:, idxs]
        pix_points = pix_points[idxs]
        hom_pc = np.vstack((pc, np.ones((1, pc.shape[1]))))
        bev_points = np.dot(self.extrin, hom_pc)

        return bev_points[:2].T, pix_points

    def bev_to_pixel(self, points):
        """BEV坐标转像素坐标"""
        if not len(points):
            return np.array([])

        hom_points = np.hstack((points[:, :2], np.ones((points.shape[0], 2))))
        hom_points[:, 2] = 0  # nx4
        hom_points = hom_points.T  # 4xn
        points_pc = np.dot(self.extrin_inv, hom_points)
        pts = np.dot(self.intrin, points_pc[:3, :])

        pts = pts[:, pts[2, :] > 0]
        pts = pts[:, pts[0, :] > 0]
        pts = pts[:, pts[1, :] > 0]
        pts[0, :] = pts[0, :] / pts[2, :]
        pts[1, :] = pts[1, :] / pts[2, :]
        pts = pts[:, pts[0, :] < self.shape[0]]
        pts = pts[:, pts[1, :] < self.shape[1]]
        return pts.T[:, :2]

    def bev_to_pixel_mask(self, points):
        """BEV坐标转像素坐标（带mask）"""
        if not len(points):
            return np.array([]), np.array([])

        hom_points = np.hstack((points[:, :2], np.ones((points.shape[0], 2))))
        hom_points[:, 2] = 0
        hom_points = hom_points.T
        points_pc = np.dot(self.extrin_inv, hom_points)
        pts = np.dot(self.intrin, points_pc[:3, :])

        pts[0, :] = pts[0, :] / pts[2, :]
        pts[1, :] = pts[1, :] / pts[2, :]

        mask = np.ones(pts.shape[1], dtype=bool)
        mask = np.logical_and(mask, pts[2, :] > 0)
        mask = np.logical_and(mask, pts[1, :] > 0)
        mask = np.logical_and(mask, pts[0, :] > 0)
        mask = np.logical_and(mask, pts[0, :] < self.shape[0] - 1)
        mask = np.logical_and(mask, pts[1, :] < self.shape[1] - 1)
        uv = pts[:2, :]
        return uv, mask


class BevProjector:
    """BEV投影器（全投影模式，后覆盖先）"""

    def __init__(self):
        self.min_x, self.max_x = BEV_MIN_X, BEV_MAX_X
        self.min_y, self.max_y = BEV_MIN_Y, BEV_MAX_Y
        self.res = BEV_RESOLUTION
        # 速腾实现：pixel_rows = X方向，pixel_cols = Y方向
        # X方向: [-20, 30] = 50m, Y方向: [-10, 10] = 20m
        self.pixel_rows = int(math.ceil((self.max_x - self.min_x) / self.res))  # X方向 = 50m/0.01 = 5000
        self.pixel_cols = int(math.ceil((self.max_y - self.min_y) / self.res))  # Y方向 = 20m/0.01 = 2000
        self.up_sample = 2

        self.cam_param = {}
        self.distortions = {}
        self.trans_dict = {}

    def init(self, calib_dict):
        """初始化相机参数和投影转换"""
        self.cam_param = calib_dict

        for name in calib_dict:
            dist = AntiDistortion(
                self.cam_param[name]["intrinsic"],
                self.cam_param[name]["distCoeff"],
                self.cam_param[name]["image_shape"],
            )
            self.cam_param[name]["intrinsic"] = dist.new_K  # update intrinsic

            intrinsic, extrinsic, image_shape, dist_coeff = (
                self.cam_param[name]["intrinsic"],
                self.cam_param[name]["extrinsic"],
                self.cam_param[name]["image_shape"],
                self.cam_param[name]["distCoeff"],
            )
            self.distortions[name] = dist
            trans = PointsTrans(intrinsic, extrinsic, image_shape)
            self.trans_dict[name] = trans

    def run(self, img_list, cam_name_list):
        """运行IPM投影，所有相机全投影，后覆盖先（使用反向投影）"""
        bev_img = np.zeros((self.pixel_rows, self.pixel_cols, 4), dtype=np.uint8)

        # 生成BEV网格点（与Robosense一致）
        x = np.linspace(self.min_x, self.max_x, self.pixel_rows)
        y = np.linspace(self.min_y, self.max_y, self.pixel_cols)
        X, Y = np.meshgrid(x, y, indexing='ij')
        grid_points = np.stack([X.ravel(), Y.ravel()], axis=1)

        for img_path, cam_name in zip(img_list, cam_name_list):
            rgb_img = cv2.imread(img_path)
            if rgb_img is None:
                continue

            # 去畸变
            rgb_img = self.distortions[cam_name].anti(rgb_img)

            # 上采样
            if self.up_sample != 1:
                new_h = rgb_img.shape[0] * self.up_sample
                new_w = rgb_img.shape[1] * self.up_sample
                rgb_img = cv2.resize(rgb_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            # BEV点反投影到像素坐标
            pixel, mask = self.trans_dict[cam_name].bev_to_pixel_mask(grid_points)
            pixel *= self.up_sample  # 考虑上采样
            valid_pixel = pixel[:, mask]
            uv = valid_pixel.round().astype(np.int32).T

            # 从上采样图像中取色
            pt_color = rgb_img[uv[:, 1], uv[:, 0]]
            pt_color = np.hstack((pt_color, np.full((pt_color.shape[0], 1), 255)))

            # 填充到BEV图像
            mask_2d = mask.reshape([self.pixel_rows, self.pixel_cols])
            bev_img[mask_2d] = pt_color

        # 旋转90度逆时针（与速腾一致）
        # 原始: (5000行, 2000列) = (高, 宽)
        # 逆时针旋转后: (2000行, 5000列) = 高2000 x 宽5000
        bev_img = cv2.rotate(bev_img, cv2.ROTATE_90_COUNTERCLOCKWISE)

        return bev_img


def load_cam_param(yaml_path: str):
    """Load camera parameters from BEV4D calibration yaml (支持polyn和ocam两种格式)"""
    with open(yaml_path, "r") as f:
        content = f.read()
    if "%YAML" in content:
        content = content.split("---", 1)[-1]
    data = yaml.safe_load(content)

    camera_model = data.get("camera_model", "polyn")

    if camera_model == "ocam":
        # OCAM模型：从affine_parameters提取内参
        affine = data.get("affine_parameters", {})
        cx = float(affine.get("cx", 960))
        cy = float(affine.get("cy", 768))
        ac = float(affine.get("ac", 1.0))

        # 使用ac作为fx和fy的近似值
        fx = fy = ac * 500  # 近似焦距

        # OCAM的畸变系数设为0（因为OCAM用多项式模型，不是传统畸变）
        dist_coeff = np.array([0, 0, 0, 0], dtype=np.float64)
    else:
        # polyn模型：使用fx, fy, cx, cy
        fx = float(data["fx"])
        fy = float(data["fy"])
        cx = float(data["cx"])
        cy = float(data["cy"])

        dist_coeff = np.array(
            [
                data.get("kc2", 0),
                data.get("kc3", 0),
                data.get("kc4", 0),
                data.get("kc5", 0),
            ],
            dtype=np.float64,
        )

    intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    r_s2b = np.array(data["r_s2b"], dtype=np.float64)
    t_s2b = np.array(data["t_s2b"], dtype=np.float64)

    # T_c2b: camera -> body
    R_c2b = R.from_rotvec(r_s2b).as_matrix()
    T_c2b = np.eye(4)
    T_c2b[:3, :3] = R_c2b
    T_c2b[:3, 3] = t_s2b

    return {
        "intrinsic": intrinsic,
        "distCoeff": dist_coeff,
        "image_shape": (int(data["width"]), int(data["height"])),  # [w, h] 与速腾ImageSize一致
        "extrinsic": T_c2b,
    }


def _find_closest_image(cam_dir: str, ref_ts: int, suffix: str = ".jpeg"):
    """Find the image file closest to ref_ts in cam_dir"""
    files = [f for f in os.listdir(cam_dir) if f.endswith(suffix)]
    if not files:
        return None
    best = None
    best_diff = None
    for f in files:
        try:
            ts = int(os.path.splitext(f)[0])
        except ValueError:
            continue
        diff = abs(ts - ref_ts)
        if best_diff is None or diff < best_diff:
            best_diff = diff
            best = os.path.join(cam_dir, f)
    return best


def generate_ipm(target_dir: str, max_frames: int = None) -> None:
    """
    Generate IPM images for BEV4D format single segment.
    All cameras project to the same BEV image, later cameras overwrite earlier ones.
    Projection order: front -> rear -> left -> right

    Args:
        target_dir: BEV4D format segment directory
        max_frames: Max frames to generate (None means all, for debugging)
    """
    calib_dir = os.path.join(target_dir, "calibration", "camera")
    if not os.path.exists(calib_dir):
        print(f"  [IPM] Skipped: calibration dir not found: {calib_dir}")
        return

    # Load camera calibration parameters
    calib_dict = {}
    for cam in FISHEYE_CAM_NAMES:
        yaml_path = os.path.join(calib_dir, f"{cam}.yaml")
        if not os.path.exists(yaml_path):
            print(f"  [IPM] Skipped: missing calibration for {cam}")
            return
        calib_dict[cam] = load_cam_param(yaml_path)

    # Create projector
    projector = BevProjector()
    projector.init(calib_dict)

    # Use front fisheye as reference, iterate all frames
    ref_cam = "camera_front_fisheye"
    ref_cam_dir = os.path.join(target_dir, "sensor_data", "camera", ref_cam)
    if not os.path.exists(ref_cam_dir):
        print(f"  [IPM] Skipped: camera dir not found: {ref_cam_dir}")
        return

    ref_images = sorted([f for f in os.listdir(ref_cam_dir) if f.endswith(".jpeg")])
    if not ref_images:
        print(f"  [IPM] Skipped: no images in {ref_cam_dir}")
        return

    output_dir = os.path.join(target_dir, "node_output", "ipm")
    os.makedirs(output_dir, exist_ok=True)

    print(f"  [IPM] Processing {len(ref_images)} frames -> {output_dir}")
    print(
        f"  [IPM] BEV range: X[{BEV_MIN_X}, {BEV_MAX_X}]m, Y[{BEV_MIN_Y}, {BEV_MAX_Y}]m"
    )
    print(f"  [IPM] Resolution: {BEV_RESOLUTION}m/pixel")
    print(f"  [IPM] Strategy: Full projection (later overwrites earlier)")

    # Limit generation to avoid timeout
    if max_frames is None:
        process_frames = len(ref_images)
    else:
        process_frames = min(len(ref_images), max_frames)

    if process_frames != len(ref_images):
        print(f"  [IPM] Limiting to {process_frames} frames (total: {len(ref_images)})")

    for ref_img_file in ref_images[:process_frames]:
        ref_ts = int(os.path.splitext(ref_img_file)[0])
        ref_img_path = os.path.join(ref_cam_dir, ref_img_file)

        # Find closest frame for each camera
        cam_img_paths = {ref_cam: ref_img_path}
        skip = False
        for cam in FISHEYE_CAM_NAMES:
            if cam == ref_cam:
                continue
            cam_dir = os.path.join(target_dir, "sensor_data", "camera", cam)
            if not os.path.exists(cam_dir):
                print(f"  [IPM] Warning: camera dir missing: {cam_dir}")
                skip = True
                break
            img_path = _find_closest_image(cam_dir, ref_ts)
            if img_path is None:
                print(f"  [IPM] Warning: no image for {cam} near ts {ref_ts}")
                skip = True
                break
            cam_img_paths[cam] = img_path

        if skip:
            continue

        # Project all cameras (later overwrites earlier)
        img_list = [cam_img_paths[cam] for cam in FISHEYE_CAM_NAMES]
        cam_name_list = FISHEYE_CAM_NAMES
        bev_img = projector.run(img_list, cam_name_list)

        # Save
        out_path = os.path.join(output_dir, f"ipm_{ref_ts}.jpeg")
        cv2.imwrite(out_path, bev_img)

    print(f"  [IPM] Done: {process_frames} frames saved to {output_dir}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", help="BEV4D format segment directory")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Max frames to generate (default: all)",
    )
    args = parser.parse_args()

    generate_ipm(args.target_dir, args.max_frames)
