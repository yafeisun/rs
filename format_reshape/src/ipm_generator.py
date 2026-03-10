"""
BEV4D格式数据的IPM生成模块

参考速腾IPM实现：所有相机全投影到同一张BEV图，后投影覆盖先投影。
投影顺序决定重叠区域的优先级。

BEV范围: X[-20m, 30m], Y[-10m, 10m], 分辨率0.05m
"""

import os
import math
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


# BEV4D格式的鱼眼相机列表
FISHEYE_CAM_NAMES = [
    "camera_front_fisheye",
    "camera_rear_fisheye",
    "camera_left_fisheye",
    "camera_right_fisheye",
]


# BEV范围（与Robosense一致）
BEV_MIN_X, BEV_MAX_X = -20.0, 30.0
BEV_MIN_Y, BEV_MAX_Y = -10.0, 10.0
BEV_RESOLUTION = 0.05  # 米/像素


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
        self.width = int(math.ceil((self.max_x - self.min_x) / self.res)) + 1
        self.height = int(math.ceil((self.max_y - self.min_y) / self.res)) + 1

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
        """运行IPM投影，所有相机全投影，后覆盖先"""
        bev_img = np.zeros((self.height, self.width, 4), dtype=np.uint8)

        for img_path, cam_name in zip(img_list, cam_name_list):
            rgb_img = cv2.imread(img_path)
            if rgb_img is None:
                continue

            rgb_img = self.distortions[cam_name].anti(rgb_img)

            y_coors, x_coors = np.mgrid[0 : rgb_img.shape[0], 0 : rgb_img.shape[1]]
            coords = np.dstack((x_coors, y_coors)).reshape(-1, 2)
            bev_coords, coords = self.trans_dict[cam_name].pixel_to_bev(coords)
            bev_img = self.trans_bev_to_img(bev_coords, coords, rgb_img, bev_img)

        return bev_img

    def bev_to_pixel(self, x, y, dtype=np.int32):
        """BEV coordinate转pixel coordinate"""
        return np.array(
            [(x - self.min_x) / self.res, (self.max_y - y) / self.res], dtype
        )

    def pixel_to_bev(self, x, y, dtype=np.int32):
        """Pixel coordinate to BEV coordinate"""
        return np.array([self.res * x + self.min_x, self.max_y - y * self.res], dtype)

    def get_pix_rgb(self, img, x, y):
        """Bilinear interpolation to get pixel RGB value"""
        # Get four neighboring integer coordinates
        x0, y0 = int(np.floor(x)), int(np.floor(y))
        x1, y1 = min(x0 + 1, img.shape[1] - 1), min(y0 + 1, img.shape[0] - 1)

        # Calculate weights
        dx, dy = x - x0, y - y0

        # Boundary check
        x0 = max(0, x0)
        y0 = max(0, y0)

        # Get RGB values of four neighboring points
        top_left = img[y0, x0]
        top_right = img[y0, x1]
        bottom_left = img[y1, x0]
        bottom_right = img[y1, x1]

        # Bilinear interpolation calculation
        interpolated = (
            (1 - dx) * (1 - dy) * top_left
            + dx * (1 - dy) * top_right
            + (1 - dx) * dy * bottom_left
            + dx * dy * bottom_right
        )

        return np.clip(interpolated, 0, 255).astype(int)

    def trans_bev_to_img(self, bev_coords, img_coords, rgb_img, bev_img=None):
        """Transform BEV coordinates back to image RGB channels"""
        if bev_img is None:
            bev_img = np.zeros((self.height, self.width, 4), dtype=np.uint8)

        # Boundary condition judgment
        cond = (
            (bev_coords[:, 0] >= self.min_x)
            & (bev_coords[:, 0] <= self.max_x)
            & (bev_coords[:, 1] >= self.min_y)
            & (bev_coords[:, 1] <= self.max_y)
        )
        bev_coords = bev_coords[cond]
        img_coords = img_coords[cond]

        # Convert BEV coordinates to pixel coordinates
        pixel_coords = bev_coords.copy()
        pixel_coords[:, 0] = (bev_coords[:, 0] - self.min_x) / self.res
        pixel_coords[:, 1] = (self.max_y - bev_coords[:, 1]) / self.res

        # Remove duplicate coordinate points
        pixel_coords, idxs = np.unique(pixel_coords.round(), return_index=True, axis=0)
        pixel_coords = pixel_coords.astype(int)
        img_coords = img_coords[idxs]

        # Update BEV image RGB values
        for i in range(img_coords.shape[0]):
            bev_img[pixel_coords[i][1], pixel_coords[i][0], :3] = rgb_img[
                img_coords[i][1], img_coords[i][0]
            ]
            bev_img[pixel_coords[i][1], pixel_coords[i][0], 3] = 255

        bev_img = np.array(bev_img, dtype=np.uint8)
        return bev_img


def load_cam_param(yaml_path: str):
    """Load camera parameters from BEV4D calibration yaml"""
    with open(yaml_path, "r") as f:
        content = f.read()
    if "%YAML" in content:
        content = content.split("---", 1)[-1]
    data = yaml.safe_load(content)

    # BEV4D format: use fx, fy, cx, cy
    fx = float(data["fx"])
    fy = float(data["fy"])
    cx = float(data["cx"])
    cy = float(data["cy"])

    intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeff = np.array(
        [
            data.get("kc2", 0),
            data.get("kc3", 0),
            data.get("kc4", 0),
            data.get("kc5", 0),
        ],
        dtype=np.float64,
    )

    r_s2b = np.array(data["r_s2b"], dtype=np.float64)
    t_s2b = np.array(data["t_s2b"], dtype=np.float64)

    # T_c2b: camera -> body
    R_c2b = R.from_rotvec(r_s2b).as_matrix()
    T_c2b = np.eye(4)
    T_c2b[:3, :3] = R_c2b
    T_c2b[:3, 3] = t_s2b

    # T_b2c: body -> camera
    T_b2c = np.linalg.inv(T_c2b)

    return {
        "intrinsic": intrinsic,
        "distCoeff": dist_coeff,
        "image_shape": (int(data["height"]), int(data["width"])),
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
