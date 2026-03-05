import glob
import logging
import os
import json
from copy import deepcopy

import cv2
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial.transform import Rotation
from argparse import ArgumentParser
from datetime import datetime
# from common.env import get_arg  # Not used, removed to avoid dependency
import concurrent.futures

logger = logging.getLogger(__name__)

camera_names = [
    "camera_front_far",
    "camera_front_wide",
    "camera_left_front",
    "camera_left_rear",
    "camera_rear_mid",
    "camera_right_front",
    "camera_right_rear",
    "camera_front_fisheye",
    "camera_left_fisheye",
    "camera_right_fisheye",
    "camera_rear_fisheye",
]

lidar_names = [
    "lidar",
    "lidar_concat",
    "lidar_fl",
    "lidar_fr",
    "lidar_front_up",
    "lidar_rear_up",
]
# 相机行扫描时间，单位ms
camera_row_scan_time = {
    "camera_front_far": 0.01389,
    "camera_front_wide": 0.01389,
    "camera_left_front": 0.01208,
    "camera_left_rear": 0.01208,
    "camera_rear_mid": 0.01483,
    "camera_right_front": 0.01208,
    "camera_right_rear": 0.01208,
    "camera_front_fisheye": 0.01904,
    "camera_left_fisheye": 0.01904,
    "camera_right_fisheye": 0.01904,
    "camera_rear_fisheye": 0.01904,
}

camera_model_type = dict(
    radial_tangential ="radial-tangential",
    polyn =  "polyn",
    ocam = "ocam",
)

def get_lidar_info(calib_root_path):
    calib_info = {}
    for calib_file_name in os.listdir(calib_root_path):
        # 只需要解析相机标定文件
        if "lidar" not in calib_file_name:
            continue
        sensor_name = calib_file_name.split(".")[0]
        if sensor_name not in lidar_names:
            continue
        yaml_path = os.path.join(calib_root_path, calib_file_name)
        info = {}
        with open(yaml_path, "r") as file:
            data = yaml.load("#" + file.read(), Loader=yaml.FullLoader)
            r_s2b = data["r_s2b"]
            t_s2b = data["t_s2b"]

            R = Rotation.from_rotvec(r_s2b).as_matrix()
            lidar_ex = np.concatenate([R, np.array(t_s2b).reshape(3, 1)], axis=1)
            comp = np.array([[0.0, 0.0, 0.0, 1.0]])
            lidar_ex = np.concatenate([lidar_ex, comp], axis=0)
            info["extrinsic"] = lidar_ex
        calib_info[sensor_name] = info
    return calib_info

def get_calib_info(calib_root_path):
    calib_info = {}
    for calib_file_name in os.listdir(calib_root_path):
        # 只需要解析相机标定文件
        if "camera" not in calib_file_name:
            continue
        sensor_name = calib_file_name.split(".")[0]
        if sensor_name not in camera_names:
            continue
        yaml_path = os.path.join(calib_root_path, calib_file_name)
        info = {}
        with open(yaml_path, "r") as file:
            data = yaml.load("#" + file.read(), Loader=yaml.FullLoader)
            r_s2b = data["r_s2b"]
            t_s2b = data["t_s2b"]

            R = Rotation.from_rotvec(r_s2b).as_matrix()
            camera_ex = np.concatenate([R, np.array(t_s2b).reshape(3, 1)], axis=1)
            comp = np.array([[0.0, 0.0, 0.0, 1.0]])
            camera_ex = np.concatenate([camera_ex, comp], axis=0)
            camera_ex_inv = np.linalg.inv(camera_ex)
            # 激光点云是车体坐标系，保存车体到相机的转换
            info["extrinsic"] = camera_ex_inv
            info["camera_model"] = data["camera_model"]
            info["intrinsic"] = []
            info["distortion"] = []


            
            if data["camera_model"] == camera_model_type["polyn"]:
                K = np.array(
                    [[data["fx"], 0, data["cx"]], [0, data["fy"], data["cy"]], [0, 0, 1]]
                )
                info["intrinsic"] = K

                info["distortion"].append(data["kc2"])
                info["distortion"].append(data["kc3"])
                info["distortion"].append(data["kc4"])
                info["distortion"].append(data["kc5"])
                info["distortion"] = np.array(info["distortion"])
            elif data["camera_model"] == camera_model_type["radial_tangential"]:
                K = np.array(
                    [[data["fx"], 0, data["cx"]], [0, data["fy"], data["cy"]], [0, 0, 1]]
                )
                info["intrinsic"] = K
                info["distortion"].append(data["k1"])
                info["distortion"].append(data["k2"])
                info["distortion"].append(data["p1"])
                info["distortion"].append(data["p2"])
                info["distortion"].append(data["k3"])
                info["distortion"].append(data["k4"])
                info["distortion"].append(data["k5"])
                info["distortion"].append(data["k6"])
                info["distortion"] = np.array(info["distortion"])
            elif data["camera_model"] == camera_model_type["ocam"]:
                ac = data["affine_parameters"]["ac"]
                ad = data["affine_parameters"]["ad"]
                ae = data["affine_parameters"]["ae"]
                cx = data["affine_parameters"]["cx"]
                cy = data["affine_parameters"]["cy"]
                info["affine_parameters"] = np.array([ac, ad, ae, cx, cy])
                info["poly_parameters"] = []
                info["poly_parameters"].append(data["poly_parameters"]["p0"])
                info["poly_parameters"].append(data["poly_parameters"]["p1"])
                info["poly_parameters"].append(data["poly_parameters"]["p2"])
                info["poly_parameters"].append(data["poly_parameters"]["p3"])
                info["poly_parameters"].append(data["poly_parameters"]["p4"])
                info["poly_parameters"] = np.array(info["poly_parameters"])
                info["inv_poly_parameters"] = []
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p0"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p1"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p2"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p3"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p4"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p5"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p6"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p7"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p8"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p9"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p10"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p11"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p12"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p13"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p14"])
                info["inv_poly_parameters"].append(data['inv_poly_parameters']["p15"])
                info["inv_poly_parameters"] = np.array(info["inv_poly_parameters"])

        calib_info[sensor_name] = info
    return calib_info

class PointCloudVisual:
    def __init__(self, bagpath, transfrom=False, processing_sensors="all"):
        calib_root_path = os.path.join(bagpath, "calibration/camera")
        self.processing_sensors = processing_sensors
        self.processing_sensors_config = {
            "all": {"camera_num": 11, "result_file_name":"result_full11v.json"},
            "camera11v_lidar": {"camera_num": 11, "result_file_name":"result_camera11v_lidar_full11v.json"},
            "camera7v_lidar": {"camera_num": 7, "result_file_name":"result_camera7v_lidar_full7v.json"},
        }
        self.result_file_name = self.processing_sensors_config[processing_sensors]["result_file_name"]
        result_json = os.path.join(
            bagpath, "node_output/peral-dataproc", self.result_file_name
        )
        self.result_json_object = {}
        if os.path.exists(result_json):
            with open(result_json) as file:
                self.result_json_object = json.load(file)
        self.sensor_data_path = os.path.join(bagpath, "sensor_data")

        # 标定参数解析
        self.calib_info = get_calib_info(calib_root_path)
        self.lidar_calib_info = get_lidar_info(os.path.join(bagpath, "calibration/lidar"))
        # 错误激光标定文件
        if os.path.exists(os.path.join(bagpath, "calibration_base/lidar")):
            self.error_lidar_calib_info = get_lidar_info(os.path.join(bagpath, "calibration_base/lidar"))
            self.change_lidar_rt = self.lidar_calib_info["lidar"]["extrinsic"] @ np.linalg.inv(self.error_lidar_calib_info["lidar"]["extrinsic"])
        else:
            self.error_lidar_calib_info = {}
            self.change_lidar_rt = np.eye(4, 4)
        
        self.transfrom = transfrom

    def car_to_cam(self, cloud, vtc_mat):
        """
        description: convert Lidar 3D coordinates to 3D camera coordinates .
        input: (PointsNum,3)
        output: (PointsNum,3)
        """
        mat = np.ones(shape=(cloud.shape[0], 4), dtype=np.float32)
        mat[:, 0:3] = cloud[:, 0:3]
        mat = np.mat(mat)
        normal = np.mat(vtc_mat)
        transformed_mat = normal * mat.T
        T = np.array(transformed_mat.T, dtype=np.float32)
        return T

    def colormap_z(self, pts_3d_cam, color):
        index_0 = np.where(pts_3d_cam[:, 2] < 10)

        color[index_0] = (0, 0, 255)
        index_1 = np.where((pts_3d_cam[:, 2] >= 10) & (pts_3d_cam[:, 2] < 20))
        color[index_1] = (255, 0, 0)
        index_2 = np.where((pts_3d_cam[:, 2] >= 20) & (pts_3d_cam[:, 2] < 30))
        color[index_2] = (0, 0, 255)
        index_3 = np.where((pts_3d_cam[:, 2] >= 30) & (pts_3d_cam[:, 2] < 40))
        color[index_3] = (255, 0, 0)
        index_4 = np.where((pts_3d_cam[:, 2] >= 40) & (pts_3d_cam[:, 2] < 50))
        color[index_4] = (0, 0, 255)
        index_5 = np.where((pts_3d_cam[:, 2] >= 50) & (pts_3d_cam[:, 2] < 60))
        color[index_5] = (255, 0, 0)
        index_6 = np.where((pts_3d_cam[:, 2] >= 60) & (pts_3d_cam[:, 2] < 70))
        color[index_6] = (0, 0, 255)
        index_7 = np.where((pts_3d_cam[:, 2] >= 70) & (pts_3d_cam[:, 2] < 80))
        color[index_7] = (255, 0, 0)
        index_8 = np.where((pts_3d_cam[:, 2] >= 80) & (pts_3d_cam[:, 2] < 90))
        color[index_8] = (0, 0, 255)
        index_9 = np.where(pts_3d_cam[:, 2] >= 90)
        color[index_9] = (255, 0, 0)
        return color

    def colormap_intensity(self, intensity, color_map):
        index_0 = np.where(intensity < 63)
        g_0 = 254 - 4 * intensity[index_0]
        b_0 = np.ones_like(g_0).astype(np.uint8) * 255
        r_0 = np.zeros_like(g_0).astype(np.uint8)
        g_0 = np.reshape(g_0, (-1, 1))
        b_0 = np.reshape(b_0, (-1, 1))
        r_0 = np.reshape(r_0, (-1, 1))
        intensity_0_bgr = np.concatenate((b_0, g_0, r_0), axis=1)
        color_map[index_0[0]] = intensity_0_bgr

        index_1 = np.where((intensity >= 63) & (intensity < 127))
        b_1 = 510 - 4 * intensity[index_1]
        g_1 = 4 * intensity[index_1] - 254
        r_1 = np.zeros_like(g_1).astype(np.uint8)
        g_1 = np.reshape(g_1, (-1, 1))
        b_1 = np.reshape(b_1, (-1, 1))
        r_1 = np.reshape(r_1, (-1, 1))
        intensity_1_bgr = np.concatenate((b_1, g_1, r_1), axis=1)
        color_map[index_1[0]] = intensity_1_bgr

        index_2 = np.where((intensity >= 127) & (intensity < 191))
        r_2 = 4 * intensity[index_2] - 254
        g_2 = np.ones_like(r_2).astype(np.uint8) * 255
        b_2 = np.zeros_like(r_2).astype(np.uint8)
        g_2 = np.reshape(g_2, (-1, 1))
        b_2 = np.reshape(b_2, (-1, 1))
        r_2 = np.reshape(r_2, (-1, 1))
        intensity_2_bgr = np.concatenate((b_2, g_2, r_2), axis=1)
        color_map[index_2[0]] = intensity_2_bgr  # 紫色

        index_3 = np.where(intensity >= 191)
        g_3 = 1022 - 4 * intensity[index_3]
        r_3 = np.ones_like(g_3).astype(np.uint8) * 255
        b_3 = np.zeros_like(g_3).astype(np.uint8)
        g_3 = np.reshape(g_3, (-1, 1))
        b_3 = np.reshape(b_3, (-1, 1))
        r_3 = np.reshape(r_3, (-1, 1))
        intensity_3_bgr = np.concatenate((b_3, g_3, r_3), axis=1)
        color_map[index_3[0]] = intensity_3_bgr  # 蓝色
        return color_map
    
    def calculate_velocity(self, quaternion_local, velocity_local):
        # 将字典中的速度值转换为 numpy 数组
        vel_local = np.array(velocity_local)
        # 将四元数转换为 Rotation 对象
        rot = Rotation.from_quat(quaternion_local)
        # transform_matrix = np.eye(4)
        # transform_matrix[:3, :3] = rot.as_matrix()
        # transform_matrix[:3, 3] = vel_local
        # print("变换矩阵:", transform_matrix)
        # 通过四元数的逆旋转将局部速度转换为车体坐标系下的速度
        car_velocity = np.linalg.inv(rot.as_matrix()) @ vel_local
        logging.info(f"lidar时刻车体坐标系下的速度: {car_velocity}")
        return car_velocity


    def get_replacement(self, row, row_time, height, car_velocity, extrinsic=None):
        time_elapsed = ((height-row) * row_time) / 1000
        car_velocity_homogeneous = np.append(car_velocity, 0)  # 转换为齐次坐标 [vx, vy, vz, 0]
        
        # 直接与完整的4x4变换矩阵相乘
        image_velocity_homogeneous = extrinsic @ car_velocity_homogeneous  # 一次性完成旋转和平移
        image_velocity = image_velocity_homogeneous[:3]  # 提取前三个分量作为结果
        logging.info(f"image_velocity shape: {image_velocity.shape}")

        # 仅打印关键信息和小样本数据，减少内存使用
        logging.info(f"车辆坐标系下的速度: {car_velocity}")
        logging.info(f"图像坐标系下的最终速度: {image_velocity}")
        logging.info(f"row_time is {row_time}")


        if row.size > 10:  # 只在数据量较小时打印详细信息
            logging.info(f"处理大数据集: {row.size}个点")
            logging.info(f"time_elapsed shape: {time_elapsed.shape}, image_velocity shape: {image_velocity.shape}")
        
        # 计算图像坐标系下的位移
        time_elapsed_expanded = time_elapsed[:, np.newaxis]
        image_replacement = time_elapsed_expanded * image_velocity
        
        # 对于大型位移数组，只打印形状信息而不是完整数组
        if image_replacement.size > 10:
            logging.info(f"图像坐标系下的最终位移形状: {image_replacement.shape}")
        else:
            logging.info(f"图像坐标系下的最终位移: {image_replacement}")
        
        return image_replacement
        # 计算时间间隔，row可能是一个数组
    def get_camera_bottom2row_transform(self, row, row_time, height, extrinsic, car_camera_bottom2local, angular_velocity_local, velocity_local):
        # ms to s
        time_elapsed = -((height-row) * row_time) / 1000
        
        # 确保angular_velocity_local和velocity_local是numpy数组
        angular_velocity_local = np.array(angular_velocity_local)
        velocity_local = np.array(velocity_local)
        
        # 获取初始位姿
        bottom_orientation = car_camera_bottom2local[:3, :3]
        bottom_position = car_camera_bottom2local[:3, 3]
        
        num_points = len(time_elapsed)
        T_camera_bottom_row = np.ones((num_points, 4, 4))
        # logging.info(f"time_elapsed shape is {time_elapsed.shape}")
        
        # 批量计算所有点的旋转矩阵：先构造角速度×时间的向量，然后将旋转向量转化成旋转矩阵
        rot_vec_batch = angular_velocity_local * time_elapsed[:, np.newaxis]  # (N,3)
        # logging.info(f"angular_velocity_local shape is {angular_velocity_local.shape}")
        # logging.info(f"time_elapsed shape is {time_elapsed.shape}")
        # logging.info(f"rot_vec_batch shape is {rot_vec_batch.shape}")
        # logging.info(time_elapsed[:, np.newaxis].shape)
        rot_matrix_batch = Rotation.from_rotvec(rot_vec_batch).as_matrix()  # (N,3,3)

        # 批量计算新朝向：旧朝向左乘各点旋转矩阵
        row_orientation_batch = rot_matrix_batch @ bottom_orientation  # (N,3,3)
        # logging.info(f"rot_matrix_batch shape is {rot_matrix_batch.shape}")
        # logging.info(f"bottom_orientation shape is {bottom_orientation.shape}")
        # logging.info(f"row_orientation_batch shape is {row_orientation_batch.shape}")

        # 批量计算新位置：旧位置 + 速度×时间
        row_pos_batch = bottom_position + velocity_local * time_elapsed[:, np.newaxis]  # (N,3)

        # 一次性构造所有点的 car_camera_row2local 矩阵 (N,4,4)
        car_camera_row2local_batch = np.tile(np.eye(4), (num_points, 1, 1))
        car_camera_row2local_batch[:, :3, :3] = row_orientation_batch
        car_camera_row2local_batch[:, :3, 3] = row_pos_batch

        # 坐标系变换逻辑，见：https://lotuscars.feishu.cn/wiki/YEgCwZIJ0iU6Nlk4yxfcegZJntb
        T_camera_bottom_row = extrinsic @ np.linalg.inv(car_camera_row2local_batch) @ car_camera_bottom2local @ np.linalg.inv(extrinsic)  # (N,4,4)
        # logging.info(f"---T_camera_bottom_row shape: {T_camera_bottom_row.shape}")
        return T_camera_bottom_row

    def tranform_point(self, points, T_camera_bottom_row):
        # 确保points是numpy数组
        points = np.array(points)
        T_camera_bottom_row = np.array(T_camera_bottom_row)
        
        points_homogeneous = np.ones((points.shape[0], 1, 4))
        points_homogeneous[:, 0, :3] = points[:, 0, :]
                
        # 转换为齐次坐标 - 现在两个数组都是2维的，可以正确hstack
        logging.info(f"points shape: {points.shape}")
        # logging.info(f"--------T_camera_bottom_row shape: {T_camera_bottom_row.shape}")
        # logging.info(f"--------points_homogeneous shape: {points_homogeneous.shape}")
        # points_homogeneous_transposed = points_homogeneous.reshape(-1, 4, 1)
        points_homogeneous_transposed = np.transpose(points_homogeneous, (0, 2, 1))
        transformed_homog = T_camera_bottom_row @ points_homogeneous_transposed
        points_transformed = transformed_homog.reshape(-1, 4)[:, :3]
        # logging.info(f"--------points_transformed shape: {points_transformed.shape}")
        points_transformed = points_transformed.reshape(-1, 1, 3)
        # logging.info(f"--------points_transformed shape: {points_transformed.shape}")

        #必须要转换，否则投影图像会有个大十字
        points_transformed = np.array(points_transformed, dtype=np.float32)
        return points_transformed

    def sample_process(self, pcd_path, images, egopose_path):
        pcd = o3d.t.io.read_point_cloud(pcd_path)
        points = pcd.point["positions"].numpy()
        intensity = pcd.point["intensity"].numpy()
        intensity = np.uint8(intensity)
        intensity_map = cv2.applyColorMap(intensity, cv2.COLORMAP_RAINBOW)
        color = intensity_map[:, 0, :]

        colormap_intensity = self.colormap_intensity(intensity, color)
        camera_img = {}
        
        lidar_egopose_file = open(egopose_path, "r")
        lidar_egopose = json.load(lidar_egopose_file)
        r = [
            lidar_egopose["orientation"]["quaternion_local"]["x"],
            lidar_egopose["orientation"]["quaternion_local"]["y"],
            lidar_egopose["orientation"]["quaternion_local"]["z"],
            lidar_egopose["orientation"]["quaternion_local"]["w"]
        ]
        t = [
            lidar_egopose["position"]["position_local"]["x"],
            lidar_egopose["position"]["position_local"]["y"],
            lidar_egopose["position"]["position_local"]["z"]
        ]
        lidar_r = Rotation.from_quat(r).as_matrix()
        lidar_rt = np.eye(4, 4)
        lidar_rt[:3, :3] = lidar_r
        lidar_rt[:3, 3] = t
        
        for camera_name, image_path in images.items():
            logging.info(f"pcd_path:{pcd_path}, camera_name: {camera_name}, image_path: {image_path}")
            image = cv2.imread(image_path)  # BGR
            image = np.asarray(image)
            H, W, _ = image.shape
            extrinsic = self.calib_info[camera_name]["extrinsic"]
            intrinsic = self.calib_info[camera_name]["intrinsic"]
            distortion = self.calib_info[camera_name]["distortion"]
            camera_model = self.calib_info[camera_name]["camera_model"]
            affine_parameters = self.calib_info.get(camera_name, {}).get("affine_parameters", None)
            inv_poly_parameters = self.calib_info.get(camera_name, {}).get("inv_poly_parameters", None)

            if self.transfrom:
                image_name = os.path.basename(image_path)
                img_egopose_path = os.path.join(self.sensor_data_path, f"egopose_opt/{camera_name}/{image_name[:-5]}.json")
                logger.info(f"img_egopose_path: {img_egopose_path}")
                img_egopose_file = open(img_egopose_path, "r")
                img_egopose = json.load(img_egopose_file)
                r_ = [
                    img_egopose["orientation"]["quaternion_local"]["x"],
                    img_egopose["orientation"]["quaternion_local"]["y"],
                    img_egopose["orientation"]["quaternion_local"]["z"],
                    img_egopose["orientation"]["quaternion_local"]["w"]
                ]
                t_ = [
                    img_egopose["position"]["position_local"]["x"],
                    img_egopose["position"]["position_local"]["y"],
                    img_egopose["position"]["position_local"]["z"]
                ]
                v_ = [
                    img_egopose["velocity"]["velocity_local"]["vx"],
                    img_egopose["velocity"]["velocity_local"]["vy"],
                    img_egopose["velocity"]["velocity_local"]["vz"]
                ]
                av_ = [
                    img_egopose["angular_velocity"]["angvelocity_local"]["vx"],
                    img_egopose["angular_velocity"]["angvelocity_local"]["vy"],
                    img_egopose["angular_velocity"]["angvelocity_local"]["vz"]
                ]

                r_ = Rotation.from_quat(r_).as_matrix()
                img_rt = np.eye(4, 4)
                img_rt[:3, :3] = r_
                img_rt[:3, 3] = t_
                
                # 定位信息和外参合并
                extrinsic = extrinsic @ (np.linalg.inv(img_rt) @ (lidar_rt @ self.change_lidar_rt))
                
            pts_3d_cam = self.car_to_cam(points, extrinsic)

            # 过滤掉相机后方的点
            show_index = np.where(pts_3d_cam[:, 2] > 0)
            pts_3d_cam = pts_3d_cam[show_index]

            bgr = colormap_intensity[show_index]
            # bgr = color[show_index]
            # bgr = self.colormap_z(pts_3d_cam, bgr)

            if pts_3d_cam.size == 0:
                continue
            # https://github.com/opencv/opencv/issues/9150

            pts_3d_cam = np.ascontiguousarray(pts_3d_cam[:, 0:3].reshape(-1, 1, 3))

            rot_mat = np.eye(3, 3)
            rvec, _ = cv2.Rodrigues(rot_mat)
            rvec = np.array([rvec[0][0], rvec[1][0], rvec[2][0]], np.float64)
            tvec = np.array([0.0, 0.0, 0.0], np.float64)
            # 相机等距模型
            if camera_model == camera_model_type["polyn"]:
                point_2d_origin, _ = cv2.fisheye.projectPoints(
                    pts_3d_cam, rvec, tvec, intrinsic, distortion
                )
            # 相机针孔模型
            elif camera_model == camera_model_type["radial_tangential"]:
                point_2d_origin, _ = cv2.projectPoints(
                    pts_3d_cam, rvec, tvec, intrinsic, distortion
                )
            elif camera_model == camera_model_type["ocam"]:
                point_2d_origin = self.projectPointsOcam(
                    pts_3d_cam, affine_parameters, inv_poly_parameters
                )
            else:
                continue
            x_origin, y_origin = point_2d_origin[:, 0, 0], point_2d_origin[:, 0, 1]

            if len(x_origin) <= 0:
                continue
            x_origin = np.clip(x_origin, 2, W - 2)
            y_origin = np.clip(y_origin, 2, H - 2)

            x_origin = x_origin.astype(np.int64)
            y_origin = y_origin.astype(np.int64)

            # car_velocity = self.calculate_velocity(r, v)
            camera_row_time = camera_row_scan_time[camera_name]
            # replacement = self.get_replacement(row=y_origin, row_time = camera_row_time,\
            #      height = H, car_velocity=car_velocity, extrinsic=extrinsic)
            # print("replacement shape:", replacement.shape)
            # print("pts_3d_cam shape:", pts_3d_cam.shape)

            T_camera_bottom_row = self.get_camera_bottom2row_transform(row=y_origin, row_time = camera_row_time,\
                 height = H, extrinsic=extrinsic, car_camera_bottom2local=img_rt, angular_velocity_local=av_, velocity_local=v_)
            pts_3d_cam_modify = self.tranform_point(pts_3d_cam, T_camera_bottom_row)

            # pts_3d_cam_modify = pts_3d_cam + replacement.reshape(-1, 1, 3)
            # print("pts_3d_cam_modify shape:", pts_3d_cam_modify.shape)

            # pts_3d_cam_modify = pts_3d_cam

            if camera_model == camera_model_type["polyn"]:
                point_2d, _ = cv2.fisheye.projectPoints(
                    pts_3d_cam_modify, rvec, tvec, intrinsic, distortion
                )
            # 相机针孔模型
            elif camera_model == camera_model_type["radial_tangential"]:
                point_2d, _ = cv2.projectPoints(
                    pts_3d_cam_modify, rvec, tvec, intrinsic, distortion
                )
            elif camera_model == camera_model_type["ocam"]:
                point_2d = self.projectPointsOcam(
                    pts_3d_cam_modify, affine_parameters, inv_poly_parameters
                )
            else:
                continue
            x, y = point_2d[:, 0, 0], point_2d[:, 0, 1]

            if len(x) <= 0:
                continue
            x = np.clip(x, 2, W - 2)
            y = np.clip(y, 2, H - 2)

            x = x.astype(np.int64)
            y = y.astype(np.int64)

            image_raw = deepcopy(image)
            image[y, x] = bgr
            x2 = x + 1
            image[y, x2] = bgr
            y2 = y + 1
            image[y2, x] = bgr
            image[y2, x2] = bgr
            alpha = 0.5
            image = image * alpha + image_raw * (1 - alpha)
            camera_img[camera_name] = image

        return camera_img
    
    def equal_interval_sampling(self, data, interval):
        sampled_data = {}
        keys = list(data.keys())
        for i in range(0, len(keys), interval):
            key = keys[i]
            sampled_data[key] = data[key]
        return sampled_data

    def _process_sample(self, save_path, key, value):
        pcd_object = os.path.join(self.sensor_data_path, f"lidar/lidar_undist/{key}")
        egopose_object = os.path.join(self.sensor_data_path, f"egopose_opt/egopose_optpose/{key[:-4]}.json")
        logger.info(f"process sample name pcd_object: {pcd_object}, egopose_object: {egopose_object}")
        image_objects = {}
        for image_key, image_value in value.items():
            if "camera" in image_key:
                image_objects[image_key] = os.path.join(self.sensor_data_path, f"camera/{image_key}/{image_value}")
                logger.info(f"process sample name image_objects: { image_objects[image_key]}")
        camera_img = self.sample_process(pcd_object, image_objects, egopose_object)
        sample_save_path = os.path.join(save_path, key[:-4])
        if not os.path.exists(sample_save_path):
            os.makedirs(sample_save_path)
        for camera_name, img in camera_img.items():
            save_img_path = os.path.join(sample_save_path, camera_name + ".jpeg")
            cv2.imwrite(save_img_path, img)
    
    def projectPointsOcam(self, pts, affine_parameters, inv_poly_parameters):
        """
        使用矩阵运算批量实现Ocam投影，提高效率
        """
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)      # (N,3)
        ac, ad, ae, cx, cy = affine_parameters

        # 1. 归一化坐标
        x = pts[:, 0] / pts[:, 2]     # (N,)
        y = pts[:, 1] / pts[:, 2]     # (N,)

        # 2. 计算 r 和 theta
        r = np.sqrt(x**2 + y**2)      # (N,)
        theta = np.arctan(-1.0 / r)   # (N,)

        # 3. 计算 rho：theta 的幂次矩阵 (N,16)
        theta_pow = np.power(theta[:, np.newaxis],
                             np.arange(16))  # (N,16)
        rho = theta_pow @ inv_poly_parameters  # (N,)

        # 4. 计算归一化平面坐标
        scale = rho / r                 # (N,)
        x_norm = x * scale              # (N,)
        y_norm = y * scale              # (N,)

        # 5. 仿射变换
        u = ac * x_norm + ad * y_norm + cx   # (N,)
        v = ae * x_norm + 1.0 * y_norm + cy  # (N,)

        # 6. 组装输出
        pts2d = np.stack([u, v], axis=-1)    # (N,2)
        return pts2d.reshape(-1, 1, 2)

    #for循环形式，效率比较低，但易于理解
    def projectPointsOcam2(self, pts, affine_parameters, inv_poly_parameters):
        # apply transform
        pts = np.array(pts, dtype=np.float64)
        pts = pts.reshape(-1, 3)

        pts2d = []
        ac, ad, ae, cx, cy = affine_parameters
        for pt in pts:
            x, y, z = pt
            x = x/z
            y = y/z

            r = np.sqrt(x*x + y*y)
            theta = np.arctan(-1/r)
            rho = 0
            temp_rho = 1.0
            for i in range(16):
                rho += inv_poly_parameters[i] * temp_rho
                temp_rho *= theta

            x = x / r * rho
            y = y / r * rho
            u = ac * x + ad * y + cx
            v = ae * x + 1.0 * y + cy

            pts2d.append([u, v])

        pts2d = np.array(pts2d, dtype=np.float64)
    
        pts2d = pts2d.reshape(-1, 1, 2)
        return pts2d

    def run(self, save_path):
        sample_data = self.equal_interval_sampling(self.result_json_object, 20)
        # 控制线程数为3，否则在平台上运行容易超内存
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            futures = []
            for key, value in sample_data.items():
                futures.append(executor.submit(self._process_sample, save_path, key, value))
            for future in concurrent.futures.as_completed(futures):
                future.result()

def parse_args():
    parser = ArgumentParser(description="base-visual")
    parser.add_argument(
        "--bev4d_path",
        type=str,
        default="/tmp/bev_4d",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    # 生成可视化数据
    pc_visual = PointCloudVisual(args.bev4d_path, transfrom=True)
    datacheck_path = os.path.join(args.bev4d_path, "visualize")
    os.makedirs(datacheck_path, exist_ok=True)
    pc_visual.run(datacheck_path)