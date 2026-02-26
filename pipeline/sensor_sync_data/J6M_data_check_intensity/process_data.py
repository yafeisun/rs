import numpy as np
import os, sys
import cv2
import multiprocessing as mp
import json
import copy
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
# from test import *
import time
import yaml

colorMap = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), 
    (0, 255, 255), (125, 0, 0), (0, 125, 0), (0, 0, 125), (125, 125, 0), 
    (0, 0, 0),  (127, 0, 255), (255, 127, 0), (128, 255, 0)
]

show_depth = 250

pixel_bias = (0, 0)

class_names = {
   'car':0, 'truck':1, 'bus':2, 'engineering_vehicle':3, 'person':4, 'pillar':5, 'cyclist':6, 'traffic_cone':7, 
   'noparking_pillar':8,'water_horse':9 ,'gate_on':10 ,'gate_off':11, 'anti_collision_bucket':12 ,'noparking_board':13,
#    'stroller': 14, 'shopping_cart': 15
}

cam_dict ={
    "front": "front",
    "front_left": "front_left",
    "front_right": "front_right",
    "back": "back",
    "back_left": "back_left",
    "back_right": "back_right",
    "a_front": "a_front",
    "a_right": "a_right",
    "a_left": "a_left",
    "a_back": "a_back",
    "front_far": "front_far",
}

cam_topic_dict ={
    "front": "cam_front_right",
    "front_left": "cam_side_left_front",
    "front_right": "cam_side_right_front",
    "back": "cam_back",
    "back_left": "cam_side_left_back",
    "back_right": "cam_side_right_back",
    "a_front": "cam_around_front",
    "a_right": "cam_around_right",
    "a_left": "cam_around_left",
    "a_back": "cam_around_back",
    "front_far": "cam_front_left",
}

undefined_classname = set()

CLASSES={
    'car': 'car',
    'truck': 'truck',
    'bus' : 'bus',
    'engineering_vehicle' : 'engineering_vehicle',
    'person':'person',
    'pillar':'pillar',
    'bicycle':'cyclist',
    'electromobile':'cyclist',
    'motorcycle':'cyclist',
    'barricade__traffic_cone':'traffic_cone',
    'barricade__noparking_pillar':'noparking_pillar',
    'barricade__water_horse':'water_horse',
    'barricade__gate':'gate',
    'barricade__anti_collision_bucket':'anti_collision_bucket',
    'barricade__noparking_board':'noparking_board',
    'stroller': 'stroller',
    'Shopping_cart': 'Shopping_cart'
}


def load_params(params_path):
    print("load_params:", params_path)
    if not os.path.exists(params_path):
        return None
    with open(params_path, 'r') as f:
        params_data = json.load(f)
    params = {}
    for cam in cam_dict.keys():
        cam_params_temp = {}
        cam_params = params_data["CAM__"+cam]
        intr = np.array(cam_params["intr"])
        D = np.array(cam_params["D"])
        extr = np.array(cam_params["extr"])
        height = cam_params["img_height"]
        width = cam_params["img_width"]
        cam_params_temp["intr"] = intr
        cam_params_temp["D"] = D
        cam_params_temp["extr"] = extr
        cam_params_temp["height"] = height
        cam_params_temp["width"] = width
        # cam_params_temp["calib_type"] = cam_params["type"]
        params[cam] = cam_params_temp
    return params

def load_params_from_frame(box_json_path):
    # if not os.path.exists(params_path):
    #     return None
    # with open(params_path, 'r') as f:
    #     params_data = json.load(f)

    # params = {}
    # for cam in cam_dict.keys():
    #     cam_params_temp = {}
    #     cam_params = params_data["CAM__"+cam]
    #     intr = np.array(cam_params["intr"])
    #     D = np.array(cam_params["D"])
    #     extr = np.array(cam_params["extr"])
    #     height = cam_params["img_height"]
    #     width = cam_params["img_width"]
    #     cam_params_temp["intr"] = intr
    #     cam_params_temp["D"] = D
    #     cam_params_temp["extr"] = extr
    #     cam_params_temp["height"] = height
    #     cam_params_temp["width"] = width
    #     cam_params_temp["calib_type"] = cam_params["type"]
    #     params[cam] = cam_params_temp
    # return params
    return None


def load_boxes(json_path):
    if not os.path.exists(json_path):
        return []
    with open(json_path, 'r') as f:
        box_json_data = json.load(f)["object"]
    boxes = []
    labels = []
    box_vis_flags = []
    for box in box_json_data:
        if not box["psr"]:
            continue
        x = box["psr"]["position"]["x"]
        y = box["psr"]["position"]["y"]
        z = box["psr"]["position"]["z"]
        l = box["psr"]["scale"]["x"]
        w = box["psr"]["scale"]["y"]
        h = box["psr"]["scale"]["z"]
        yaw = box["psr"]["rotation"]["z"]

        boxes.append([x,y,z,l,w,h,yaw])

    boxes = np.array(boxes)
    return boxes

def points_to_3d_bounding_box(points):
    if points.shape != (8, 3):
        raise ValueError("Input points must be an array of shape (8, 3).")

    # 创建一个点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 计算有向边界框
    oriented_bounding_box = pcd.get_oriented_bounding_box()

    # 获取中心点、尺寸和旋转矩阵
    center = oriented_bounding_box.center
    extent = oriented_bounding_box.extent
    R = oriented_bounding_box.R

    return center, extent, R

def rotation_matrix_to_euler_angles(R):
    """
    将旋转矩阵转换为欧拉角（按 ZYX 顺序）。
    
    参数:
    R (numpy.ndarray): 形状为 (3, 3) 的旋转矩阵。
    
    返回:
    tuple: 包含欧拉角 (roll, pitch, yaw) 的元组，单位为弧度。
    """
    if R.shape != (3, 3):
        raise ValueError("Input must be a 3x3 rotation matrix.")

    # 计算欧拉角
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)      # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])  # Roll
        y = np.arctan2(-R[2, 0], sy)       # Pitch
        z = 0.0                            # Yaw

    return x, y, z

def load_boxes_from_eight_point(data_json):
    BoxA = []
    pointA = []
    BoxLabel = []
    BoxTime = []

    # gtBoxPath = os.path.join(BasePath ,"bbox", str(frame) + '.json')

    with open(data_json, 'r', encoding='utf-8') as file:
        data = json.load(file)
    #print("data:",data.keys(),data['boxes'][0],data['num_boxes'])

    for indBox in range(data['num_boxes']):
        BoxT = data['boxes'][indBox]
        point = np.ones([8,3])
        point[0,:] = [BoxT['p_back_left_bottom']['x'],BoxT['p_back_left_bottom']['y'],BoxT['p_back_left_bottom']['z']]
        point[1,:] = [BoxT['p_back_right_bottom']['x'],BoxT['p_back_right_bottom']['y'],BoxT['p_back_right_bottom']['z']]
        point[2,:] = [BoxT['p_front_right_bottom']['x'],BoxT['p_front_right_bottom']['y'],BoxT['p_front_right_bottom']['z']]
        point[3,:] = [BoxT['p_front_left_bottom']['x'],BoxT['p_front_left_bottom']['y'],BoxT['p_front_left_bottom']['z']]

        point[4,:] = [BoxT['p_back_left_top']['x'],BoxT['p_back_left_top']['y'],BoxT['p_back_left_top']['z']]
        point[5,:] = [BoxT['p_back_right_top']['x'],BoxT['p_back_right_top']['y'],BoxT['p_back_right_top']['z']]
        point[6,:] = [BoxT['p_front_right_top']['x'],BoxT['p_front_right_top']['y'],BoxT['p_front_right_top']['z']]
        point[7,:] = [BoxT['p_front_left_top']['x'],BoxT['p_front_left_top']['y'],BoxT['p_front_left_top']['z']]

        center, extent, R = points_to_3d_bounding_box(point)
        pointA.append(point)
        x,y,z = rotation_matrix_to_euler_angles(R)
        Box = [center[0],center[1],center[2],extent[0],extent[1],extent[2],z]
        BoxA.append(Box)
        BoxLabel.append('car')
    BoxTime = data['timestamp']
    BoxA=np.array(BoxA)
    BoxLabel = np.array(BoxLabel)
    return BoxA, BoxLabel

def read_pcd_binary(pcd_file):
    with open(pcd_file, 'rb') as f:
        # 读取pcd文件头信息
        header = ""
        while True:
            line = f.readline().decode('utf-8')
            if "POINTS" in line:
                point_num = int(line.split("POINTS")[1])
            header += line
            if line.startswith("DATA"):
                break
        #dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # points = np.fromfile(f, dtype=np.float32)[:point_num*4].reshape(-1,4)

        # print("point_num POINTS:", point_num)
        points = np.fromfile(f, dtype=np.float32)
        point_num = points.shape[0] // 4
        # print("point_num:", point_num)
        points = points[:point_num*4].reshape(-1,4)
        # print("points:", points.shape)
        # exit()
        intensity = points[:,3]

        #print(points.shape,pcd_file)
        source_data = points[:,0:3]  #10000x3     
    return source_data, intensity, header

def point_project(coords, image, M1, M2):
    """
    points: Nx3
    image: opencv img, 表示要投影的图像
    M1: 内参矩阵 K, 4*4
    M2: 外参矩阵， 4*4

    return: points 在像素坐标系下的坐标 N*4, 实际只用 N*2
    """
    #import pdb
    #pdb.set_trace()
   # coords=coords[:,:3]
    # tmp = np.eye(4)
    # tmp[:3,:3] = M1[:3,:3]
    # M1 = tmp
   # import pdb
   # pdb.set_trace()
    resolution = image.shape
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)
    # coords = coords[np.where((coords[:, 0] > -50) * (coords[:, 0] < 50)* (coords[:, 1] < 30)* (coords[:, 0] > -30))]

    transform = copy.deepcopy(M2).reshape(4, 4)
    coords = (transform @ coords.T).T

    coords = coords[np.where((coords[:, 2] > 0) * (coords[:, 2] < show_depth))]
    zzz = coords[:,2].copy()

    coords = (M1 @ coords.T).T
    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]
    coords[:, 2] = 1
    vaild = (coords[:, 0] >=0) & (coords[:, 0] < resolution[1])  & (coords[:, 1] >=0) & (coords[:, 1] < resolution[0]) & (zzz >0)  

    return coords[vaild],zzz[vaild]


def point_project_distort(coords, image, intr, extr, D=None, intensity=None):
    resolution = image.shape
    points = copy.deepcopy(coords)
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)

    transform = copy.deepcopy(extr).reshape(4, 4)
    coords = coords @ transform.T

    flag = np.where((coords[:, 2] > 0) * (coords[:, 2] < show_depth))
    coords = coords[flag]
    points = points[flag]
    zzz = coords[:,2].copy()

    rvec = cv2.Rodrigues(extr[:3,:3])[0]
    tvec = extr[:3,3]
    if D is None:
        points = cv2.projectPoints(points.reshape(-1, 1, 3), rvec, tvec, intr[:3,:3], D)[0].reshape(-1, 2)
    else:
        points = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), rvec, tvec, intr[:3,:3], D)[0].reshape(-1, 2)

    vaild = (zzz > 0) & (points[:, 0] >=0) & (points[:, 0] < resolution[1])  & (points[:, 1] >=0) & (points[:, 1] < resolution[0])

    zzz = zzz/show_depth*255

    if intensity is not None:
        intensity_ = copy.deepcopy(intensity)[flag][vaild]
    else:
        intensity_ = None

    return points[vaild],zzz[vaild],intensity_


def get_color(depth):
    color =(100,0,0)
    if depth<32:
        color = (128+4*depth,0,0)
    elif depth==32:
        color=(255,0,0)
    elif depth<95:
        color = (255, 4+4*depth, 0)
    elif depth==96:
        color = (254, 255, 2)
    elif depth<158:
        color = (254, 255, 2)
    elif depth == 159:
        color = (1, 255, 254)
    elif depth <223:
        color = (0, 252 - 4*depth, 255)
    elif depth <255:
        color = (0, 0, 252-4*depth)
    return color

def rotation_3d_in_aixs(corners, yaw):
    rot_sin = np.sin(yaw)
    rot_cos = np.cos(yaw)
    ones    = np.ones_like(rot_sin)
    zeros   = np.zeros_like(rot_sin)

    rot_mat_T = np.stack([
        np.stack([rot_cos, rot_sin, zeros]),
        np.stack([-rot_sin, rot_cos,  zeros]),
        np.stack([zeros,   zeros,    ones])
    ])

    return np.einsum('aij, jka->aik', corners, rot_mat_T)

def get_corners(boxes):
    if len(boxes)<=0:
        return []
    corners_norm = np.stack(np.unravel_index(np.arange(8), [2]*3), axis=1)
    corners_norm = corners_norm[[0, 1, 3, 2, 4, 5, 7, 6]]
    corners_norm = corners_norm - np.array([0.5, 0.5, 0.5])
    corners = boxes[:, 3:6].reshape([-1, 1, 3]) * corners_norm.reshape([1, 8, 3])
    corners = rotation_3d_in_aixs(corners, boxes[:, -1])
    corners += boxes[:, :3].reshape(-1, 1, 3)
    return corners

def draw_corners(img, corner, color, projection, img_size):
    # pdb.set_trace()
    corners_3d_4 = np.concatenate((corner, np.ones((8, 1))), axis=1)
    corners_2d_3 = corners_3d_4 @ projection.T
    z_mask = corners_2d_3[:, 2] > 0
    corners_2d = corners_2d_3[:, :2] / corners_2d_3[:, 2:]

    corners_2d[:] += pixel_bias

    visible_mask = (corners_2d_3[:, 2] > 0) & (0 < corners_2d[:, 0]) & (corners_2d[:, 0] < img_size[0]) & (0 < corners_2d[:, 1]) & (corners_2d[:, 1] < img_size[1])
    if not visible_mask.any():
        return
    corners_2d = corners_2d.astype(np.int64)
    for i, j in [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
        [4, 6], [5, 7]
    ]:
        try:
            if z_mask[i] and z_mask[j]:
                cv2.line(img, tuple(corners_2d[i]), tuple(corners_2d[j]), color, 1)
        except:
            pass

def draw_corners_distort(img, corner, color, intrinsic, extrinsic, img_size, D=np.zeros(4)):

    corners_2d_3 = corner @ extrinsic[:3,:3].T + extrinsic[:3,3].T
    z_mask = corners_2d_3[:, 2] > 0

    corner = corner.reshape(-1, 1, 3)
    rvec = cv2.Rodrigues(extrinsic[:3,:3])[0]
    tvec = extrinsic[:3,3]
    corners_2d = cv2.fisheye.projectPoints(corner, rvec, tvec, intrinsic, D)[0].reshape(-1, 2)


    visible_mask = (corners_2d_3[:, 2] > 0) & (0 < corners_2d[:, 0]) & (corners_2d[:, 0] < img_size[0]) & (0 < corners_2d[:, 1]) & (corners_2d[:, 1] < img_size[1])
    if not visible_mask.any():
        return
    corners_2d = corners_2d.astype(np.int64)
    for i, j in [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
        [4, 6], [5, 7]
    ]:
        try:
            if z_mask[i] and z_mask[j]:
                cv2.line(img, tuple(corners_2d[i]), tuple(corners_2d[j]), color, 1)
        except:
            pass
    
def visualize_box(frame_path, save_path, params, boxes, box_oclu_infos=None,  undistort_flag=True):
    # print("********************"+os.path.basename(frame_path)+"********************")
    frame = os.path.basename(frame_path)
    pics = []
    corners = get_corners(boxes)
    if box_oclu_infos is None:
        box_oclu_infos = ["" for _ in range(len(boxes))]

    for cam in cam_dict.keys():
        cam_value = cam_dict[cam]
        img_path = os.path.join(frame_path, frame+"__"+cam_value+".jpg")
        img = cv2.imread(img_path)
        intrinsic = copy.deepcopy(params[cam_value]["intr"])
        extrinsic = copy.deepcopy(params[cam_value]["extr"])
        D = params[cam_value]["D"]
        img_height = params[cam_value]["height"]
        img_width = params[cam_value]["width"]

        if not "a_" in cam:
            new_height = 1920
            new_width = 1080
        else:
            new_height = img_height
            new_width = img_width


        # if undistort_flag:
        #     if not "a_" in cam:
        #         # return
        #         alpha = 0.3
        #         new_intrinsic, validPixROI = cv2.getOptimalNewCameraMatrix(intrinsic, D[:4], (img_height,img_width), alpha)
        #         map1, map2 = cv2.fisheye.initUndistortRectifyMap(intrinsic,D[:4],None,new_intrinsic,(img_height,img_width), cv2.CV_16SC2)
        #     else:
        #         map1, map2 = cv2.fisheye.initUndistortRectifyMap(intrinsic,D[:4],None,intrinsic,(img_height,img_width),cv2.CV_16SC2)
        #         new_intrinsic = intrinsic

        #     img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        #     intrinsic = new_intrinsic

        if (not cam_value in ["front", "back"]) and (img_width == 1280):
            img = img[100:1280-100, :, :]
            intrinsic[1,2] -= 100
        if cam_value in ["front", "back", "front_far"]:
            if not (0.9 < intrinsic[0,2]*2/img_height < 1.1):
                intrinsic = intrinsic * 2
                intrinsic[2,2] = 1.0
        new_intrinsic = intrinsic * new_height / img_height
        new_intrinsic[2,2] = 1.0
        img = cv2.resize(img, (new_height, new_width))
        projection = new_intrinsic @ extrinsic[:3]
        for corner in corners:
            if (not undistort_flag) and "a_" in cam:
                draw_corners_distort(img, corner, (0,0,255), new_intrinsic[:3,:3], extrinsic, (new_height, new_width), D[:4])
            else:
                draw_corners(img, corner, (0,0,255), projection, (new_height, new_width))
        pics.append(img)

    pic1 = cv2.hconcat([pics[1], pics[0], pics[2]])
    pic2 = cv2.hconcat([pics[4], pics[3], pics[5]])
    img_save_6v = cv2.vconcat([pic1, pic2])
    # cv2.imwrite(os.path.join(save_path,"vis_box", frame+"_6v.jpg"), img_save_6v)

    cv2.imwrite(os.path.join(save_path,"vis_box", "6v_"+frame+".jpg"), img_save_6v)
    # print(os.path.join(save_path,"vis_box", "6v_"+frame+".jpg"))

    pic3 = cv2.hconcat([pics[6], pics[7]])
    pic4 = cv2.hconcat([pics[8], pics[9]])
    img_save_4v = cv2.vconcat([pic3, pic4])
    cv2.imwrite(os.path.join(save_path,"vis_box", "4v_"+frame+".jpg"), img_save_4v)

    cv2.imwrite(os.path.join(save_path,"vis_box", "far_"+frame+".jpg"), pics[10])

def draw_pcd(img, points, intr, extr, D=None):

    points, zzz = point_project_distort(points, img, intr, extr, D)

    circle_thickness = 1

    if points is not None:
        for index in range(points.shape[0]):
            p = (int(points[index, 0]), int(points[index, 1]))
            cv2.drawMarker(img,position=p,color=get_color(zzz[index]),markerSize = circle_thickness*2, markerType=cv2.MARKER_CROSS, thickness=circle_thickness)

def get_color_from_intensity(point_intensity):
    intensity_colors = [
        # (0, 0, 0),  # Black
        (255, 0, 0),  # Blue
        (0, 0, 255),  # Red
        (255, 255, 0),  # Cyan
        (0, 255, 255),  # Yellow
        (255, 0, 255),  # Purple
        (255, 255, 255),  # White

    ]
    color = intensity_colors[int((point_intensity-1) * len(intensity_colors) / 255)]

    # if point_intensity > 20 and point_intensity < (255*2 / len(intensity_colors)):
    #     color = intensity_colors[1]

    return color

def visualize_pcd(frame_path, save_path, params, points, intensity=None, vis_intensity=False):
    frame = os.path.basename(frame_path)
    pics = []

    for cam in cam_dict.keys():
        cam_value = cam_dict[cam]
        img_path = os.path.join(frame_path, frame+"__"+cam_value+".jpg")
        img = cv2.imread(img_path)
        intrinsic = copy.deepcopy(params[cam_value]["intr"])
        extrinsic = copy.deepcopy(params[cam_value]["extr"])
        D = params[cam_value]["D"]
        img_height = max(list(img.shape)[:2])
        img_width = min(list(img.shape)[:2])
        if not "a_" in cam:
            new_height = 1920
            new_width = 1080
        else:
            new_height = img_height
            new_width = img_width
        circle_thickness = int(img_height/1920)
        if cam_value == "front":
            circle_thickness = 1


        # print("intrinsic", intrinsic)

        # if undistort_flag:
        #     if not "a_" in cam:
        #         alpha = 0.3
        #         new_intrinsic, validPixROI = cv2.getOptimalNewCameraMatrix(intrinsic, D[:4], (img_height,img_width), alpha)
        #         map1, map2 = cv2.fisheye.initUndistortRectifyMap(intrinsic,D[:4],None,new_intrinsic,(img_height,img_width), cv2.CV_16SC2)
        #     else:
        #         map1, map2 = cv2.fisheye.initUndistortRectifyMap(intrinsic,D[:4],None,intrinsic,(img_height,img_width),cv2.CV_16SC2)
        #         new_intrinsic = intrinsic

        #     img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        #     intrinsic = new_intrinsic     
    
            
        if (not cam_value in ["front", "back", "front_far"]) and (img_width == 1280):
            img = img[100:1280-100, :, :]
            intrinsic[1,2] -= 100
        if cam_value in ["front", "back", "front_far"]:
            # print(img_height, img_width, img.shape, intrinsic)
            if 1.1 < intrinsic[0,2]*2/img_height:
                intrinsic = intrinsic / 2
                intrinsic[2,2] = 1.0

        new_intr = np.identity(4)
        new_intr[:3, :3] = intrinsic # 4x4
        # if True:
        if "a_" in cam:
            # img = cv2.fisheye.undistortImage(img, intrinsic, D[:4], None, intrinsic)
            # coords, zzz, intensity_ = point_project_distort(copy.deepcopy(points), img, new_intr, extrinsic, None, intensity)

            coords, zzz, intensity_ = point_project_distort(copy.deepcopy(points), img, new_intr, extrinsic, D[:4], intensity)
        else:
            coords, zzz, intensity_ = point_project_distort(copy.deepcopy(points), img, new_intr, extrinsic, None, intensity)
        
        if "a_" in cam:
            show_depth = 50
        else:
            show_depth = 250
        zzz = zzz/show_depth*255
        if coords is not None:
            coords[:, 0] += pixel_bias[0]
            coords[:, 1] += pixel_bias[1]
            for index in range(coords.shape[0]):
                p = (int(coords[index, 0]), int(coords[index, 1]))
                ps=3
                # cv2.circle(img, p, ps, color=get_color(zzz[index]), thickness=circle_thickness)
                if vis_intensity or (intensity is None):
                    cv2.drawMarker(img,position=p,color=get_color_from_intensity(intensity_[index]),markerSize = circle_thickness*2, markerType=cv2.MARKER_CROSS, thickness=circle_thickness)
                else:
                    cv2.drawMarker(img,position=p,color=get_color(zzz[index]),markerSize = circle_thickness*2, markerType=cv2.MARKER_CROSS, thickness=circle_thickness)

        img = cv2.resize(img, (new_height, new_width))
        pics.append(img)
    pic1 = cv2.hconcat([pics[1], pics[0], pics[2]])
    pic2 = cv2.hconcat([pics[4], pics[3], pics[5]])
    img_save_6v = cv2.vconcat([pic1, pic2])
    cv2.imwrite(os.path.join(save_path,"vis_pcd", "6v_"+frame+".jpg"), img_save_6v)
    # print(os.path.join(save_path,"vis_pcd", "6v_"+frame+".jpg"))

    pic3 = cv2.hconcat([pics[6], pics[7]])
    pic4 = cv2.hconcat([pics[8], pics[9]])
    img_save_4v = cv2.vconcat([pic3, pic4])
    cv2.imwrite(os.path.join(save_path,"vis_pcd", "4v_"+frame+".jpg"), img_save_4v)

    cv2.imwrite(os.path.join(save_path,"vis_pcd", "far_"+frame+".jpg"), pics[10])

    
def visualize_box_in_pcd(boxes, points, Pcd_color, frame_name="box in pcd"):
    #创建窗口对象
    vis = o3d.visualization.Visualizer()
    #设置窗口标题
    # vis.create_window(window_name="box in pcd")
    vis.create_window(window_name=frame_name)
    #设置点云大小
    opt = vis.get_render_option()
    opt.point_size = 1
    #设置颜色背景为白色
    opt.background_color = np.asarray([1, 1, 1])


    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors =  o3d.utility.Vector3dVector(Pcd_color)

    # o3d.visualization.draw_geometries_with_editing([pcd])

    corners = get_corners(boxes)
    for corner in corners:
        # color = np.array(colorMap[class_names[label]]) / 255.0
        color = np.array((0,0,1.0))
        lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
        [4, 6], [5, 7]
        ]
        line_colors = np.tile(color, len(lines)).reshape(-1,3)
        line_set = o3d.geometry.LineSet()
        line_set.lines = o3d.utility.Vector2iVector(lines) 
        line_set.colors = o3d.utility.Vector3dVector(line_colors)
        line_set.points = o3d.utility.Vector3dVector(corner)
        vis.add_geometry(line_set)
        # vis.add_3d_label(corner[1], "{}".format(0.88))

    FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    # o3d.visualization.draw_geometries([pcd, FOR1]) 

    vis.add_geometry(pcd)
    vis.add_geometry(FOR1)

    vis.run()
    vis.destroy_window()


def save_box_json(json_path, box_json_data, box_vis_flags):
    for box, flag in zip(box_json_data, box_vis_flags):
        box["obj_attr"] = flag

    with open(json_path, 'w') as f:
        json.dump(box_json_data, f, indent=4)

def process_by_frame(frame, bag_dir_path, save_path, params, vis_box=False, vis_pcd=False, vis_box_in_pcd=False):
    

    frame_path = os.path.join(bag_dir_path, frame)

    if not os.path.isdir(frame_path):
        return
    box_json_path = os.path.join(frame_path, frame+".json")

    if params is None:
        params = load_params_from_frame(box_json_path)

    # print("frame:", frame)

    # print(box_json_path)
    boxes = load_boxes(box_json_path)


    # box_json = os.path.join(frame_path, frame+"_robosense.json")
    # boxes, labels = load_boxes_from_eight_point(box_json)
    # print("len(boxes)", len(boxes))
    pcd_path = os.path.join(frame_path, frame+".pcd")

    o3d_pcd = o3d.io.read_point_cloud(os.path.join(frame_path, frame+".pcd"))
    # print("points num:", len(o3d_pcd.points))
    points, intensity, header = read_pcd_binary(pcd_path)
    # print(len(points))
    # print(len(intensity))
    # exit()
    # points = np.asarray(o3d_pcd.points)
    # print("points num:", points.shape[0])
    # print("max(x):", np.max(points[:,0]), "min(x):", np.min(points[:,0]))
    # print("max(y):", np.max(points[:,1]), "min(y):", np.min(points[:,1]))
    # print("max(z):", np.max(points[:,2]), "min(z):", np.min(points[:,2]))

    flag = (np.abs(points[:,0] <=500)) & (np.abs(points[:,1] <=500)) & (np.abs(points[:,2] <=50))
    points = points[flag]
    # print("valid points (x<500m, y<500m, z<50m) num:", points.shape[0])
    # print("max(x):", np.max(points[:,0]), "min(x):", np.min(points[:,0]))
    # print("max(y):", np.max(points[:,1]), "min(y):", np.min(points[:,1]))
    # print("max(z):", np.max(points[:,2]), "min(z):", np.min(points[:,2]))

    intensity = intensity[flag]

    # intensity_flag = (intensity > 20)
    # points = points[intensity_flag]
    # intensity = intensity[intensity_flag]

    # print("boxes num:", len(boxes))
    # print("points = points[(np.abs(points[:,0] <=500)) & (np.abs(points[:,1] <=500)) & (np.abs(points[:,2] <=50))]")
    # print("points num:", points.shape[0])
    if vis_box:
        visualize_box(frame_path, save_path, params, boxes, None, False)
    if vis_pcd:
        visualize_pcd(frame_path, save_path, params, points, intensity, True)
    if vis_box_in_pcd:
        visualize_box_in_pcd(boxes, points, points*0+np.array([1,0,0]).reshape(1,3), frame)


def process_by_bag(bag_dir_path, save_path, vis_box=False, vis_pcd=False, vis_box_in_pcd=False):
    # params = load_params(os.path.join(bag_dir_path, "calib_0308.json"))

    pcd_path = os.path.join(save_path, "vis_pcd")
    box_path = os.path.join(save_path, "vis_box")
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    if not os.path.exists(pcd_path) and vis_pcd:
        os.makedirs(pcd_path)
    if not os.path.exists(box_path) and vis_box:
        os.makedirs(box_path)

    params = load_params(os.path.join(bag_dir_path, "..", "calib_new.json"))
    if params is None:
        params = load_params(os.path.join(bag_dir_path, "calib.json"))
        if params is None:
            print("no cali.json or calib.json in bag dir: ", bag_dir_path)
            # return None




    frameList = os.listdir(bag_dir_path)
    frameList.sort()

    with mp.Pool(processes=50) as pool:
        pool.starmap_async(process_by_frame, [(frame, bag_dir_path, save_path, params, vis_box, vis_pcd, vis_box_in_pcd) for frame in frameList if len(frame) == 13])
        pool.close()
        pool.join()

    # for frame in frameList:
    #     if len(frame) != 13:
    #         continue
    #     process_by_frame(frame, bag_dir_path, save_path, params, vis_box, vis_pcd, vis_box_in_pcd)

def get_frame_loc(desc_path):
    with open(desc_path, 'r') as f:
        desc_data = json.load(f)
    return np.array(desc_data["desc"]["ego2global"])

# def load

def vis_pcd_compare(points1, points2):
    # points = np.asarray(points.points)
    #创建窗口对象
    vis = o3d.visualization.Visualizer()
    #设置窗口标题
    vis.create_window(window_name="box in pcd")
    #设置点云大小
    opt = vis.get_render_option()
    opt.point_size = 1
    #设置颜色背景为白色
    opt.background_color = np.asarray([1, 1, 1])

    FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])

    color1 = np.expand_dims(np.array([1, 0, 0]), axis=0).repeat(len(points1.points), axis=0)
    color2 = np.expand_dims(np.array([0, 0, 1]), axis=0).repeat(len(points2.points), axis=0)
    points1.colors = o3d.utility.Vector3dVector(color1)
    points2.colors = o3d.utility.Vector3dVector(color2)
    vis.add_geometry(points1)
    vis.add_geometry(points2)
    vis.add_geometry(FOR1)

    vis.run()
    vis.destroy_window()

def vis_pcd_stack(points_list):
    # points = np.asarray(points.points)
    #创建窗口对象
    vis = o3d.visualization.Visualizer()
    #设置窗口标题
    vis.create_window(window_name="box in pcd")
    #设置点云大小
    opt = vis.get_render_option()
    opt.point_size = 1
    #设置颜色背景为白色
    opt.background_color = np.asarray([1, 1, 1])

    # FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
    # vis.add_geometry(FOR1)

    for o3d_points in points_list:
        color = np.expand_dims(np.array([0, 0, 0]), axis=0).repeat(len(o3d_points.points), axis=0)
        o3d_points.colors = o3d.utility.Vector3dVector(color)
        vis.add_geometry(o3d_points)

    vis.run()
    vis.destroy_window()


def process_stacking_frame(bag_dir_path, gap):

    print("processing:", bag_dir_path, "gap:", gap)
    frameList = sorted([name for name in os.listdir(bag_dir_path) if not ".json" in name])

    
    # desc_path = os.path.join(frame_path, frame+"__desc.json")
    # frame_loc = get_frame_loc(desc_path)
    # o3d_pcd = o3d.io.read_point_cloud(os.path.join(frame_path, frame+".pcd"))
    # points = np.asarray(o3d_pcd.points)

    points_list = []
    pre_global_loc = None
    for index in range(0, len(frameList), gap):
        frame = frameList[index]
        frame_path = os.path.join(bag_dir_path, frame)
        if not os.path.isdir(frame_path):
            continue
        print("frame:", frame)
        desc_path = os.path.join(frame_path, frame+"__desc.json")
        frame_loc = get_frame_loc(desc_path)
        # global_loc = np.linalg.inv(frame_loc)
        # print("global_loc:", global_loc[:3, 3])
        # if pre_global_loc is not None:
        #     relative_loc = pre_global_loc @ frame_loc
        #     print("relative_loc:", relative_loc[:3, 3])
        # pre_global_loc = global_loc
        o3d_pcd = o3d.io.read_point_cloud(os.path.join(frame_path, frame+".pcd"))
        points = np.array(o3d_pcd.points)
        valid = ~((np.abs(points[:,0]) < 2.5) & (np.abs(points[:,1]) < 1.0))
        points = points[valid]
        points = points @ frame_loc[:3, :3].T + frame_loc[:3, 3]
        o3d_pcd.points = o3d.utility.Vector3dVector(points)
        points_list.append(o3d_pcd)
    # vis_pcd_compare(points_list[0], points_list[1])
    vis_pcd_stack(points_list)
    






def process_by_date(date_path, save_path, vis_box=False, vis_pcd=False, vis_box_in_pcd=False):
    if not os.path.isdir(date_path):
        print("date_path is not a dir")
        return
    bag_dir_list = os.listdir(date_path)
    for bag_dir in bag_dir_list:
        process_by_bag(os.path.join(date_path, bag_dir), save_path, vis_box, vis_pcd, vis_box_in_pcd)

def process(gt_data_path, save_path, vis_box=False, vis_pcd=False, vis_box_in_pcd=False):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.isdir(gt_data_path):
        print("gt_data_path is not a dir")
        return
    date_dir_list = os.listdir(gt_data_path)

    if vis_box and not os.path.exists(os.path.join(save_path, "vis_box")):
        os.makedirs(os.path.join(save_path, "vis_box"))
    if vis_pcd and not os.path.exists(os.path.join(save_path, "vis_pcd")):
        os.makedirs(os.path.join(save_path, "vis_pcd"))

    for date_dir in date_dir_list:
        process_by_date(os.path.join(gt_data_path, date_dir), save_path, vis_box, vis_pcd, vis_box_in_pcd)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 process_data.py <path_to_gt_data> (<path_to_save>)")
        exit()
    gt_data_path = sys.argv[1]
    if len(sys.argv) < 3:
        save_path = "vis_output"
    else:
        save_path = sys.argv[2]
        save_path = os.path.join("./save_output", save_path)
    process(gt_data_path, save_path, vis_box=False, vis_pcd=True, vis_box_in_pcd=False)