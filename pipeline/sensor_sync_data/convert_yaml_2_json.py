import numpy as np
import os, sys
import yaml, json
import cv2


cam_dict ={
    "front": "cam_front_right",
    "front_right": "cam_side_right_front",
    "front_left": "cam_side_left_front",
    "back": "cam_back",
    "back_left": "cam_side_left_back",
    "back_right": "cam_side_right_back",
    "a_front": "cam_around_front",
    "a_back": "cam_around_back",
    "a_left": "cam_around_left",
    "a_right": "cam_around_right",
    "front_far": "cam_front_left",
    
}

lidar_topic = "/middle/rslidar_packets_unique"

def process_convert(yaml_file, json_path):

    with open(yaml_file, "r") as f:
        params_yaml = yaml.load(f, Loader=yaml.FullLoader)

    camera_params = params_yaml["sensors"]["camera"]

    params = {}
    for cam in cam_dict.keys():
        cam_params = None
        for cam_param in camera_params:
            if cam_dict[cam] in cam_param["topic"]:
                cam_params = cam_param
                break
        if cam_params is None:
            print("Camera params not found for cam: ", cam)
            continue


        cam_json_params = {}
        cam_cali = cam_params["calibration"]
        intr = np.array(cam_cali["CameraIntMat"]).reshape(3,3)
        D = np.zeros(8)
        D[:4] = np.array(cam_cali["DistCoeff"])
        x, y, z, roll, pitch, yaw = cam_cali["CameraExt"]["x"], cam_cali["CameraExt"]["y"], cam_cali["CameraExt"]["z"], cam_cali["CameraExt"]["roll"], cam_cali["CameraExt"]["pitch"], cam_cali["CameraExt"]["yaw"]
        trans = np.array([x, y, z])
        Rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
        Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        Rot = Rz@Ry@Rx
        extr = np.eye(4)
        extr[:3,:3] = Rot
        extr[:3,3] = trans
        extr = np.linalg.inv(extr)

        img_height, img_width = cam_cali["ImageSize"]

        if not "a_" in cam:
            alpha = 0.3
            new_intr, validPixROI = cv2.getOptimalNewCameraMatrix(intr, D[:4], (img_height,img_width), alpha)
        else:
            new_intr = intr

        cam_json_params["type"] = "fisheye Equidistant"
        cam_json_params["intr"] = new_intr.tolist()
        cam_json_params["D"] = D.tolist()
        cam_json_params["extr"] = extr.tolist()
        cam_json_params["img_height"], cam_json_params["img_width"] = img_height, img_width
        if not "a_" in cam:
            cam_json_params["origin_intr"] = intr.tolist()

        # cam_params["type"] = cam_param["topic"]

        params["CAM__"+cam] = cam_json_params

    lidar_param = params_yaml["sensors"]["lidar"]

    middle_lidar_param = None
    for lidar_param in params_yaml["sensors"]["lidar"]:
        if lidar_param["topic"] == lidar_topic:
            middle_lidar_param = lidar_param
    if middle_lidar_param is None:
        print("middle lidar params not found")
        exit()

    x, y, z, roll, pitch, yaw = [
        middle_lidar_param["calibration"]["x"],
        middle_lidar_param["calibration"]["y"],
        middle_lidar_param["calibration"]["z"],
        middle_lidar_param["calibration"]["roll"],
        middle_lidar_param["calibration"]["pitch"],
        middle_lidar_param["calibration"]["yaw"]
    ]
    trans = np.array([x, y, z])
    Rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    Rot = Rz@Ry@Rx
    extr = np.eye(4)
    extr[:3,:3] = Rot
    extr[:3,3] = trans
    # extr = np.linalg.inv(extr)

    params["trans_lidar2ann"] = extr.tolist()

    with open(json_path, "w") as f:
        json.dump(params, f, indent=4)

    print("saved to ", json_path)


if __name__=="__main__":
    yaml_file = sys.argv[1]

    json_path = os.path.join(os.path.dirname(yaml_file), "calib_new.json")

    process_convert(yaml_file, json_path)