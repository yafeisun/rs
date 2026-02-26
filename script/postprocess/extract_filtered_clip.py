import os
import shutil
import re
import subprocess

vision_folder_list = {
    "cam_around_back",
    "cam_around_front",
    "cam_around_left",
    "cam_around_right",
    "cam_back",
    "cam_front_left",
    "cam_front_right",
    "cam_side_left_back",
    "cam_side_left_front",
    "cam_side_right_back",
    "cam_side_right_front",
}

def extract_bev_files(bag, flag=False):
    # 使用 os.makedirs 递归创建目标目录
    target_root = bag + "-clip-20df"
    target_bev = os.path.join(target_root, "bev")
    if not os.path.exists(target_bev):
        os.mkdir(target_bev)
    bev_folder = os.path.join(bag, "result", "bev")
    for file in ["BEV.jpeg", "InBEV.jpeg", "HBEV.png", "BEV_label.json", \
        "IBEV.pcd", "RGBBEV.pcd"]:
        file_path = os.path.join(bev_folder, file)
        target_file_path = os.path.join(target_bev, file)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            if flag or not os.path.exists(target_file_path):
                shutil.copy2(file_path, target_file_path)
        else:
            print(f"{file} 不存在于 {bev_folder}")

    for file in ["map_intensity.pcd", "map_intensity_bev.pcd", "map_rgb_bev.pcd"]:
        map_pcd = os.path.join(bag, "result", "test_calibration", file)
        target_map_pcd_path = os.path.join(target_bev, os.path.basename(map_pcd))
        if os.path.exists(map_pcd) and os.path.isfile(map_pcd):
            if flag or not os.path.exists(target_map_pcd_path):
                shutil.copy2(map_pcd, target_map_pcd_path)
        else:
            print(f"{file} not in {os.path.join(bag, 'result', 'test_calibration')}")

    #cp pose txt
    target_pose = os.path.join(target_root, "pose")
    if not os.path.exists(target_pose):
        os.mkdir(target_pose)
    for img_folder in vision_folder_list:
        pose_txt = os.path.join(bag, "result", "test_calibration", img_folder, "sync_sensors.txt")
        if os.path.exists(pose_txt):
            img_pose = os.path.join(target_pose, img_folder)
            if not os.path.exists(img_pose):
                os.mkdir(img_pose)
            shutil.copy2(pose_txt, os.path.join(img_pose, os.path.basename(pose_txt)))

img_root = "/home/geely/nas/J6M-3/4D-OD/4D-OD-BAG"  #筛选完成的路径
bag_root = "/home/geely/nas/J6M-3/bag"                               #原始路径
target_root = "/home/geely/nas/J6M-3/bag-clip"            #拷贝路径

if os.path.isdir(img_root):
    img_car_paths = [os.path.abspath(os.path.join(img_root, d)) for d in os.listdir(img_root)]
    img_car_paths = [d for d in img_car_paths if os.path.isdir(d)]
img_car_paths.sort()

for car_dir in img_car_paths:
    img_bags = os.listdir(car_dir)
    img_bags.sort()
    for img_bag in img_bags:
        od_frame_path = os.path.join(bag_root, car_dir.split('_')[2], os.path.basename(car_dir), img_bag) + "-clip-20df"
        if os.path.exists(od_frame_path):
            target_dir = os.path.join(target_root, os.path.basename(car_dir), os.path.basename(car_dir).split('_')[1] + img_bag + "-clip-20df")
            if not os.path.exists(target_dir):
                os.makedirs(target_dir)

            img_clips = os.listdir(os.path.join(car_dir, img_bag))
            img_clips.sort()
            for clip in img_clips:
                od_clip = os.path.join(od_frame_path, clip)
                if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_dir, clip)):
                    try:
                        print(f"cp {clip} to {target_dir}")
                        subprocess.run(["cp", "-rp", od_clip, target_dir], check=True)
                    except Exception as e:
                        print(f"cp failed: {e}")

            if not os.path.exists(os.path.join(od_frame_path, "bev")):
                bag_dir = od_frame_path.replace("-clip-20df", "")
                extract_bev_files(bag_dir)

            for item in ["bev", "pose"]:
                od_clip = os.path.join(od_frame_path, item)
                if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_dir, item)):
                    try:
                        print(f"cp {item} to {target_dir}")
                        subprocess.run(["cp", "-rp", od_clip, target_dir], check=True)
                    except Exception as e:
                        print(f"cp failed: {e}")



