import open3d as o3d
import os
import numpy as np
import json
import shutil
import math


def remove_static_frame(label_seman_dir):
    pc_lists = os.listdir(label_seman_dir)
    pc_lists = [i for i in pc_lists if os.path.isdir(os.path.join(label_seman_dir, i))]
    pc_lists = sorted(pc_lists)
    
    if len(pc_lists) > 0:
        curr_timestamp = pc_lists[0]
    else:
        return None
    curr_loc_file = os.path.join(label_seman_dir, curr_timestamp, curr_timestamp + '__desc.json')
    if not os.path.exists(curr_loc_file):
        print(f"{curr_loc_file} is not exists!")
    with open(curr_loc_file) as f:
        curr_loc_content = json.load(f)
    curr_trans = np.asarray(curr_loc_content["desc"]["ego2global"])
    curr_x, curr_y, curr_z = curr_trans[:3, 3]
    
    for i in range(1, len(pc_lists)):
        timestamp = pc_lists[i]
        if "cam" in timestamp:
            continue
        loc_file = os.path.join(label_seman_dir, timestamp, timestamp + '__desc.json')
        if not os.path.exists(loc_file):
            print(f"{loc_file} is not exists!")
            continue
        with open(loc_file) as f:
            loc_content = json.load(f)
        trans = np.asarray(loc_content["desc"]["ego2global"])
        x, y, z = trans[:3, 3]
        distance = math.sqrt((x - curr_x) ** 2 + (y - curr_y) ** 2 + (z - curr_z) ** 2)
        print(f'distance from {timestamp} to {curr_timestamp} is {distance}m')
        if distance < 0.05:
            print(f'remove {os.path.join(label_seman_dir, timestamp)}')
            shutil.rmtree(os.path.join(label_seman_dir, timestamp))
        else:
            curr_x, curr_y, curr_z = x, y, z
            curr_timestamp = timestamp
    return 1


if __name__ == "__main__":

    # command
    # python3 clips_location_checking.py /home/geely/nas/J6M-3B/20250309/E371_3483_20250309_2049/2025-03-09-21-21-13-clip-20df

    # clips_path = sys.argv[1]
    # clips_path = "/home/geely/nas/J6M-4A/bag/20250404/L946_0036_20250404_1416/2024-11-20-01-51-42-clip-20df"
    batch_path = "/home/geely/nas/PLD-S6F/OCC/贵阳基地数据-12619/P181_0251_20250724_1116"
    # clips_path = "/home/geely/nas/J6M-3C/bag/20250317/E371_3483_20250317_0835/2025-03-17-08-52-06-clip-20df"

    # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])  # create coordinate frame
 
    for bag_pathi in os.listdir(batch_path):

        # if bag_pathi != "0576-2025-05-31-10-38-42-clip-20df":
        #     continue

        if not os.path.isdir(os.path.join(batch_path, bag_pathi)):
            continue
        if "clip" not in bag_pathi:
            continue
        pc_lists = []
        bag_path = os.path.join(batch_path, bag_pathi)
        clips_all = os.listdir(bag_path)
        
        for i, clip in enumerate(clips_all):
            # 跳过 pose 文件夹
            if clip == "pose":
                continue
            clips_path = os.path.join(bag_path, clip)
            flag = remove_static_frame(clips_path)


    