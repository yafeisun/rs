import os
import open3d as o3d
import numpy as np
import json
import re
import subprocess

def extract_frame_OD(bags):
    # jiaxin写的7V周视去畸变 抽帧1s2帧，在bag同目录输出结果-df结尾
    valid_bags = bags
    if valid_bags:
        script_path = "/home/geely/jiaxin/EA-LSS_extract_frame_version_bag/run_clip.sh"
        command = f"cd {os.path.dirname(script_path)} && ./run_clip.sh {' '.join(valid_bags)}"
        subprocess.run(["bash", "-c", command], check=True)



def lidar_transfer(pc_input, trans_src, trans_dst=None):
    pc = pc_input.copy()
    points = pc[:, :3]
    one_array = np.ones((points.shape[0], 1))
    points = np.hstack((points, one_array)).T
    if trans_dst is not None:
        points = (np.linalg.inv(trans_dst) @ (trans_src @ points)).T
    else:
        points = (trans_src @ points).T
    pc[:, :3] = points[:, :3]
    return pc

def read_samantic_cloud(label_seman_dir, percentage_per_clips):
    pc_lists = os.listdir(label_seman_dir)
    pc_lists = [i for i in pc_lists if os.path.isdir(os.path.join(label_seman_dir, i)) and i.isdigit()]
    pc_lists = sorted(pc_lists)[::int(1 / percentage_per_clips)]
    trans_0 = None
    pc_list = []

    for timestamp in pc_lists:
        if "cam" in timestamp:
            continue
        loc_file = os.path.join(label_seman_dir, timestamp, timestamp + '__desc.json')
        if not os.path.exists(loc_file):
            print(f"{loc_file} is not exists!")
            continue
        with open(loc_file) as f:
            loc_content = json.load(f)
        trans = loc_content["desc"]["ego2global"]
        if trans_0 is None:
            trans_0 = trans

        pc_file = os.path.join(label_seman_dir, timestamp, timestamp + '.pcd')
        if not os.path.exists(pc_file):
            print("not exists ", pc_file)
            continue
        pcd = o3d.io.read_point_cloud(pc_file)

        lidar_pc = lidar_transfer(np.asarray(pcd.points), trans)
        mask = (lidar_pc[:, 0] > -10000) & (lidar_pc[:, 0] < 10000) & (lidar_pc[:, 1] > -10000) & (lidar_pc[:, 1] < 10000)
        pc_list.append(lidar_pc[mask])

    return pc_list


def hecheng(bag):
    clips = os.listdir(bag)
    clips = [d for d in clips if os.path.isdir(os.path.join(bag, d))]
    clips.sort()
    combined_pcd = o3d.geometry.PointCloud()
    pc_lists = []
    for clip in clips:
        clip_path = os.path.join(bag, clip)    
        pc_list = read_samantic_cloud(clip_path, 1)
        pc_lists.extend(pc_list)

    pc_all = np.vstack(pc_lists)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_all)
        # frames = os.listdir(os.path.join(bag, clip))
        # frames = [d for d in frames if os.path.isdir(os.path.join(bag, clip, d)) and d.isdigit()]
        # for frame in frames:
        #     pcd_file = os.path.join(bag, clip, frame, frame + ".pcd")
        #     if os.path.exists(pcd_file):
        #         pcd = o3d.io.read_point_cloud(pcd_file)
        #         combined_pcd += pcd
    if not pcd.is_empty():        
        o3d.io.write_point_cloud(os.path.join(bag, "middle.pcd"), pcd)

def combine_removeObj(bag):
    pc_lists = []
    pcd_path = os.path.join(bag, "result", "remove_obj")    
    pc_list = read_samantic_cloud(pcd_path, 1)
    pc_lists.extend(pc_list)

    pc_all = np.vstack(pc_list)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_all)

    if not pcd.is_empty():        
        o3d.io.write_point_cloud(os.path.join(bag, "result", "remove_obj.pcd"), pcd)

od_frame_path = "/home/geely/nas/PLD-S6B/J6M-S08/20250612/J6M_0302_20250612_1136-x"
bags = os.listdir(od_frame_path)
# bags = [d for d in bags if d.endswith("-clip-20df")]
bags = [d for d in bags if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', d)]
bags = []
bags.append("/home/geely/nas/PLD-S6B/J6M-S08/20250612/J6M_0302_20250612_1016-x/2025-06-12-10-28-00")
bags.sort()
extract_frame_OD(bags)
for bag in bags:
    print(bag)
    if not os.path.exists(os.path.join(od_frame_path, bag + "-clip-20df", "middle.pcd")):
        hecheng(os.path.join(od_frame_path, bag + "-clip-20df"))
    # if not os.path.exists(os.path.join(od_frame_path, bag, "result", "remove_obj.pcd")):
        # combine_removeObj(os.path.join(od_frame_path, bag))



    