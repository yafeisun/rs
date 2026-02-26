import open3d as o3d
import os
import numpy as np
import json
import sys
import multiprocessing as mp
from functools import partial

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

def read_samantic_cloud(label_seman_dir, percentage_per_clips, hight_limit):
    pc_lists = os.listdir(label_seman_dir)
    pc_lists = [i for i in pc_lists if os.path.isdir(os.path.join(label_seman_dir, i))]
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
        mask = (lidar_pc[:, 0] > -10000) & (lidar_pc[:, 0] < 10000) & (lidar_pc[:, 1] > -10000) & (lidar_pc[:, 1] < 10000) & (lidar_pc[:, 2] < hight_limit)
        pc_list.append(lidar_pc[mask])

    return pc_list

if __name__ == "__main__":

    # command
    # python3 clips_location_checking.py /home/geely/nas/J6M-3B/20250309/E371_3483_20250309_2049/2025-03-09-21-21-13-clip-20df

    # clips_path = sys.argv[1]
    # clips_path = "/home/geely/nas/PLD-S6D/OCC/P181-地锁-S24/P181_0252_20250729_2118"
    # batch_path = "/home/geely/nas/J6M-S6C/1138/20251125/1138_0143_20251125_1200"
    batch_path = "/home/geely/nas/J6M-S6C/真值bag/0109p/20251126"
    
    
    # batch_path = "/media/geely/sda/4D-OD/J6M-3/SJT-4-20-4514_3484"
    # batch_path = "/media/geely/sdb/4D-OD-PCD-json/J6M-3"
    # batch_path = "/home/geely/nas/J6M-5B/bag/20250508/L946_0036_20250508_1948"
    # batch_path='/home/geely/nas/PLD-S6B/OCC/P177-OCC'

    percentage_per_bag = 1  # 单个包可视化clip百分比
    percentage_per_clips = 0.4 # 单clip可视化帧数百分比
    next_bag_name = ""   # 可视化中断后，从哪个包开始可视化，不使用该功能时值为空字符串“”
    hight_limit = 100 # 地库场景去掉天花板 0.8   地上场景改为100
    num_process = 32 # 加载点云进程数量

    # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])  # create coordinate frame
    next_visual_flag = False
    func = partial(read_samantic_cloud, percentage_per_clips=percentage_per_clips, hight_limit=hight_limit)
    bath_list_sorted = sorted(os.listdir(batch_path))

    if next_bag_name == "":
        next_bag_name = bath_list_sorted[0]
    
    for bag_pathi in bath_list_sorted:

        if bag_pathi == next_bag_name:
            next_visual_flag = True
        if not next_visual_flag:
            print(f"INFO: skip {bag_pathi} !")
            continue

        # if bag_pathi != "2025-11-25-12-55-41-clip-20df":
        #     continue

        if not os.path.isdir(os.path.join(batch_path, bag_pathi)):
            continue
        if "clip" not in bag_pathi:
            print(f"INFO: skip {bag_pathi} !")
            continue
        pc_lists = []
        bag_path = os.path.join(batch_path, bag_pathi)
        clips_all = os.listdir(bag_path)[::int(1 / percentage_per_bag)]
        clips_path_list = []
        
        for i, clip in enumerate(clips_all):
            if "2025" not in clip:
                continue
            clips_path = os.path.join(bag_path, clip)  
            clips_path_list.append(clips_path)
        
        with mp.Pool(processes=num_process) as pool:
            results = pool.map(func, clips_path_list)
        for pc in results:
            pc_lists.extend(pc)
            # pc_list = read_samantic_cloud(clips_path,  percentage_per_clips)
            # pc_lists.extend(pc_list)
        # pc_lists = pc_lists[10:]
        pc_all = np.vstack(pc_lists)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_all)
        # pcd.downsample(voxel_size=0.01)
            # 可视化点云
        # o3d.io.write_point_cloud("test.pcd", pcd)
        o3d.visualization.draw_geometries_with_editing(geometry_list=[pcd],
                                            window_name=f"{bag_path}, fps_num:{len(pc_lists)}, pc_num:{len(pc_all)}")

    
