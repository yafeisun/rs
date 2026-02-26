import open3d as o3d
import os
import numpy as np
import json
import sys

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
        mask = (lidar_pc[:, 0] > -10000) & (lidar_pc[:, 0] < 10000) & (lidar_pc[:, 1] > -10000) & (lidar_pc[:, 1] < 10000)
        pc_list.append(lidar_pc[mask])

    return pc_list

if __name__ == "__main__":

    # command
    # python3 clips_location_checking.py /home/geely/nas/J6M-3B/20250309/E371_3483_20250309_2049/2025-03-09-21-21-13-clip-20df

    # clips_path = sys.argv[1]
    # clips_path = "/home/geely/nas/J6M-4A/bag/20250404/L946_0036_20250404_1416/2024-11-20-01-51-42-clip-20df"
    batch_path = "/home/geely/nas/PLD-S6B/P177/20250704/P177_0287_20250704_1545"
    # clips_path = "/home/geely/nas/J6M-3C/bag/20250317/E371_3483_20250317_0835/2025-03-17-08-52-06-clip-20df"


    percentage_per_bag = 1  # 单个包可视化clip百分比
    percentage_per_clips = 0.4 # 单clip可视化帧数百分比
    

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])  # create coordinate frame
 
    for bag_pathi in os.listdir(batch_path):

        # if bag_pathi != "0576-2025-05-31-10-38-42-clip-20df":
        #     continue

        if not os.path.isdir(os.path.join(batch_path, bag_pathi)):
            continue
        if "clip" not in bag_pathi:
            continue
        pc_lists = []
        bag_path = os.path.join(batch_path, bag_pathi)
        clips_all = os.listdir(bag_path)[::int(1 / percentage_per_bag)]
        
        for i, clip in enumerate(clips_all):
            clips_path = os.path.join(bag_path, clip)    
            pc_list = read_samantic_cloud(clips_path,  percentage_per_clips)
            pc_lists.extend(pc_list)
        pc_all = np.vstack(pc_lists)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_all)
        # pcd.downsample(voxel_size=0.01)
            # 可视化点云
        o3d.visualization.draw_geometries(geometry_list=[pcd, mesh_frame],
                                            window_name=f"{bag_path}, fps_num:{len(pc_lists)}, pc_num:{len(pc_all)}")

    