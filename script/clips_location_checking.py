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
        pc_list.append(lidar_pc)

    return pc_list

if __name__ == "__main__":

    # command
    # python3 /home/geely/Documents/robosense/script/clips_location_checking.py /home/geely/nas/J6M-3B/20250309/E371_3483_20250309_1810/2025-03-09-18-37-45-clip-20df

    # clips_path = sys.argv[1]
    clips_path = '/home/geely/nas/J6M-4A/bag/20250325/L946_0143_20250325_1339/2025-03-25-13-58-42-clip-20df'
    percentage_per_bag = 1  # 单个包可视化clip百分比
    percentage_per_clips = 1  # 单clip可视化帧数百分比

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])  # create coordinate frame

    clips_all = os.listdir(clips_path)[::int(1 / percentage_per_bag)]
    for i, slip in enumerate(clips_all):
        # if slip != '2025-03-16-19-00-06_12':
        #     continue
        pc_lists = read_samantic_cloud(os.path.join(clips_path, slip), percentage_per_clips)
        pc_all = np.vstack(pc_lists)
        print(f"vis clip {i}:{os.path.join(clips_path, slip)}, fps_num:{len(pc_lists)}, pc_num:{len(pc_all)}")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_all)

        # 可视化点云
        o3d.visualization.draw_geometries(geometry_list=[pcd, mesh_frame],
                                          window_name=f"{i}/{len(clips_all)},{slip}, fps_num:{len(pc_lists)}, pc_num:{len(pc_all)}")

        # 使用Visualizer可视化点云，设置点云大小
        # vis = o3d.visualization.Visualizer()
        # vis.create_window(window_name=f"{i}/{len(clips_all)},{slip}, fps_num:{len(pc_lists)}, pc_num:{len(pc_all)}")  # 设置窗口标题
        # vis.get_render_option().point_size = 0.5  # 设置点云大小
        # opt = vis.get_render_option()
        # opt.background_color = np.array([0, 0, 0])  # 设置颜色背景为黑色
        # vis.add_geometry(mesh_frame)
        # vis.add_geometry(pcd)

        # vis.run()
        # vis.destroy_window()