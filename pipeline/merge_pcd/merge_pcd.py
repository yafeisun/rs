import os
import shutil
import open3d as o3d
import numpy as np
import multiprocessing as mp
from functools import partial
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description='merge pcd from different lidar in a bag')
    parser.add_argument('--InputPath', nargs="+", default="", help='train config file path')
    # parser.add_argument('--OutputPath', default="/home/robosense/Documents/yafeisun/0115_OD/2025-01-15-17-11-06", help='train config file path')

    args = parser.parse_args()
    return args

def merge_points(index, bag, folder_files, sync_dict, folders):
    data = []
    data.append(str(index + 1))
    data.append(str(sync_dict[folder_files[folders[0]][index]]))
    middle_name = f"{folder_files['middle'][index]}" + ".pcd"
    middle_path = os.path.join(bag, "result", "test_calibration", 'middle', middle_name)
    save_pcd = o3d.t.io.read_point_cloud(middle_path)
    middle_points_num = str(len(save_pcd.point['positions']))
    data.append(middle_points_num)
    for lidar_name in folders[1:]:
        file_name = f"{folder_files[lidar_name][index]}" + ".pcd"
        file_path = os.path.join(bag, "result", "test_calibration", lidar_name, file_name)
        lidar_pcd = o3d.t.io.read_point_cloud(file_path)
        points_num = str(len(lidar_pcd.point['positions']))
        data.append(str(points_num))
        for key, tensor in lidar_pcd.point.items():
            save_pcd.point[key] = o3d.core.concatenate([save_pcd.point[key], tensor], axis=0)
    save_path = os.path.join(bag, "result", "test_calibration", "merge", str(index + 1) + '.pcd')
    o3d.t.io.write_point_cloud(save_path, save_pcd)
    return data

def merge_pcd(bag, num_process=60):
    # 创建merge文件夹
    merge_dir = os.path.join(bag, "result", "test_calibration", 'merge')
    os.makedirs(merge_dir, exist_ok=True)
    
    # 获取所有文件夹的PCD文件列表
    folder_files = {}
    max_index = None
    
    # 定义文件夹顺序和对应的索引位置
    folders = ["middle", "front", "back", "left", "right"]
    pcd_num = os.path.join(bag, "result", "test_calibration", 'merge', "pcd_num.txt")

    # 收集所有文件夹中的PCD文件并排序
    for folder in folders:
        folder_path = os.path.join(bag, "result", "test_calibration", folder)
        if not os.path.exists(folder_path):
            print(f"警告: 文件夹 {folder_path} 不存在")
            return
        
        # 获取所有PCD文件并按名称排序
        pcd_files = [f for f in os.listdir(folder_path) if f.endswith('.pcd')]
        pcd_files = [int(f.split('.')[0]) for f in pcd_files]
        pcd_files.sort()
        folder_files[folder] = pcd_files
        
        # 确定最小文件数量以对齐索引
        if max_index is None or len(pcd_files) < max_index:
            max_index = len(pcd_files)

    # 读取middle/sync_sensors.txt 保存第一列index和第二列timestamp
    sync_sensors = os.path.join(bag, "result", "test_calibration", "sync_sensors.txt")
    sync_dict = {}  # 创建字典用于index到timestamp的映射
    if os.path.exists(sync_sensors):
        with open(sync_sensors, 'r') as f:
            next(f)
            lines = f.readlines()
            for line in lines:
                line = line.strip()
                if not line:  # 跳过空行
                    continue
                parts = line.split()  # 使用默认split()处理任意数量空格
                if len(parts) < 2:  # 确保至少有两列数据
                    continue
                try:
                    index = int(parts[0])
                    timestamp = int(parts[1])
                    sync_dict[index] = timestamp  # 建立index到timestamp的映射
                except ValueError:
                    continue  # 跳过格式错误的行

    param = partial(merge_points, bag=bag, folder_files=folder_files, sync_dict=sync_dict, folders=folders)

    # 写入pcd_num.txt文件
    with open(pcd_num, 'w') as f:
        # 写入头部
        header = ["index", "timestamp", "middle", "front", "back", "left", "right"]
        f.write(' '.join(header) + '\n')

        with mp.Pool(processes=num_process) as pool:
            results = pool.map(param, range(max_index))
        records = []
        for data in results:
            if len(data) != 7:
                print(f"警告: 数据长度不匹配, 预期7列, 实际{len(data)}列。数据: {data}")
                data.append("error")  # 如果数据长度不匹配，补充0
            records.append(data)
        for data_sorted in sorted(records, key=lambda x: int(x[0])):
            f.write(' '.join(data_sorted) + '\n')

if __name__ == '__main__':
    args = parse_args()
    bags = args.InputPath
    # bags = []
    # bags.append("/home/geely/nas/PLD-S6D/J6M-S24/20250730/P181_0252_20250730_1243/2025-07-30-12-44-57")
    for bag_path in bags:
        if os.path.isdir(bag_path):
            print(f"Merge pcd: {bag_path}")
            merge_pcd(bag_path)
            middle_sync = os.path.join(bag_path, "result", "test_calibration", "middle", "sync_sensors.txt")
            if os.path.exists(middle_sync):
                shutil.copy(middle_sync, os.path.join(bag_path, "result", "test_calibration", "merge", "sync_sensors.txt"))
            for folder in ["middle", "front", "back", "left", "right"]:
                folder_path = os.path.join(bag_path, "result", "test_calibration", folder)
                if os.path.exists(folder_path):
                    shutil.rmtree(folder_path)
            print(f"Merge completed for: {bag_path}")
            os.rename(os.path.join(bag_path, "result", "test_calibration", "merge"), os.path.join(bag_path, "result", "test_calibration", "middle"))
        else:
            print(f"Warning: {bag_path} is not a valid directory.")
