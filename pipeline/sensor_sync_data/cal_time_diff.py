import os
import re
import shutil
import csv
from datetime import datetime
import time
import open3d as o3d
import numpy as np
import subprocess
import multiprocessing as mp
from functools import partial
# from ../script/extract_bev_jpg import get_original_bags, get_filtered_bags

# ODD切包需求 原始包不需要切包，直接提取图片1秒1张
vision_folder_list = [
    "cam_front_right",
    "cam_back",
    "cam_side_left_front",
    "cam_side_right_front",
    "cam_side_left_back",
    "cam_side_right_back",
    "cam_front_left",
    "cam_around_front",
    "cam_around_left",
    "cam_around_right",
    "cam_around_back",
]

def find_common_images(bag_path):
    # 存储每个相机的图片时间戳列表
    timestamp_lists = {}
    for item in vision_folder_list:
        image_folder_path = os.path.join(bag_path, item)
        image_list = os.listdir(image_folder_path)
        timestamps = [int(os.path.splitext(img)[0]) for img in image_list if img.endswith('.jpg')]
        timestamps.sort()
        timestamp_lists[item] = timestamps
        # print(f"total {item} {len(timestamp_lists[item])}")

    # 读取 sync_time_list.txt 文件的第一列时间戳
    time_list_path = os.path.join(bag_path, "raw", "sync_time_list.txt")
    middle_timestamps = []
    try:
        with open(time_list_path, 'r') as f:
            # 跳过第一行标题
            next(f)
            for line in f:
                parts = line.strip().split()
                if parts:
                    middle_timestamps.append(int(parts[0]))
        middle_timestamps.sort()
    except FileNotFoundError:
        print(f"未找到同步时间列表文件: {time_list_path}")
        return {}
    except Exception as e:
        print(f"读取同步时间列表文件时出错: {e}")
        return {}

    timestamp_lists["middle"] = middle_timestamps

    # Bug fix: Create a new set that includes "middle"
    check_list = vision_folder_list + ["middle"]
    # 初始化指针
    pointers = {item: 0 for item in check_list}
    common_dict = {item: [] for item in check_list}

    while all(pointers[item] < len(timestamp_lists[item]) for item in check_list):
        # 获取当前指针指向的时间戳
        current_timestamps = {item: timestamp_lists[item][pointers[item]] for item in check_list}
        # 计算时间戳的最大值和最小值
        max_timestamp = max(current_timestamps.values())
        min_timestamp = min(current_timestamps.values())

        # 检查时间差是否在 20ms 内（20ms = 20 * 1000000 ns）
        if max_timestamp - min_timestamp <= 50 * 1000000:
            # 如果时间差在 20ms 内，将该时间戳对应的图片添加到公共字典中
            for item in check_list:
                timestamp = current_timestamps[item]
                # 根据 item 决定文件扩展名
                if item == "middle":
                    common_dict[item].append(str(timestamp) + ".pcd")
                else:
                    common_dict[item].append(str(timestamp) + ".jpg")
            # 所有指针向后移动一位
            for item in check_list:
                pointers[item] += 1
        else:
            # 如果时间差超过 20ms，将最小时间戳对应的相机指针向后移动一位
            min_item = min(current_timestamps, key=current_timestamps.get)
            pointers[min_item] += 1
    for item in check_list:
        common_dict[item].sort()
        # print(f"common {item} {len(common_dict[item])}")
    return common_dict

def select_images_per_halfsec(common_dict):
    # 把图片按照帧率抽帧，1秒2张
    selected_dict = {}
    for item, image_names in common_dict.items():
        selected_images = []
        # 记录上一个 0.5 秒的时间戳
        last_half_second = -1
        image_names.sort()
        for image in image_names:
            timestamp = int(os.path.splitext(image)[0])
            # 将时间戳转换为 0.5 秒的间隔
            current_half_second = timestamp // 500000000  

            if current_half_second > last_half_second:
                selected_images.append(image)
                last_half_second = current_half_second

        selected_dict[item] = selected_images
    return selected_dict

def select_images_per_sec(common_dict):
    selected_dict = {}
    for item, image_names in common_dict.items():
        selected_images = []
        last_second = -1
        image_names.sort()
        for image in image_names:
            timestamp = int(os.path.splitext(image)[0])
            current_sec = timestamp // 1000000000  # 将纳秒转换为秒

            if current_sec > last_second:
                selected_images.append(image)
                last_second = current_sec
        selected_dict[item] = selected_images
    return selected_dict

def copy_pcds(source_folder, target_folder, middle_pcd_names):
    source_pcd_path = os.path.join(source_folder, "raw", "pcd")
    target_pcd_path = os.path.join(target_folder, "raw", "pcd")
    if not os.path.exists(target_pcd_path):
        os.makedirs(target_pcd_path)
    time_list_path = os.path.join(source_folder, "raw", "sync_time_list.txt")

    sync_dict = {}
    if os.path.exists(time_list_path):
        try:
            # 复制 sync_time_list.txt 文件
            target_time_list_path = os.path.dirname(time_list_path.replace(source_folder, target_folder))
            shutil.copy2(time_list_path, target_time_list_path)

            # 读取 sync_time_list.txt 文件
            with open(time_list_path, 'r') as f:
                # 跳过第一行标题，标题包含 #/middle /front /left /right /back /rslidar_mx_packets /rtk_imu /rtk_gps /rtk_odom
                # 仅使用前 6 列数据，后三列数据忽略
                next(f)
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 6:
                        middle_ts = parts[0]
                        sync_dict[middle_ts] = parts[1:6]
        except Exception as e:
            print(f"处理 sync_time_list.txt 时出错: {e}")
            return
    else:
        print(f"未找到同步时间列表文件: {time_list_path}")
        return

    # 定义需要处理的文件夹名称
    other_folders = ["front", "left", "right", "back", "rslidar_mx_packets"]

    for middle_pcd in middle_pcd_names:
        middle_ts = os.path.splitext(middle_pcd)[0]
        if middle_ts in sync_dict:
            # 复制 middle 文件夹的 pcd 文件
            middle_source_path = os.path.join(source_pcd_path, "middle", middle_pcd)
            middle_target_path = os.path.join(target_pcd_path, "middle")
            if not os.path.exists(middle_target_path):
                os.makedirs(middle_target_path)
            shutil.copy2(middle_source_path, middle_target_path)

            # 复制其他文件夹的 pcd 文件
            other_timestamps = sync_dict[middle_ts]
            for folder, ts in zip(other_folders, other_timestamps):
                pcd_name = f"{ts}.pcd"
                source_path = os.path.join(source_pcd_path, folder, pcd_name)
                target_path = os.path.join(target_pcd_path, folder)
                if not os.path.exists(target_path):
                    os.makedirs(target_path)
                if os.path.exists(source_path):
                    shutil.copy2(source_path, target_path)

def cal_bp_diff(bag):
    diff = []
    sync_time_list = os.path.join(bag, "raw", "sync_time_list.txt")
    # 读取 sync_time_list.txt 文件
    with open(sync_time_list, 'r') as f:
        # 跳过第一行标题，标题包含 #/middle /front /left /right /back /rslidar_mx_packets /rtk_imu /rtk_gps /rtk_odom
        # 仅使用前 6 列数据，后三列数据忽略
        header = next(f).strip().split()

        # 建立列名到索引的映射
        column_map = {}
        for idx, col in enumerate(header):
            if col in ['#/middle', '/left', '/right', '/front', '/back']:
                column_map[col] = idx

        for line in f:
            parts = line.strip().split()
            if len(parts) >= len(header):
                middle_val = int(parts[column_map['#/middle']])
                current_diff = 0
                # 计算各方向与middle的差值
                for col in ['/left', '/right', '/front', '/back']:
                    if col in column_map:
                        val = int(parts[column_map[col]])
                        current_diff = max(current_diff, abs(val - middle_val))
                diff.append(current_diff)
    diff_val = {}
    for i in range(len(diff)):
        diff_val[i] = diff[i]
    return diff_val

def cal_diff(bag, common_dict):
    # 计算时间戳差值
    diff_bp = {}        # bp / middle
    diff_11v = {}       # 10V / cam_front_right
    diff_middle = {}    # middle / cam_front_right
    img_len = len(common_dict["cam_front_right"])
    for i in range(img_len):
        for item in ["middle"] + vision_folder_list:
            if item not in common_dict:
                continue
            if i < len(common_dict[item]):
                common_dict[item][i] = common_dict[item][i].split('.')[0]
            else:
                common_dict[item].append("0")
        diff_11v[i] = 0
        # 计算 cam_front_right 和其他相机的时间戳差值
        for item in vision_folder_list:
            if item != "cam_front_right":
                diff_11v[i] = max(diff_11v[i], abs(int(common_dict["cam_front_right"][i]) - int(common_dict[item][i])))
        # 计算 middle 和 cam_front_right 的时间戳差值
        diff_middle[i] = abs(int(common_dict["middle"][i]) - int(common_dict["cam_front_right"][i]))

        # 计算时间戳差值
        # max_timestamp = max(int(common_dict["middle"][i]), *[int(common_dict[item][i]) for item in vision_folder_list if i < len(common_dict[item])])
        # min_timestamp = min(int(common_dict["middle"][i]), *[int(common_dict[item][i]) for item in vision_folder_list if i < len(common_dict[item])])
    #     max_timestamp = max(*[int(common_dict[item][i]) for item in vision_folder_list if i < len(common_dict[item])])
    #     min_timestamp = min(*[int(common_dict[item][i]) for item in vision_folder_list if i < len(common_dict[item])])
        
    #     diff_dict[i] = max_timestamp - min_timestamp
    # avgdiffns = sum(diff_dict.values()) / len(diff_dict) if diff_dict else 0
    # avgdiffms = avgdiffns / 1000000  # 转换为毫秒
    # return avgdiffms

    diff_11v_avg = sum(diff_11v.values()) / len(diff_11v) if diff_11v else 0
    diff_11v_avg_ms = diff_11v_avg / 1000000
    diff_11v_max_ms = max(diff_11v.values()) / 1000000

    diff_middle_avg = sum(diff_middle.values()) / len(diff_middle) if diff_middle else 0
    diff_middle_avg_ms = diff_middle_avg / 1000000
    diff_middle_max_ms = max(diff_middle.values()) / 1000000

    diff_bp = cal_bp_diff(bag)
    diff_bp_avg = sum(diff_bp.values()) / len(diff_bp) if diff_bp else 0
    diff_bp_avg_ms = diff_bp_avg / 1000000
    diff_bp_max_ms = max(diff_bp.values()) / 1000000

    # return diff_11v_avg_ms, diff_middle_avg_ms, diff_bp_avg_ms
    return diff_11v_max_ms, diff_middle_max_ms, diff_bp_max_ms

def get_valid_bag_folders(target_folder):
    """
    获取目标文件夹下符合 YYYY-MM-DD-HH-MM-SS 格式的子文件夹路径
    
    :param target_folder: 目标文件夹路径
    :return: 有效子文件夹路径列表
    """
    valid_bag_dirs = []
    if not os.path.isdir(target_folder):
        print(f"错误：目标文件夹 {target_folder} 不存在！")
        return valid_bag_dirs

    for item in os.listdir(target_folder):
        item_path = os.path.join(target_folder, item)
        if os.path.isdir(item_path) and not item.startswith('.'):
            if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item):
                # if not (item)    \
                # or not os.path.isdir(os.path.join(item_path + "-clip-20df")):
                    valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")

    return valid_bag_dirs


def merge_points(index ,folder_files, sync_dict, folders):
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


def merge_pcd(bag, num_process):
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

    param = partial(merge_points, folder_files=folder_files, sync_dict=sync_dict, folders=folders)

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
                print(f"警告: 数据长度不匹配，预期7列，实际{len(data)}列。数据: {data}")
                data.append("error")  # 如果数据长度不匹配，补充0
            records.append(data)
        for data_sorted in sorted(records, key=lambda x: int(x[0])):
            f.write(' '.join(data_sorted) + '\n')

        # 按索引合并PCD文件
        
            # cmd = f"cd {bag} && bash -c pcl_concatenate_points_pcd "
            # pcds = []
            # data = []
            # data.append(str(folder_files[folder][idx]))
            # # 把sync_sensors里面index对应的timestamp写入
            # data.append(str(sync_dict[folder_files[folder][idx]]))
            # for folder in folders:
            #     file_name = f"{folder_files[folder][idx]}" + ".pcd"
            #     file_path = os.path.join(bag, "result", "test_calibration", folder, file_name)
            #     pcds.append(file_path)
            #     # cmd += f"{file_path} "
            #     # 读取PCD文件
            #     pcd = o3d.io.read_point_cloud(file_path)
            #     data.append(str(len(pcd.points)))  # 收集点云数量
            #     print(f"{len(pcd.points)}")
            # f.write(' '.join(data) + '\n')

            # output_path = os.path.join(merge_dir, file_name)
            # # cmd += f"-o '{output_path}'"
            # # cmd = f"pcl_concatenate_points_pcd {' '.join(pcds)} -o {output_path}"
            # cmd = f"cd {merge_dir} && pcl_concatenate_points_pcd {' '.join(pcds)}"
            
            # subprocess.run(cmd, shell=True, check=True)
            # os.rename(os.path.join(merge_dir, "output.pcd"), output_path)
            

def split_merged_pcd(bag):
    pcd_num = os.path.join(bag, "result", "test_calibration", 'merge', "pcd_num.txt")
    pcd_info = {}
    if os.path.exists(pcd_num):
        with open(pcd_num, 'r') as f:
            next(f)
            lines = f.readlines()
            for line in lines:
                parts = line.split()
                index = int(parts[0])
                timestamp = int(parts[1])
                pcd_info[index] = {}
                pcd_info[index]["timestamp"] = int(parts[1])
                pcd_info[index]["middle"] = int(parts[2])
                pcd_info[index]["front"] = int(parts[3])
                pcd_info[index]["back"] = int(parts[4])
                pcd_info[index]["left"] = int(parts[5])
                pcd_info[index]["right"] = int(parts[6])

    merge_pcds = os.listdir(os.path.join(bag, "result", "test_calibration", 'merge'))
    merge_pcds = [pcd for pcd in merge_pcds if pcd.endswith('.pcd')]
    for merge_pcd in merge_pcds:
        pcd_path = os.path.join(bag, "result", "test_calibration", 'merge', merge_pcd)
        pcd = o3d.io.read_point_cloud(pcd_path)
        index = int(merge_pcd.split('.')[0])
        point_index = 0
        for item in ['middle', 'front', 'back', 'left', 'right']:
            # 从pcd中依次取出pcd_info[index][item]个点，保留强度信息，另存为新的pcd文件
            pcd_item = pcd.select_by_index(range(point_index, point_index + pcd_info[index][item]), invert=False)
            
            # indices = range(point_index, point_index + pcd_info[index][item])            
            # # Convert Open3D point cloud data to numpy arrays
            # points = np.asarray(pcd.points)[indices]
            # intensities = np.asarray(pcd.colors)[indices]  # Assumes intensity is stored in colors
            
            # # Create new point cloud and assign selected data
            # pcd_item = o3d.geometry.PointCloud()
            # pcd_item.points = o3d.utility.Vector3dVector(points)
            # pcd_item.colors = o3d.utility.Vector3dVector(intensities)  # Preserve intensity




            point_index += pcd_info[index][item]
            # 构建新的输出文件名，添加方向标识
            base_name = os.path.splitext(merge_pcd)[0]
            new_filename = f"{base_name}_{item}.pcd"
            new_pcd_path = os.path.join(os.path.dirname(os.path.dirname(pcd_path)), "split", base_name, new_filename)
            os.makedirs(os.path.dirname(new_pcd_path), exist_ok=True)
            o3d.io.write_point_cloud(new_pcd_path, pcd_item, write_ascii=True)

if __name__ == "__main__":
    dir_root = "/home/geely/PLD-S6F/bag/J6M-S23"
    num_process = 8   # 多进程数量
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250717")
    # day_dirs.append("20250718")
    
    vin_codes = []
    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            if vin_codes:
                car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)

    # abs_car_dirs = []
    # abs_car_dirs.append("/home/geely/nas/PLD-S6B/J6M-S08/20250612/J6M_0302_20250612_1016")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))
        # total_bag_dirs.extend(get_night_bag_folders(dir))
        # total_bag_dirs.extend(get_day_bag_folders(dir))
    # 指定处理某个bag文件夹
    # total_bag_dirs = []
    # total_bag_dirs.extend(get_filtered_bags("/home/geely/nas/PLD-S6C/J6M-S07", "/home/geely/nas/PLD-S6B/OCC/0721-OD-OCC-zhaji/BD-33-83-9450-5220-zj"))      # 33 E245-S07
    # total_bag_dirs.extend(get_filtered_bags("/home/geely/nas/PLD-S6D/J6M-S08", "/home/geely/nas/PLD-S6B/OCC/0721-OD-OCC-zhaji/BD-34-84-S08-S23-2155-zj"))   # 34 P181-S08
    # total_bag_dirs.extend(get_filtered_bags("/home/geely/nas/PLD-S6B/J6M-S23", "/home/geely/nas/PLD-S6B/OCC/0721-OD-OCC-zhaji/BD-34-84-S08-S23-2155-zj"))   # 34 P181-S23
    # total_bag_dirs.extend(get_filtered_bags("/home/geely/nas/PLD-S6D/J6M-S23", "/home/geely/nas/PLD-S6B/OCC/0721-OD-OCC-zhaji/ZHAJI-OCC-P181-S23/BD-35-87-0251-1871"))  # 35 P181-S23
    # total_bag_dirs.extend(get_filtered_bags("/home/geely/nas/PLD-S6D/J6M-S24", "/home/geely/nas/PLD-S6B/OCC/0721-OD-OCC-zhaji/ZHAJI-P181-S24/BD-37-90-0252-1798"))      # 37 P181-S24
    # total_bag_dirs.append("/home/geely/nas/PLD-S6D/J6M-S08/20250729/J6M_0302_20250729_1706/2025-07-29-17-28-05")
    # bag_dir = "/home/geely/nas/PLD-S6D/J6M-S23/20250712/P181_0251_20250712_1519/2025-07-12-15-32-15"
    total_bag_dirs.sort()
    print(f'bag  10v/cam_front-right  middle/cam_front-right  bp/middle')
    for bag in total_bag_dirs:
        common_images = {}
        common_images = find_common_images(bag)
        if common_images:
            diff_11v, diff_middle, diff_bp = cal_diff(bag, common_images)
            print(f"{bag}: {diff_11v:.2f}ms {diff_middle:.2f}ms {diff_bp:.2f}ms")
        else:
            print(f"time sync fail {bag}")
        # merge_pcd(bag, num_process)
        # split_merged_pcd(bag)
    



