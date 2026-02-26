import os
import re
import shutil
import csv
from datetime import datetime
import time
from find_bag_by_tag import get_pld_bags

# ODD切包需求 原始包不需要切包，直接提取图片1秒1张
vision_folder_list = {
    "cam_around_back",
    "cam_around_front",
    "cam_around_left",
    "cam_around_right",
    # "cam_back",
    # "cam_front_left",
    "cam_front_right",
    # "cam_side_left_back",
    # "cam_side_left_front",
    # "cam_side_right_back",
    # "cam_side_right_front",
}

def check_vision_folders(bag_folder):
    existing_folders = [name for name in os.listdir(bag_folder)
                        if os.path.isdir(os.path.join(bag_folder, name))]
    for item in vision_folder_list:
        if item not in existing_folders:
            print("{bag_folder} {item} folder lost")
            return False
    return True

def get_images_from_clips(bag_path):
    # 做不到所有路相机每帧对齐，放弃该方案
    imagelist = [[] for _ in range(len(vision_folder_list))]
    data_dict = dict(zip(vision_folder_list, imagelist))

    clips = os.listdir(os.path.join(bag_path, "clips"))
    clips.sort(key=lambda x: int(x.split('_')[-1]))
    for clip in clips:
        for item in vision_folder_list:
            image_folder_path = os.path.join(bag_path, "clips", clip, item)
            current_imagelist = os.listdir(image_folder_path)
            data_dict[item].extend(current_imagelist)
    for item in vision_folder_list:
        data_dict[item] = list(set(data_dict[item]))
        print(f"{item} {len(data_dict[item])}")
    return data_dict

def write_to_file(bag_path, common_dict, filename):
    csv_path = os.path.join(bag_path, filename)
    # 新增列名
    new_fieldnames = list(vision_folder_list) + ['min_timestamp', 'max_timestamp', 'timestamp_diff']
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=new_fieldnames)
        writer.writeheader()
        rows = list(zip(*[common_dict[item] for item in vision_folder_list]))
        for row in rows:
            # 去掉 .jpg 后缀并转换为整数
            row = [int(os.path.splitext(img)[0]) for img in row]
            min_timestamp = min(row)
            max_timestamp = max(row)
            timestamp_diff = max_timestamp - min_timestamp
            row_dict = dict(zip(vision_folder_list, row))
            # 添加新增列的数据
            row_dict['min_timestamp'] = '{:d}'.format(min_timestamp)
            row_dict['max_timestamp'] = '{:d}'.format(max_timestamp)
            row_dict['timestamp_diff'] = '{:d}'.format(timestamp_diff)
            writer.writerow(row_dict)

def find_common_images(bag_path):
    # 存储每个相机的图片时间戳列表
    timestamp_lists = {}
    for item in vision_folder_list:
        image_folder_path = os.path.join(bag_path, item)
        # timestamps = [int(os.path.splitext(img)[0]) for img in os.listdir(image_folder_path) if '.@__thumb' not in img]
        image_list = os.listdir(image_folder_path)
        for img in image_list:
            if "复制" in img:
                os.remove(os.path.join(image_folder_path, img))
        image_list = os.listdir(image_folder_path)
        timestamps = []
        # timestamps = [int(os.path.splitext(img)[0]) for img in os.listdir(image_folder_path) if img != '.@__thumb']
        timestamps = [int(os.path.splitext(img)[0]) for img in image_list if img.endswith('.jpg')]
        timestamps.sort()
        timestamp_lists[item] = timestamps
        # print(f"total {item} {len(timestamp_lists[item])}")

    # 初始化指针
    pointers = {item: 0 for item in vision_folder_list}
    common_dict = {item: [] for item in vision_folder_list}

    while all(pointers[item] < len(timestamp_lists[item]) for item in vision_folder_list):
        # 获取当前指针指向的时间戳
        current_timestamps = {item: timestamp_lists[item][pointers[item]] for item in vision_folder_list}
        # 计算时间戳的最大值和最小值
        max_timestamp = max(current_timestamps.values())
        min_timestamp = min(current_timestamps.values())

        # 检查时间差是否在 20ms 内（20ms = 20 * 1000000 ns）
        if max_timestamp - min_timestamp <= 100 * 1000000:
            # 如果时间差在 20ms 内，将该时间戳对应的图片添加到公共字典中
            for item in vision_folder_list:
                timestamp = current_timestamps[item]
                common_dict[item].append(str(timestamp) + ".jpg")
            # 所有指针向后移动一位
            for item in vision_folder_list:
                pointers[item] += 1
        else:
            # 如果时间差超过 20ms，将最小时间戳对应的相机指针向后移动一位
            min_item = min(current_timestamps, key=current_timestamps.get)
            pointers[min_item] += 1
    for item in vision_folder_list:
        common_dict[item].sort()
        # print(f"common {item} {len(common_dict[item])}")
    return common_dict

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

def copy_images(source_folder, target_folder, image_names):
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
    for image in image_names:
        source_path = os.path.join(source_folder, image)
        target_path = os.path.join(target_folder, image)
        shutil.copy2(source_path, target_path)

def frame_extraction(source_bag_dir, target_dir):
    """
    并在 target_dir 下创建同名文件夹，然后在其中创建同名文件夹，"""
    print(f"extract {source_bag_dir}")
    start_time_total = time.time()
    common_images = find_common_images(source_bag_dir)
    # write_to_file(source_bag_dir, common_images, "common_images.csv")

    selected_images = select_images_per_halfsec(common_images)
    # write_to_file(source_bag_dir, selected_images, "selected_dict.csv")

    clip_index = 0
    for item in vision_folder_list:
        index = 0
        for image in selected_images[item]:
            clip_index = str(int(index / 40))
            index = index + 1
            target_folder = os.path.join(target_dir, os.path.basename(source_bag_dir) + '_' + clip_index, item)
            if not os.path.exists(target_folder):
                os.makedirs(target_folder)
            source_path = os.path.join(source_bag_dir, item, image)
            shutil.copy2(source_path, os.path.join(target_folder, os.path.basename(source_path)))

    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"consumed time: {total_duration:.2f} s")


def extract_bag(daily_dir, target_dir, VIN_CODE):
    # 获取 daily_dir 的文件夹名称
    daily_dir_name = os.path.basename(daily_dir)
    # 在 target_dir 下创建同名文件夹
    new_daily_path = os.path.join(target_dir, daily_dir_name)
    if not os.path.exists(new_daily_path):
        os.makedirs(new_daily_path)

    dirs = os.listdir(daily_dir)
     # 筛选出以下划线分割的第二个元素为VIN_CODE的目录
    # dirs = [d for d in dirs if len(d.split('_')) > 1 and d.split('_')[1] == VIN_CODE]
    # dirs = [d for d in dirs if "-zw" in d]
    dirs.sort()
    abs_car_paths = [os.path.abspath(os.path.join(daily_dir, d)) for d in dirs]

    for car_path in abs_car_paths:
        car_dir_name = os.path.basename(car_path)
        # 在 car_path 下创建同名文件夹
        new_car_path = os.path.join(new_daily_path, car_dir_name)
        if not os.path.exists(new_car_path):
            os.makedirs(new_car_path)
        bag_dirs = os.listdir(car_path)
        # 使用正则表达式匹配 YYYY-MM-DD-HH-MM-SS 格式
        date_pattern = re.compile(r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$')
        bag_dirs = [d for d in bag_dirs if date_pattern.match(d)]
        # 检查当前采集包11路相机是否缺路数
        bag_dirs = [d for d in bag_dirs if check_vision_folders(os.path.abspath(os.path.join(car_path, d)))]
        bag_dirs.sort()
        abs_bag_paths = [os.path.abspath(os.path.join(car_path, d)) for d in bag_dirs]
        
        for bag_path in abs_bag_paths:
        # 创建bag同名文件夹
            new_bag_path = os.path.join(new_car_path, os.path.basename(bag_path))
            if not os.path.exists(new_bag_path):
                os.makedirs(new_bag_path)
                frame_extraction(bag_path, new_bag_path)
    

if __name__ == "__main__":
    target_root = "/home/geely/nas/PLD-S6B/S07-OCC-img-nas-D"
    if not os.path.exists(target_root):
        os.makedirs(target_root)

    bag_root = "/home/geely/nas/PLD-S6D/J6M-S07"
    total_bag_dirs = get_pld_bags(bag_root)
    for bag in total_bag_dirs:
        car_dir = os.path.dirname(bag)
        target_dir = os.path.join(target_root, os.path.basename(car_dir), os.path.basename(bag))
        if not os.path.isdir(target_dir):
            os.makedirs(target_dir)
            frame_extraction(bag, target_dir)

    # bag_path = "/media/geely/sda/20250304/E371_3483_20250304_1355/2025-03-04-13-58-38"
    # new_bag_path = "/media/geely/sdb/frame_3384_bag/2025-03-04-13-58-38"
    # frame_extraction(bag_path, new_bag_path)


