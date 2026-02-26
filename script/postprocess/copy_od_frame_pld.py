import os
import shutil
from datetime import datetime
import time
import subprocess
import re
# OD抽帧需求 原始包不需要切包，直接提取图片1秒1张

def copy_pld_clip(src, dst):
    start_time_total = time.time()
    img_bag = src.replace("bag", "img").replace("-clip-20df", "")
    img_clips = os.listdir(img_bag)
    clips = os.listdir(src)
    clips = [d for d in clips if os.path.isdir(os.path.join(src, d))]
    for img_clip in img_clips:
        for clip in clips:
            if img_clip == clip and os.path.isdir(os.path.join(src, clip)):
                try:
                    print(f"cp {clip} to {dst}")
                    subprocess.run(["cp", "-rp", os.path.join(src, clip), dst], check=True)
                except Exception as e:
                    print(f"cp failed: {e}")


    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"consumed time: {total_duration:.2f}s")

def get_total_clip_folders(bag_dirs):
    valid_clips = []
    clip_num = 0
    for bag_dir in bag_dirs:
        clips_path = os.path.join(bag_dir, "clips")
        if os.path.isdir(clips_path):
            # 遍历 clips 子文件夹下的一级子文件夹
            for item in os.listdir(clips_path):
                clip_num += 1
                item_path = os.path.join(clips_path, item)
                if os.path.isdir(item_path):
                    valid_clips.append(item_path)
    valid_clips.sort()
    return valid_clips

def is_night_bag(bag_dir):
    """
    判断是否为晚包，晚包的时间戳在10点之后
    :param bag_dir: 包路径"""    

    bag_hour = int(bag_dir.split('-')[3])
    bag_minute = int(bag_dir.split('-')[4])
    if bag_hour * 100 + bag_minute >= 1830:
        return True
    else:
        return False
    
def get_day_bag_folders(target_folder):
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
                if not is_night_bag(item):
                    valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")
    return valid_bag_dirs

def get_night_bag_folders(target_folder):
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
                if is_night_bag(item):
                    valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")
    return valid_bag_dirs

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
                valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"bag folder format invalid:{item_path}")

    return valid_bag_dirs

def extract_od_frame(car_path, target_root):
    car_dir_name = os.path.basename(car_path)
    # 在 car_path 下创建同名文件夹
    new_car_path = os.path.join(target_root, car_dir_name)
    car_path = car_path.replace("img", "bag")
    bag_dirs = os.listdir(car_path)
    # 匹配-clip-20df的OD抽帧文件夹
    bag_dirs = [d for d in bag_dirs if d.endswith('-clip-20df')]
    bag_dirs.sort()
    abs_bag_paths = [os.path.abspath(os.path.join(car_path, d)) for d in bag_dirs]
    
    for item in abs_bag_paths:
        clips = os.listdir(item)
        clips = [d for d in clips if os.path.isdir(os.path.join(item, d))]
        if not os.path.exists(new_car_path):
            os.makedirs(new_car_path)
        new_bag_path = os.path.join(new_car_path, os.path.basename(item))
        if not os.path.exists(new_bag_path):
            os.makedirs(new_bag_path)
        copy_pld_clip(item, new_bag_path)

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    dir_root = "/home/geely/nas/PLD-S6A/img"
    target_root = "/home/geely/nas/PLD-S6A/PLD-OD/20250605"
    if not os.path.exists(target_root):
        os.makedirs(target_root)
    
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    day_dirs = []
    # day_dirs.append("20250531")
    day_dirs.append("20250601")
    day_dirs.append("20250602")

    vincodes = []
    vincodes.append("0576")

    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            if vincodes:
                car_dirs = [d for d in car_dirs if d.split('_')[1] in vincodes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
    # abs_car_dirs = []
    # abs_car_paths.append("/home/robosense/nas/J6M-3D/bag/20250326/E371_6477_20250326_1840")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-3C/bag/20250306-zw/E371_6477_20250306_1930/2025-03-06-19-45-58")
    
    for item in abs_car_dirs:
        extract_od_frame(item, target_root)

