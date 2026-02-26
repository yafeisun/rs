import os
import shutil
from datetime import datetime
import time
import subprocess
import re
# OD抽帧需求 原始包不需要切包，直接提取图片1秒1张

def copy_cc_clip(src, dst):
    """
    并在 target_dir 下创建同名文件夹，然后在其中创建同名文件夹，"""
    start_time_total = time.time()
    clips = os.listdir(src)
    clips = [d for d in clips if os.path.isdir(os.path.join(src, d))]
    for clip in clips:
        # if clip.endswith("-cc"):
            if os.path.isdir(os.path.join(src, clip)) and "-cc" in clip and not os.path.isdir(os.path.join(dst, clip)):
                try:
                    # cp cc clip
                    print(f"cp {clip} to {dst}")
                    subprocess.run(["cp", "-rp", os.path.join(src, clip), dst], check=True)                
                except Exception as e:
                    print(f"cp failed: {e}")
            elif not "-cc" in clip and os.path.isdir(os.path.join(dst, clip)):
                print(f"remove {os.path.join(dst, clip)}")
                # shutil.rmtree(os.path.join(dst, clip))            
    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"consumed time: {total_duration:.2f}s")

def get_cc_clip_folders(bag_dirs):
    total_clips = get_total_clip_folders(bag_dirs)
    clips = [d for d in total_clips if "-cc" in d]
    print(f"total clip {len(total_clips)} cc clip {len(clips)}")
    return clips

def check_clip(clip):
    jpg_counts = []
    cam_folders = [f for f in os.listdir(clip) 
                if f.startswith('cam_') and os.path.isdir(os.path.join(clip, f))]
    for cam_folder in cam_folders:
        cam_path = os.path.join(clip, cam_folder)
        jpg_files = [f for f in os.listdir(cam_path) if f.endswith('.jpg')]
        jpg_counts.append(len(jpg_files))
    
    if not jpg_counts:
        return False
    
    max_count = max(jpg_counts)
    min_count = min(jpg_counts)
    record_time = 20
    if min_count < 10 * record_time:
        print(f"{clip} jpg num too small {min_count} < 90% * {10 * record_time}")        
        # remove_folder(clip)
        return False
    return True

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
                if os.path.isdir(item_path) and check_clip(item_path):
                    valid_clips.append(item_path)
        else:
            print(f"警告: {bag_dir}/clips 文件夹不存在！跳过...")
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

def extract_od_frame(car_path, target_root, period):
    car_dir_name = os.path.basename(car_path)
    # 在 car_path 下创建同名文件夹
    new_car_path = os.path.join(target_root, car_dir_name)

    bag_dirs = os.listdir(car_path)
    # 匹配-clip-20df的OD抽帧文件夹
    bag_dirs = [d for d in bag_dirs if d.endswith('-clip-20df')]
    if period == "day":
        bag_dirs = [d for d in bag_dirs if not is_night_bag(d)]
    elif period == "night":
        bag_dirs = [d for d in bag_dirs if is_night_bag(d)]
    bag_dirs.sort()
    abs_bag_paths = [os.path.abspath(os.path.join(car_path, d)) for d in bag_dirs]
    
    for item in abs_bag_paths:
        cc_clip = False
        clips = os.listdir(item)
        clips = [d for d in clips if os.path.isdir(os.path.join(item, d))]
        for clip in clips:
            if os.path.isdir(os.path.join(item, clip)) and "-cc" in clip:
                cc_clip = True
                break
        if cc_clip:
            if not os.path.exists(new_car_path):
                os.makedirs(new_car_path)
            new_bag_path = os.path.join(new_car_path, os.path.basename(item))
            if not os.path.exists(new_bag_path):
                os.makedirs(new_bag_path)
            copy_cc_clip(item, new_bag_path)

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    dir_root = "/home/geely/nas/J6M-3C/bag"
    target_root = "/home/geely/nas/J6M-3C/frame_od_0415_day"
    if not os.path.exists(target_root):
        os.makedirs(target_root)
    
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250307")

    vincodes = []
    vincodes.append("3483")
    vincodes.append("6956")

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

    period = "day" # "" == total, "day", "night"
    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        if period == "day":
            total_bag_dirs.extend(get_day_bag_folders(dir))
        elif period == "night":
            total_bag_dirs.extend(get_night_bag_folders(dir))
        else:
            total_bag_dirs.extend(get_valid_bag_folders(dir))
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-3C/bag/20250306-zw/E371_6477_20250306_1930/2025-03-06-19-45-58")
    
    total_clip_dirs = []
    # total_clip_dirs = get_cc_clip_folders(total_bag_dirs)

    for item in abs_car_dirs:
        extract_od_frame(item, target_root, period)

