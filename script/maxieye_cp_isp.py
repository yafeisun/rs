import os
import shutil
import re
import time
import subprocess

def copy_files(source_folder, target_folder, file_list):
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)
    for file in file_list:
        source_path = os.path.join(source_folder, file)
        target_path = os.path.join(target_folder, file)
        shutil.copy2(source_path, target_path)
        

def sample_images_per_sec(image_list):
    """
    Sample one frame per second from the image list based on nanosecond timestamps,
    with a time difference error not exceeding 10%.

    :param image_list: A list of dictionaries containing 'img' (nanosecond timestamp) and 'pcd' fields.
    :return: A list of sampled image timestamps.
    """
    if not image_list:
        return []
    selected_images = []
    one_second_ns = 1000000000  # One second in nanoseconds

    last_second = -1
    for item in image_list[0:]:
        current_timestamp = item['img']
        current_sec = current_timestamp // one_second_ns  # 将纳秒转换为秒
        if current_sec > last_second:
            selected_images.append(current_timestamp)
            last_second = current_sec

    return selected_images

def copy_isp_clip(clip, target_dir):
    start_time_total = time.time()    
    image_list = []
    # Read the sync_sensors.txt file
    sync_sensors_path = os.path.join(clip, "result", "test_calibration", "middle", "sync_sensors.txt")
    if os.path.exists(sync_sensors_path):
        # Specify the encoding when opening the file
        with open(sync_sensors_path, 'r', encoding='utf-8') as f:
            next(f)
            for line in f:
                pcd_num, img_num = line.strip().split(' ')[0:2]
                if img_num != "null" and pcd_num != "null":
                    image_list.append({'img': int(img_num), 'pcd': int(pcd_num)})
    
    image_list.sort(key=lambda x: x['img'])
    result = sample_images_per_sec(image_list)
    print(len(result))

    # Get the grandparent directory of clip
    grandparent_dir = os.path.dirname(os.path.dirname(clip))
    source_image_folder = os.path.join(grandparent_dir, "cam_front_right")
    target_image_folder = os.path.join(target_dir, "cam_front_right")

    # Create the target folder if it doesn't exist
    if not os.path.exists(target_image_folder):
        os.makedirs(target_image_folder)

    # Copy the corresponding images
    for img_num in result:
        img_name = f"{img_num}.jpg"
        source_img_path = os.path.join(source_image_folder, img_name)
        target_img_path = os.path.join(target_image_folder, img_name)
        if os.path.exists(source_img_path):
            shutil.copy2(source_img_path, target_img_path)
            print(f"Copied {img_name} to {target_image_folder}")
        else:
            print(f"Image {img_name} not found in {source_image_folder}")

    # Copy the corresponding PCD files
    pcd_source_folder = os.path.join(clip, "result", "test_calibration", "middle")
    pcd_target_folder = os.path.join(target_dir, "middle")

    # Create the target PCD folder if it doesn't exist
    if not os.path.exists(pcd_target_folder):
        os.makedirs(pcd_target_folder)

    # Create a mapping from image numbers to PCD numbers
    img_to_pcd = {item['img']: item['pcd'] for item in image_list}

    for img_num in result:
        if img_num in img_to_pcd:
            pcd_num = img_to_pcd[img_num]
            pcd_name = f"{pcd_num}.pcd"
            pcd_source_path = os.path.join(pcd_source_folder, pcd_name)
            pcd_target_path = os.path.join(pcd_target_folder, pcd_name)
            if os.path.exists(pcd_source_path):
                shutil.copy2(pcd_source_path, pcd_target_path)
                print(f"Copied {pcd_name} to {pcd_target_folder}")
            else:
                print(f"PCD file {pcd_name} not found in {pcd_source_folder}")
        else:
            print(f"No corresponding PCD number found for image number {img_num}")

    # Copy the sync_sensors.txt file
    if os.path.exists(sync_sensors_path):
        try:
            file_name = os.path.basename(sync_sensors_path)
            target_file_path = os.path.join(pcd_target_folder, file_name)
            shutil.copy2(sync_sensors_path, target_file_path)
            print(f"Copied {file_name} to {pcd_target_folder}")
        except Exception as e:
            print(f"Failed to copy {sync_sensors_path} to {pcd_target_folder}: {e}")
    else:
        print(f"{sync_sensors_path} does not exist.")

    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"Consumed time: {total_duration:.2f}s")


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

def get_od_bag_folders(car_path, period_time):
    # period_time: "" == total, "day", "night"
    valid_bag_dirs = []
    if not os.path.isdir(car_path):
        print(f"错误：目标文件夹 {car_path} 不存在！")
        return valid_bag_dirs
    bags = os.listdir(car_path)
    bags = [d for d in bags if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', d)]
    bags = [d for d in bags if not d.startswith('.')]
    for item in bags:
        bag_path = os.path.join(car_path, item)
        if os.path.isdir(bag_path):
            if period_time == "day":
                if not is_night_bag(item):
                    valid_bag_dirs.append(bag_path)
            elif period_time == "night":
                if is_night_bag(item):
                    valid_bag_dirs.append(bag_path)
            else:
                valid_bag_dirs.append(bag_path)                
    return valid_bag_dirs

def get_day_bag_list(old_target_root):
    total_bag_dirs = []
    for item in old_target_root:
        if os.path.isdir(item):
            car_dirs = os.listdir(item)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(item, d))]
            car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            for car_dir in car_dirs:
                bag_dirs = os.listdir(os.path.join(item, car_dir))
                total_bag_dirs.extend(bag_dirs)
    return total_bag_dirs

def extract_isp_frame(bag_dir, target_root):
    new_car_path = os.path.join(target_root, os.path.basename(os.path.dirname(bag_dir)))
    if not os.path.exists(new_car_path):
        os.makedirs(new_car_path)
    new_bag_path = os.path.join(new_car_path, os.path.basename(bag_dir))
    if not os.path.exists(new_bag_path):
        os.makedirs(new_bag_path)
        
    clips = os.listdir(os.path.join(bag_dir, "clips"))
    clips = [d for d in clips if not "-x" in d]
    clips.sort()
    for clip in clips:
        clip_path = os.path.join(bag_dir, "clips", clip)        
        if os.path.isdir(os.path.join(clip_path, "result", "test_calibration", "middle")):
            new_clip_path = os.path.join(new_bag_path, clip)
            if not os.path.exists(new_clip_path):
                os.makedirs(new_clip_path)
                copy_isp_clip(clip_path, new_clip_path)


if __name__ == "__main__":
    dir_root = "/home/geely/nas/J6M-3C/bag"
    target_root = "/media/geely/fangzhen03/maxieye-isp-0425"

    if not os.path.exists(target_root):
        os.makedirs(target_root)
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    else:
        day_dirs = []

    day_dirs = []
    day_dirs.append("20250320")
    day_dirs.append("20250322")

    vin_codes = []
    # vin_codes.append("3483")
    vin_codes.append("6477")
    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
            
    # abs_car_dirs = []
    # abs_car_dirs.append("/home/robosense/nas/Geely-d/AEB/20250412/E371_3483_20250412_1557ELK")
    
    period = "day" 
    period = "" 
    total_bag_dirs = []
    for dir in abs_car_dirs:
        total_bag_dirs.extend(get_od_bag_folders(dir, period))        
    total_bag_dirs.sort()
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-3C/bag/20250320/E371_6477_20250320_1833/2025-03-20-19-09-49")

    for item in total_bag_dirs:
        extract_isp_frame(item, target_root)

        

