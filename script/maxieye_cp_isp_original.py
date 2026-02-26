import os
import shutil
import re
import time
import subprocess

def copy_isp_clip(src, dst):
    start_time_total = time.time()
    clips = os.listdir(src)
    clips = [d for d in clips if os.path.isdir(os.path.join(src, d))]
    for clip in clips:
        if os.path.isdir(os.path.join(src, clip)) and "-isp" in clip and not os.path.isdir(os.path.join(dst, clip)):
            try:
                # cp cc clip
                print(f"cp {clip} to {dst}")
                subprocess.run(["cp", "-rp", os.path.join(src, clip), dst], check=True)                
            except Exception as e:
                print(f"cp failed: {e}")
        elif not "-isp" in clip and os.path.isdir(os.path.join(dst, clip)):
            print(f"remove {os.path.join(dst, clip)}")
            shutil.rmtree(os.path.join(dst, clip))
    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"consumed time: {total_duration:.2f}s")

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
    bags = [d for d in bags if d.endswith('-clip-20df')]
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
            # car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            for car_dir in car_dirs:
                bag_dirs = os.listdir(os.path.join(item, car_dir))
                total_bag_dirs.extend(bag_dirs)
    return total_bag_dirs

def extract_od_frame(item, target_root):
    new_car_path = os.path.join(target_root, os.path.basename(os.path.dirname(item)))
    isp_clip = False
    clips = os.listdir(item)
    for clip in clips:
        if os.path.isdir(os.path.join(item, clip)) and "-isp" in clip:
            isp_clip = True
            break
    if isp_clip:
        if not os.path.exists(new_car_path):
            os.makedirs(new_car_path)
        new_bag_path = os.path.join(new_car_path, os.path.basename(item))
        if not os.path.exists(new_bag_path):
            os.makedirs(new_bag_path)
        copy_isp_clip(item, new_bag_path)

        bag_path = item.replace("-clip-20df", "")
        clips = os.listdir(os.path.join(bag_path, "clips"))
        for clip in clips:
            target_dir = os.path.join(new_bag_path, clip)
            if os.path.isdir(target_dir):
                for file in ["mapping_pose.txt", "mapping_pose_quaterniond.txt"]:
                    file_path = os.path.join(clip, "result", "mapping", file)
                    shutil.copy2(file_path, target_dir)
                file_path = os.path.join(clip, "result", "test_calibration", "sync_sensors.txt")
                shutil.copy2(file_path, target_dir)

if __name__ == "__main__":
    dir_root = "/home/geely/nas/J6M-3C/bag"
    target_root = "/media/geely/hq18/maxieye-isp-0417-day"
    # old_target_root = []
    # old_target_root.append("/media/geely/sdb/maxieye-isp-0412-night")
    # old_target_root.append("/media/geely/sdb/maxieye-isp-0415-day")

    # if not os.path.exists(target_root):
    #     os.makedirs(target_root)
    isp_list = os.listdir(target_root)
    for item in isp_list:
        source_car_dir = os.path.join(dir_root, item.split('_')[2], item)
        abs_car_isp = os.path.join(target_root, item)
        isp_bag_list = os.listdir(abs_car_isp)
        isp_bag_list = [d for d in isp_bag_list if d.endswith("-clip-20df") and os.path.isdir(os.path.join(abs_car_isp, d))]
        for isp_bag in isp_bag_list:
            source_bag_dir = os.path.join(source_car_dir, isp_bag.replace("-clip-20df", ""))
            target_bag_dir = os.path.join(abs_car_isp, isp_bag.replace("-clip-20df", ""))
            if not os.path.exists(target_bag_dir):
                os.makedirs(target_bag_dir)
            img_path = os.path.join(source_bag_dir, "cam_front_right")
            subprocess.run(["cp", "-rp", img_path, target_bag_dir], check=True)
    



    # day_dirs = os.listdir(dir_root)
    # day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # # day_dirs = []
    # # day_dirs.append("20250309")
    # vin_codes = []
    # vin_codes.append("3483")
    # vin_codes.append("6956")
    # abs_car_dirs = []
    # for day in day_dirs:
    #     day_dir = os.path.join(dir_root, day)
    #     if os.path.isdir(day_dir):
    #         car_dirs = os.listdir(day_dir)
    #         car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
    #         car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
    #         car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
    #         car_dirs.sort()
    #         abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
    # # abs_car_dirs = []
    # # abs_car_paths.append("/home/robosense/nas/J6M-3D/bag/20250326/E371_6477_20250326_1840")
    # period = "day" 
    # total_bag_dirs = []
    # for dir in abs_car_dirs:
    #     total_bag_dirs.extend(get_od_bag_folders(dir, period))        
    # total_bag_dirs.sort()
    # if old_target_root:
    #     bag_list = get_day_bag_list(old_target_root)
    # for item in total_bag_dirs:
    #     if old_target_root:
    #         if os.path.basename(item) not in bag_list:
    #             extract_od_frame(item, target_root)
    #     else:
    #         extract_od_frame(item, target_root)

        

