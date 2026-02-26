import os
import re
import shutil
import subprocess

PASSWORD = "123"

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
            if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item) or "-clip-20df" in item:
                valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")

    return valid_bag_dirs

def get_batch_bags(batch):
    total_bags = []
    bags = os.listdir(batch)
    bags = [d for d in bags if os.path.isdir(os.path.join(batch, d))]
    bags = [d for d in bags if "-clip-20df" in d]
    bags = [os.path.abspath(os.path.join(batch, d)) for d in bags]
    total_bags.extend(bags)
    total_bags.sort()
    return total_bags

def print_same_name_bag(bags):
    for bag in bags:
        for i in bags:
            if bag != i and os.path.basename(bag) == os.path.basename(i):
                print(f"{bag} <-> {i}")

def get_total_bags(dir_root, day=''):
    day_dirs = []
    if day:
        day_dirs.append(day)
    else:
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))

    abs_car_dirs = []
    for day_dir in day_dirs:
        car_dirs = os.listdir(os.path.join(dir_root, day_dir))
        car_dirs.sort()
        car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
        abs_car_dirs.extend(os.path.abspath(os.path.join(dir_root, day_dir, d)) for d in car_dirs)
    abs_car_dirs.sort()
    abs_bag_dirs = []
    for car_dir in abs_car_dirs:
        bag_dirs = os.listdir(car_dir)
        bag_dirs = [d for d in bag_dirs if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', d)]
        bag_dirs = [d for d in bag_dirs if "-clip-20df" not in d]
        abs_bag_dirs.extend(list(os.path.abspath(os.path.join(car_dir, d)) for d in bag_dirs))
    abs_bag_dirs.sort()
    return abs_bag_dirs

def get_match_batch_bags(bag_root, batch_dir):
    total_bags = get_total_bags(bag_root)
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)
    total_bags = [d for d in total_bags if "-clip-20df" not in d]

    abs_car_dirs = []
    abs_car_dirs.append(batch_dir)

    filtered_bags = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        filtered_bags.extend(get_valid_bag_folders(dir))
    filtered_bags = list(set(filtered_bags))
    print(f"filtered bags {len(filtered_bags)}")

    valid_bags = []
    for bag in total_bags:
        exist = False
        for filter_bag in filtered_bags:
            if os.path.basename(bag) in filter_bag:
                exist = True
                break
        if exist:
            valid_bags.append((bag, filter_bag))
    print(f"valid bags {len(valid_bags)}")
    return valid_bags

def fullin_frames(bag, batch_bag, target_dir):
    bag_name = os.path.basename(bag).replace('-clip-20df', '')
    batch_clips = os.listdir(batch_bag)
    batch_clips = [d for d in batch_clips if os.path.basename(bag_name) in d]
    batch_clips.sort()
    for batch_clip in batch_clips:
        target_clip = os.path.join(target_dir, batch_clip)
        if not os.path.exists(target_clip):
            os.makedirs(target_clip)
        frames = os.listdir(os.path.join(batch_bag, batch_clip))
        frames = [int(d) for d in frames if d.isdigit()]
        if not frames:
            continue
        time_zone_min = min(frames)
        time_zone_max = max(frames)

        clips = os.listdir(bag)
        clips = [d for d in clips if os.path.basename(bag_name) in d]
        clips.sort()
        for clip in clips:
            od_clip = os.path.join(bag, clip)
            frames = os.listdir(os.path.join(bag, clip))
            frames = [d for d in frames if d.isdigit() and int(d) >= time_zone_min and int(d) < time_zone_max]
            if frames:
                for frame in frames:
                    if not os.path.exists(os.path.join(target_clip, frame)):
                        try:
                            # print(f"cp {frame} to {target_clip}")
                            cmd = f"sshpass -p '{PASSWORD}' sudo cp -rp {os.path.join(od_clip, frame)} {target_clip}"
                            subprocess.run(cmd, shell=True, check=True)
                        except Exception as e:
                            print(f"cp failed: {e}")

                file_path = os.path.join(od_clip, "calib.json")
                if os.path.exists(file_path):
                    shutil.copy2(file_path, os.path.join(target_clip, os.path.basename(file_path)))

    target_pose = os.path.join(target_dir, "pose")
    if not os.path.exists(target_pose) and os.path.exists(os.path.join(bag, "pose")):
        # os.makedirs(target_pose)
        try:
            cmd = f"sshpass -p '{PASSWORD}' sudo cp -r {os.path.join(bag, 'pose')} {target_dir}"
            subprocess.run(cmd, shell=True, check=True)
        except Exception as e:
            print(f"cp failed: {e}")

def remove_item(item):
    if os.path.exists(item):
        print(f"remove {item}")
        cmd = f"sshpass -p '{PASSWORD}' sudo rm -rf {item}"
        subprocess.run(cmd, shell=True, check=True)

if __name__ == "__main__":
    batchs = []
    batch_root = "/home/geely/nas/J6M-S9G/LD_OD_2hz/已下发/PICI"
    batchs = os.listdir(batch_root)
    batchs = [os.path.join(batch_root, d) for d in batchs if os.path.isdir(os.path.join(batch_root, d))]
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250520/BD-15-58_1-10011_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250520/BD-15-58_2-9981_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250520/BD-15-58_3-9989_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250520/XC-6-57-6065_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250521/SJT-20-59-10009_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250521/SJT-21-60-10018_0036")
    # batchs.append("/home/geely/nas/J6M-4A/4D-OD-J4A/20250521/SJT-22-61-10K")
    
    bag_root = "/home/geely/nas/J6M-S9G/bag/行车/J6M-S10"
    target_root = "/home/geely/nas/J6M-S9G/bag/行车/LD_OD_10HZ"
    for batch in batchs:
        valid_bags = get_match_batch_bags(bag_root, batch)
        for bag, batch_bag in valid_bags:
            od_folder = bag + "-clip-20df"
            if os.path.exists(od_folder):
                if not os.path.exists(os.path.join(target_root, os.path.basename(batch), os.path.basename(batch_bag)) + ".zip"):
                    target_dir = os.path.join(target_root, os.path.basename(batch), os.path.basename(batch_bag))
                    print(f"cp {od_folder} \nto {target_dir}")
                    fullin_frames(od_folder, batch_bag, target_dir)
                    
                    # 压缩target_dir文件夹并删除原文件夹
                    zip_path = target_dir + ".zip"
                    try:
                        # 创建ZIP压缩包
                        shutil.make_archive(target_dir, 'zip', target_dir)
                        print(f"成功压缩文件夹到 {zip_path}")

                        if os.path.exists(zip_path):
                            remove_item(target_dir)
                    except Exception as e:
                        print(f"压缩或删除文件夹时出错: {e}")                
            else:
                print(f"{od_folder} not exist")


