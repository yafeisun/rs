import os
import shutil
import re
import subprocess

vision_folder_list = {
    "cam_around_back",
    "cam_around_front",
    "cam_around_left",
    "cam_around_right",
    "cam_back",
    "cam_front_left",
    "cam_front_right",
    "cam_side_left_back",
    "cam_side_left_front",
    "cam_side_right_back",
    "cam_side_right_front",
}

def extract_bev_files(bag, flag=False):
    # 使用 os.makedirs 递归创建目标目录
    target_root = bag + "-clip-20df"
    target_bev = os.path.join(target_root, "bev")
    if not os.path.exists(target_bev):
        os.mkdir(target_bev)
    bev_folder = os.path.join(bag, "result", "bev")
    for file in ["BEV.jpeg", "InBEV.jpeg", "HBEV.png", "BEV_label.json", \
        "IBEV.pcd", "RGBBEV.pcd"]:
        file_path = os.path.join(bev_folder, file)
        target_file_path = os.path.join(target_bev, file)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            if flag or not os.path.exists(target_file_path):
                shutil.copy2(file_path, target_file_path)
        else:
            print(f"{file} 不存在于 {bev_folder}")

    for file in ["map_intensity.pcd", "map_intensity_bev.pcd", "map_rgb_bev.pcd"]:
        map_pcd = os.path.join(bag, "result", "test_calibration", file)
        target_map_pcd_path = os.path.join(target_bev, os.path.basename(map_pcd))
        if os.path.exists(map_pcd) and os.path.isfile(map_pcd):
            if flag or not os.path.exists(target_map_pcd_path):
                shutil.copy2(map_pcd, target_map_pcd_path)
        else:
            print(f"{file} not in {os.path.join(bag, 'result', 'test_calibration')}")

    #cp pose txt
    target_pose = os.path.join(target_root, "pose")
    if not os.path.exists(target_pose):
        os.mkdir(target_pose)
    for img_folder in vision_folder_list:
        pose_txt = os.path.join(bag, "result", "test_calibration", img_folder, "sync_sensors.txt")
        if os.path.exists(pose_txt):
            img_pose = os.path.join(target_pose, img_folder)
            if not os.path.exists(img_pose):
                os.mkdir(img_pose)
            shutil.copy2(pose_txt, os.path.join(img_pose, os.path.basename(pose_txt)))


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

def get_total_bags(dir_root):
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250704")
    # day_dirs.append("20250705")
    # day_dirs.append("20250706")
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

def get_filtered_bags(bag_root, filter_root):
    total_bags = get_total_bags(bag_root)
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)
    total_bags = [d for d in total_bags if "-clip-20df" not in d]

    # 之前筛选过的结果
    abs_car_dirs = []
    car_dirs = os.listdir(filter_root)
    car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(filter_root, d))]
    car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
    car_dirs.sort()
    abs_car_dirs.extend(os.path.abspath(os.path.join(filter_root, d)) for d in car_dirs)

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

def extract_od_occ_files(od_frame_path, img_bag, target_dir):
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    img_clips = os.listdir(img_bag)
    img_clips.sort()
    for clip in img_clips:
        od_clip = os.path.join(od_frame_path, clip)
        if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_dir, clip)):
            try:
                print(f"cp {clip} to {target_dir}")
                subprocess.run(["cp", "-rp", od_clip, target_dir], check=True)
            except Exception as e:
                print(f"cp failed: {e}")

def extract_ldod_files(bag, img_bag, target_dir):
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
    # if not os.path.exists(target_dir):
    #     shutil.rmtree(od_frame_path)

    imgs = os.listdir(img_bag)
    imgs = [int(d.replace(".jpg", "")) for d in imgs if d.endswith(".jpg")]
    imgs.sort()
    imgs = [int(d / 1000000) for d in imgs]

    img_lists = [[] for _ in range(20)]
    last_img = imgs[0]
    index = 0
    img_lists[0] = []
    for img in imgs:
        if img - last_img >= 5000:
            index += 1
            img_lists[index] = []
        img_lists[index].append(img)
        last_img = img
    img_lists = list(filter(None, img_lists))

    for idx, img_list in enumerate(img_lists):
        time_zone_min = min(img_list)
        time_zone_max = max(img_list) + 4000 # 延后4S
    
        clips = os.listdir(bag)
        clips = [d for d in clips if os.path.basename(img_bag) in d]
        clips.sort()
        for clip in clips:
            od_clip = os.path.join(bag, clip)
            if os.path.exists(od_clip):
                frames = os.listdir(os.path.join(bag, clip))
                frames = [d for d in frames if d.isdigit() and int(d) >= time_zone_min and int(d) <= time_zone_max]
                new_clip = clip.split('_')[0] + "_" + str(idx)
                target_clip = os.path.join(target_dir, new_clip)
                if not os.path.isdir(target_clip):
                    os.makedirs(target_clip)
                for frame in frames:
                    try:
                        subprocess.run(["cp", "-rp", os.path.join(od_clip, frame), target_clip], check=True)
                    except Exception as e:
                        print(f"cp failed: {e}")

                file_path = os.path.join(od_clip, "calib.json")
                if os.path.exists(file_path):
                    shutil.copy2(file_path, os.path.join(target_clip, os.path.basename(file_path)))


def get_filtered_ld_bags(filter_root):
    dir_roots = os.listdir(filter_root)
    dir_roots = []
    # dir_roots.append("/home/geely/nas/J6M-S8D/20250818-data/J6M-S05")
    # dir_roots.append("/home/geely/nas/J6M-S8D/20250818-data/J6M-S06")
    # dir_roots.append("/home/geely/nas/J6M-S8D/20250818-data/J6M-S11")
    # dir_roots.append("/home/geely/nas/J6M-S8D/20250818-data/J6M-S14")
    dir_roots.append("/home/geely/nas/J6M-S8D/20250818-data/J6M-S23")
    total_bag_dirs = []
    for dir_root in dir_roots:
        dir_root = os.path.join(filter_root, dir_root)
        if os.path.isdir(dir_root):
            day_dirs = os.listdir(dir_root)
            day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))

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

        for dir in abs_car_dirs:
            # 获取目标文件夹下的有效bag文件夹
            total_bag_dirs.extend(get_valid_bag_folders(dir))

    total_bag_dirs = [d.replace(filter_root, "/home/geely/nas/J6M-S8D/bag") for d in total_bag_dirs]
    total_bag_dirs = list(set(total_bag_dirs))
    total_bag_dirs.sort()

    return total_bag_dirs

if __name__ == "__main__":
    img_root = "/home/geely/nas/J6M-S8D/20250818-data"
    bag_root = "/home/geely/nas/J6M-S8D/bag"
    target_root = "/home/geely/nas/J6M-S8D/LD_OD_10hz"

    # img_root = "/home/geely/nas/PLD-S6B/IMG/p181-zhaji-img"
    # bag_root = "/home/geely/nas/PLD-S6B/J6M-S23"
    # target_root = "/home/geely/nas/PLD-S6B/OCC/ZHAJI-P181-S23-S6B"
    
    exist_bags = []
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/J6M_0302_20250617_1148-qie/BD-PLD-318-0302"))
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-5728-0302"))
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-6103-0302"))
    total_bag_dirs = []
    total_bag_dirs.extend(get_filtered_ld_bags(img_root))

    for bag in total_bag_dirs:
        od_frame_path = bag + '-clip-20df'
        if os.path.isdir(od_frame_path):
            upper_folder = os.path.basename(os.path.dirname(bag))
            target_dir = os.path.join(target_root, upper_folder, upper_folder.split('_')[1] + '-' + os.path.basename(od_frame_path))
            filter_bag = bag.replace(bag_root, img_root)
            extract_ldod_files(od_frame_path, filter_bag, target_dir)

