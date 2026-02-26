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
    # day_dirs.append("20250730")
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

    abs_car_dirs = []
    days = os.listdir(filter_root)
    for day in days:
        day_dir = os.path.join(filter_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)

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

def extract_occ_clips_with_strict_frames(od_frame_path, img_bag, target_dir, frequency=2):
    img_clips = os.listdir(img_bag)
    img_clips.sort()
    for img_clip in img_clips:
        target_clip = os.path.join(target_dir, img_clip)
        if not os.path.exists(target_clip):
            os.makedirs(target_clip)

        img_list = os.listdir(os.path.join(img_bag, img_clip))
        img_list = [int(d.replace(".jpg", "")) for d in img_list if d.endswith('.jpg')]
        img_list = [int(d / 1000000) for d in img_list]
        img_list.sort()
        time_zone_min = min(img_list)
        time_zone_max = max(img_list)

        # 筛选出对应的帧
        clips = os.listdir(od_frame_path)
        clips = [d for d in clips if os.path.basename(img_bag) in d]
        clips.sort()
        for clip in clips:
            od_clip = os.path.join(od_frame_path, clip)
            if os.path.exists(od_clip):
                frames = os.listdir(os.path.join(od_frame_path, clip))
                frames = [d for d in frames if d.isdigit() and int(d) >= time_zone_min and int(d) < time_zone_max]
                if frames:
                    for frame in frames:
                        try:
                            subprocess.run(["cp", "-rp", os.path.join(od_clip, frame), target_clip], check=True)
                        except Exception as e:
                            print(f"cp failed: {e}")

                    file_path = os.path.join(od_clip, "calib.json")
                    if os.path.exists(file_path):
                        shutil.copy2(file_path, os.path.join(target_clip, os.path.basename(file_path)))
                
    pose_path = os.path.join(od_frame_path, "pose")
    if os.path.exists(pose_path):
        shutil.copytree(pose_path, os.path.join(target_dir, "pose"))
    
    if frequency == 2:
        clips = os.listdir(target_dir)
        clips = [d for d in clips if os.path.basename(img_bag) in d]
        for clip in clips:
            frames = os.listdir(os.path.join(target_dir, clip))
            frames = [i for i in frames if os.path.isdir(os.path.join(target_dir, clip, i))]
            frames.sort()
            for frame_index, frame in enumerate(frames):
                frame_path = os.path.join(target_dir, clip, frame)
                if frame_index % 5 != 0:
                    shutil.rmtree(frame_path)
    
def extract_occ_match_clips(od_frame_path, img_bag, target_dir):
    # extract od clip if img_root is filtered img, where clips you want
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    img_clips = os.listdir(img_bag)
    # img_clips = [d for d in img_clips if '-s' in d and '-cc' in d and os.path.isdir(os.path.join(img_bag, d))]
    # img_clips = [d for d in img_clips if '-s' not in d and '-cc' in d and os.path.isdir(os.path.join(img_bag, d))]
    img_clips.sort()
    for img_clip in img_clips:
        clip = img_clip.replace('-cc', '').replace('-s1', '').replace('-s2', '').replace('-s3', '').replace('-s4', '')
        od_clip = os.path.join(od_frame_path, clip)
        if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_dir, clip)):
            print(f"cp {clip} to {target_dir}")
            # cp total clip
            try:
                subprocess.run(["cp", "-rp", od_clip, target_dir], check=True)
            except Exception as e:
                print(f"cp failed: {e}")
                
    pose_path = os.path.join(od_frame_path, "pose")
    if os.path.exists(pose_path):
        shutil.copytree(pose_path, os.path.join(target_dir, "pose"))
    

def extract_occ_match_frames(od_frame_path, img_bag, target_dir):
    # extract frame if img_root is filtered frames, where frames not stationary that already sent
    # only extract frame that exist in bag + -clip-20df
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)

    img_clips = os.listdir(img_bag)
    img_clips.sort()
    for clip in img_clips:
        od_clip = os.path.join(od_frame_path, clip)
        if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_dir, clip)):
            print(f"cp {clip} to {target_dir}")
            frames = os.listdir(os.path.join(img_bag, clip))
            frames = [d for d in frames if d.isdigit()]
            target_clip = os.path.join(target_dir, clip)
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

def extract_occ_files_from_bag(bag, img_bag, target_dir):
    # extract od files if img_root is filtered frames,  where frames not stationary that already sent
    # cannot find frames in -clip-20df, rerun sdk until projection finish
    # extract pcd from middle, no need to gen -clip-20df
    middle = os.path.join(bag, "result", "test_calibration", "middle")
    if os.path.isdir(middle):
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        image_list = []
        # Read the sync_sensors.txt file
        sync_sensors_path = os.path.join(bag, "result", "test_calibration", "middle", "sync_sensors.txt")
        if os.path.exists(sync_sensors_path):
            # Specify the encoding when opening the file
            with open(sync_sensors_path, 'r', encoding='utf-8') as f:
                next(f)
                for line in f:
                    pcd_num, img_num = line.strip().split(' ')[0:2]
                    if img_num != "null" and pcd_num != "null":
                        image_list.append({'img': int(img_num), 'pcd': int(pcd_num)})        
        image_list.sort(key=lambda x: x['img'])

        img_clips = os.listdir(img_bag)
        img_clips.sort()
        for clip in img_clips:
            od_clip = os.path.join(img_bag, clip)
            if os.path.isdir(middle) and not os.path.exists(os.path.join(target_dir, clip)):
                print(f"cp {clip} to {target_dir}")
                # cp match frames
                frames = os.listdir(os.path.join(img_bag, clip))
                frames = [d for d in frames if d.isdigit()]
                target_clip = os.path.join(target_dir, clip)
                if not os.path.isdir(target_clip):
                    os.makedirs(target_clip)
                    for frame in frames:
                        try:
                            subprocess.run(["cp", "-rp", os.path.join(od_clip, frame), target_clip], check=True)
                        except Exception as e:
                            print(f"cp failed: {e}")

                        # 取 frame 的前 13 位
                        frame_prefix = int(frame[:13])
                        # 查找对应的 pcd 编号
                        pcd_num = None
                        for item in image_list:
                            img_timestamp = str(item['img'])[:13]
                            if int(img_timestamp) == frame_prefix:
                                pcd_num = item['pcd']
                                break
                        if pcd_num is not None:
                            pcd_file = os.path.join(middle, str(pcd_num) + '.pcd')
                            if os.path.exists(pcd_file):
                                shutil.copy2(pcd_file, os.path.join(target_clip, frame, str(frame) + '.pcd'))
                
                    file_path = os.path.join(od_clip, "calib.json")
                    if os.path.exists(file_path):
                        shutil.copy2(file_path, os.path.join(target_clip, os.path.basename(file_path)))


if __name__ == "__main__":
    img_root = "/home/geely/nas/J6M-S6E/S10/已筛选/立柱不带悬空障碍物"
    bag_root = "/home/geely/nas/J6M-S6E/S10"
    target_root = "/home/geely/nas/J6M-S6E/S10/下发/立柱不带悬空障碍物"
    
    exist_bags = [] 
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/J6M_0302_20250617_1148-qie/BD-PLD-318-0302"))
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-5728-0302"))
    # exist_bags.extend(get_batch_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-6103-0302"))

    total_filtered_bags = get_filtered_bags(bag_root, img_root)
    for bag, filter_bag in total_filtered_bags:
        od_frame_path = bag + '-clip-20df'
        if os.path.isdir(od_frame_path):
            already_exist = False
            for exist_bag in exist_bags:
                if os.path.basename(bag) in os.path.basename(exist_bag):
                    already_exist = True
                    break
            if not already_exist:
                upper_folder = os.path.basename(os.path.dirname(bag))
                target_dir = os.path.join(target_root, os.path.basename(os.path.dirname(filter_bag)), upper_folder.split('_')[1] + '-' + os.path.basename(od_frame_path))
                # target_dir = os.path.join(target_root, os.path.basename(os.path.dirname(filter_bag)), os.path.basename(filter_bag))
                # if not os.path.isdir(os.path.join(target_root, os.path.basename(os.path.dirname(filter_bag)), os.path.basename(filter_bag))):
                #     shutil.rmtree(od_frame_path)
                print(f"extract {od_frame_path} to {target_dir}")
                extract_occ_clips_with_strict_frames(od_frame_path, filter_bag, target_dir, 2)
                # extract_occ_match_clips(od_frame_path, filter_bag, target_dir)
                # extract_occ_match_frames(od_frame_path, filter_bag, target_dir)
                # extract_occ_files_from_bag(bag, filter_bag, target_dir)
