import os
import shutil
import re
import subprocess
from find_bag_by_tag import get_pld_bags

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

def get_batch_bags(bag_root, batch_root):
    # bag_root bag根目录
    # filter_root 筛选后的根路径
    total_bags = get_total_bags(bag_root)
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)
    total_bags = [d for d in total_bags if "-clip-20df" not in d]

    filtered_bags = os.listdir(batch_root)
    filtered_bags = [os.path.join(batch_root, d) for d in filtered_bags]
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
        # else:
        #     shutil.rmtree(bag)
    print(f"valid bags {len(valid_bags)}")
    return valid_bags

def get_original_bags(bag_root):
    total_bags = get_total_bags(bag_root)
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)
    total_bags = [d for d in total_bags if "-clip-20df" not in d]

    # 之前筛选过的处理结果
    exist_bags = []
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6B/FS-PLD-0609"))
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6B/FS-PLD-0619"))
    exist_bags = list(set(exist_bags))
    print(f"exist bags {len(exist_bags)}")

    valid_bags = []
    for bag in total_bags:
        exist = False
        for exist_bag in exist_bags:
            if os.path.basename(bag) in exist_bag:
                exist = True
                break
        if exist:
            valid_bags.append(bag)
    print(f"valid bags {len(valid_bags)}")
    return valid_bags


def get_exist_bags(od_root="/home/geely/nas/PLD-S6B/FS-PLD-0624"):
    total_bags = []
    if os.path.isdir(od_root):
        batchs = os.listdir(od_root)
        batchs = [d for d in batchs if os.path.isdir(os.path.join(od_root, d))]
        batchs = [os.path.abspath(os.path.join(od_root, d)) for d in batchs]
        for batch in batchs:
            bags = os.listdir(batch)
            bags = [d for d in bags if os.path.isdir(os.path.join(batch, d))]
            bags = [d for d in bags if "-clip-20df" in d]
            bags = [os.path.abspath(os.path.join(batch, d)) for d in bags]
            total_bags.extend(bags)
    total_bags.sort()
    return total_bags

def get_total_bags(dir_root):
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250621")
    # day_dirs.append("20250622")
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

def print_same_name_bag(bags):
    for bag in bags:
        for i in bags:
            if bag != i and os.path.basename(bag) == os.path.basename(i):
                print(f"{bag} <-> {i}")

def get_valid_bags(bag_root):
    total_bags = get_total_bags(bag_root)
    total_bags = get_pld_bags(bag_root)
    # total_bags = [d for d in total_bags if os.path.isdir(d + "-clip-20df")]
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)

    # od_root="/home/geely/nas/PLD-S6B/FS-PLD-0609"       # 之前提取过的处理结果
    # exist_bags = get_exist_bags(od_root)
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6B/FS-PLD-0619"))
    # exist_bags.extend(get_pld_bags(bag_root))

    # 之前筛选过的处理结果
    exist_bags = []
    exist_bags.extend(get_exist_bags("/media/geely/BCCJ5t/FS-PLD/img-0618"))
    exist_bags.extend(get_exist_bags("/media/geely/BCCJ5t/FS-PLD/img-0619"))
    exist_bags = list(set(exist_bags))
    print(f"exist bags {len(exist_bags)}")

    valid_bags = []
    for bag in total_bags:
        exist = False
        for exist_bag in exist_bags:
            if os.path.basename(bag) in exist_bag:
                exist = True
                break
        if not exist:
            valid_bags.append(bag)
    print(f"valid bags {len(valid_bags)}")
    return valid_bags

def extract_pld_img(bags, img_root):
    total_frame_num = 0
    for bag in bags:
        car_dir = os.path.dirname(bag)
        clips = os.listdir(bag)
        clips = [d for d in clips if "2025" in d]
        clips.sort()
        for clip in clips:
            target_clip_dir = os.path.join(car_dir, bag, clip).replace(os.path.dirname(car_dir), img_root)
            if not os.path.isdir(target_clip_dir):
                os.makedirs(target_clip_dir)
            frames = [d for d in os.listdir(os.path.join(car_dir, bag, clip))
                if os.path.isdir(os.path.join(car_dir, bag, clip, d)) and d.isdigit()]
            total_frame_num += len(frames)
            for frame in frames:
                img = os.path.join(car_dir, bag, clip, frame, frame + "__front.jpg")
                shutil.copy2(img, os.path.join(target_clip_dir, os.path.basename(img)))
    print(f"total frame {total_frame_num}")

def extract_clip_from_img(img_root, bag_root, target_root):    
    img_bags = get_exist_bags(img_root)
    for img_bag in img_bags:
        car_dir = os.path.basename(os.path.dirname(img_bag))
        od_frame_path = os.path.join(bag_root, car_dir.split('_')[2], car_dir, os.path.basename(img_bag))
        target_bag_dir = os.path.join(target_root, car_dir, os.path.basename(img_bag))
        if not os.path.exists(target_bag_dir):
            os.makedirs(target_bag_dir)
        bev = os.path.join(od_frame_path, "bev")
        pose = os.path.join(od_frame_path, "pose")
        if os.path.exists(od_frame_path) and os.path.exists(bev) and os.path.exists(pose):
            clips = os.listdir(img_bag)
            clips = [d for d in clips if "2025" in d]
            clips.sort()
            for clip in clips:
                od_clip = os.path.join(od_frame_path, clip)
                if os.path.exists(od_clip): # and not os.path.exists(os.path.join(target_bag_dir, clip)):
                    frames = os.listdir(od_clip)
                    frames = [d for d in frames if d.isdigit()]
                    target_clip = os.path.join(target_bag_dir, clip)
                    # 只取每帧中的5路相机和json
                    for frame in frames:
                        target_frame = os.path.join(target_clip, frame)
                        if not os.path.isdir(target_frame):
                            os.makedirs(target_frame)
                        for file in ['__a_back.jpg', '__a_front.jpg', '__a_left.jpg', '__a_right.jpg', '__front.jpg', '__desc.json']:
                            file_path = os.path.join(od_clip, frame, frame + file)
                            shutil.copy2(file_path, os.path.join(target_frame, os.path.basename(file_path)))
                    
                    file_path = os.path.join(od_clip, "calib.json")
                    if os.path.exists(file_path):
                        shutil.copy2(file_path, os.path.join(target_clip, os.path.basename(file_path)))

                    # try:
                    #     print(f"cp {frame} to {target_clip}")
                    #     subprocess.run(["cp", "-rp", os.path.join(od_clip, frame), target_clip], check=True)
                    # except Exception as e:
                    #     print(f"cp failed: {e}")

            for item in ["bev", "pose"]:
                od_clip = os.path.join(od_frame_path, item)
                if os.path.exists(od_clip) and not os.path.exists(os.path.join(target_bag_dir, item)):
                    try:
                        print(f"cp {item} to {target_bag_dir}")
                        subprocess.run(["cp", "-rp", od_clip, target_bag_dir], check=True)
                    except Exception as e:
                        print(f"cp failed: {e}")
        else:
            print(f"file missing {od_frame_path}")
    return

def get_total_frames(bag_dir):
    bag_name = os.path.basename(bag_dir).replace("-clip-20df", "")
    total_frames = []
    clips = os.listdir(bag_dir)
    clips = [d for d in clips if "2025" in d]
    clips.sort()
    for clip in clips:
        clip_path = os.path.join(bag_dir, clip)
        frames = os.listdir(clip_path)
        frames = [d for d in frames if d.isdigit()]
        frames = [os.path.join(clip_path, d) for d in frames]
        total_frames.extend(frames)
    total_frames.sort()
    return total_frames

def replace_bag_files(od_folder, batch_bag):
    print(f"replace {od_folder} to {batch_bag}")
    total_batch_frames = get_total_frames(batch_bag)
    total_od_frames = get_total_frames(od_folder)
    for batch_frame in total_batch_frames:
        exist_flag = False
        for od_frame in total_od_frames:
            if os.path.basename(od_frame) == os.path.basename(batch_frame):
                exist_flag = True
                try:
                    subprocess.run(["cp", "-rp", od_frame, os.path.dirname(batch_frame)], check=True)
                except Exception as e:
                    print(f"cp failed: {e}")
        if not exist_flag:
            print(f"frame {batch_frame} not in {od_folder}")

if __name__ == "__main__":
    # bag_root = "/home/geely/nas/PLD-S6C/J6M-S23"        # bag根路径
    # img_root = "/media/geely/BCCJ5t/FS-PLD/img-0626"    # 给算法筛选场景的目标目录 抽取结果中的前视相机
    # # img_root = "/media/geely/BCCJ5t/FS-PLD/img-0619-backup"    # 给算法筛选场景的目标目录 抽取结果中的前视相机
    # target_root = "/home/geely/nas/PLD-S6B/FS-PLD-0625" # 当前批次要抽取的目标位置

    # valid_bags = get_valid_bags(bag_root)

    # extract_pld_img(valid_bags, img_root)
    
    # extract_clip_from_img(img_root, bag_root, target_root)


    dir_root = "/home/geely/nas/J6M-S9G/OCC/J6M-S22"

    total_bag_dirs = []
    total_filtered_bags = []
    total_filtered_bags.extend(get_batch_bags(dir_root, "/home/geely/nas/J6M-S9G/OCC/待下发/HY-2-102-0509-198"))
    total_filtered_bags.extend(get_batch_bags(dir_root, "/home/geely/nas/J6M-S9G/OCC/待下发/HY-2-102-0509-2381"))

    for bag, batch_bag in total_filtered_bags:
        od_folder = bag + "-clip-20df"
        if os.path.exists(od_folder):
            replace_bag_files(od_folder, batch_bag)
        else:
            print(f"{od_folder} not exists")



        

    