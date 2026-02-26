import os
import shutil
import re
import subprocess
# from find_bag_by_tag import get_pld_bags

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

def get_filtered_bags(bag_root, filter_root):
    # bag_root bag根目录
    # filter_root 筛选后的根路径
    total_bags = get_total_bags(bag_root)
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)
    total_bags = [d for d in total_bags if "-clip-20df" not in d]

    # 之前筛选过的结果
    # abs_car_dirs = []
    # car_dirs = os.listdir(filter_root)
    # car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(filter_root, d))]
    # car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
    # car_dirs.sort()
    # abs_car_dirs.extend(os.path.abspath(os.path.join(filter_root, d)) for d in car_dirs)
    abs_car_dirs = [filter_root]
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
            valid_bags.append(bag)
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
    # total_bags = get_pld_bags(bag_root)
    # total_bags = [d for d in total_bags if os.path.isdir(d + "-clip-20df")]
    print(f"total bags {len(total_bags)}")
    print_same_name_bag(total_bags)

    # od_root="/home/geely/nas/PLD-S6B/FS-PLD-0609"       # 之前提取过的处理结果
    # exist_bags = get_exist_bags(od_root)
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6B/FS-PLD-0619"))
    # exist_bags.extend(get_pld_bags(bag_root))

    # 之前筛选过的处理结果
    exist_bags = []
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/J6M_0302_20250617_1148-qie/BD-PLD-318-0302"))
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-5728-0302"))
    # exist_bags.extend(get_exist_bags("/home/geely/nas/PLD-S6C/FS-PLD/PLD-S6C-20250708/PLD/WU-3-30-6103-0302"))
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
        car_dir = img_root
        clips = os.listdir(bag)
        clips = [d for d in clips if "2025" in d]
        clips.sort()
        for clip in clips:
            car_name = os.path.basename(os.path.dirname(bag))
            bag_name = os.path.basename(bag)
            target_clip_dir = os.path.join(car_dir, car_name, bag_name, clip)
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
    img_bags = get_filtered_bags(img_root)
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

if __name__ == "__main__":
    bag_root = "/home/geely/nas/PLD-S6F/bag/J6M-S24"        # bag根路径
    img_root = "/home/geely/nas/PLD-S6F/bag/J6M-S24/20250812/J6M_0252_20250812_2041"    # 给算法筛选场景的目标目录 抽取结果中的前视相机
    # img_root = "/media/geely/BCCJ5t/FS-PLD/img-0619-backup"    # 给算法筛选场景的目标目录 抽取结果中的前视相机
    target_root = "/home/geely/nas/PLD-S6B/FS-PLD-0625" # 当前批次要抽取的目标位置

    valid_bags = os.listdir(img_root)
    valid_bags = [d for d in valid_bags if "-clip-20df" in d]
    valid_bags = [os.path.join(img_root, d) for d in valid_bags]

    extract_pld_img(valid_bags, "/home/geely/nas/PLD-S6F/关闸机-S24")
    
    # extract_clip_from_img(img_root, bag_root, target_root)


        

    