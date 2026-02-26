import os
import re
import shutil

# extract all the label from the dir
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

    return valid_bag_dirs

def extract_label(bag_dir, target_dir):
    label_file = os.path.join(bag_dir, "label.yaml")
    if os.path.exists(label_file):
        shutil.copy(label_file, target_dir)
        print(f"copy label.yaml to {target_dir}")
    else:
        print(f"label.yaml 不存在于 {bag_dir}")

    info_file = os.path.join(bag_dir, "info.yaml")
    if os.path.exists(info_file):
        shutil.copy(info_file, target_dir)
        print(f"copy info.yaml to {target_dir}")
    else:
        print(f"label.yaml 不存在于 {bag_dir}")

def extract_all_label(bag_root, target_root):
    if not os.path.isdir(target_root):
        os.makedirs(target_root)

    if os.path.exists(bag_root):
        day_dirs = os.listdir(bag_root)
        day_dirs = [d for d in day_dirs if os.path.isdir(os.path.join(bag_root, d))]
        day_dirs = [d for d in day_dirs if d.isdigit()]
        day_dirs = [os.path.join(bag_root, d) for d in day_dirs]
        day_dirs.sort()

    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(bag_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
    # abs_car_dirs = []
    # abs_car_dirs.append("/home/robosense/Documents/verify/20250318/E371_3483_20250318_0830")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))

    # 指定处理某个bag文件夹
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-3/20250301/E371_3484_20250301_0942/2025-03-01-09-48-09")
    for bag_dir in total_bag_dirs:
        car_dir = os.path.dirname(bag_dir)
        day_dir = os.path.dirname(car_dir)        
        target_dir = os.path.join(target_root, os.path.basename(day_dir), os.path.basename(car_dir), os.path.basename(bag_dir))
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)
        extract_label(bag_dir, target_dir)

if __name__ == "__main__":
    bag_root = "/home/geely/nas/J6M-4c/bag"
    target_root = "/home/geely/Documents/labels/J6M-4c_label"
    extract_all_label(bag_root, target_root)
    