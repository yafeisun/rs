import os
import re
import shutil

vision_folder_list = [
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
]

def get_valid_bag_folders(target_folder):
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

if __name__ == "__main__":
    dir_root = "/home/geely/nas/J6M-3C/bag"
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    
    abs_car_dirs = []
    for day_dir in day_dirs:        
        car_dirs = os.listdir(os.path.join(dir_root, day_dir))
        car_dirs = [d for d in car_dirs if "-x" not in d and "-zw" not in d]
        car_dirs.sort()
        abs_car_dirs.extend(os.path.abspath(os.path.join(dir_root, day_dir, d)) for d in car_dirs)

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))
    
    for bag in total_bag_dirs:
        for item in ['raw', 'result', 'state']:
            item_dir = os.path.join(bag, item)
            if os.path.isdir(item_dir):
                shutil.rmtree(item_dir)