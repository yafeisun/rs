import os
import re
import shutil

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
                if not os.path.isdir(item_path + "-clip-20df"):
                    valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")

    return valid_bag_dirs

def copy_img_patch():
    dir_root = "/home/geely/nas/Geely-f/img_tuomin/J6M-S07"
    target_root = "/media/geely/sda/tuomin-20250612"
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    day_dirs = []
    day_dirs.append("20250525")
    day_dirs.append("20250526")
    day_dirs.append("20250527")
    # day_dirs.append("20250518")
    # day_dirs.append("20250519")
    # day_dirs.append("20250520")
    # day_dirs.append("20250523")
    # day_dirs.append("20250524")

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

    # abs_car_dirs = []
    # abs_car_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_0712")
    # abs_car_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_0945")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))

    # 指定处理某个bag文件夹
    # total_bag_dirs = [] 
    # total_bag_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_1358/2025-05-17-14-35-24")
    # total_bag_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_1358/2025-05-17-14-40-24")
    # total_bag_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_1358/2025-05-17-14-45-24")
    # total_bag_dirs.append("/home/geely/nas/Geely-f/img_tuomin/J6M-S03/20250517/J6M_0153_20250517_1358/2025-05-17-14-50-24")

    total_img = 0
    total_bag_dirs.sort()
    for bag in total_bag_dirs:
        folders = os.listdir(bag)
        for folder in folders:
            imgs = os.listdir(os.path.join(bag, folder))
            imgs = [d for d in imgs if ".jpg" in d]
            if len(imgs) == 0:
                shutil.rmtree(os.path.join(bag, folder))
            else:
                bag_path = bag.replace("img_tuomin/", "")
                car_path = os.path.basename(os.path.dirname(bag_path))
                target_path = os.path.join(target_root, car_path, os.path.basename(bag_path), folder)
                if not os.path.exists(target_path):
                    os.makedirs(target_path)
                for img in imgs:
                    source_path = os.path.join(bag_path, folder, img)
                    shutil.copy2(source_path, target_path)
            total_img += len(imgs)
    print(f"total image: {total_img}")

def tuomin_img_count(target_root):
    abs_car_dirs = []
    if os.path.isdir(target_root):
        car_dirs = os.listdir(target_root)
        car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(target_root, d))]
        car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
        car_dirs.sort()
        abs_car_dirs.extend(os.path.abspath(os.path.join(target_root, d)) for d in car_dirs)

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))

    total_img = 0
    total_bag_dirs.sort()
    for bag in total_bag_dirs:
        folders = os.listdir(bag)
        for folder in folders:
            imgs = os.listdir(os.path.join(bag, folder))
            imgs = [d for d in imgs if ".jpg" in d]
            if not imgs:
                shutil.rmtree(os.path.join(bag, folder))
            total_img += len(imgs)
        if not os.listdir(bag):
            shutil.rmtree(bag)
    print(f"total image: {total_img}")
    return total_img

if __name__ == "__main__":
    img_root = "/home/geely/nas/Geely-f/img_tuomin/J6M-S11"
    days = os.listdir(img_root)
    total_imgs = 0
    for day in days:
        total_imgs = total_imgs + tuomin_img_count(os.path.join(img_root, day))
    print(total_imgs)