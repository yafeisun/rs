import os
import shutil
import re
import cv2
from skimage.metrics import structural_similarity as ssim

# wget http://fishros.com/install -O fishros && . ./fishros

def get_valid_bag_folders(target_folder):
    valid_bag_dirs = []
    if not os.path.isdir(target_folder):
        print(f"错误：目标文件夹 {target_folder} 不存在！")
        return valid_bag_dirs

    for item in os.listdir(target_folder):
        item_path = os.path.join(target_folder, item)
        if os.path.isdir(item_path) and not item.startswith('.'):
            if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item):
                # if not os.path.isdir(item_path + "-clip-20df"):
                    valid_bag_dirs.append(item_path)
            # bug 修复：将 endwith 改为 endswith
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")
    return valid_bag_dirs


def is_vehicle_stationary(image_folder, threshold=0.9, stationary_percentage=0.3):
    """
    判断车辆是否静止。

    :param image_folder: 包含车辆图片的文件夹路径
    :param threshold: SSIM 阈值，用于判断两张图片是否相似
    :param stationary_percentage: 静止图片比例阈值，达到该比例则认为车辆静止
    :return: 车辆是否静止的布尔值
    """
    image_files = sorted([os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
    if len(image_files) < 2:
        return False

    similar_count = 0
    total_comparisons = len(image_files) - 1

    for i in range(total_comparisons):
        # 读取相邻两张图片
        img1 = cv2.imread(image_files[i], cv2.IMREAD_GRAYSCALE)
        img2 = cv2.imread(image_files[i + 1], cv2.IMREAD_GRAYSCALE)

        # 调整图片尺寸使其一致
        img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))

        # 计算 SSIM 值
        similarity = ssim(img1, img2)

        if similarity >= threshold:
            similar_count += 1

    # 计算相似图片的比例
    similarity_ratio = similar_count / total_comparisons
    return similarity_ratio >= stationary_percentage

def check_bag_standby(bag):
    img_bag = bag.replace("bag", "img")
    print(f"{img_bag}")
    clips = os.listdir(img_bag)
    clips.sort()
    for clip in clips:
        clip_path = os.path.join(img_bag, clip)
        if is_vehicle_stationary(clip_path):
            print(f"{clip_path} is stationary")
            shutil.rmtree(clip_path)

def get_od_bag_folders(target_folder):
    valid_bag_dirs = []
    if not os.path.isdir(target_folder):
        print(f"错误：目标文件夹 {target_folder} 不存在！")
        return valid_bag_dirs

    for item in os.listdir(target_folder):
        item_path = os.path.join(target_folder, item)
        if "-clip-20df" in item:
            valid_bag_dirs.append(item_path)
    return valid_bag_dirs

def gen_pic_link_summary(bag):
    target_day_path = os.path.dirname(os.path.dirname(bag)).replace("bag", "img_summary")
    pics = os.listdir(os.path.join(bag, "cam_front_right"))
    for pic in pics[::1000]:
        # 创建软连接
        pic_dir = os.path.join(bag, "cam_front_right", pic)
        soft_link_path = os.path.join(target_day_path, pic)
        # 计算相对路径
        relative_img_save_path = os.path.relpath(target_day_path, start=os.path.dirname(soft_link_path))
        # os.symlink(relative_img_save_path, soft_link_path)
        if not os.path.exists(relative_img_save_path):
            os.symlink(pic_dir, relative_img_save_path)

def gen_pic_link(bag):
    target_bag_dir = bag.replace("bag", "img")
    pics = os.listdir(os.path.join(bag, "cam_front_right"))
    pics.sort()
    index = 0
    for pic in pics[::5]:
        # 创建软连接
        clip_index = int(index / 40)
        index += 1
        target_path = os.path.join(target_bag_dir, os.path.basename(target_bag_dir) + "_" + str(clip_index))
        if not os.path.exists(target_path):
            os.makedirs(target_path)
        pic_dir = os.path.join(bag, "cam_front_right", pic)
        soft_link_path = os.path.join(target_path, pic)
        # 计算相对路径
        relative_img_save_path = os.path.relpath(pic_dir, start=os.path.dirname(soft_link_path))
        os.symlink(relative_img_save_path, soft_link_path)

def get_total_bags(dir_root):
    target_root = dir_root.replace("bag", "img")
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    day_dirs = []
    day_dirs.append("20250531")
    day_dirs.append("20250601")
    day_dirs.append("20250602")

    vin_codes = []
    vin_codes.append("0576")
    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            target_day = os.path.join(target_root, day)
            if not os.path.exists(target_day):
                os.makedirs(target_day)
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            if vin_codes:
                car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)

    total_bags = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bags.extend(get_valid_bag_folders(dir))
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-4A/bag/20250323/E371_6956_20250323_1443/2025-03-23-14-45-13")
    total_bags.sort()
    return total_bags

def gen_bag_pic(dir_root):
    total_bag_dirs = get_total_bags(dir_root, get_valid_bag_folders)
    for bag in total_bag_dirs:
        # gen_pic_link_summary(bag)
        gen_pic_link(bag)

def rename_od_cc_clips(dir_root):
    total_od_dirs = get_total_bags(dir_root, get_od_bag_folders)
    for od_bag in total_od_dirs:
        img_bag = od_bag.replace("-clip-20df", "").replace("J6M-S03", "img/J6M-S03")
        if os.path.isdir(img_bag):
            for img_clip in os.listdir(img_bag):
                clips = os.listdir(od_bag)
                for clip in clips:
                    if img_clip == clip + "-cc":
                        abs_clip = os.path.join(od_bag, clip)
                        os.rename(abs_clip, abs_clip + "-cc")

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    dir_root = "/home/geely/nas/PLD-S6A/bag"
    # gen_bag_pic(dir_root)
    # rename_od_cc_clips(dir_root)
    total_bags = get_total_bags(dir_root)
    for bag in total_bags:
        check_bag_standby(bag)