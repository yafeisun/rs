import os
import shutil
import re

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

def gen_pic_link(bag, target_bag_dir):
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

        # if not os.path.exists(relative_img_save_path):
        #     os.symlink(soft_link_path, relative_img_save_path)
def copy_with_override(src, dst):
    if os.path.exists(dst):
        os.remove(dst)
    shutil.copy(src, dst)

def recover_pic_link(bag, source_bag):
    folders = os.listdir(bag)
    for folder in folders:
        imgs = os.listdir(os.path.join(bag, folder))
        imgs = [d for d in imgs if ".jpg" in d]
        if imgs:
            for img in imgs:
                img_path = os.path.join(bag, folder, img)
                source_img = os.path.join(source_bag, folder, img)
                copy_with_override(source_img, img_path)


if __name__ == "__main__":
    # 获取目标文件夹路径参数
    dir_root = "/home/geely/nas/Geely-f/img_tuomin/J6M-S11"
    target_root = "/home/geely/nas/Geely-f/J6M-S11"
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250531")
    # day_dirs.append("20250601")
    # day_dirs.append("20250602")
    vin_codes = []
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

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))

    total_bag_dirs.sort()
    for bag in total_bag_dirs:
        # gen_pic_link_summary(bag)
        # gen_pic_link(bag, bag.replace(dir_root, target_root))
        recover_pic_link(bag, bag.replace(dir_root, target_root))