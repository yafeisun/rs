import os
import re
from PIL import Image

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

side_list_ordered = [
    "cam_front_right",
    "cam_side_left_front",
    "cam_side_right_front",
    "cam_back",
    "cam_side_left_back",
    "cam_side_right_back",
]

def stitch_images(image_paths):
    """
    Stitch 4 images into a 2x2 grid or 6 images into a 2x3 grid composite image.
    Resize all images to a uniform resolution before stitching.
    """
    num_images = len(image_paths)
    if num_images not in [2, 4, 6]:
        raise ValueError("This function is designed to stitch either 4 or 6 images.")
    
    images = [Image.open(img_path) for img_path in image_paths]
    
    # Calculate the average width and height
    total_width = sum(img.size[0] for img in images)
    total_height = sum(img.size[1] for img in images)
    avg_width = total_width // num_images
    avg_height = total_height // num_images
    
    # Resize all images to the average width and height
    resized_images = [img.resize((avg_width, avg_height), Image.LANCZOS) for img in images]
    
    if num_images == 2:
        # Stitch 4 images in 2x2 grid
        total_width = avg_width * 2
        total_height = avg_height
        new_image = Image.new('RGB', (total_width, total_height))
        new_image.paste(resized_images[0], (0, 0))
        new_image.paste(resized_images[1], (avg_width, 0))
        # new_image.paste(resized_images[2], (0, avg_height))
        # new_image.paste(resized_images[3], (avg_width, avg_height))

    elif num_images == 4:
        # Stitch 4 images in 2x2 grid
        total_width = avg_width * 2
        total_height = avg_height * 2
        new_image = Image.new('RGB', (total_width, total_height))
        new_image.paste(resized_images[0], (0, 0))
        new_image.paste(resized_images[1], (avg_width, 0))
        new_image.paste(resized_images[2], (0, avg_height))
        new_image.paste(resized_images[3], (avg_width, avg_height))
    else:
        # Stitch 6 images in 2x3 grid
        total_width = avg_width * 3
        total_height = avg_height * 2
        new_image = Image.new('RGB', (total_width, total_height))
        # Paste first row
        x_offset = 0
        for i in range(3):
            new_image.paste(resized_images[i], (x_offset, 0))
            x_offset += avg_width
        # Paste second row
        x_offset = 0
        for i in range(3, 6):
            new_image.paste(resized_images[i], (x_offset, avg_height))
            x_offset += avg_width

    return new_image


def find_common_images(bag_path):
    # 存储每个相机的图片时间戳列表
    timestamp_lists = {}
    for item in vision_folder_list:
        image_folder_path = os.path.join(bag_path, item)
        timestamps = [int(os.path.splitext(img)[0]) for img in os.listdir(image_folder_path) if '.@__thumb' not in img]
        timestamps.sort()
        timestamp_lists[item] = timestamps
        # print(f"total {item} {len(timestamp_lists[item])}")

    # 初始化指针
    pointers = {item: 0 for item in vision_folder_list}
    common_dict = {item: [] for item in vision_folder_list}

    while all(pointers[item] < len(timestamp_lists[item]) for item in vision_folder_list):
        # 获取当前指针指向的时间戳
        current_timestamps = {item: timestamp_lists[item][pointers[item]] for item in vision_folder_list}
        # 计算时间戳的最大值和最小值
        max_timestamp = max(current_timestamps.values())
        min_timestamp = min(current_timestamps.values())

        # 检查时间差是否在 20ms 内（20ms = 20 * 1000000 ns）
        if max_timestamp - min_timestamp <= 40 * 1000000:
            # 如果时间差在 20ms 内，将该时间戳对应的图片添加到公共字典中
            for item in vision_folder_list:
                timestamp = current_timestamps[item]
                common_dict[item].append(str(timestamp) + ".jpg")
            # 所有指针向后移动一位
            for item in vision_folder_list:
                pointers[item] += 1
        else:
            # 如果时间差超过 20ms，将最小时间戳对应的相机指针向后移动一位
            min_item = min(current_timestamps, key=current_timestamps.get)
            pointers[min_item] += 1
    for item in vision_folder_list:
        common_dict[item].sort()
        # print(f"common {item} {len(common_dict[item])}")
    return common_dict

def gen_around_side(bag, img_list, common_imgs, target_dir):
    flag = False    
    img_count = len(common_imgs["cam_around_back"])
    new_target_dir = target_dir.replace(target_root, target_root_2)
    if not os.path.exists(target_dir):
        if not os.path.exists(new_target_dir):
            os.makedirs(new_target_dir)
            flag = True
    # elif img_count != len(os.listdir(target_dir)):
    #     flag = True
    if flag:
        for i in range(img_count):
            imgs = []
            clip_index = str(int(i / 40))
            for folder in img_list:
                imgs.append(os.path.join(bag, folder, common_imgs[folder][i]))
            target_clip_dir = os.path.join(new_target_dir, os.path.basename(bag) + '_' + clip_index)
            if img_list == around_list:
                # target_folder = os.path.join(target_clip_dir, "around")
                target_folder = target_clip_dir
            elif img_list == side_list_ordered:
                target_folder = os.path.join(target_clip_dir, "side")
            else:
                target_folder = target_clip_dir
            
            if not os.path.exists(target_folder):
                os.makedirs(target_folder)
            composite_image = stitch_images(imgs)
            composite_image_path = os.path.join(target_folder, f"{i}.jpg")
            composite_image.save(composite_image_path)

def select_images_per_halfsec(common_dict):
    # 把图片按照帧率抽帧，1秒2张
    selected_dict = {}
    for item, image_names in common_dict.items():
        selected_images = []
        # 记录上一个 0.5 秒的时间戳
        last_half_second = -1
        image_names.sort()
        for image in image_names:
            timestamp = int(os.path.splitext(image)[0])
            # 将时间戳转换为 0.5 秒的间隔
            current_half_second = timestamp // 500000000  

            if current_half_second > last_half_second:
                selected_images.append(image)
                last_half_second = current_half_second

        selected_dict[item] = selected_images
    return selected_dict

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

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    # dir_root = "/home/geely/nas/PLD-S6D/J6M-S08"
    # target_root = "/home/geely/nas/PLD-S6B/p181-huochebanche-img"

    dir_root = "/home/geely/nas/Data11/record/zhaji"
    target_root = "/home/geely/nas/Data11/record/img"
    target_root_2 = target_root
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250724")
    # day_dirs.append("20250725")

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
    # abs_car_dirs.append("/home/geely/nas/PLD-S6D/J6M-S23/20250626/P181_0251_20250626_1346")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))

    # 指定处理某个bag文件夹
    total_bag_dirs = []
    total_bag_dirs.append("/home/geely/nas/Data11/record/zhaji/20251113/s23_0251_20251113_2108/2025-11-13-21-18-23")
    total_bag_dirs.append("/home/geely/nas/Data11/record/zhaji/20251113/s23_0251_20251113_2108/2025-11-13-21-23-23")
    total_bag_dirs.sort()
    for bag in total_bag_dirs:
            common_imgs = find_common_images(bag)
            selected_images = select_images_per_halfsec(common_imgs)
            target_dir = os.path.join(target_root, os.path.basename(os.path.dirname(bag)), os.path.basename(bag))
            around_list = [
                
                "cam_front_right",
                "cam_around_front",
                
                # "cam_around_back",
                # "cam_around_left",
                # "cam_around_right",
            ]
            gen_around_side(bag, around_list, selected_images, target_dir)

            # target_dir = os.path.join(target_root, os.path.basename(os.path.dirname(bag)), os.path.basename(bag))
            # side_list_ordered = [
            #     "cam_front_right",
            #     "cam_side_left_front",
            #     "cam_side_right_front",
            #     "cam_back",
            #     "cam_side_left_back",
            #     "cam_side_right_back",
            # ]            
            # gen_around_side(bag, side_list_ordered, selected_images, target_dir)


    