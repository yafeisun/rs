import os
from ultralytics import YOLO
import re
import shutil

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

def detect_people_in_image_yolo(image_path, model):
    # 使用 YOLOv8 模型进行检测
    results = model(image_path)
    
    # 检查检测结果中是否有人
    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = int(box.cls[0])
            if class_id == 0:  # 0 表示人
                return True
    return False

def check_images_in_folders_yolo(folder_list):
    # 加载 YOLOv8 模型
    model = YOLO('yolov8n.pt')

    for folder in folder_list:
        if not os.path.isdir(folder):
            print(f"文件夹不存在: {folder}")
            continue
        files = os.listdir(folder)
        for file in files:
            if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(folder, file)
                if detect_people_in_image_yolo(image_path, model):
                    new_img_path = image_path.replace("bag", "img_tuomin")
                    target_folder = os.path.dirname(new_img_path)
                    if not os.path.exists(target_folder):
                        os.makedirs(target_folder)
                    shutil.copy2(image_path, new_img_path)
                    print(f"图片 {image_path} 中有人")

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
   
if __name__ == "__main__":
    # 获取目标文件夹路径参数
    dir_root = "/home/geely/nas/PLD-S6A/bag"
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250531")

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

    total_bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(dir))
    total_bag_dirs.sort()

    total_img_dirs = []
    for bag in total_bag_dirs:
        for img in vision_folder_list:
            total_img_dirs.append(os.path.join(bag, img))

    check_images_in_folders_yolo(total_img_dirs)
