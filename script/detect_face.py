import os
# os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "cpp"
# from mtcnn import MTCNN
from PIL import Image
import numpy as np
import re
import shutil
from ultralytics import YOLO


# 初始化 MTCNN 检测器
# model_mtcnn = MTCNN()
# 加载 YOLOv8 模型
model_yolo = YOLO('yolov8n.pt')

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

def detect_faces_in_image_mtcnn(image_path, detector):
    try:
        # 读取图片
        image = Image.open(image_path)
        image = np.array(image)

        # 检测人脸
        results = detector.detect_faces(image)

        # 如果检测到人脸，返回 True
        return len(results) > 0
    except Exception as e:
        print(f"处理图片 {image_path} 时出错: {e}")
        return False

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

def check_images_in_folder(folder, target_folder, detector):
    files = os.listdir(folder)
    files.sort()
    files = files[::10]
    for file in files:
        if file.lower().endswith(('.png', '.jpg', '.jpeg')):
            image_path = os.path.join(folder, file)
            new_img_path = os.path.join(target_folder, file)
            # if detect_faces_in_image_mtcnn(image_path, model_mtcnn):# and
            if detect_people_in_image_yolo(image_path, model_yolo):
                soft_link_path = os.path.join(target_folder, file)
                relative_img_save_path = os.path.relpath(image_path, start=os.path.dirname(soft_link_path))
                os.symlink(relative_img_save_path, soft_link_path)

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
    dir_root = "/home/geely/nas/Geely-f/J6M-S03"
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


    for folder in total_img_dirs:
        if not os.path.isdir(folder):
            print(f"文件夹不存在: {folder}")
            continue

        target_folder = folder.replace("Geely-f", "Geely-f/img_face")
        if not os.path.exists(target_folder):
            os.makedirs(target_folder)
            check_images_in_folder(folder, target_folder)

