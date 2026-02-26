import os
from ultralytics import YOLO
import re
import shutil
import cv2
from skimage.metrics import structural_similarity as ssim

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

def detect_face_in_image_yolo(image_path, model):
    # 使用 YOLOv8 模型进行检测
    results = model(image_path)
    
    # 检查检测结果中是否有人
    for result in results:
        faces = result.faces
        for face in faces:
            class_id = int(face.cls[0])
            if class_id == 0:  # 0 表示human face
                return True
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

def get_car_num(image_path, model):
    # 使用 YOLOv8 模型进行检测
    results = model(image_path)
    total_boxes = 0
    # 检查检测结果中是否有人
    for result in results:
        boxes = result.boxes
        total_boxes += len(boxes)
        # for box in boxes:
        #     class_id = int(box.cls[0])
        #     if class_id == 10:  # 2 表示car
        #         return True
    return total_boxes

def check_images_in_folders_yolo(folder_list):
    # 加载 YOLOv8 模型
    model = YOLO('yolov8n.pt')
    # print(model.names)
    for folder in folder_list:
        if not os.path.isdir(folder):
            print(f"文件夹不存在: {folder}")
            continue

        # target_folder = folder.replace("bag", "img_tuomin")
        target_folder = folder.replace("Geely-f", "Geely-f/img_tuomin_yolo")
        if not os.path.exists(target_folder):
            os.makedirs(target_folder)
            files = os.listdir(folder)
            files.sort()
            files = files[::10]
            for file in files:
                if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                    image_path = os.path.join(folder, file)
                    new_img_path = os.path.join(target_folder, file)
                    if detect_people_in_image_yolo(image_path, model):
                    # if detect_face_in_image_yolo(image_path, model):
                        # shutil.copy2(image_path, new_img_path)
                        soft_link_path = os.path.join(target_folder, file)
                        # 计算相对路径
                        relative_img_save_path = os.path.relpath(image_path, start=os.path.dirname(soft_link_path))
                        os.symlink(relative_img_save_path, soft_link_path)
                        # print(f"图片 {image_path} 中有人")

def is_clip_object_enough(clip, model):
    frames = os.listdir(clip)
    frames = [d for d in frames if os.path.isdir(os.path.join(clip, d)) and d.isdigit()]
    frames.sort()
    frame_num = 0 # 可标注帧
    for frame in frames:
        files = os.listdir(os.path.join(clip, frame))
        files = [d for d in files if ".jpg" in d]
        files.sort()

        object_num = 0
        for file in files:
            image_path = os.path.join(clip, frame, file)
            if os.path.getsize(image_path) > 0:
                object_num += get_car_num(image_path, model)
        # 11路相机中可标注物数量太少，不予下发标注
        if object_num > 15:
            frame_num += 1

    if frame_num > int(0.7 * len(frames)):
        return True
    else:
        return False

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

def is_clip_stationary(clip):
    frames = os.listdir(clip)
    frames = [d for d in frames if os.path.isdir(os.path.join(clip, d)) and d.isdigit()]
    frames.sort()
    img_path = os.path.join(clip, "cam_front_right")
    if not os.path.exists(img_path):
        os.makedirs(img_path)
        for frame in frames:
            pic_dir = os.path.join(clip, frame, frame + "__front.jpg")
            if os.path.exists(pic_dir):
                soft_link_path = os.path.join(img_path, os.path.basename(pic_dir))
                relative_img_save_path = os.path.relpath(pic_dir, start=os.path.dirname(soft_link_path))
                if not os.path.exists(relative_img_save_path):
                    os.symlink(relative_img_save_path, soft_link_path)
    if is_vehicle_stationary(img_path):
        return True
    else:
        # shutil.rmtree(img_path)
        return False

def filter_od_clips(bag):
    clips = os.listdir(bag)
    clips = [d for d in clips if "2025" in d]
    clips.sort()
    for clip in clips:
        clip_path = os.path.join(bag, clip)
        # if "-x" in clip:
        #     shutil.rmtree(clip_path)
        # else:
        #     img_path = os.path.join(clip_path, "cam_front_right")
        #     if os.path.exists(img_path):
        #         shutil.rmtree(img_path)
        if is_clip_stationary(clip_path) or not is_clip_object_enough(clip_path, model):
            # 可标注帧<30 整个clip不予标注
            os.rename(clip_path, clip_path + "-x")
            # shutil.rmtree(clip_path)

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    # dir_root = "/home/geely/nas/PLD-S6A/bag"
    dir_root = "/home/geely/nas/PLD-S6B/4D-OD"
    if os.path.isdir(dir_root):
        day_dirs = os.listdir(dir_root)
        day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250603")
    # day_dirs.append("20250604")
    # day_dirs.append("20250606")

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
    abs_car_dirs = []
    # abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250611_1442-5855")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250611_1839-5354")
    abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250612_1051-5722")
    abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250612_1522-5877")
    abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250612_1622-5785")
    abs_car_dirs.append("/home/geely/nas/PLD-S6B/4D-OD/J6M_0302_20250612_1921-6883")

    total_bag_dirs = []
    for dir in abs_car_dirs:
        bags = os.listdir(dir)
        bags = [d for d in bags if os.path.isdir(os.path.join(dir, d))]
        bags = [d for d in bags if d.endswith("-clip-20df")]
        total_bag_dirs.extend(os.path.abspath(os.path.join(dir, d)) for d in bags)
    total_bag_dirs.sort()
    # total_bag_dirs.append("/home/geely/nas/PLD-S6A/PLD-OD/20250609/J6M_0576_20250602_1556/2025-06-02-18-12-37-clip-20df")
    # total_bag_dirs.append("/home/geely/nas/PLD-S6A/PLD-OD/20250609/J6M_0576_20250602_1556/2025-06-02-18-17-37-clip-20df")
    model = YOLO('yolov8n.pt')
    print(model.names)

    for bag in total_bag_dirs:
        filter_od_clips(bag)

