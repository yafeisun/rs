import os

dir_root = "/home/geely/nas/J6M-S6C/真值bag/0109p"

days = os.listdir(dir_root)
for day in days:
    day_path = os.path.join(dir_root, day)
    if not os.path.isdir(day_path):  # 确保是目录
        continue
        
    bags = os.listdir(day_path)
    for bag in bags:
        bag_path = os.path.join(dir_root, day, bag)
        if not os.path.isdir(bag_path):  # 确保是目录
            continue
            
        cams = os.listdir(bag_path)
        cams = [d for d in cams if d.startswith("cam") and os.path.isdir(os.path.join(bag_path, d))]
        for cam in cams:
            cam_path = os.path.join(dir_root, day, bag, cam)
            jpgs = os.listdir(cam_path)
            jpgs = [d for d in jpgs if d.endswith(".jpeg")]
            for jpg in jpgs:
                old_path = os.path.join(dir_root, day, bag, cam, jpg)
                new_jpg_name = jpg.replace(".jpeg", ".jpg")
                new_path = os.path.join(dir_root, day, bag, cam, new_jpg_name)
                
                try:
                    os.rename(old_path, new_path)
                    print(f"重命名成功: {old_path} -> {new_path}")
                except Exception as e:
                    print(f"重命名失败: {old_path}, 错误: {e}")