import os
import shutil

def copy_folder(src, dst):
    """
    复制整个文件夹
    :param src: 源文件夹路径
    :param dst: 目标文件夹路径
    """
    try:
        # 如果目标路径已经存在，先删除
        if os.path.exists(dst):
            shutil.rmtree(dst)
        
        # 复制整个文件夹
        shutil.copytree(src, dst)
        print(f"copy {src} to {dst}")
    except Exception as e:
        print(f"复制文件夹时出错: {e}")


def extract_car_folder(car_path, target_dir):
    bag_dirs = os.listdir(car_path)
    bag_dirs = [d for d in bag_dirs if d.endswith("-clip-20df")]
    bag_dirs.sort()
    abs_bag_paths = [os.path.abspath(os.path.join(car_path, d)) for d in bag_dirs]
    
    for bag_path in abs_bag_paths:
        # 在 car_path 下创建同名文件夹
        new_car_path = os.path.join(target_dir, os.path.basename(car_path))
        if not os.path.exists(new_car_path):
            os.makedirs(new_car_path)

        # 创建bag同名文件夹
        new_bag_path = os.path.join(new_car_path, os.path.basename(bag_path))
        if not os.path.exists(new_bag_path):
            os.makedirs(new_bag_path)
            copy_folder(bag_path, new_bag_path)

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    VIN_CODE = "0036"
    target_dir = "/media/geely/MyPassort1/4D-OD/20250527"
    dirs = []
    dirs.append("20250508") 

    abs_car_paths = []
    for dir in dirs:
        daily_dir = os.path.join("/home/geely/nas/J6M-5B/bag", dir)   
        car_dirs = os.listdir(daily_dir)
        # 筛选出以下划线分割的第二个元素为VIN_CODE的目录
        car_dirs = [d for d in car_dirs if len(d.split('_')) > 1 and d.split('_')[1] == VIN_CODE]
        car_dirs.sort()
        abs_car_paths.extend(os.path.abspath(os.path.join(daily_dir, d)) for d in car_dirs)

    # abs_car_paths = []
    # abs_car_paths.append("/home/geely/nas/J6M-3B/verify/20250326/L946_0014_20250326_0530")

    for car_path in abs_car_paths:
        extract_car_folder(car_path, target_dir)