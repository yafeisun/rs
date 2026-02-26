import os
import re
# import yaml
from ruamel.yaml import YAML  # 替换原来的yaml导入
import shutil

def check_calibration_yaml(yaml_path):
    """
    检查并修改标定yaml文件中的parse字段
    :param yaml_path: 标定yaml文件路径
    """
    if os.path.exists(yaml_path):
        yaml = YAML()
        yaml.preserve_quotes = True  # 保留引号样式
        yaml.compact_seq_seq = True  # 紧凑序列格式
        yaml.indent(mapping=2, sequence=4, offset=2)  # 设置缩进
        
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.load(f)
        
        modified = False
        if 'sensors' in data and 'camera' in data['sensors']:
            # 检查每个camera的topic字段
            nodes = data['sensors'].get('camera', [])
            if not nodes:
                print(f"{yaml_path}的'camera'节点为空")
            else:
                for node in nodes:
                    topic = node.get('topic', '')
                    parse = node.get('parse', '')
                    if topic != "" and parse == True:
                        node['parse'] = False
                        modified = True
        
        if modified:
            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f)
            print(f"{yaml_path} 已修改parse字段为false")

    else:
        print(f"{yaml_path} not exist")

def distribute_yaml(yaml_path, dst_dir):
    retval = True  # 初始化为 True
    info_yaml_file_path = os.path.join(dst_dir, 'info.yaml')
    if os.path.exists(info_yaml_file_path):
        # Create an instance of the YAML class
        yaml = YAML()
        with open(info_yaml_file_path, 'r', encoding='utf-8') as f:
            # Use the instance to call the load method
            data = yaml.load(f)
            car_label = data.get('car_label', '')
            # 检查是否有标定yaml格式文件
            car_yaml_files = [f for f in os.listdir(dst_dir) 
                                if f.startswith('car_') and f.endswith('.yaml')]
            # 添加删除逻辑
            for file in car_yaml_files:
                file_path = os.path.join(dst_dir, file)
                os.remove(file_path)
            # 构造目标文件路径
            dst_file_path = os.path.join(dst_dir, f"{car_label}.yaml")
            try:
                # 拷贝文件
                shutil.copy(yaml_path, dst_file_path)
                print(f"copy {yaml_path} to {dst_dir}")
            except Exception as e:
                print(f"拷贝文件时出错: {e}")

def distribute_bag_folder(bag_path, yaml_path):
    distribute_yaml(yaml_path, bag_path)
    if os.path.isdir(os.path.join(bag_path, "clips")):
        total_clip_paths = os.listdir(os.path.join(bag_path, "clips"))
        for item in total_clip_paths:
            distribute_yaml(yaml_path, os.path.join(bag_path, "clips", item))

if __name__ == "__main__":
    # 给定yaml地址
    VIN_CODE = "0036"
    yaml_path = "/home/geely/nas/J6M-S6E/1131/car_L946_L6T79END5RT000036__2024-11-12.yaml"
    # 检查标定yaml文件内容是否正确
    # check_calibration_yaml(yaml_path)

    dir_root = "/home/geely/nas/Data/record"
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20251207")
    # day_dirs.append("20251208")
    # day_dirs.append("20251130")
    # day_dirs.append("20251201")
    
    abs_car_dirs = []
    for day_dir in day_dirs:        
        car_dirs = os.listdir(os.path.join(dir_root, day_dir))
        # 筛选出以下划线分割的第二个元素为VIN_CODE的目录
        # car_dirs = [d for d in car_dirs if len(d.split('_')) > 1 and d.split('_')[1] == VIN_CODE]
        car_dirs.sort()
        abs_car_dirs.extend(os.path.abspath(os.path.join(dir_root, day_dir, d)) for d in car_dirs)
    
    abs_bag_dirs = []
    for car_dir in abs_car_dirs:
        bag_dirs = os.listdir(car_dir)
        # 使用正则表达式匹配 YYYY-MM-DD-HH-MM-SS 格式
        bag_pattern = re.compile(r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$')
        # bag_dirs = [d for d in bag_dirs if bag_pattern.match(d)]
        # current_abs_bag_dirs = list(os.path.join(car_dir, d) for d in bag_dirs)
        abs_bag_dirs.extend(list(os.path.abspath(os.path.join(car_dir, d)) for d in bag_dirs))
    
    for bag in abs_bag_dirs:
        distribute_bag_folder(bag, yaml_path)