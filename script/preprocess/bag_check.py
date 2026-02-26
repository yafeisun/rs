import os
import re
import yaml
from ruamel.yaml import YAML  # 替换原来的yaml导入
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

YAML_ROOT = "/home/geely/Documents/robosense/yaml"
vin_dict = {
    "1170": "3484",
    "1171": "3483",
    "1172": "6956",
    "1174": "6477",
    "1173": "3486",
    "1131": "0036",
    "1138": "0143",
    "1139": "0014"
}
car_type_dict = {
    "3484": "E371",
    "3483": "E371",
    "6956": "E371",
    "6477": "E371",
    "3486": "E371",
    "0036": "L946",
    "0143": "L946",
    "0014": "L946",
    "0252": "J6M",
    "1900": "P162"
}
def is_valid_time_format(folder_name):
    """
    检查文件夹名称是否符合 YYYY-MM-DD-HH-MM-SS 的时间格式
    :param folder_name: 文件夹名称
    :return: 如果符合时间格式返回 True,否则返回 False
    """
    pattern = r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$'
    return bool(re.match(pattern, folder_name))

def check_jpg_num(bag_folder_path, record_time):
    jpg_counts = []
    cam_folders = [f for f in os.listdir(bag_folder_path) 
                if f.startswith('cam_') and os.path.isdir(os.path.join(bag_folder_path, f))]
    for cam_folder in cam_folders:
        cam_path = os.path.join(bag_folder_path, cam_folder)
        jpg_files = [f for f in os.listdir(cam_path) if f.endswith('.jpg')]
        jpg_counts.append(len(jpg_files))
    
    if not jpg_counts:
        return False
    
    max_count = max(jpg_counts)
    min_count = min(jpg_counts)
    
    if min_count < 9 * record_time:
        print(f"{bag_folder_path} jpg num too small {min_count} < 90% * {10 * record_time}")
        remove_folder(bag_folder_path)
    return True

def check_vision_folder(bag_folder_path):
    """
    检查 cam_ 开头的文件夹
    :param bag_folder_path: bag 文件夹路径
    :return: 如果 jpg 文件数量差异小于等于 150 返回 True，否则返回 False
    """
    cam_folders = [f for f in os.listdir(bag_folder_path) 
                  if f.startswith('cam_') and os.path.isdir(os.path.join(bag_folder_path, f))]
    
    if not cam_folders:
        print(f"{bag_folder_path} 下没有 cam_ 开头的文件夹")
        return False
    
    if len(cam_folders) < 11:
        print(f"{bag_folder_path} 11V漏采相机")
        return False    
    return True

def distribute_yaml(yaml_path):
    vin_code = os.path.basename(yaml_path).split('_')[2][-4:]
    for item in os.listdir(YAML_ROOT):
        if vin_code == item.split('_')[2][-4:]:
            shutil.copy(os.path.join(YAML_ROOT, item), yaml_path)
            break
        
def check_bag_folder(bag_folder_path):
    """
    处理 label.yaml 文件，计算每个 text 的时间总和
    :param yaml_file_path: label.yaml 文件的路径
    :return: 如果所有检查通过返回 True，否则返回 False
    """
    retval = True  # 初始化为 True
            
    # 检查 rsOriGnssInfo.txt 文件是否存在
    if not os.path.exists(os.path.join(bag_folder_path, "rsOriGnssInfo.txt")):
        print(f"{bag_folder_path}/rsOriGnssInfo.txt not exist")
        # retval = False
        
    # 检查 label.yaml 文件是否存在
    if not os.path.exists(os.path.join(bag_folder_path, 'label.yaml')):
        print(f"{bag_folder_path}/label.yaml not exist")
        retval = False
        
    # 检查 info.yaml 文件是否存在
    info_yaml_file_path = os.path.join(bag_folder_path, 'info.yaml')
    if os.path.exists(info_yaml_file_path):
        with open(info_yaml_file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            record_time = data.get('record_time', '')
            bag_name = data.get('bag_name', '')
            car_label = data.get('car_label', '')
            
            # bag包是否存在
            if not os.path.exists(os.path.join(bag_folder_path, bag_name)):
                print(f"{bag_folder_path}/{bag_name} not exist")
                retval = False
        
            # 检查是否有标定yaml格式文件
            car_yaml_files = [f for f in os.listdir(bag_folder_path) 
                                if f.startswith('car_') and f.endswith('.yaml')]
            # 检查标定yaml文件名称是否与info.yaml配置一致
            if len(car_yaml_files) == 1:
                # 只有一个car_开头的yaml文件
                existing_file = car_yaml_files[0]
                calibration_yaml_path = os.path.join(bag_folder_path, existing_file)
                if existing_file != car_label + '.yaml':
                    # 文件名不一致，进行重命名
                    old_path = os.path.join(bag_folder_path, existing_file)
                    new_path = os.path.join(bag_folder_path, car_label + '.yaml')
                    os.rename(old_path, new_path)
                    print(f"{bag_folder_path} 标定文件名称不一致 rename {existing_file} -> {car_label}.yaml")
                    calibration_yaml_path = new_path
                # 检查标定yaml文件内容是否正确
                check_calibration_yaml(calibration_yaml_path)
            elif len(car_yaml_files) > 1:
                # 有多个car_开头的yaml文件
                print(f"{bag_folder_path} 路径下有多个标定文件，请检查")
                retval = False
            else:
                # 没有car_开头的yaml文件
                print(f"{bag_folder_path} {car_label}.yaml not exist, get and distribute...")
                distribute_yaml(os.path.join(bag_folder_path, car_label + '.yaml'))
                
            # 检查录制时长
            if record_time < 20:
                print(f"{bag_folder_path} 录制时长小于20s")
                remove_folder(bag_folder_path)
                # retval = False
            elif record_time > 300:
                print(f"{bag_folder_path} 录制时长大于300s")
                retval = False
            else:
                retval = check_jpg_num(bag_folder_path, record_time)
    else:
        print(f"{info_yaml_file_path} not exist")
        retval = False

    clips_path = os.path.join(bag_folder_path, "clips")
    if os.path.isdir(clips_path):
        clips = os.listdir(clips_path)
        for item in clips:
            item_path = os.path.join(clips_path, item)
            if not check_jpg_num(item_path, 20):
                new_item = item
                new_item = new_item.replace("-cc", "")
                new_item = new_item.replace("-isp", "")
                new_folder_path = os.path.join(clips_path, new_item) + "-x"
                print(f"rename {item} -> {new_folder_path}")
                os.rename(item_path, new_folder_path)
    return retval

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
        
def get_x_bag_folders(target_folder):
    x_bag_dirs = []
    if not os.path.isdir(target_folder):
        print(f"Error: folder {target_folder} not exist!")
        return x_bag_dirs

    for item in os.listdir(target_folder):
        item_path = os.path.join(target_folder, item)
        if os.path.isdir(item_path) and not item.startswith('.'):
            if item.endswith("-x"):
                x_bag_dirs.append(item_path)
                
    return x_bag_dirs

def get_valid_bag_folders(car_dir):
    """
    获取目标文件夹下符合 YYYY-MM-DD-HH-MM-SS 格式的子文件夹路径
    
    :param target_folder: 目标文件夹路径
    :return: 有效子文件夹路径列表
    """
    valid_bag_dirs = []
    if not os.path.isdir(car_dir):
        print(f"Error: folder {car_dir} not exist!")
        return valid_bag_dirs
    
    bag_dirs = os.listdir(car_dir)
    bag_dirs.sort()
    car_time = int(os.path.basename(car_dir).split('_')[3][0:4])
    for item in bag_dirs:
        if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item):
            bag_time = int(item.split('-')[3]) * 100 + int(item.split('-')[4])
            if car_time - 10 > bag_time:
                print(f"{car_dir} time dismatch {item}")
    
    for item in bag_dirs:
        item_path = os.path.join(car_dir, item)
        if os.path.isdir(item_path) and not item.startswith('.'):
            if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item):
                valid_bag_dirs.append(item_path)
            elif not item.endswith('-clip-20df'):
                if item.isdigit() and len(item) == 10:
                    print(f"removing {item_path}")
                    shutil.rmtree(item_path)
                elif not item.endswith("-x"):
                    print(f"bag folder format invalid {item_path}")
    valid_bag_dirs.sort()
    return valid_bag_dirs

def get_valid_car_folders(abs_day_dirs, vin_codes):
    retval = []
    abs_day_dirs.sort()
    for day_dir in abs_day_dirs:
        car_dirs = os.listdir(day_dir)
        car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
        car_dirs.sort()
        for item in car_dirs:
            new_folder_name = item
            if "__" in item:
                # __ invalid 
                new_folder_name = item.replace("__", "_")
            if len(item.split('_')[2]) != 8:
                # YYYYMMDD
                print(f"{os.path.join(day_dir, item)} invalid")
            else:
                # vincode error
                key = item.split('_')[1]
                if key in vin_dict:
                    vincode = vin_dict[key]
                    new_folder_name = new_folder_name.replace(key, vin_dict[key])
                else:
                    vincode = key
                # car_type error
                car_type = item.split('_')[0]
                if car_type != car_type_dict[vincode]:
                    new_folder_name = new_folder_name.replace(car_type, car_type_dict[vincode])
            if new_folder_name != item:
                os.rename(os.path.join(day_dir, item), os.path.join(day_dir, new_folder_name))
                print(f"rename {item} -> {os.path.join(day_dir, new_folder_name)}")
            
        car_dirs = os.listdir(day_dir)
        car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
        car_dirs.sort()
        # 筛选出以下划线分割的第二个元素为VIN_CODE的目录
        if not vin_codes:
            retval.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
        else:
            retval.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs if d.split('_')[1] in vin_codes)
    retval.sort()
    return retval

def remove_folder(item):
    try:
        print(f"delete {item}")
        shutil.rmtree(item)
    except Exception as e:
        print(f"delete {item} error")

if __name__ == '__main__':
    # 输入文件夹路径
    dir_root = "/home/geely/nas/Data/record"
    day_dirs = os.listdir(dir_root)
    day_dirs = [d for d in day_dirs if d.isdigit()]
    # day_dirs = []
    # day_dirs.append("20251207")
    # day_dirs.append("20251208")
    # day_dirs.append("20251130")
    # day_dirs.append("20251201")

    vin_codes = []
    # vin_codes.append("0014")
    # vin_codes.append("6956")
    abs_day_dirs = list(os.path.abspath(os.path.join(dir_root, d)) for d in day_dirs if os.path.isdir(os.path.join(dir_root, d)))
    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            if vin_codes:
                car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            # car_dirs = [d for d in car_dirs if "J6M" in d]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)
    # abs_car_dirs = get_valid_car_folders(abs_day_dirs, vin_codes)

    # abs_car_dirs = []
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1344")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1510")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1610")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1745")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1753")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1758")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1828")
    # abs_car_dirs.append("/home/geely/nas/PLD-S6A/bag/20250530/J6M_0576_20250530_1933")

    total_bag_dirs = []
    for car_path in abs_car_dirs:
        # 获取目标文件夹下的bag文件夹
        total_bag_dirs.extend(get_valid_bag_folders(car_path)) # first check
        # total_bag_dirs.extend(get_x_bag_folders(car_path)) # double check

    # 指定处理某个bag文件夹
    # total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/J6M-3C/bag/20250319/E371_6956_20250319_0829/2025-03-19-08-48-42")
    
    total_bag_dirs.sort()
    valid_bag_num = 0
    for item in total_bag_dirs:
        if not check_vision_folder(item):
            remove_folder(item)
        elif check_bag_folder(item):
            valid_bag_num += 1
        elif not item.endswith("-x"):
            # 修改不通过的文件夹名称
            print(f"rename -> {new_folder_path}")
            new_folder_path = item + "-x"
            os.rename(item, new_folder_path)

    print(f"total bag:{len(total_bag_dirs)}; valid bag:{valid_bag_num}")
    
