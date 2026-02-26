import os
import shutil
import re
import time
import subprocess
import yaml
import pandas as pd
import csv

sceneDict = {
    "夜间眩光": ["夜晚有光照-B5", "雨天夜间眩光"],
    "闸机": ["闸机"],
    "正前方极近车辆":["正前方极近车辆"],
    "行车：异形车":["异形车"],
    "路边并排行人":["并排打伞行人"],
    "人推车":["人推车"],
    "雾霾":["雾霾-B14"],
    "被遮挡的锥桶":["被遮挡的锥桶"],
    "地上地下园区/地库/停车场":["地上标准-B49","地上园区停车场-B49","地下车库-B44"],
    "跟车":["跟车-A28"],
    "非机动车参与交通":["跟二三轮车行使-A29"],
    "密集场景": ["密集场景", "集市场景", "密集行人", "拥堵-B53", "人流量高-B54"]
    
    # "地下车库大曲率坡道": ["地下车库大曲率坡道-A35", "地下车库大曲率坡道"],
    
    # "路口场景": ["匝道汇入主干道-A35","主干道汇入匝道-A36","丁字路口-B19","分合流-B15","匝道-B45","Y型匝道","十字路口-B21",
    #          "横穿路口-B45","左转待转区-B38","直行待转区-B37","无保护路口-B46","路口直行","路口掉头-A5","特殊路口（Y字，5岔路以上）"],
    # "大弯道":["大弯道"],
    # "收费站":["收费站-B27"],
    # "三轮车":["二三轮车"],
    # "环岛场景":["环岛-B41"],
    # "切入切出":["cutin-A60","cutout-A61"],
    # "变道超车":["超车/被超车-A57","自车超大车大车超自车"],
    # "城区道路场景左转右转掉头环岛":["右转专用道-B42","环岛-B41","左转待转区-B38","掉头车道-B43","左转待转区-B38","路口掉头-A5","向左转弯-A4","向右转弯-A6"]
            }

testDict = {
    # scene: [light:title, weather:text, road_type:title, labels:text]
    "白天城区": [["光照-白天"], ["晴天-B7", "阴天-B8"], ["道路类型-城区"], 
                ["地下-B44", "地下-B28", "道路静态障碍物", "道路上有静态障碍物", "被遮挡的锥桶", "闸机", "禁停牌"]],

    "白天高架": [["光照-白天"], ["晴天-B7", "阴天-B8"], ["道路类型-高速高架"], 
                ["道路静态障碍物", "被遮挡的锥桶", "临时减速/刹停避开非机动车-A40", "非机动车道", "道路上有静态障碍物"]],

    "夜晚城区": [["光照-夜晚"], ["晴天-B7", "阴天-B8"], ["道路类型-城区"], 
                ["地下-B44", "地下-B28", "道路静态障碍物", "道路上有静态障碍物", "被遮挡的锥桶", "闸机", "道路静态障碍物地下-B28", "禁停牌"]],

    "夜晚高架": [["光照-夜晚"], ["晴天-B7", "阴天-B8"], ["道路类型-高速高架"], 
                ["异形车", "异形车辆", "道路静态障碍物", "道路上有静态障碍物", "被遮挡的锥桶", "临时减速/刹停避开非机动车-A40", "非机动车道"]],

    "白天雨天城区": [["光照-白天"], ["雨天-B9"], ["道路类型-城区"], 
                ["异形车", "异形车辆", "地下-B44", "地下-B28", "道路静态障碍物", "道路上有静态障碍物", "被遮挡的锥桶", "道路施工", "闸机", 
                 "禁停牌", "密集行人", "人流量高", "二三轮车", "并排打伞行人", "横穿行人-A80", "等待行人通过-A34", "停车避让车前行人-A12", 
                 "跟在行人后面-A32", "人行横道的行人-A44", "跟在可行驶路段上的行人后面-A19", "跟在接送地点的行人后面-A20", "靠近有领头车辆的77", "临时减速/刹停避开非机动车-A40", "非机动车道"]],

    "白天雨天高架": [["光照-白天"], ["雨天-B9"], ["道路类型-高速高架"], 
                ["异形车", "异形车辆", "地下-B44", "地下-B28", "道路静态障碍物","道路上有静态障碍物", "被遮挡的锥桶", "道路施工", "闸机", 
                 "禁停牌","密集行人" , "并排打伞行人", "横穿行人-A80", "等待行人通过-A34", "停车避让车前行人-A12", "跟在行人后面-A32", 
                 "人行横道的行人-A44", "跟在可行驶路段上的行人后面-A19", "跟在接送地点的行人后面-A20", "临时减速/刹停避开非机动车-A40", "非机动车道", "靠近有领头车辆的77"]],

    "夜晚雨天城区": [["光照-夜晚"], ["雨天-B9"], ["道路类型-城区"], 
                ["异形车", "异形车辆", "地下-B44", "地下-B28", "道路静态障碍物", "被遮挡的锥桶", "道路施工", "闸机", "禁停牌", "密集行人", 
                 "人流量高", "二三轮车", "并排打伞行人", "横穿行人-A80", "等待行人通过-A34", "停车避让车前行人-A12", "跟在行人后面-A32", 
                 "人行横道的行人-A44", "跟在可行驶路段上的行人后面-A19", "跟在接送地点的行人后面-A20", "临时减速/刹停避开非机动车-A40", "非机动车道", "靠近有领头车辆的77"]],

    "夜晚雨天高架": [["光照-夜晚"], ["雨天-B9"], ["道路类型-高速高架"], 
                ["异形车", "异形车辆", "地下-B44", "地下-B28", "道路静态障碍物", "道路上有静态障碍物", "被遮挡的锥桶", "道路施工", "闸机", 
                 "禁停牌", "密集行人", "并排打伞行人", "横穿行人-A80", "等待行人通过-A34", "停车避让车前行人-A12", "跟在行人后面-A32", 
                 "人行横道的行人-A44", "跟在可行驶路段上的行人后面-A19", "跟在接送地点的行人后面-A20", "临时减速/刹停避开非机动车-A40", "非机动车道", "靠近有领头车辆的77"]],
}

pldDict = {
    # scene: [light:text, weather:text, labels:text]
    "白天巡航": [["光照-白天"], ["晴天-B7", "阴天-B8"], ["巡航"]],

    "白天泊入泊出": [["光照-白天"], ["晴天-B7", "阴天-B8"], ["泊入泊出"]],

    "雨天巡航": [["光照-夜晚"], ["雨天-B9"], ["巡航"]],

    "雨天泊入泊出": [["光照-夜晚"], ["雨天-B9"], ["泊入泊出"]],

    "昏暗巡航": [["低照明-B4"], ["夜晚-B2"], ["巡航"]],

    "昏暗泊入泊出": [["低照明-B4"], ["夜晚-B2"], ["泊入泊出"]],

    "机械": [["低照明-B4"], ["夜晚-B2"], ["机械"]],
    "坡度": [["低照明-B4"], ["夜晚-B2"], ["坡度"]],
    "草砖": [["低照明-B4"], ["夜晚-B2"], ["草砖材质"]],
    "石砖": [["低照明-B4"], ["夜晚-B2"], ["石砖材质"]],
    "坡度": [["低照明-B4"], ["夜晚-B2"], ["坡度"]],
    "开关车门": [["低照明-B4"], ["夜晚-B2"], ["开关车门"]],
    "悬空障碍物":[["低照明-B4"], ["夜晚-B2"], ["悬空障碍物白天","悬空障碍物夜晚"]],

    "立柱":[["低照明-B4"], ["夜晚-B2"], ["不同颜色涂层的立柱"]],
    "轮挡":[["低照明-B4"], ["夜晚-B2"], ["常规轮挡", "石体轮挡", "铁质轮挡"]],
    "库位障碍物":[["低照明-B4"], ["夜晚-B2"], ["库位内各种标识","库位内纸箱","库位内行人","库位内有水渍"]],
    "悬空物体":[["低照明-B4"], ["夜晚-B2"], ["悬空障碍物水管","消防箱","有电箱墙体"]],

    "室内停车楼":[["低照明-B4"], ["夜晚-B2"], ["室内停车楼"]],
    "室内大型园区":[["低照明-B4"], ["夜晚-B2"], ["室内大型园区"]],
    "室外住宅小区":[["低照明-B4"], ["夜晚-B2"], ["室外住宅小区"]],
    "室外狭窄快速路":[["低照明-B4"], ["夜晚-B2"], ["室外狭窄快速路"]],

    "冯诗羽":[["低照明-B4"], ["夜晚-B2"], ["人推车", "婴儿车", "购物车", "停车位内二三轮车"]],

    "地库":[["低照明-B4"], ["夜晚-B2"], ["地下车库","地上路边","地上标准", "地上园区停车场", "地上弱光",
                                   "地面有睡姿", "地上草砖库位", "库位地面障碍物阴影", "地上夜晚暗光"]],
    "车位":[["低照明-B4"], ["夜晚-B2"], ["地砖库位","机械库位","T型库位","U型库位","II型库位","坡度库位","车位线破损",
                                  "车位线被遮挡","车位内反光","车位线内模糊","车位内有水渍"]],
}

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

    valid_bag_dirs.sort()
    return valid_bag_dirs
    
def is_bag_match_label(bag_dir, label_list):
    light = label_list[0]  # title in FullTime and Interval
    weather = label_list[1]  # text in FullTime and Interval
    road_type = label_list[2]  # title in FullTime and Interval
    labels = label_list[3]  # text in Interval
    # labels = ["S弯"]  # text

    light_match = False
    weather_match = False
    road_type_match = False
    label_match = False
    label_yaml = os.path.join(bag_dir, "label.yaml")
    if os.path.exists(label_yaml) :
        with open(label_yaml, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            nodes_fulltime = data.get('FullTime', [])
            nodes_interval = data.get('Interval', [])

            if nodes_fulltime:
                for node in nodes_fulltime:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip()
                    text = text.replace(' ', '').replace('–', '').upper()
                    if "夜晚" in light[0]:
                        if "夜" in text:
                            light_match = True
                    else:
                        light_match = True
                    if "雨" in weather[0]:
                        if "雨" in text:
                            weather_match = True
                    else:
                        weather_match = True
                    if "城区" in road_type[0]:
                        if "城区" in title or "城区" in text:
                            road_type_match = True
                    else:
                        if "高速" in title or "高速" in text or "高架" in title or "高架" in text:
                            road_type_match = True
                    if text in labels:
                        label_match = True
            if nodes_interval:
                for node in nodes_interval:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip() 
                    text = text.replace(' ', '').replace('–', '').upper()
                    if "夜晚" in light[0]:
                        if "夜" in text:
                            light_match = True
                    else:
                        light_match = True
                    if "雨" in weather[0]:
                        if "雨" in text:
                            weather_match = True
                    else:
                        weather_match = True
                    if "城区" in road_type[0]:
                        if "城区" in title or "城区" in text:
                            road_type_match = True
                    else:
                        if "高速" in title or "高速" in text or "高架" in title or "高架" in text:
                            road_type_match = True
                    start_time = node.get('start_time', '')
                    end_time = node.get('end_time', '')
                    if text in labels and end_time != "":
                        label_match = True
    if light_match and weather_match and road_type_match and label_match:
    # if light_match and weather_match and label_match:
    # if label_match:
        print(bag_dir)
        return True
    return False

def is_match_label(bag_dir, label_list):
    labels = label_list

    label_match = False
    label_yaml = os.path.join(bag_dir, "label.yaml")
    if os.path.exists(label_yaml) :
        with open(label_yaml, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            nodes_fulltime = data.get('FullTime', [])
            nodes_interval = data.get('Interval', [])

            if nodes_fulltime:
                for node in nodes_fulltime:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip()
                    text = text.replace(' ', '').replace('–', '').upper()

                    for label in labels:
                        if label in text or text in label:
                            label_match = True
            if nodes_interval:
                for node in nodes_interval:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip() 
                    text = text.replace(' ', '').replace('–', '').upper()

                    for label in labels:
                        if label in text or text in label:
                            label_match = True

    if label_match:
        print(bag_dir)
        return True
    return False

def is_PLD_match_label(bag_dir, label_list):
    light = label_list[0]  # title in FullTime and Interval
    weather = label_list[1]  # text in FullTime and Interval
    labels = label_list[2]  # text in Interval
    # labels = ["S弯"]  # text

    light_match = False
    weather_match = False
    road_type_match = False
    label_match = False
    label_yaml = os.path.join(bag_dir, "label.yaml")
    if os.path.exists(label_yaml) :
        with open(label_yaml, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            nodes_fulltime = data.get('FullTime', [])
            nodes_interval = data.get('Interval', [])

            if nodes_fulltime:
                for node in nodes_fulltime:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip()
                    text = text.replace(' ', '').replace('–', '').upper()
                    if "夜晚" in light[0]:
                        if "夜" in text:
                            light_match = True
                    else:
                        light_match = True
                    if "雨" in weather[0]:
                        if "雨" in text:
                            weather_match = True
                    else:
                        weather_match = True
                    for label in labels:
                        if label in text or text in label:
                            label_match = True
            if nodes_interval:
                for node in nodes_interval:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip() 
                    text = text.replace(' ', '').replace('–', '').upper()
                    if "巡航" in light[0]:
                        if "巡" in text:
                            light_match = True
                    else:
                        light_match = True
                    if "泊入泊出" in weather[0]:
                        if "泊" in text:
                            weather_match = True
                    else:
                        weather_match = True

                    end_time = node.get('end_time', '')
                    for label in labels:
                        if label in text or text in label:
                            label_match = True
                    # if text in labels:# and end_time != "":
                    #     label_match = True
    # if light_match and weather_match and road_type_match and label_match:
    # if light_match and weather_match and label_match:
    if label_match:
        print(bag_dir)
        return True
    return False

def get_all_bags(dir_root):
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    # day_dirs = []
    # day_dirs.append("20250622")
    # day_dirs.append("20250617")
    # day_dirs.append("20250621")
    vin_codes = []
    # vin_codes.append("3483")
    # vin_codes.append("6596")
    abs_car_dirs = []
    for day in day_dirs:
        day_dir = os.path.join(dir_root, day)
        if os.path.isdir(day_dir):
            car_dirs = os.listdir(day_dir)
            car_dirs = [d for d in car_dirs if os.path.isdir(os.path.join(day_dir, d))]
            if len(vin_codes) > 0 :
                car_dirs = [d for d in car_dirs if d.split('_')[1] in vin_codes]
            car_dirs = [d for d in car_dirs if not d.endswith('-x') and not d.endswith('-zw')]
            car_dirs.sort()
            abs_car_dirs.extend(os.path.abspath(os.path.join(day_dir, d)) for d in car_dirs)

    # abs_car_dirs = []
    # abs_car_dirs.append("/home/geely/nas/Geely-d/AEB/20250410/E371_3483_20250410_1522CCRS")

    bag_dirs = []
    for dir in abs_car_dirs:
        # 获取目标文件夹下的有效bag文件夹
        bag_dirs.extend(get_valid_bag_folders(dir))
    bag_dirs = [d for d in bag_dirs if not "-x" in d]
    return bag_dirs

def get_scene_bags(bag_root):
    total_bag_dirs = []
    total_bag_dirs.extend(get_all_bags(bag_root))
    scene = "白天城区"
    # print(testDict[scene])
    label_bag_dirs = {}
    # scenes = ["白天城区", "白天高架", "夜晚城区", "夜晚高架", "白天雨天城区", "白天雨天高架", "夜晚雨天城区", "夜晚雨天高架"]
    scenes = ["夜晚高架", "白天雨天城区", "白天雨天高架", "夜晚雨天城区", "夜晚雨天高架"]
    print(f"total bag: {len(total_bag_dirs)}")
    total_scene_bags = []
    for scene in scenes:
        label_bag_dirs[scene] = [d for d in total_bag_dirs if is_bag_match_label(d, testDict[scene])]
        print(f"{scene} bag: {len(label_bag_dirs[scene])}")
        total_scene_bags.extend(label_bag_dirs[scene])
    return total_scene_bags

def get_pld_bags(bag_root):
    total_bag_dirs = []
    total_bag_dirs.extend(get_all_bags(bag_root))
    label_bag_dirs = {}
    scenes= []
    # scenes = ["冯诗羽"]
    # scenes = ["开关车门"]
    scenes = ["悬空障碍物"]
    print(f"total bag: {len(total_bag_dirs)} in {bag_root}")
    total_scene_bags = []
    for scene in scenes:
        label_bag_dirs[scene] = [d for d in total_bag_dirs if is_PLD_match_label(d, pldDict[scene])]
        print(f"{scene} bag: {len(label_bag_dirs[scene])}")
        total_scene_bags.extend(label_bag_dirs[scene])  
    total_scene_bags = list(set(total_scene_bags))  
    return total_scene_bags

def get_all_pld_bags():
    bag_roots = []
    # bag_roots.append("/home/geely/nas/PLD-S6B/J6M-S05")
    # bag_roots.append("/home/geely/nas/PLD-S6B/J6M-S07")
    bag_roots.append("/home/geely/nas/PLD-S6C/J6M-S08")
    # bag_roots.append("/home/geely/nas/PLD-S6B/J6M-S10")
    total_bag_dirs = []
    for bag_root in bag_roots:
        total_bag_dirs.append(get_pld_bags(bag_root))
    print(f"matched bag: {len(total_bag_dirs)}")

if __name__ == "__main__":
    get_all_pld_bags()


    