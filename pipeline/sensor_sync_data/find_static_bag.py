import os, sys
import numpy as np
from tqdm import tqdm
import rosbag
import re


file_check = ["cam_front_right", "cam_back", "cam_side_left_front", "cam_side_right_front", "cam_side_left_back", "cam_side_right_back", "cam_front_left", "cam_around_front", "cam_around_left", "cam_around_right", "cam_around_back"]

def get_bag_list(input_path):
    bag_list = []
    if not os.path.isdir(input_path):
        return bag_list
    
    if "clip-20df" in input_path:
        return bag_list

    list_dir = os.listdir(input_path)
    list_dir.sort()
    ok_flag = True

    for file in file_check:
        if not file in list_dir:
            ok_flag = False
            break
    
    if ok_flag:
        bag_list.append(input_path)
    else:
        for file in list_dir:
            res = get_bag_list(os.path.join(input_path, file))
            bag_list.extend(res)
    return bag_list

def check_static_bag(bag_file_path):
    bag = rosbag.Bag(bag_file_path)
    static_count = 0
    for topic, msg, t in bag.read_messages(topics=["/rs/odom"]):
        velocity_sep = msg.twist.twist.linear
        velocity = np.array([velocity_sep.x, velocity_sep.y, velocity_sep.z])
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm < 0.1:
            static_count += 1
        else:
            static_count = 0
        if static_count > 50:
            return True
    return False

if __name__ == "__main__":
    input_path = sys.argv[1]

    bag_list = get_bag_list(input_path)

    print(f"bag_list: {len(bag_list)}")

    for bag_path in tqdm(bag_list):
        bag_basename = os.path.basename(bag_path)
        result = re.search(r"[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}", bag_basename)
        if result:
            bag_name = result.group()
            bag_file_path = os.path.join(bag_path, bag_name + ".bag")
        else:
            continue

        static_flag = check_static_bag(bag_file_path)
        if static_flag:
            print(f"static bag: {bag_path}")
