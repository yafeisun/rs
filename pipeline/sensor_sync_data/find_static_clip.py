import os, sys
import numpy as np
import json
import re

def get_frame_loc(desc_path):
    with open(desc_path, "r") as f:
        data = json.load(f)
        
    return np.array(data['desc']['ego2global'])

def check_static_clip(clip_path):
    frame_list = sorted([name for name in os.listdir(clip_path) if not ".json" in name])
    # print(os.path.basename(clip_path))
    frame = frame_list[0]
    frame_path = os.path.join(clip_path, frame)
    desc_path = os.path.join(frame_path, frame+"__desc.json")
    loc_pre = get_frame_loc(desc_path)
    slow_flag = False
    very_slow_flag = False
    time_pre = float(frame) / 1000.0
    for frame in frame_list[1:]:
        frame_path = os.path.join(clip_path, frame)
        desc_path = os.path.join(frame_path, frame+"__desc.json")
        
        loc = get_frame_loc(desc_path)
        
        relative_motion = loc @ np.linalg.inv(loc_pre)

        loc_pre = loc

        time_cur = float(frame) / 1000.0

        dt = time_cur - time_pre
        time_pre = time_cur
        
        relative_trans = np.linalg.norm(relative_motion[:3,3])
        
        # print(os.path.basename(clip_path), frame, relative_trans)


        velocity = relative_trans / dt
        
        if (not slow_flag) and (0.1 <= velocity < 1):
            print(os.path.basename(clip_path), frame, velocity, "slow motion")
            slow_flag = True

        if (not very_slow_flag) and (velocity < 0.1):
            print(os.path.basename(clip_path), frame, velocity, "very slow motion !!!!!!!!!")
            very_slow_flag = True
            break


        

def process_by_bag(bag_path):
    if not "clip-20df" in os.path.basename(bag_path):
        return
    clip_list = sorted(os.listdir(bag_path))
    print("*****************************************")
    print(os.path.basename(bag_path))
    
    for clip_name in clip_list:
        if "_visualization" in clip_name:
            continue
        result = re.search(r"[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}", clip_name)
        if result is None:
            continue
        clip_path = os.path.join(bag_path, clip_name)
        if check_static_clip(clip_path):
            print(clip_path)

if __name__=="__main__":
    if len(sys.argv) < 2:
        print("python3 find_static_clip.py bag1 bag2 bag3")
        
    bag_list = sys.argv[1:]
    
    # print(bag_list)
    
    for bag_path in bag_list:
        process_by_bag(bag_path)