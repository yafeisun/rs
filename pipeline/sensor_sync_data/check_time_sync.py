import os, sys
import numpy as np
from tqdm import tqdm
import shutil

def check_time_sync(sync_sensor_file):
    with open(sync_sensor_file, "r") as f:
        lines = f.readlines()
    sensor_keys = lines[0].split(" ")
    #frame /middle /cam_front_right /cam_back /cam_side_left_front /cam_side_right_front /cam_side_left_back /cam_side_right_back /cam_front_left /cam_around_front /cam_around_left /cam_around_right /cam_around_back

    sensor_map = {sensor_key: index for index, sensor_key in enumerate(sensor_keys[1:])}
    sync_target = "/cam_front_right"
    frame_count = 0

    # print(sensor_key)
    gap_max_pos = 0
    gap_max_neg = 0
    gap_sum = np.zeros(len(sensor_map))
    for line in lines[1:]:
        frame_sync = line.strip().split(" ")
        # print(frame_sync)
        all_sensor_ok = True
        for timestamp in frame_sync[1:]:
            if len(timestamp) != 19: # timestamp 1750499345199839232
                all_sensor_ok = False
                break
        if not all_sensor_ok:
            continue
        frame_count += 1
        frame_time_stamp = np.array([float(timestamp)/(1e6)for timestamp in frame_sync[1:]]) 
        target_timestmap = frame_time_stamp[sensor_map[sync_target]]
        frame_gap = frame_time_stamp - target_timestmap
        gap_max_pos = max(gap_max_pos, np.max(frame_gap))

        gap_max_neg = min(gap_max_neg, np.min(frame_gap))
        gap_sum += frame_gap
    avg_gap = gap_sum / frame_count

    print(f"frame_count: {frame_count} frames")
    print(f"gap_max_pos: {int(gap_max_pos)} ms")
    print(f"gap_max_neg: {int(gap_max_neg)} ms")
    print("avg_gap", np.array(avg_gap, np.int32))

    return np.max(np.abs(avg_gap))

def binary_search_nearest(timestamp_list, target):
    left = 0
    right = len(timestamp_list) - 1
    while left < right:
        mid = (left + right) // 2
        if timestamp_list[mid] < target:
            left = mid + 1
        else:
            right = mid
    res_timestamp = [timestamp_list[index] for index in [left-1, left, left+1] if 0 <= index < len(timestamp_list)]
    return min(res_timestamp, key=lambda x: abs(x - target))

def sync_frames(cam_timestamp_all, target_cam, sync_thresh=35):
    target_timestamp_list = cam_timestamp_all[target_cam]
    sync_all_frames = []
    for timestamp in target_timestamp_list:
        frame_timestamp = {}
        for cam, timestamp_list in cam_timestamp_all.items():
            # sync_time = min(timestamp_list, key=lambda x: abs(x - timestamp))
            frame_timestamp[cam] = binary_search_nearest(timestamp_list, timestamp)
        if max(np.abs(np.array(list(frame_timestamp.values())) - timestamp)) > sync_thresh:
            continue
        sync_all_frames.append(frame_timestamp)
    return sync_all_frames

def check_time_sync_file(bag_path, target_cam="cam_front_right", gap_thresh=6, drop_ratio=0.01, unusual_gap_thresh=10, unusual_gap_ratio=0.05):

    #frame /middle /cam_front_right /cam_back /cam_side_left_front /cam_side_right_front /cam_side_left_back /cam_side_right_back /cam_front_left /cam_around_front /cam_around_left /cam_around_right /cam_around_back
    cam_list = ["cam_front_right", "cam_back", "cam_side_left_front", "cam_side_right_front", "cam_side_left_back", "cam_side_right_back", "cam_front_left", "cam_around_front", "cam_around_left", "cam_around_right", "cam_around_back"]
    
    cam_timestamp_all = {cam: [] for cam in cam_list}

    for cam in cam_list:
        cam_path = os.path.join(bag_path, cam)
        timestamp_list = [float(filename.split(".jpg")[0])/1e6 for filename in os.listdir(cam_path) if ".jpg" in filename]
        timestamp_list.sort()
        # print(f"{cam} {len(timestamp_list)}")
        cam_timestamp_all[cam] = timestamp_list

    sync_all_frames = sync_frames(cam_timestamp_all, target_cam)
    sync_list = [[frame[cam] for cam in cam_list] for frame in sync_all_frames]

    # print(f"sync_all_frames: {len(sync_all_frames)}")

    gap_max_pos = 0
    gap_max_neg = 0
    unusual_gap_count = 0
    gap_sum = np.zeros(len(cam_list))
    frame_count = len(sync_list)
    if frame_count <= 0 or (1 - frame_count/len(cam_timestamp_all[target_cam]) > drop_ratio):
        print(f"drop_ratio: {1 - frame_count/len(cam_timestamp_all[target_cam])}")
        # return False

    for frame in sync_list:
        target_timestamp = frame[cam_list.index(target_cam)]
        gap = np.array(frame) - target_timestamp
        gap_max_pos = max(gap_max_pos, np.max(gap))
        gap_max_neg = min(gap_max_neg, np.min(gap))
        gap_sum += gap

        cur_max_gap = np.max(np.abs(gap))
        if cur_max_gap > unusual_gap_thresh:
            unusual_gap_count += 1

    avg_gap = gap_sum / frame_count
    max_avg_gap = np.max(np.abs(avg_gap))
    unusual_ratio = unusual_gap_count / frame_count
    
    print(f"frame_count: {frame_count} frames")
    print(f"gap_max_pos: {int(gap_max_pos)} ms")
    print(f"gap_max_neg: {int(gap_max_neg)} ms")
    print("avg_gap", np.array(avg_gap, np.int32))
    print(f"max_avg_gap: {max_avg_gap} ms")
    print(f"unusual_ratio: {unusual_ratio}")

    if max_avg_gap > gap_thresh or unusual_ratio > unusual_gap_ratio:
        return False
    else:
        return True

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

if __name__ == "__main__":
    input_path = sys.argv[1]
    if ".txt" in input_path:
        sync_sensor_file = input_path
        max_avg_gap = check_time_sync(sync_sensor_file)
        print(f"max_avg_gap: {max_avg_gap} ms")
        exit()
        

    bag_list = get_bag_list(input_path)

    print(f"bag_list: {len(bag_list)}")

    # bag_ok = check_time_sync_file(bag_path)
    # print(f"bag_ok: {bag_ok}")

    save_folder = "./check_time_sync"
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    gap_thresh = 6 # ms
    drop_ratio_thresh = 0.01
    target_cam = "cam_front_right"

    print(f"gap_thresh: {gap_thresh} ms")
    print(f"drop_ratio_thresh: {drop_ratio_thresh}")
    print(f"target_cam: {target_cam}")
    
    sync_error_count = 0
    count = 0
    for bag_path in tqdm(bag_list):
        count += 1
        # print(f"count: {count} / {len(bag_list)}")
        bag_ok = check_time_sync_file(bag_path, target_cam, gap_thresh, drop_ratio_thresh)
        # print(f"bag_ok: {bag_ok}")
        if not bag_ok:
            sync_error_count += 1
            with open(os.path.join(save_folder, os.path.basename(bag_path)+".txt"), "w") as f:
                f.write(bag_path+"\n")
                f.write("sync_error\n")

            if not "time_sync_error" in bag_path:
                mask_path = bag_path + "_time_sync_error"
                shutil.move(bag_path, mask_path)
        
    print(f"sync_error_count: {sync_error_count} / {len(bag_list)}")