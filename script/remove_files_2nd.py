import os
import re
import shutil

vision_folder_list = [
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
]

def get_total_clip_folders(bag_dirs):
    valid_clips = []
    clip_num = 0
    for bag_dir in bag_dirs:
        clips_path = os.path.join(bag_dir, "clips")
        if os.path.isdir(clips_path):
            # 遍历 clips 子文件夹下的一级子文件夹
            for item in os.listdir(clips_path):
                clip_num += 1
                item_path = os.path.join(clips_path, item)
                if os.path.isdir(item_path):
                    valid_clips.append(item_path)
    valid_clips.sort()
    # print(f"total clip {clip_num}")
    return valid_clips

def get_isp_clip_folders(bag_dirs):
    total_clips = get_total_clip_folders(bag_dirs)
    isp_clips = [d for d in total_clips if "-isp" in d]
    print(f"total clip {len(total_clips)} isp clip {len(isp_clips)}")
    return isp_clips

def get_cc_clip_folders(bag_dirs):
    total_clips = get_total_clip_folders(bag_dirs)
    cc_clips = [d for d in total_clips if "-cc" in d]
    print(f"total clip {len(total_clips)} cc clip {len(cc_clips)}")
    return cc_clips

def remove_clip_files(clip):
    print(f'remove {clip} files')
    for item in vision_folder_list + ['raw']:
        if os.path.isdir(os.path.join(clip, item)):
            shutil.rmtree(os.path.join(clip, item))

    if clip.endswith('-cc'):
        bag_file = os.path.join(clip, os.path.basename(clip)[:-3] + '.bag')
    else:
        bag_file = os.path.join(clip, os.path.basename(clip) + '.bag')
    if os.path.exists(bag_file):
        os.remove(bag_file)

    for item in ['debug', 'local_motion_pcdbin', 'local_pcdbin', 'motion_pcdbin', 'pcd', 'tmp']:
        if os.path.isdir(os.path.join(clip, 'result', item)):
            shutil.rmtree(os.path.join(clip, 'result', item))
    
    for item in vision_folder_list:
        pic_folder = os.path.join(clip, 'result', 'test_calibration', item)
        if os.path.isdir(pic_folder):
            for pic in os.listdir(pic_folder):
                if pic.endswith('.jpg'):
                    os.remove(os.path.join(pic_folder, pic))

def is_night_bag(bag_dir):
    """
    判断是否为晚包，晚包的时间戳在10点之后
    :param bag_dir: 包路径"""    

    bag_hour = int(bag_dir.split('-')[3])
    bag_minute = int(bag_dir.split('-')[4])
    if bag_hour * 100 + bag_minute >= 1830:
        return True
    else:
        return False

def save_clip_info(bag_dir):
    """
    将bag_dir下clips文件夹中的所有子文件夹名称保存到bag_dir/state/clip_state.txt文件中。

    :param bag_dir: 包含clips文件夹的目录路径
    """
    clips_dir = os.path.join(bag_dir, 'clips')
    state_dir = os.path.join(bag_dir, 'state')
    state_file = os.path.join(state_dir, 'clip_state.txt')

    # 检查clips文件夹是否存在
    if not os.path.isdir(clips_dir):
        print(f"警告: {clips_dir} 文件夹不存在！跳过...")
        return

    # 检查state文件夹是否存在，不存在则创建
    if not os.path.isdir(state_dir):
        os.makedirs(state_dir)

    try:
        # 获取clips文件夹下的所有子文件夹名称
        sub_folders = [d for d in os.listdir(clips_dir) if os.path.isdir(os.path.join(clips_dir, d))]
        sub_folders.sort()
        # 将子文件夹名称写入文件
        with open(state_file, 'w', encoding='utf-8') as f:
            for folder in sub_folders:
                f.write(folder + '\n')
        print(f"已成功将clips子文件夹名称保存到 {state_file}")
    except Exception as e:
        print(f"保存clips子文件夹名称时出错: {e}")

if __name__ == "__main__":
    # 给定yaml地址
    vin_codes = []
    vin_codes.append("3483")
    vin_codes.append("6956")
    vin_codes.append("6477")
    dir_root = "/home/geely/nas/J6M-3C/bag"
    day_dirs = os.listdir(dir_root)
    day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))
    
    abs_car_dirs = []
    for day_dir in day_dirs:        
        car_dirs = os.listdir(os.path.join(dir_root, day_dir))
        # 筛选出以下划线分割的第二个元素为VIN_CODE的目录
        car_dirs = [d for d in car_dirs if len(d.split('_')) > 1 and d.split('_')[1] in vin_codes]
        car_dirs = [d for d in car_dirs if "-x" not in d and "-zw" not in d]
        car_dirs.sort()
        abs_car_dirs.extend(os.path.abspath(os.path.join(dir_root, day_dir, d)) for d in car_dirs)
    
    abs_bag_dirs = []
    for car_dir in abs_car_dirs:
        bag_dirs = os.listdir(car_dir)
        bag_dirs = [d for d in bag_dirs if os.path.isdir(os.path.join(car_dir, d))]
        bag_dirs = [d for d in bag_dirs if is_night_bag(d)]
        bag_dirs = [d for d in bag_dirs if d.endswith("-clip-20df")]
        bag_dirs = [d.replace("-clip-20df", "") for d in bag_dirs]
        # current_bag_dirs = list(os.path.abspath(os.path.join(car_dir, d)) for d in bag_dirs)
        abs_bag_dirs.extend(os.path.abspath(os.path.join(car_dir, d)) for d in bag_dirs)
    
    for bag in abs_bag_dirs:
        save_clip_info(bag)
        clips_path = os.path.join(bag, "clips")
        if os.path.isdir(clips_path):
            shutil.rmtree(clips_path)
    # valid_total_clips = get_total_clip_folders(abs_bag_dirs)
    # valid_cc_clips = get_cc_clip_folders(abs_bag_dirs)
    # valid_isp_clips = get_isp_clip_folders(abs_bag_dirs)
    # unused_clips = list(d for d in valid_total_clips if d not in valid_cc_clips and d not in valid_isp_clips)
    # for clip in unused_clips:
    #     print(f'delete {clip}')
    #     shutil.rmtree(clip)
        
    # for item in valid_cc_clips + valid_isp_clips:
    #     remove_clip_files(item)