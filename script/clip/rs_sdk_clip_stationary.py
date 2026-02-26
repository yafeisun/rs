import os
import re
import time
import shutil
import yaml
import subprocess
import concurrent.futures  # 导入多线程库

from bag_clips_optimized import write_bag_clips

PASSWORD = "123"
sdk_root = "/home/geely/Documents/robosense/11v_SDK_250522"

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

class SdkParams:
    def __init__(self, folder_name, thread_num, filter_func):
        self.folder_name = folder_name
        self.thread_num = thread_num
        self.filter_func = filter_func
        
# 各步骤文件夹
# rosrun rviz rviz -d vis.rviz
sdk_folders = [
    "1_bag_clip_11v",
    "2_parser_11v",
    "3_slam",
    "4_postprocess",
    "5_projection_11v",
    "6_colormap_11v",
    "7_height_colormap"
]
thread_per_gpu = 3
thread_per_cpu = 4 * thread_per_gpu
thread_nums = [thread_per_cpu, 
               thread_per_cpu, 
               thread_per_cpu, 
               thread_per_gpu, 
               thread_per_cpu, 
               thread_per_gpu,
               thread_per_gpu]

def filter_clip(bag, flag=True):
    if os.path.isdir(os.path.join(bag, 'clips')):
        return True
    if not os.path.exists(os.path.join(bag, 'state', 'clip_successed.txt')):
        return False
    if flag:
        if not os.path.isdir(os.path.join(bag, 'clips')):
            return False
    return True

def filter_parser(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'parse_successed.txt')):
        return False
    if flag:
        for item in ['raw', 'result']:
            if not os.path.isdir(os.path.join(bag, item)):
                return False
        for item in ['datacheck.csv', 'task_file.yaml']:
            if not os.path.exists(os.path.join(bag, item)):
                return False
        for item in ['rtk_gps.txt', 'rtk_imu.txt', 'rtk_odom.txt', 'sync_time_list.txt']:
            if not os.path.exists(os.path.join(bag, 'raw', item)):
                return False
        if not os.path.isdir(os.path.join(bag, 'raw', 'pcd')):
            return False
        
        results = os.listdir(os.path.join(bag, "result"))
        for file in results:
            yaml = False
            if file.endswith(".yaml"):
                yaml = True
                break
        # if not yaml:
        #     return False
    return True

def filter_slam(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'slam_successed.txt')):
        return False
    if flag:
        for item in ['lane_image', 'local_motion_pcdbin', 'local_pcdbin', 
                    'mapping', 'motion_pcdbin', 'sensor_data', 'slam_debug', 'slot_image']:
            if not os.path.isdir(os.path.join(bag, 'result', item)):
                return False
        for item in ['bumpy_log.txt', 'exception_odom.txt', 'slam_time.txt', 'sync_sensor_data.txt', 
                    'traj.txt', 'traj_alignment.txt', 'traj_evo.txt', 'traj_vehicle.txt']:
            if not os.path.exists(os.path.join(bag, 'result', item)):
                return False

        middle_pcds = os.listdir(os.path.join(bag, 'result', 'local_motion_pcdbin'))
        middle_pcds = [pcd for pcd in middle_pcds if '.pcd' in pcd]
        if not middle_pcds:
            return False
        
        bp_pcds = []
        for item in ['left', 'right', 'front', 'back']:
            pcds = os.listdir(os.path.join(bag, 'result', 'local_motion_pcdbin', item))
            pcds = [pcd for pcd in pcds if '.pcd' in pcd]
            if pcds:
                bp_pcds.extend(pcds)
        # if not bp_pcds:   # 没有补盲的p
        #     return False #没用补盲注释掉
    return True

def filter_postprocess(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'postprocess_successed.txt')):
        return False
    if flag:
        for item in ['pcd', 'sensor_data']:
            if not os.path.isdir(os.path.join(bag, 'result', item)):
                return False
        for item in ['global_GT.bin', 'global_GT.csv', 'GT.bin', 'GT.csv', 
                     'GT_ROI.bin', 'GT_ROI.csv', 'road.bin']:
            if not os.path.exists(os.path.join(bag, 'result', item)):
                return False
    return True

def filter_projection(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'project_successed.txt')):
        return False
    if flag:
        if not os.path.isdir(os.path.join(bag, 'result', 'test_calibration')):
            return False
        for img in vision_folder_list or ["middle"]:
            if not os.path.isdir(os.path.join(bag, 'result', 'test_calibration', img)):
                return False
        # for item in ['map_intensity.pcd', 'map_intensity_bev.pcd', 'map_rgb_bev.pcd', 'sync_sensors.txt']:
        for item in ['sync_sensors.txt']:
            if not os.path.exists(os.path.join(bag, 'result', 'test_calibration', item)):
                return False
    return True

def filter_colormap(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'colormap_successed.txt')):
        return False
    if flag:
        if not os.path.isdir(os.path.join(bag, 'result', 'bev')):
            return False
        for item in ['BEV_gps.json', 'BEV_info.json', 'BEV.jpeg', 'BEV_label.json', 
                     'BEV_pose.json', 'HBEV.png', 'IBEV.jpeg', 'IBEV.pcd', 'InBEV.jpeg', 
                     'origin_IBEV.jpeg', 'RGBBEV.pcd', 'sensor_timestamp_map.json', 
                     'utm.json', 'version.json']:
            if not os.path.exists(os.path.join(bag, 'result', 'bev', item)):
                return False
    return True

def filter_bevheight(bag, flag=True):
    if not os.path.exists(os.path.join(bag, 'state', 'height_colormap_successed.txt')):
        return False
    if flag:
        if not os.path.isdir(os.path.join(bag, "result", "test_calibration", "bev_height")):
            return False
        for item in ['BEV.jpeg', 'BEV_label.json', 'HBEV.png', 'InBEV.jpeg', 
                     'sensor_timestamp_map.json']:
            if not os.path.exists(os.path.join(bag, "result", "test_calibration", "bev_height", item)):
                return False
    return True

filter_functions = [
    filter_clip,
    filter_parser,
    filter_slam,
    filter_postprocess,
    filter_projection,
    filter_colormap,
    filter_bevheight
]

paras_list = [
    SdkParams(folder_name, thread_num, filter_func)
    for folder_name, thread_num, filter_func in zip(sdk_folders, thread_nums, filter_functions)
]

def split_stationary_clip(bag_path):
    try:
        print(f"execute clip: {bag_path}")
        clip_num = write_bag_clips(bag_path)
        # Check if clip_num is None or zero
        if clip_num is not None and clip_num != 0:
            if filter_clip(bag_path):
                # print(f"split stationary clip successed: {bag_path}")
                return bag_path, True
            else:
                print(f"split stationary clip failed: {bag_path}")
                return bag_path, False
        # If clip_num is None or 0, return as failed
        return bag_path, False
    except Exception as e:
        print(f"split stationary clip exception {bag_path}: {str(e)}")
        return bag_path, False

def execute_clip_sh(bag_folders, thread_num=12):
    if not bag_folders:
        return [], [], 0
    
    start_time = time.time()  # 记录总开始时间
    succeed_bags = []
    failed_bags = []
    
    # 过滤掉已经处理过的bag
    valid_bags = [i for i in bag_folders if not filter_clip(i, False)]
    if not valid_bags:
        return [], [], 0

    # 使用ThreadPoolExecutor进行多线程处理
    with concurrent.futures.ThreadPoolExecutor(max_workers=thread_num) as executor:
        future_to_bag = {executor.submit(split_stationary_clip, bag_path): bag_path for bag_path in valid_bags}
        
        for future in concurrent.futures.as_completed(future_to_bag):
            bag_path, success = future.result()
            if success:
                succeed_bags.append(bag_path)
            else:
                failed_bags.append(bag_path)
    
    # 计算总耗时
    total_time = time.time() - start_time
    # print(f"split clip done: {len(succeed_bags)}, fail: {len(failed_bags)}, time: {total_time:.2f}s")
    
    return succeed_bags, failed_bags, total_time

def execute_SDK_scripts(bags, para):
    """
    按顺序执行多个文件夹中的 start.sh 脚本，并记录每个脚本的执行时间。
    
    :param valid_subdirs: 目标文件夹路径
    :param step_folder: 包含多个步骤文件夹路径的列表
    """
    succeed_clips = []
    failed_clips = []
    consumed_time = 0

    # 定义当前步骤的脚本路径
    script_dir = os.path.join(sdk_root, para.folder_name)
    script_path = os.path.join(script_dir, "start.sh")

    if not os.path.isfile(script_path):
        print(f"警告：脚本 {script_path} 不存在，跳过当前步骤！")
    valid_dirs = bags
    if valid_dirs:
        # 动态构建命令参数
        if "7" in para.folder_name:
            args = valid_dirs
        else:
            args = ["0", str(para.thread_num)] + valid_dirs
        command = f"cd {script_dir} && ./start.sh {' '.join(args)}"

        # 执行脚本并记录时间
        start_time_step = time.time()
        print(f"\n正在执行 [{para.folder_name}]: {command}")
        try:
            # cmd = f"sshpass -p '{PASSWORD}' sudo bash -c '{command}'"
            cmd = f"bash -c '{command}'"
            subprocess.run(cmd, shell=True, check=True)
            end_time_step = time.time()
            consumed_time = end_time_step - start_time_step
            # print(f"[{para.folder_name}] 执行完成！耗时: {consumed_time:.2f} 秒")
        except subprocess.CalledProcessError as e:
            print(f"[{para.folder_name}] 执行失败: {e}")
            
        # check sdk output
        succeed_clips = [i for i in valid_dirs if para.filter_func(i)]
        failed_clips = [i for i in valid_dirs if not para.filter_func(i)]
        # for bag in failed_clips:
        #     if os.path.exists(bag):
        #         #delete_bag_files(bag, True)
        #         os.rename(bag, bag + "-x")
    return succeed_clips, failed_clips, consumed_time

def check_jpg_num(bag, record_time):
    cam_folders = [f for f in os.listdir(bag) 
                if f.startswith('cam_') and os.path.isdir(os.path.join(bag, f))]
    
    if len(cam_folders) < len(vision_folder_list):
        print(f"{bag} cam folder num error {len(cam_folders)} != {len(vision_folder_list)}")
        return False
    
    for cam_folder in cam_folders:
        cam_path = os.path.join(bag, cam_folder)
        jpg_files = [f for f in os.listdir(cam_path) if f.endswith('.jpg')]
        if len(jpg_files) < 9 * record_time:
            print(f"{bag} jpg num too small {len(jpg_files)} < 90% * {10 * record_time}")
            return False
    return True

def check_bag_files(bag):
    # 检查 rsOriGnssInfo.txt 文件是否存在
    if not os.path.exists(os.path.join(bag, "rsOriGnssInfo.txt")):
        print(f"{bag}/rsOriGnssInfo.txt not exist")
        # return False
        
    # 检查 label.yaml 文件是否存在
    if not os.path.exists(os.path.join(bag, 'label.yaml')):
        print(f"{bag}/label.yaml not exist")
        return False
        
    # 检查 info.yaml 文件是否存在
    info_yaml_path = os.path.join(bag, 'info.yaml')
    if not os.path.exists(info_yaml_path):
        print(f"{info_yaml_path} not exist")
        return False

    with open(info_yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
        record_time = data.get('record_time', '')
        bag_name = data.get('bag_name', '')
        car_label = data.get('car_label', '')
        
        # bag包是否存在
        if not os.path.exists(os.path.join(bag, bag_name)):
            print(f"{bag}/{bag_name} not exist")
            return False
    
        # 检查是否有标定yaml格式文件
        car_yaml_files = [f for f in os.listdir(bag) 
                            if f.startswith('car_') and f.endswith('.yaml')]
        # 检查标定yaml文件名称是否与info.yaml配置一致
        if len(car_yaml_files) == 1:
            # 只有一个car_开头的yaml文件
            yaml_file = car_yaml_files[0]
            if yaml_file != car_label + '.yaml':
                # 文件名不一致，进行重命名
                print(f"{bag} 标定文件名称不一致 {yaml_file} != {car_label}.yaml")
                if 0:
                    old_path = os.path.join(bag, yaml_file)
                    new_path = os.path.join(bag, car_label + '.yaml')
                    os.rename(old_path, new_path)
                    print(f"rename {yaml_file} -> {car_label}.yaml")
                else: # 修改info.yaml中car_label为yaml文件名称
                    data['car_label'] = yaml_file.replace('.yaml', '')
                    with open(info_yaml_path, 'w', encoding='utf-8') as f:
                        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
                    print(f"modify info.yaml {car_label} -> {data['car_label']}")

        elif len(car_yaml_files) > 1:
            # 有多个car_开头的yaml文件
            print(f"{bag} 路径下有多个标定yaml文件")
            return False
        else:
            # 没有car_开头的yaml文件
            print(f"{bag}/{car_label}.yaml not exist!")
            return False
            
        # 检查录制时长
        if record_time < 20:
            print(f"{bag} 录制时长小于20s")
            # shutil.rmtree(bag)
            return False
        elif record_time > 300:
            print(f"{bag} 录制时长大于300s")
            return False
        else:
            return check_jpg_num(bag, record_time)
    return True

def get_valid_bag_folders(target_folder):
    valid_bag_dirs = []
    if not os.path.isdir(target_folder):
        print(f"错误：目标文件夹 {target_folder} 不存在！")
        return valid_bag_dirs

    for item in os.listdir(target_folder):
        item_path = os.path.join(target_folder, item)
        if os.path.isdir(item_path) and not item.startswith('.'):
            if re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', item):
                valid_bag_dirs.append(item_path)
            elif not item.endswith('-clip-20df'):
                print(f"跳过无效文件夹：{item} (不符合 YYYY-MM-DD-HH-MM-SS 格式)")

    return valid_bag_dirs

def extract_frame_OD(bags, freq = 10):
    # jiaxin写的7V周视去畸变
    succeed_clips = []
    failed_clips = []
    consumed_time = 0
    if freq == 10:
        script_path = "/home/geely/jiaxin/EA-LSS_extract_frame_version_bag_10hz"
    else:
        script_path = "/home/geely/jiaxin/EA-LSS_extract_frame_version_bag"
    
    if bags:
        command = f"cd {script_path} && ./run_clip.sh {' '.join(bags)}"
        try:
            # 执行脚本并记录时间
            start_time_step = time.time()
            subprocess.run(["bash", "-c", command], check=True)
            end_time_step = time.time()
            consumed_time = end_time_step - start_time_step
        except subprocess.CalledProcessError as e:
            print(f"EA-LSS_extract_frame 执行失败: {e}")
        
        # check sdk output
        for bag in bags:
            od_frame_folder = bag + "-clip-20df"
            if os.path.isdir(od_frame_folder):
                succeed_clips.append(bag)
            else:
                failed_clips.append(bag)
    return succeed_clips, failed_clips, consumed_time

def execute_merge_pcd(bags):
    # 合并pcd
    if bags:
        script_path = "/home/geely/Documents/robosense/script/merge_pcd/start.sh"
        command = f"cd {os.path.dirname(script_path)} && ./start.sh {' '.join(bags)}"
        # 执行脚本
        try:
            subprocess.run(["bash", "-c", command], check=True)
        except subprocess.CalledProcessError as e:
            print(f"合并PCD失败: {e}")
    return

def delete_thumbs_db(folder_path):
    for folder in vision_folder_list:
        files = os.listdir(os.path.join(folder_path, folder))
        files = [d for d in files if ".jpg" not in d]
        files = [d for d in files if "clips" not in d]
        for d in files:
            file = os.path.join(folder_path, folder, d)
            if os.path.isdir(file):
                os.rmdir(file)
                print(f"Delete folder {file}")
            else:
                try:
                    os.remove(file)
                    print(f"Delete file {file}")
                except Exception as e:
                    print(f"Error deleting {file}: {e}")

def clear_bag_state(bag):
    # if os.path.exists(os.path.join(clip, 'state', 'parse_successed.txt')):
    #     os.remove(os.path.join(clip, 'state', 'parse_successed.txt'))
    # if os.path.exists(os.path.join(bag, 'state', 'slam_successed.txt')):
    #     os.remove(os.path.join(bag, 'state', 'slam_successed.txt'))
    # if os.path.exists(os.path.join(bag, 'state', 'postprocess_successed.txt')):
    #     os.remove(os.path.join(bag, 'state', 'postprocess_successed.txt'))
    if os.path.exists(os.path.join(bag, 'state', 'project_successed.txt')):
        os.remove(os.path.join(bag, 'state', 'project_successed.txt'))
    # if os.path.exists(os.path.join(bag, 'state', 'colormap_successed.txt')):
    #     os.remove(os.path.join(bag, 'state', 'colormap_successed.txt'))

def extract_bev_files(bag, flag=False):
    # 使用 os.makedirs 递归创建目标目录
    target_root = bag + "-clip-20df"
    target_bev = os.path.join(target_root, "bev")
    if not os.path.exists(target_bev):
        os.mkdir(target_bev)
    bev_folder = os.path.join(bag, "result", "bev")
    for file in ["BEV.jpeg", "InBEV.jpeg", "HBEV.png", "BEV_label.json", \
        "IBEV.pcd", "RGBBEV.pcd"]:
        file_path = os.path.join(bev_folder, file)
        target_file_path = os.path.join(target_bev, file)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            if flag or not os.path.exists(target_file_path):
                shutil.copy2(file_path, target_file_path)
        else:
            print(f"{file} 不存在于 {bev_folder}")

    for file in ["map_intensity.pcd", "map_intensity_bev.pcd", "map_rgb_bev.pcd"]:
        map_pcd = os.path.join(bag, "result", "test_calibration", file)
        target_map_pcd_path = os.path.join(target_bev, os.path.basename(map_pcd))
        if os.path.exists(map_pcd) and os.path.isfile(map_pcd):
            if flag or not os.path.exists(target_map_pcd_path):
                shutil.copy2(map_pcd, target_map_pcd_path)
        else:
            print(f"{file} not in {os.path.join(bag, 'result', 'test_calibration')}")

def extract_pose(bag, target_dir):
    print(f"extract pose from {bag}")
    #cp pose txt
    target_pose = os.path.join(target_dir, "pose")
    if not os.path.exists(target_pose):
        os.makedirs(target_pose)
    for img_folder in vision_folder_list:
        pose_txt = os.path.join(bag, "result", "test_calibration", img_folder, "sync_sensors.txt")
        if os.path.exists(pose_txt):
            img_pose = os.path.join(target_pose, img_folder)
            if not os.path.exists(img_pose):
                os.makedirs(img_pose)
            shutil.copy(pose_txt, os.path.join(img_pose, os.path.basename(pose_txt)))

    sync_sensors = os.path.join(bag, "result", "test_calibration", "sync_sensors.txt")
    if os.path.exists(sync_sensors):
        shutil.copy(sync_sensors, os.path.join(target_pose, os.path.basename(sync_sensors)))
    
    sync_time_list = os.path.join(bag, "raw", "sync_time_list.txt")
    if os.path.exists(sync_time_list):
        shutil.copy(sync_time_list, os.path.join(target_pose, os.path.basename(sync_time_list)))

    middle_path = os.path.join(bag, "result", "test_calibration", "middle")
    os.makedirs(os.path.join(target_pose, "middle"), exist_ok=True)
    for file in ['sync_sensors.txt', 'pcd_num.txt']:
        if os.path.exists(os.path.join(middle_path, file)):
            shutil.copy(os.path.join(middle_path, file), os.path.join(target_pose, "middle", file))

def replace_occ_files_from_bag(bag, filter_bag):
    # extract od files if img_root is filtered frames,  where frames not stationary that already sent
    # cannot find frames in -clip-20df, rerun sdk until projection finish
    # extract pcd from middle, no need to gen -clip-20df
    middle = os.path.join(bag, "result", "test_calibration", "middle")
    if os.path.isdir(middle):
        image_list = []
        # Read the sync_sensors.txt file
        sync_sensors_path = os.path.join(bag, "result", "test_calibration", "middle", "sync_sensors.txt")
        if os.path.exists(sync_sensors_path):
            # Specify the encoding when opening the file
            with open(sync_sensors_path, 'r', encoding='utf-8') as f:
                next(f)
                for line in f:
                    pcd_num, img_num = line.strip().split(' ')[0:2]
                    if img_num != "null" and pcd_num != "null":
                        image_list.append({'img': int(img_num), 'pcd': int(pcd_num)})        
        image_list.sort(key=lambda x: x['img'])

        img_clips = os.listdir(filter_bag)
        img_clips = [d for d in img_clips if os.path.basename(bag) in d]
        img_clips.sort()
        for clip in img_clips:
            od_clip = os.path.join(filter_bag, clip)
            if os.path.isdir(middle): # and not os.path.exists(os.path.join(filter_bag, clip)):
                frames = os.listdir(od_clip)
                frames = [d for d in frames if d.isdigit()]
                frames.sort()
                target_clip = od_clip

                for frame in frames:
                    # 取 frame 的前 13 位
                    frame_prefix = int(frame[:13])
                    # 查找对应的 pcd 编号
                    pcd_num = None
                    for item in image_list:
                        img_timestamp = str(item['img'])[:13]
                        if int(img_timestamp) == frame_prefix:
                            pcd_num = item['pcd']
                            break
                    if pcd_num is not None:
                        pcd_file = os.path.join(middle, str(pcd_num) + '.pcd')
                        if os.path.exists(pcd_file):
                            target_pcd = os.path.join(target_clip, frame, str(frame) + '.pcd')
                            if not os.path.exists(target_pcd):
                                shutil.copy2(pcd_file, target_pcd)
                            elif os.path.getsize(pcd_file) != os.path.getsize(target_pcd):
                                shutil.copy(pcd_file, target_pcd)
                        else:
                            print(f"{pcd_file} not exist")

def remove_item(item):
    if os.path.exists(item):
        print(f"remove {item}")
        cmd = f"sshpass -p '{PASSWORD}' sudo rm -rf {item}"
        subprocess.run(cmd, shell=True, check=True)

def recover_locked_result(bag):
    # remove result
    result = os.path.join(bag, 'result')
    remove_item(result)

    # creat result folder
    if not os.path.isdir(result):
        os.makedirs(result)
        # copy yaml
        files = os.listdir(bag)
        for file in files:
            if file.endswith(".yaml") and file.startswith("car_"):
                shutil.copy(os.path.join(bag, file), os.path.join(result, file))

def run_with_sudo(func_name, args):
    script_name = os.path.basename(__file__).replace('.py', '')
    cmd = f"sshpass -p '{PASSWORD}' sudo python3 -c 'import sys; sys.path.append(\"{os.path.dirname(os.path.abspath(__file__))}\"); from {script_name} import {func_name}; {func_name}(\"{args}\")'"
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running {func_name} as sudo: {e}")

def get_unfinished_bags(step, bags):
    retval = []
    for bag in bags:
        flag = True
        parse_state = filter_parser(bag, flag)
        slam_state = filter_slam(bag, flag)
        postprocess_state = filter_postprocess(bag, flag)
        project_state = filter_projection(bag, flag)
        colormap_state = filter_colormap(bag, flag)
        bevheigh_state = filter_bevheight(bag, flag)

        folders = []
        bp_enable = False
        for folder in ["front", "back", "left", "right"]:
            folder_path = os.path.join(bag, "raw", "pcd", folder)
            if os.path.exists(folder_path):
                folders.append(folder)
        if len(folders) > 1:
            bp_enable = True

        if step == 1:
            if not parse_state:
                retval.append(bag)
        elif step == 2:
            if parse_state and not slam_state:
                recover_locked_result(bag)
                retval.append(bag)
        elif step == 3:
           if parse_state and slam_state and not postprocess_state:
                remove_item(os.path.join(bag, 'result', 'tmp'))
                retval.append(bag)
        elif step == 4:
            if parse_state and slam_state and postprocess_state and not project_state:
                remove_item(os.path.join(bag, 'result', 'test_calibration'))
                retval.append(bag)
        elif step == 5:
            if parse_state and slam_state and not colormap_state:
                print(f"remove {os.path.join(bag, 'result', 'bev')}")
                cmd = f"sshpass -p '{PASSWORD}' sudo cd {os.path.join(bag, 'result')} & rm -rf bev*"
                subprocess.run(cmd, shell=True, check=True)
                retval.append(bag)
        elif step == 6:
            if parse_state and slam_state and not bevheigh_state:
                retval.append(bag)
        elif step == 7:
            if parse_state and slam_state and postprocess_state and project_state and bp_enable \
                and not os.path.exists(os.path.join(bag, 'result', 'test_calibration', 'middle', 'pcd_num.txt')):
                retval.append(bag)
        elif step == 8:
            if parse_state and slam_state and postprocess_state and project_state \
                and not os.path.isdir(bag + "-clip-20df"):
                if not bp_enable:
                    retval.append(bag)
                elif os.path.exists(os.path.join(bag, 'result', 'test_calibration', 'middle', 'pcd_num.txt')):
                    retval.append(bag)

    # 取12个当前step未完成的bag
    retval.sort()
    retval = retval[0 : max(thread_nums)]
    # retval = retval[0 : 7]
    return retval

def is_label_match(bag_dir, label_list):
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
                            return True
            if nodes_interval:
                for node in nodes_interval:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip() 
                    text = text.replace(' ', '').replace('–', '').upper()

                    end_time = node.get('end_time', '')
                    for label in labels:
                        if label in text or text in label:
                            label_match = True
                            return True
    if label_match:
        return True
    return False

def delete_bag_files(bag, flag=False):
    if flag:
        for folder in ["raw", "result"]:
            remove_item(os.path.join(bag, folder))
    else:
        print(f"delete bag files {bag}")

    result_path = os.path.join(bag, 'result')
    if not os.path.isdir(result_path):
        return
    
    # 要保留的路径
    test_path = os.path.join(result_path, 'test_calibration')
    for item in os.listdir(result_path):
        item_path = os.path.join(result_path, item)
        if item == "test_calibration":
            for i in os.listdir(test_path):
                i_path = os.path.join(test_path, i)
                if os.path.isdir(i_path):
                    continue
                elif '.jpeg' in i or '.pcd' in i:
                    os.remove(i_path)
            continue
        if item_path.endswith(".yaml"):
            continue
        if os.path.isdir(item_path):
            shutil.rmtree(item_path)
        else:
            os.remove(item_path)

def is_bag_finished(bag):
    if not os.path.exists(bag):
        return True
    for state in ["parse_successed.txt", "slam_successed.txt", "postprocess_successed.txt", "project_successed.txt"]:
        if not os.path.exists(os.path.join(bag, 'state', state)):
            return False    
    if not os.path.isdir(bag + "-clip-20df"):
        return False
    return True

def get_filtered_ld_bags(bag_root, filter_root):
    dir_roots = os.listdir(filter_root)
    # dir_roots = []
    # dir_roots.append("J6M-S22")
    # dir_roots.append("J6M-S06")
    # dir_roots.append("J6M-S11")
    # dir_roots.append("J6M-S14")
    # dir_roots.append("J6M-S23")
    total_bag_dirs = []
    for dir_root in dir_roots:
        dir_root = os.path.join(filter_root, dir_root)
        if os.path.isdir(dir_root):
            day_dirs = os.listdir(dir_root)
            day_dirs = list(d for d in day_dirs if len(d) == 8 and d.isdigit() and os.path.isdir(os.path.join(dir_root, d)))

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

        for dir in abs_car_dirs:
            # 获取目标文件夹下的有效bag文件夹
            current_bags = get_valid_bag_folders(dir)
            current_bags = [d.replace(os.path.join(filter_root, dir_root), bag_root) for d in current_bags]
            total_bag_dirs.extend(current_bags)

    total_bag_dirs = list(set(total_bag_dirs))
    total_bag_dirs.sort()

    return total_bag_dirs

if __name__ == "__main__":
    # 获取目标文件夹路径参数
    start_time_total = time.time()
    total_bag_dirs = []
    # total_bag_dirs.append("/home/geely/nas/Data11/图达同雷达-12-22/S11_1015_90/cam_cal/sta_dyna_sta1/2025-10-15-10-28-10")
    total_bag_dirs.append("/home/geely/nas/Data11/图达同雷达-12-22/S22_0930_80/Sta/2025-09-30-17-09-56")
    # total_bag_dirs.append("/home/geely/nas/Data11/图达同雷达-12-22/S15_1016_69/2025-10-16-17-01-24")
    # total_bag_dirs.append("/home/geely/nas/Data11/图达同雷达-12-22/S29_1108_97/cam_cal/sta_dyna_sta1/2025-11-08-14-00-41")
    # total_bag_dirs.append("/home/geely/nas/Data-4/标定数据/S17_1016_91/cam_cal/sta_dyna_sta/2025-10-16-11-17-27")
    # total_bag_dirs.append("/home/geely/nas/Data-4/标定数据/S18_1015_92/cam_cal/sta_dyna_sta/2025-10-15-15-47-21")

    total_bag_dirs = [d for d in total_bag_dirs if check_bag_files(d)]
    total_bag_dirs = list(set(total_bag_dirs))
    total_bag_dirs.sort()

    execute_clip_sh(total_bag_dirs)

    total_clip_dirs = []
    for item in total_bag_dirs:
        clips = os.path.join(item, "clips")
        if os.path.isdir(clips):
            clips_dirs = os.listdir(clips)
            clips_dirs = [d for d in clips_dirs if "clip-20df" not in d]
            clips_dirs = [os.path.join(clips, d) for d in clips_dirs if os.path.isdir(os.path.join(clips, d)) and os.path.basename(item) in d]
            total_clip_dirs.extend(clips_dirs)

    for item in total_clip_dirs:
        # print(f"delete {item}")
        # delete_thumbs_db(item)
        # files = os.listdir(item)
        # files = [d for d in files if '.jpg' in d]
        # for file in files:
        #     if os.path.exists(os.path.join(item, file)):
        #         os.remove(os.path.join(item, file))
        # clear_bag_state(item)
        # delete_bag_files(item, True)
        od_folder = item + "-clip-20df"
        if os.path.exists(od_folder):
            if not os.path.exists(os.path.join(od_folder, 'pose')):
                extract_pose(item, od_folder)
            # delete_bag_files(item, True)
            # shutil.rmtree(od_folder)
        # else:
        #     clear_bag_state(item)

    sdk_results = {}
    sdk_group_size = max(thread_nums)
    for i in range(0, len(total_clip_dirs), sdk_group_size):
        bag_list = [d for d in total_clip_dirs if not is_bag_finished(d)]
        sdk_results[1] = execute_SDK_scripts(get_unfinished_bags(1, bag_list), paras_list[1])
        sdk_results[2] = execute_SDK_scripts(get_unfinished_bags(2, bag_list), paras_list[2])

        sdk_results[3] = execute_SDK_scripts(get_unfinished_bags(3, bag_list), paras_list[3])
        # sdk_results[5] = execute_SDK_scripts(get_unfinished_bags(5, bag_list), paras_list[5])
        sdk_results[4] = execute_SDK_scripts(get_unfinished_bags(4, bag_list), paras_list[4])

        execute_merge_pcd(get_unfinished_bags(7, bag_list))
        # 最后一步 7V周视去畸变 抽帧
        sdk_results[7] = extract_frame_OD(get_unfinished_bags(8, bag_list))
        for item in bag_list:
            od_frame_folder = item + "-clip-20df"
            if os.path.isdir(od_frame_folder):
                # extract_bev_files(item)
                extract_pose(item, od_frame_folder)
                # if not os.path.exists(os.path.join(od_frame_folder, 'pose', 'middle', 'pcd_num.txt')):
                #     print(f'replace {item} files')
                #     replace_occ_files_from_bag(item, od_frame_folder)
                # delete_bag_files(item, True)
    
    # print each step result
    for i in sdk_results:
        if i < 7:
            stage = paras_list[i].folder_name
        else: 
            stage = "8_od_frame_extract"
        done_str = "Done-" + str(len(sdk_results[i][0]))
        fail_str = "Fail-" + str(len(sdk_results[i][1]))
        print("{:<20}{:<10}{:<10}{:<10}".format(stage, done_str, fail_str, "{:.2f}s".format(sdk_results[i][2])))

    # 计算总耗时 打印结束时间
    end_time_total = time.time()
    total_duration = end_time_total - start_time_total
    print(f"total {len(total_clip_dirs)} clip, consume:{total_duration:.2f}s")
    print(f"finish time {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(end_time_total))}")