import logging
import os
import shutil
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import subprocess
# 设置日志配置
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()  # 输出到控制台
    ]
)
logger = logging.getLogger(__name__)

def copy_file_to_directory(src_file_path, dest_dir_path):
    """
    将指定文件拷贝到给定目录。

    :param src_file_path: 源文件路径
    :param dest_dir_path: 目标目录路径
    :return: 如果成功则返回新文件的路径，否则返回None
    """
    try:
        # 确保源文件存在
        if not os.path.isfile(src_file_path):
            logger.info(f"Source file does not exist: {src_file_path}")
            return None

        # 确保目标目录存在，如果不存在则创建
        os.makedirs(dest_dir_path, exist_ok=True)

        # 构建目标文件路径（保持原文件名）
        dest_file_path = os.path.join(dest_dir_path, os.path.basename(src_file_path))

        # 拷贝文件
        shutil.copy2(src_file_path, dest_file_path)  # 使用copy2以保留元数据
        logger.info(f"File copied successfully to: {dest_file_path}")
        return dest_file_path

    except Exception as e:
        logger.info(f"An error occurred while copying the file: {e}")
        traceback.print_exc()
        return None

def delete_dir(tmp_dir):
    # 删除非空目录及其所有内容
    try:
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
            logger.info(f"Directory {tmp_dir} and all its contents have been removed.")
    except Exception as e:
        logger.info(f"An error occurred while deleting the directory: {e}")

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE) # 30分钟超时
        logger.info(f"Command executed successfully: {command}")
        return result.stdout.decode('utf-8')
    except subprocess.TimeoutExpired as e:
        raise
    except subprocess.CalledProcessError as e:
        raise


import json
import threading
lock = threading.Lock()
def append_file_content(output_file, text):
    # 确保每次写入时都是一个新的行
    with lock:
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'a', encoding='utf-8') as f:
            # 如果text是一个字典或列表等可序列化对象，则先转换为JSON字符串
            if isinstance(text, (dict, list)):
                json_str = json.dumps(text, ensure_ascii=False)
                f.write(f"{json_str}\n")
            else:
                # 如果text已经是字符串，则直接写入
                f.write(f"{text}\n")


def get_file_size_with_pathlib(filepath):
    path = Path(filepath)
    try:
        return path.stat().st_size
    except FileNotFoundError:
        print(f"The file {filepath} does not exist.")
        raise
    except PermissionError:
        print(f"Permission denied when accessing the file {filepath}.")
        raise


import yaml
def extract_topics(yaml_file):
    with open(yaml_file, 'r', encoding='utf-8') as file:
        yaml_content = file.read()
    # 将YAML内容转换为Python对象
    # print(yaml_content)
    data = yaml.safe_load(yaml_content)

    topics = []

    # 获取sensors下的所有传感器类型
    sensors = data.get('sensors', {})
    # 遍历每个传感器类型（例如lidar, camera等）
    for sensor_item in sensors['lidar']:
        # 获取每个传感器类型下的topics
        topic = sensor_item.get('topic')
        topics.append(topic)
    for sensor_item in sensors['camera']:
        # 获取每个传感器类型下的topics
        topic = sensor_item.get('topic')
        topics.append(topic)
    # 获取sensors下的rtk部分pip i
    rtk = sensors.get('rtk', {})
    # 提取topics
    imu_topic = rtk.get('imu_topic')
    odom_topic = rtk.get('odom_topic')
    gps_topic = rtk.get('gps_topic')

    if imu_topic:
        topics.append(imu_topic)
    if odom_topic:
        topics.append(odom_topic)
    if gps_topic:
        topics.append(gps_topic)

    return topics

import re
def get_topics_from_rosbag_info(bag_file):
    # 构建并执行命令
    command = f"source /opt/ros/noetic/setup.sh; rosbag info {bag_file};"
    output = run_command(command)
    pattern = re.compile(r'topics:\s*((?:\s*\S+.*?\n?)*)', re.DOTALL)
    match = pattern.search(output)
    topics = []
    if match:
        # 获取topics部分
        topics_info = match.group(1).strip()
        # 每个topic的信息按行分割
        topic_lines = topics_info.split('\n')
        for line in topic_lines:
            # 提取topic名称和消息类型
            parts = line.split()
            if len(parts) > 1:
                topic = parts[0]
                topics.append(topic)
    return topics

def process_bag(bag_dir, compare_size_flag, size_rate):
    """
    处理单个bag目录，将所有文件拷贝到目标目录。

    :param bag_dir: bag目录路径
    """
    # 构建目标目录路径
    logger.info(f"Processing bag directory: {bag_dir}")

    work_dir = '/dev/shm/rosbag'
    bag_name = Path(bag_dir).name
    local_bag_dir = os.path.join(work_dir, bag_name)

    try:
        # 获取采集时长
        info_yaml = os.path.join(bag_dir, 'info.yaml')
        with open(info_yaml, 'r', encoding='utf-8') as file:
            info_yaml_content = file.read()
        data = yaml.safe_load(info_yaml_content)
        record_time = data['record_time']
        car_label = data['car_label']

        # 判断参数文件名
        if not os.path.isfile(os.path.join(bag_dir, f'{car_label}.yaml')):
            append_file_content(output_error_file, f'{bag_dir} QI unpass: the file name matches the parameter file name.')
            return 1
        yaml_topics = extract_topics(os.path.join(bag_dir, f'{car_label}.yaml'))
        source_dirs = []
        # 判断摄像头是否丢失
        for topic in yaml_topics:
            if topic.startswith('/cam'):
                source_dirs.append(str(Path(topic).parent).replace('/', ''))
        for sub_dir in source_dirs:
            if not os.path.isdir(os.path.join(bag_dir, sub_dir)):
                append_file_content(output_error_file, f'{bag_dir} QI unpass: {sub_dir} is empty')
                return 3
            img_files = [f for f in os.listdir(os.path.join(bag_dir, sub_dir)) if f.endswith('.jpg')]
            if len(img_files) < 200:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: {sub_dir}  {len(img_files)} < 200')
                return 4
            if len(img_files) < record_time * 9:
                append_file_content(output_error_file,
                                    f'{bag_dir} QI unpass: {sub_dir}  {len(img_files)} < {record_time * 10}')
                return 4


        if not compare_size_flag:
            os.makedirs(local_bag_dir, exist_ok=True)
            copy_file_to_directory(os.path.join(bag_dir, f'{bag_name}.bag'), local_bag_dir)
            bag_topics = get_topics_from_rosbag_info(os.path.join(local_bag_dir, f'{bag_name}.bag'))
            bag_yaml_file_origin = ['/m1p/rslidar_packets_unique', '/middle/rslidar_packets_unique', '/rs/gps', '/rs/imu', '/rs/odom']
            bag_in_bag_origin = ['/rs/4D_radar', '/rs/chassis', '/rs/front_corner_radar', '/rs/milliwareRadar_PointCloud_Msg', '/rs/rear_corner_radar']

            # 判断是否漏采数据
            if '/seyond/rslidar_packets_unique' in yaml_topics and '/falcon_packets' not in bag_topics:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data (falcon_packets).')
                return 2
            if  '/falcon_packets' in bag_topics and '/seyond/rslidar_packets_unique' not in yaml_topics:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data. (falcon_packets)')
                return 2
            # 判断单独有的
            for topic in bag_in_bag_origin:
                if topic not in bag_topics:
                    append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data. ({topic})')
                    return 2
            # 判断bag有yaml也有的
            for topic in bag_yaml_file_origin:
                if topic in yaml_topics and topic not in bag_topics:
                    append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data.  ({topic})')
                    return 2
        else:
            bag_file_size = get_file_size_with_pathlib(os.path.join(bag_dir, f'{bag_name}.bag'))
            if bag_file_size < record_time * 7 * 1024 * 1024 * 1024 / 299 * size_rate:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data.')
                return 3

        return 0
    except Exception as e:
        logger.info(f"An error occurred while processing bag directory: {bag_dir} - {e}")
        traceback.print_exc()
        append_file_content(output_error_file, f'{bag_dir} QI unpass: exception {e}  ')
        return -1
    finally:
        delete_dir(local_bag_dir)

def process_bag_no_download(bag_dir, compare_size_flag, size_rate):
    """
    处理单个bag目录，将所有文件拷贝到目标目录。

    :param bag_dir: bag目录路径
    """
    # 构建目标目录路径
    logger.info(f"Processing bag directory: {bag_dir}")

    bag_name = Path(bag_dir).name

    try:
        # 获取采集时长
        info_yaml = os.path.join(bag_dir, 'info.yaml')
        with open(info_yaml, 'r', encoding='utf-8') as file:
            info_yaml_content = file.read()
        data = yaml.safe_load(info_yaml_content)
        record_time = data['record_time']
        car_label = data['car_label']

        # 判断参数文件名
        if not os.path.isfile(os.path.join(bag_dir, f'{car_label}.yaml')):
            append_file_content(output_error_file,
                                f'{bag_dir} QI unpass: the file name matches the parameter file name.')
            return 1
        yaml_topics = extract_topics(os.path.join(bag_dir, f'{car_label}.yaml'))
        source_dirs = []
        # 判断摄像头是否丢失
        for topic in yaml_topics:
            if topic.startswith('/cam'):
                source_dirs.append(str(Path(topic).parent).replace('/', ''))
        for sub_dir in source_dirs:
            if not os.path.isdir(os.path.join(bag_dir, sub_dir)):
                append_file_content(output_error_file, f'{bag_dir} QI unpass: {sub_dir} is empty')
                return 3
            img_files = [f for f in os.listdir(os.path.join(bag_dir, sub_dir)) if f.endswith('.jpg')]
            if len(img_files) < 200:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: {sub_dir}  {len(img_files)} < 200')
                return 4
            if len(img_files) < record_time * 9:
                append_file_content(output_error_file,
                                    f'{bag_dir} QI unpass: {sub_dir}  {len(img_files)} < {record_time * 10}')
                return 4

        if not compare_size_flag:
            bag_topics = get_topics_from_rosbag_info(os.path.join(bag_dir, f'{bag_name}.bag'))
            bag_yaml_file_origin = ['/m1p/rslidar_packets_unique', '/middle/rslidar_packets_unique', '/rs/gps',
                                    '/rs/imu', '/rs/odom']
            bag_in_bag_origin = ['/rs/4D_radar', '/rs/chassis', '/rs/front_corner_radar',
                                 '/rs/milliwareRadar_PointCloud_Msg', '/rs/rear_corner_radar']

            # 判断是否漏采数据
            if '/seyond/rslidar_packets_unique' in yaml_topics and '/falcon_packets' not in bag_topics:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data (falcon_packets).')
                return 2
            if '/falcon_packets' in bag_topics and '/seyond/rslidar_packets_unique' not in yaml_topics:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data. (falcon_packets)')
                return 2
            # 判断单独有的
            for topic in bag_in_bag_origin:
                if topic not in bag_topics:
                    append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data. ({topic})')
                    return 2
            # 判断bag有yaml也有的
            for topic in bag_yaml_file_origin:
                if topic in yaml_topics and topic not in bag_topics:
                    append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data.  ({topic})')
                    return 2
        else:
            bag_file_size = get_file_size_with_pathlib(os.path.join(bag_dir, f'{bag_name}.bag'))
            if bag_file_size < record_time * 7 * 1024 * 1024 * 1024 / 299 * size_rate:
                append_file_content(output_error_file, f'{bag_dir} QI unpass: omitted data.')
                return 3

        return 0
    except Exception as e:
        logger.info(f"An error occurred while processing bag directory: {bag_dir} - {e}")
        traceback.print_exc()
        append_file_content(output_error_file, f'{bag_dir} QI unpass: exception {e}  ')
        return -1



def get_second_level_subdirectories(root_dir):
    """
    获取指定目录下第二级子目录的所有路径。
    """
    # 将字符串转换为Path对象
    root_path = Path(root_dir)

    # 检查根目录是否存在
    if not root_path.exists() or not root_path.is_dir():
        raise ValueError(f"The directory {root_dir} does not exist or is not a directory.")

    # 初始化一个集合以避免重复的目录
    second_level_dirs = set()

    # 遍历根目录下的第一级子目录
    for first_level in root_path.iterdir():
        if first_level.is_dir():
            # 遍历第一级子目录下的第二级子目录
            for second_level in first_level.iterdir():
                if second_level.is_dir():
                    second_level_dirs.add(str(second_level))

    return list(second_level_dirs)

def mv_bag_dir():
    # 读取文件
    bag_dirs = []
    with open(output_error_file, 'r', encoding='utf-8') as file:
        for line in file:
            bag_dirs.append(line.strip().split(" ")[0])

    for bag_dir in bag_dirs:
        try:
            os.makedirs(os.path.join(output_dir, "Bag_QI_error", Path(bag_dir).name), exist_ok=True)
            run_command(f'mv {bag_dir} {os.path.join(output_dir, "Bag_QI_error")}')
        except Exception as e:
            logger.info(f"An error occurred while moving bag directory: {bag_dir} - {e}")
            traceback.print_exc()

def main(time_dir_str, n, mv_flag, download_flag, compare_size_flag, size_rate):
    # 遍历t_dir下的所有目录
    bag_dirs = []
    root_dirs = time_dir_str.split(',')
    for root_dir in root_dirs:
        bag_dirs.extend(get_second_level_subdirectories(root_dir))
    # 过滤掉非法目录
    bag_dirs = [bag_dir for bag_dir in bag_dirs if "-" in Path(bag_dir).name]
    bag_dirs = [bag_dir for bag_dir in bag_dirs if "." not in Path(bag_dir).name]

    logger.info(bag_dirs)
    logger.info(f"total bag dirs: {len(bag_dirs)}")

    # 启动多线程处理
    total_count = len(bag_dirs)
    count = 0
    success_count = 0
    error_count = 0
    error_type_1_count = 0
    error_type_2_count = 0
    error_type_3_count = 0
    error_type_4_count = 0
    error_type_other_count = 0
    with ThreadPoolExecutor(max_workers=n) as executor:
        futures = None
        if download_flag:
            futures = {executor.submit(process_bag, bag_dir, compare_size_flag, size_rate): bag_dir for bag_dir in bag_dirs}
        else:
            futures = {executor.submit(process_bag_no_download, bag_dir, compare_size_flag, size_rate): bag_dir for bag_dir in bag_dirs}
        # 等待所有线程完成，并打印进度
        for future in as_completed(futures):
            bag_dir = futures[future]
            try:
                flag = future.result()  # 获取结果或抛出异常
                if flag == 0:
                    append_file_content(output_pass_file, bag_dir)
                    success_count += 1
                else:
                    error_count += 1
                if flag == 1:
                    error_type_1_count += 1
                elif flag == 2:
                    error_type_2_count += 1
                elif flag == 3:
                    error_type_3_count += 1
                elif flag == 4:
                    error_type_4_count += 1
                elif flag == -1:
                    error_type_other_count += 1
                count += 1
            except Exception as exc:
                logger.error(f"Clip processing generated an exception: {exc}")
            finally:
                logger.info(f"Processed {count} out of {total_count} bags." + "=" * 40)

    append_file_content(output_summary_file, f'total: {total_count}')
    append_file_content(output_summary_file, f'valid bag: {success_count} , percent: {success_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'error bag: {error_count} , percent: {error_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'    - The file name matches the parameter file name: {error_type_1_count} , percent:{error_type_1_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'    - Omitted data: {error_type_2_count} , percent:{error_type_2_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'    - Camera lost: {error_type_3_count} , percent:{error_type_3_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'    - Insufficient frame rate(< 90%): {error_type_4_count} , percent:{error_type_4_count / total_count * 100:.2f}%')
    append_file_content(output_summary_file, f'    - other error: {error_type_other_count} , percent:{error_type_other_count / total_count * 100:.2f}%')


    if mv_flag:
        mv_bag_dir()
def delete_file(input_file):
    try:
        if os.path.isfile(input_file):
            os.remove(input_file)
    except Exception as exc:
        logger.info(f"Task failed with exception: {exc}")
        return False
def create_output_files(*args):
    for file in args:
        os.makedirs(os.path.dirname(file), exist_ok=True)
        delete_file(file)

"""
功能描述: 对bag进行QI检测
算法名称: bag_qi_2025.py
参数说明:
-timeDir        输入采集时间的路径，多个用逗号隔开，例如: /data/wyc-test/202501251710/20250121,...
-o              输出相关统计信息文件的目录
-mvSwitch       是否移动文件到指定目录(选填,默认不移动) 
-download       是否下载数据到本地进行算法操作(选填,默认不下载)
-compareSize    是否使用比较bag大小的方式判断数据是否完整(选填,默认不使用)
-n              线程数(选填,默认10)
使用案例: 
python3 /data/maogen/tools/bag_qi_2025.py -timeDir /data/wyc-test/202502120830/20250208/ -o /data/maogen/test/bag_qi_2025_test2 -n 4 -download
python3 /data/maogen/tools/bag_qi_2025.py -timeDir /data/wyc-test/202502120830/20250208/ -o /data/maogen/test/bag_qi_2025_test3 -n 4 -compareSize 
python3 /data/maogen/tools/bag_qi_2025.py -timeDir /data/wyc-test/202502120830/20250208/ -o /data/maogen/test/bag_qi_2025_test4 -n 4
python3 /data/maogen/tools/bag_qi_2025.py -timeDir /data/gri-ziyandata/202502071226/20250123/ -o /data/maogen/test/bag_qi_2025_test5 -n 4


python3 /data/maogen/tools/bag_qi_2025.py -timeDir /data/gri-ziyandata/202502071226/20250123/ -o /data/maogen/test/bag_qi_2025_test5 -mvSwitch
"""
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-timeDir', type=str)
    parser.add_argument('-o', type=str)
    parser.add_argument('-mvSwitch', action='store_true')
    parser.add_argument('-download', action='store_true')  # 是否下载到本地
    parser.add_argument('-compareSize', action='store_true')  # 是否下载到本地
    parser.add_argument('-size_rate', type=float, default=0.6)  # 是否下载到本地
    parser.add_argument('-n', type=int, default=5)

    args = parser.parse_args()
    output_dir = args.o
    # 创建必要的输出文件
    output_error_file = os.path.join(output_dir, "error.txt")
    output_pass_file = os.path.join(output_dir, "pass.txt")
    output_summary_file = os.path.join(output_dir, "summary.txt")
    create_output_files(output_error_file
                        , output_pass_file
                        , output_summary_file
                        )

    main(args.timeDir, args.n, args.mvSwitch, args.download, args.compareSize, args.size_rate)