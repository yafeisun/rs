import shutil
import sys
import yaml
import rospy
import rosbag
import math
from sensor_msgs.msg import NavSatFix
import time
import os

# =========================================================
# 配置参数
# =========================================================
# 相机相关配置
save_camera_images = True  # 默认启用图像切割
image_handling_method = 'copy'  # 图片处理方式: 'copy' 或 'symlink'，默认使用拷贝
camera_folder = [
    "cam_around_back", "cam_around_front", "cam_around_left", "cam_around_right", "cam_back",
    "cam_front_left", "cam_front_right", "cam_side_left_back", "cam_side_left_front",
    "cam_side_right_back", "cam_side_right_front"
]
camera_forward_backwork_shift = 0  # 图像时间偏移量，单位：秒
camera_link_filename = "clips_imgs.index"  # 映射表文件名

# GPS静止检测配置
gps_topic = '/rs/gps'
default_distance_threshold = 0.002  # 距离阈值（米）
default_time_window_threshold = 5.0  # 时间窗口阈值（秒）


# =========================================================
# GPS分析器类
# =========================================================
class GPSAnalyzer:
    def __init__(self):
        self.gps_data = []
        self._distances_cache = None
        self._timestamps_cache = None
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the Earth surface distance between two GPS coordinates (meters)"""
        R = 6371000  # Earth radius (meters)
        
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def read_gps_from_bag(self, bag_path, gps_topic='/rs/gps'):
        """Read GPS data from bag file"""
        self._distances_cache = None
        self._timestamps_cache = None
        self.gps_data = []
        
        if not os.path.exists(bag_path):
            print(f"Error: Bag file does not exist: {bag_path}")
            return False
            
        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=[gps_topic]):
                    if msg._type == 'sensor_msgs/NavSatFix':
                        self.gps_data.append((
                            t.to_nsec(),  # Timestamp (nanoseconds)
                            msg.latitude,
                            msg.longitude,
                            msg.header.seq
                        ))
            
            return True
            
        except Exception as e:
            print(f"Error reading bag file: {e}")
            return False
    
    def read_gps_from_bag_direct(self, bag_data, gps_topic='/rs/gps'):
        """Read GPS data directly from an opened bag object"""
        self._distances_cache = None
        self._timestamps_cache = None
        self.gps_data = []
        
        try:
            for topic, msg, t in bag_data.read_messages(topics=[gps_topic]):
                if msg._type == 'sensor_msgs/NavSatFix':
                    self.gps_data.append((
                        t.to_nsec(),
                        msg.latitude,
                        msg.longitude,
                        msg.header.seq
                    ))
            
            return True
            
        except Exception as e:
            print(f"Error reading GPS data: {e}")
            return False
    
    def calculate_distances(self):
        """Calculate distances between adjacent GPS frames with caching"""
        if self._distances_cache is not None and self._timestamps_cache is not None:
            return self._distances_cache, self._timestamps_cache
        
        if len(self.gps_data) < 2:
            self._distances_cache, self._timestamps_cache = [], []
            return self._distances_cache, self._timestamps_cache
        
        distances = [0.0] * (len(self.gps_data) - 1)
        timestamps = [None] * (len(self.gps_data) - 1)
        
        data = self.gps_data
        haversine = self.haversine_distance
        
        for i in range(1, len(data)):
            prev = data[i-1]
            curr = data[i]
            
            distance = haversine(prev[1], prev[2], curr[1], curr[2])
            time_diff = curr[0] - prev[0]
            
            distances[i-1] = distance
            timestamps[i-1] = (prev[0], curr[0], time_diff)
        
        self._distances_cache = distances
        self._timestamps_cache = timestamps
        
        return distances, timestamps
    
    def detect_stationary_periods(self, distance_threshold=0.01, time_window_threshold=5.0):
        """Detect vehicle stationary periods"""
        if len(self.gps_data) < 2:
            return []
        
        distances, timestamps = self.calculate_distances()
        time_window_threshold_ns = time_window_threshold * 1000000000
        
        stationary_periods = []
        current_stationary_start = None
        current_stationary_start_frame = None
        
        len_distances = len(distances)
        
        for i in range(len_distances):
            if distances[i] <= distance_threshold:
                if current_stationary_start is None:
                    current_stationary_start = timestamps[i][0]
                    current_stationary_start_frame = i
            else:
                if current_stationary_start is not None:
                    duration_ns = timestamps[i-1][1] - current_stationary_start
                    
                    if duration_ns >= time_window_threshold_ns:
                        duration_sec = convert_ns_to_sec(duration_ns)
                        stationary_periods.append({
                            'start_time': current_stationary_start,
                            'end_time': timestamps[i-1][1],
                            'duration_ns': duration_ns,
                            'duration_sec': duration_sec,
                            'start_frame': current_stationary_start_frame,
                            'end_frame': i
                        })
                    
                    current_stationary_start = None
                    current_stationary_start_frame = None
        
        if current_stationary_start is not None:
            duration_ns = self.gps_data[-1][0] - current_stationary_start
            if duration_ns >= time_window_threshold_ns:
                duration_sec = convert_ns_to_sec(duration_ns)
                stationary_periods.append({
                    'start_time': current_stationary_start,
                    'end_time': self.gps_data[-1][0],
                    'duration_ns': duration_ns,
                    'duration_sec': duration_sec,
                    'start_frame': current_stationary_start_frame,
                    'end_frame': len_distances
                })
        
        return stationary_periods


# =========================================================
# 辅助函数
# =========================================================
def get_info(yaml_path):
    """Read information from YAML file"""
    try:
        info = yaml.load(open(yaml_path, 'r'), Loader=yaml.FullLoader)
    except FileNotFoundError:
        print(f"info.yaml not found in {yaml_path}")
        return None
    return info

def get_bag_data(path):
    """Get bag file information and opened bag object"""
    start_time = time.time()  # 开始计时
    
    bag_data = rosbag.Bag(path, 'r')
    bag_start_time = bag_data.get_start_time()
    bag_end_time = bag_data.get_end_time()
    bag_total_messages = bag_data.get_message_count()
    bag_time = bag_end_time - bag_start_time

    bag_info = {
        "data": bag_data,
        "start_time": bag_start_time,
        "end_time": bag_end_time,
        "total_messages": bag_total_messages,
        "interval": bag_time
    }
    
    elapsed_time = time.time() - start_time  # 计算耗时
    print(f"read {os.path.basename(path)} info, consume: {elapsed_time:.2f}s")

    return bag_info

def info_yaml_writer(ori_info, name, start_time, end_time, path):
    """Write information to YAML file"""
    write_info_data = {
        "bag_name": name,
        "car_label": ori_info["car_label"],
        "record_time": end_time - start_time,  # Actual recording duration
        "start_record_timestamp": start_time,
        "stop_record_timestamp": end_time
    }
    with open(os.path.join(path, "info.yaml"), 'w') as file:
        yaml.safe_dump(write_info_data, file, allow_unicode=True, default_flow_style=False)

def convert_ns_to_sec(nanoseconds):
    """Convert nanoseconds to seconds"""
    return nanoseconds / 1e9


# =========================================================
# 核心功能模块
# =========================================================
def prepare_directories(bag_dir):
    """Prepare necessary directories and check files"""
    state_path = os.path.join(bag_dir, "state")
    success_flag = os.path.join(state_path, "clip_successed.txt")

    if not os.path.exists(state_path):
        os.makedirs(state_path)
    if os.path.exists(success_flag):
        print(f"remove {success_flag}")
        os.remove(success_flag)

    # Read info.yaml
    info_yaml_path = os.path.join(bag_dir, "info.yaml")
    info_data = get_info(info_yaml_path)
    if info_data is None:
        return None, None, None

    # Find bag file
    bag_path = ""
    for item in os.listdir(bag_dir):
        if str(item).endswith(".bag"):
            bag_path = os.path.join(bag_dir, item)
            break

    if bag_path == "":
        print(f"bag file not found in {bag_dir}")
        return None, None, None

    return state_path, success_flag, info_data, bag_path

def detect_stationary_periods(bag_info, distance_threshold, time_window_threshold):
    """Detect stationary periods using GPS data"""
    analyzer = GPSAnalyzer()
    success = analyzer.read_gps_from_bag_direct(bag_info["data"], gps_topic)
    
    if not success:
        print(f"Failed to read GPS data from bag file")
        return None

    stationary_periods = analyzer.detect_stationary_periods(distance_threshold, time_window_threshold)
    
    if not stationary_periods:
        print(f"No stationary segments detected")
        return None
    
    durations = [round(period['duration_sec'], 2) for period in stationary_periods]
    print(f"{len(stationary_periods)} stationary segments detected: {durations}")
        
    return stationary_periods

def process_single_segment(bag_info, period, path, info_data, segment_index):
    """Process a single stationary segment"""
    start_time = time.time()  # 开始计时
    
    # Prepare save path and file name
    file_name = os.path.basename(path)
    file_name_index = f"{file_name}_{segment_index}"
    save_path = os.path.join(path, "clips", file_name_index)
    bag_name = f"{file_name_index}.bag"

    # Create directories and flags
    clip_state_path = os.path.join(save_path, "state")
    clip_success_flag = os.path.join(clip_state_path, "clip_successed.txt")

    if not os.path.exists(clip_state_path):
        os.makedirs(clip_state_path)
    if os.path.exists(clip_success_flag):
        print(f"remove {clip_success_flag}")
        os.remove(clip_success_flag)

    if not os.path.exists(save_path):
        os.makedirs(save_path)
        
    # Convert timestamps
    start_time_sec = convert_ns_to_sec(period['start_time'])
    end_time_sec = convert_ns_to_sec(period['end_time'])
    
    write_bag_message = bag_info["data"].read_messages(
        start_time=rospy.Time.from_sec(start_time_sec),
        end_time=rospy.Time.from_sec(end_time_sec))

    with rosbag.Bag(os.path.join(save_path, bag_name), "w") as clips:
        for topic, msg, t in write_bag_message:
            clips.write(topic, msg, t)
    
    # Process camera images
    if save_camera_images:
        process_camera_images(path, save_path, start_time_sec, end_time_sec)
    
    # Copy configuration files
    copy_config_files(path, save_path, info_data, bag_name, start_time_sec, end_time_sec)
    
    # Mark as successful
    with open(clip_success_flag, 'w') as file:
        pass
        
    elapsed_time = time.time() - start_time  # 计算耗时
    print(f"create clip {file_name_index} consume: {elapsed_time:.2f}s")

def process_camera_images(source_path, target_path, start_time_sec, end_time_sec):
    """Process camera images for the segment"""
    camera_path = {}
    for item in camera_folder:
        camera_path[item] = os.path.join(source_path, item)
        
    for item in camera_path:
        if not os.path.exists(camera_path[item]):
            continue
            
        camera_link_detail = []
        
        for jpg_file in os.listdir(camera_path[item]):
            if ".jpg" not in jpg_file:
                continue
                
            try:
                # Get image timestamp
                jpg_file_time_ns = int(jpg_file.split(".")[0])
                # 将纳秒转秒替换为使用函数
                jpg_file_time_sec = convert_ns_to_sec(jpg_file_time_ns)
                
                # Check if image is within time range
                if (start_time_sec - camera_forward_backwork_shift <= jpg_file_time_sec <= 
                    end_time_sec + camera_forward_backwork_shift):
                    jpg_file_save_path = os.path.join(target_path, item, jpg_file)
                    if not os.path.exists(os.path.dirname(jpg_file_save_path)):
                        os.makedirs(os.path.dirname(jpg_file_save_path))

                    # Remove existing file or link
                    if os.path.exists(jpg_file_save_path):
                        if os.path.islink(jpg_file_save_path):
                            os.remove(jpg_file_save_path)
                        else:
                            os.unlink(jpg_file_save_path)

                    # Handle image based on configuration
                    if image_handling_method.lower() == 'copy':
                        # Copy image file
                        src_img_path = os.path.join(camera_path[item], jpg_file)
                        shutil.copy2(src_img_path, jpg_file_save_path)
                    else:
                        # Create symbolic link
                        os.symlink(
                            os.path.relpath(os.path.join(camera_path[item], jpg_file), 
                                            start=os.path.dirname(jpg_file_save_path)),
                            jpg_file_save_path
                        )

                    camera_link_detail.append(jpg_file.split(".")[0])
            except Exception as e:
                print(f"Error processing image {jpg_file}: {e}")
                continue

        # Write mapping table
        if camera_link_detail and os.path.exists(os.path.dirname(jpg_file_save_path)):
            with open(os.path.join(os.path.dirname(jpg_file_save_path), camera_link_filename), 'w') as file:
                for timestamp in camera_link_detail:
                    file.write(timestamp + "\n")

def copy_config_files(source_path, target_path, info_data, bag_name, start_time_sec, end_time_sec):
    """Copy configuration files to the segment directory"""
    # Write info.yaml
    info_yaml_writer(info_data, bag_name, start_time_sec, end_time_sec, target_path)

    # Copy vehicle configuration files
    if os.path.exists(os.path.join(source_path, str(info_data["car_label"] + ".yaml"))):
        shutil.copy(os.path.join(source_path, str(info_data["car_label"] + ".yaml")),
                    os.path.join(target_path, str(info_data["car_label"] + ".yaml")))
    if os.path.exists(os.path.join(source_path, "label.yaml")):
        shutil.copy(os.path.join(source_path, "label.yaml"), os.path.join(target_path, "label.yaml"))


# =========================================================
# 主函数
# =========================================================
def write_bag_clips(path, distance_threshold=default_distance_threshold, 
                    time_window_threshold=default_time_window_threshold, 
                    fast_mode=False):
    """Main function to process bag file and create clips based on stationary periods"""
    try:
        # Prepare directories and check files
        result = prepare_directories(path)
        if result is None:
            return 0
        
        state_path, success_flag, info_data, bag_path = result
        
        # Record overall start time
        overall_start_time = time.time()
        
        # Get basic bag information
        bag_info = get_bag_data(bag_path)
        
        # Detect stationary periods
        stationary_periods = detect_stationary_periods(bag_info, distance_threshold, time_window_threshold)
        if stationary_periods is None:
            bag_info["data"].close()
            with open(success_flag, 'w') as file:
                pass
            return 0
        else:        
            # Process each segment
            for i, period in enumerate(stationary_periods):
                process_single_segment(bag_info, period, path, info_data, i)
        
        # Close bag file
        bag_info["data"].close()
        with open(success_flag, 'w') as file:
            pass

        # Calculate and print overall processing time
        overall_elapsed_time = time.time() - overall_start_time
        print(f"All clips created successfully. Total time: {overall_elapsed_time:.2f}s")
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        # Ensure bag file is closed in case of exception
        if 'bag_info' in locals() and 'data' in bag_info:
            bag_info["data"].close()
    return len(stationary_periods)


# =========================================================
# 入口点
# =========================================================
if __name__ == "__main__":
    # if len(sys.argv) < 2:
    #     print("Usage: python bag_clips_optimized.py <bag_file_directory>")
    #     sys.exit(1)
    # directory_path = sys.argv[1]
    # write_bag_clips(directory_path)
    bag_dirs = []
    bag_dirs.append("/media/geely/sdb/S10_0820/sta_dyna_sta1/2025-08-20-16-40-35")
    for bag_dir in bag_dirs:
        print(f"Processing {bag_dir}")
        write_bag_clips(bag_dir)