"""
ROS Bag 读取器 - 统一支持 ROS 1 和 ROS 2
"""

import os
from typing import Optional, Iterator, Tuple, Any, List
import glob


# 尝试导入 ROS 1 和 ROS 2 的 bag 库
ROS_AVAILABLE = None  # 'ros1', 'ros2', or None

try:
    import rosbag

    ROS_AVAILABLE = "ros1"
except ImportError:
    pass

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    ROS_AVAILABLE = "ros2"
except ImportError:
    pass


# 源 bag 目录路径（全局变量）
SRC_BAG_DIR: Optional[str] = None

# Bag 文件路径（全局变量）
BAG_PATH: Optional[str] = None
SRC_BAG_DIR: Optional[str] = None


def find_bag_file(src_bag_dir: str) -> Optional[str]:
    """自动查找目录中的 bag 文件 (ROS 1: .bag, ROS 2: .db3, .mcap)"""
    """自动查找目录中的 bag 文件 (ROS 1: .bag, ROS 2: .db3, .mcap)"""
    bag_files = glob.glob(os.path.join(src_bag_dir, "*.bag"))
    if bag_files:
        # 找到了 ROS 1 格式的 bag 文件
        if ROS_AVAILABLE != "ros1":
            print(f"  注意: 检测到 ROS 1 格式的 .bag 文件，但当前环境只支持 ROS 2")
            print(f"  提示: 要处理 ROS 1 bag 文件，请安装 ROS Noetic 或使用 Docker")
        return bag_files[0]

    # 查找 ROS 2 格式的 bag 文件
    bag_files = glob.glob(os.path.join(src_bag_dir, "*.db3"))
    if not bag_files:
        bag_files = glob.glob(os.path.join(src_bag_dir, "*.mcap"))

    # 查找包含 db3/mcap 文件的目录
    if not bag_files:
        db3_dirs = glob.glob(os.path.join(src_bag_dir, "*/*.db3"))
        if db3_dirs:
            bag_files = [os.path.dirname(db3_dirs[0])]

    if not bag_files:
        return None

    if len(bag_files) > 1:
        print(
            f"  警告: 发现多个 bag 文件，使用第一个: {os.path.basename(bag_files[0])}"
        )
    return bag_files[0]


class BagReader:
    """统一的 Bag 读取器，支持 ROS 1 和 ROS 2"""

    def __init__(self, bag_path: str):
        self.bag_path = bag_path

        # 根据 bag 文件扩展名自动选择 API
        if bag_path.endswith(".bag"):
            # ROS 1 格式
            if ROS_AVAILABLE != "ros1":
                raise RuntimeError(
                    f"ROS 1 bag 文件需要 ROS 1 的 rosbag 库，但当前环境只支持 ROS 2"
                )
            self.ros_version = "ros1"
            self._bag = rosbag.Bag(bag_path)
        else:
            # ROS 2 格式 (.db3, .mcap 或目录)
            if ROS_AVAILABLE != "ros2":
                raise RuntimeError(
                    f"ROS 2 bag 文件需要 ROS 2 的 rosbag2_py 库，但当前环境只支持 ROS 1"
                )
            self.ros_version = "ros2"
            storage_options = StorageOptions(uri=bag_path)
            converter_options = ConverterOptions("", "")
            self._bag = SequentialReader()
            self._bag.open(storage_options, converter_options)

    def read_messages(
        self, topics: Optional[List[str]] = None
    ) -> Iterator[Tuple[Any, Any, Any]]:
        """
        读取 bag 中的消息
        返回: (topic, message, timestamp)
        """
        if self.ros_version == "ros1":
            for topic, msg, timestamp in self._bag.read_messages(topics=topics):
                yield (topic, msg, timestamp)
        elif self.ros_version == "ros2":
            topic_types = self._bag.get_all_topics_and_types()
            type_map = {topic.type: topic.name for topic, _ in topic_types}

            for msg in self._bag:
                topic = msg.topic
                if topics and topic not in topics:
                    continue

                # 反序列化消息
                msg_type = get_message(type_map[msg.type_id])
                deserialized_msg = deserialize_message(msg.data, msg_type)

                # ROS 2 的 timestamp 需要从消息 header 中获取
                timestamp = 0
                if hasattr(deserialized_msg, "header"):
                    timestamp = deserialized_msg.header.stamp

                yield (topic, deserialized_msg, timestamp)

    def close(self):
        """关闭 bag 文件"""
        if self._bag:
            if self.ros_version == "ros1":
                self._bag.close()
            elif self.ros_version == "ros2":
                self._bag = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
