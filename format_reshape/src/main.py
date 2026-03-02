#!/usr/bin/env python3
"""
ROS Bag 格式转换工具 (主入口点)

功能：
1. 相机图像拷贝
2. 点云数据拷贝
3. 标定参数转换
4. Pose数据转换（在线/离线优化）
5. IMU/GNSS/轮速数据提取

使用：
    python3 main.py <数据根目录> [--target-root <目标根目录>]

示例：
    python3 main.py /path/to/data_root
    python3 main.py /path/to/data_root --target-root /path/to/output

说明：
    程序会递归查找符合 YYYY-MM-DD-HH-MM-SS 格式且包含 .bag 文件的目录，
    然后批量进行格式转换。
"""

import os
import subprocess
import re
import argparse
from typing import List
from .bag_reader import ROS_AVAILABLE, find_bag_file
from .extractors import (
    extract_camera_images,
    extract_lidar_pcd,
    extract_lidar_concat,
    extract_calibration,
    extract_lidar_map,
)
from .calibration import (
    generate_calib_anno,
    generate_calib_anno_vc,
    generate_node_output,
    generate_camera_poses,
    generate_lidar_main_pose,
)
from .converters import (
    convert_pose_online,
    convert_pose_offline,
    extract_imu_from_bag,
    extract_gnss_from_bag,
    extract_wheel_from_bag,
)
from .utils import create_directory


# 时间戳目录格式: YYYY-MM-DD-HH-MM-SS
TIMESTAMP_DIR_PATTERN = re.compile(r"^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$")



def find_valid_bag_dirs(root_path: str) -> List[str]:
    """
    递归查找符合时间戳格式且包含 bag 文件的有效目录

    Args:
        root_path: 搜索根目录

    Returns:
        有效目录路径列表
    """
    valid_dirs = []

    for dirpath, dirnames, filenames in os.walk(root_path):
        # 检查目录名是否符合时间戳格式
        dir_name = os.path.basename(dirpath)
        if not TIMESTAMP_DIR_PATTERN.match(dir_name):
            continue

        # 检查是否有 bag 文件
        bag_files = [f for f in filenames if f.endswith(".bag")]
        if bag_files:
            valid_dirs.append(dirpath)
    valid_dirs.sort()
    return valid_dirs


def process_single_bag(src_dir: str, target_root: str) -> bool:
    """
    处理单个 bag 目录

    Args:
        src_dir: 源 bag 目录路径
        target_root: 目标根目录

    Returns:
        处理是否成功
    """


    # 从源路径提取最后两级目录作为目标子目录
    src_bag_dir = src_dir
    target_root_param = target_root  # 临时保存参数以避免混淆

    src_path_parts = src_bag_dir.rstrip("/").split("/")
    if len(src_path_parts) >= 2:
        target_dir = os.path.join(target_root_param, src_path_parts[-2], src_path_parts[-1])
    else:
        target_dir = os.path.join(target_root_param, src_path_parts[-1])

    # 自动查找 bag 文件
    bag_path = find_bag_file(src_bag_dir)

    # 检查是否找到 bag 文件
    if bag_path is None:
        raise FileNotFoundError(f"No bag file found in {src_bag_dir}")

    try:
        create_directory(target_dir)

        # 第一部分：相机、点云、标定
        extract_camera_images(src_bag_dir, target_dir)
        extract_lidar_pcd(src_bag_dir, target_dir)
        extract_lidar_concat(src_bag_dir, target_dir)
        extract_calibration(src_bag_dir, target_dir)

        # 生成 calib_anno 和 calib_anno_vc
        generate_calib_anno(target_dir)
        generate_calib_anno_vc(target_dir)

        # 生成 node_output
        generate_node_output(src_bag_dir, target_dir)

        # 生成相机位姿
        generate_camera_poses(src_bag_dir, target_dir)

        # 生成主雷达位姿
        generate_lidar_main_pose(src_bag_dir, target_dir)

        # 拷贝 lidar 地图（需要在 generate_lidar_main_pose 之后，因为需要使用 egopose_optpose 中的时间戳）
        extract_lidar_map(src_bag_dir, target_dir)

        # 第二部分：Pose数据
        convert_pose_online(src_bag_dir, target_dir)
        convert_pose_offline(src_bag_dir, target_dir)

        # 第三部分：IMU/GNSS/Wheel
        extract_imu_from_bag(bag_path, target_dir)
        extract_gnss_from_bag(bag_path, target_dir)
        extract_wheel_from_bag(bag_path, target_dir)
        # 第四部分：可视化验证
        print(f"\n  [Visualization] Running projection verification...")
        projection_script = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "scripts/project_calib.py",
        )
        if os.path.exists(projection_script):
            try:
                subprocess.run(
                    [
                        "python3",
                        projection_script,
                        "--data-dir",
                        target_dir,
                        "--output-dir",
                        "visualize",
                    ],
                    check=True,
                )
                print(
                    f"                 Done: Visualization generated in {target_dir}/visualize/"
                )
            except subprocess.CalledProcessError as e:
                print(f"                 Warning: Projection verification failed: {e}")
        else:
            print(
                f"                 Warning: Projection script not found at {projection_script}"
            )

        return True

        return True

    except Exception as e:
        print(f"  Error: {e}")
        return False


def main():
    """主函数 - 批量处理符合条件的数据目录"""
    parser = argparse.ArgumentParser(
        description="ROS Bag Format Converter (Batch Processing)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python3 main.py /path/to/data_root
  python3 main.py /path/to/data_root --target-root /path/to/output
  
Description:
  Recursively find directories matching YYYY-MM-DD-HH-MM-SS format with .bag files,
  then perform batch format conversion.
        """,
    )
    parser.add_argument(
        "data_root",
        type=str,
        help="Data root directory, will recursively search for timestamp-format bag directories",
    )
    parser.add_argument(
        "--target-root",
        "-t",
        type=str,
        default=None,
        help="Target root directory (optional, default: current directory)",
    )

    args = parser.parse_args()

    # 检查数据根目录
    data_root = args.data_root
    if not os.path.isdir(data_root):
        print(f"Error: Data root directory not found: {data_root}")
        return

    # 设置目标根目录
    target_root = args.target_root if args.target_root else os.getcwd()

    print(f"Data root: {data_root}")
    print(f"Target root: {target_root}")

    # 查找有效的 bag 目录
    valid_dirs = find_valid_bag_dirs(data_root)

    if not valid_dirs:
        print("No valid data directories found")
        return

    print(f"Found {len(valid_dirs)} data directories")

    # 批量处理
    success_count = 0
    fail_count = 0

    for i, src_dir in enumerate(valid_dirs, 1):
        print(f"\n[{i}/{len(valid_dirs)}] {src_dir}")
        if process_single_bag(src_dir, target_root):
            success_count += 1
        else:
            fail_count += 1

    # 汇总
    print(f"\nDone! Success: {success_count}, Failed: {fail_count}")


if __name__ == "__main__":
    # 显示检测到的 ROS 版本
    if ROS_AVAILABLE == "ros1":
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print(f"检测到 ROS 版本: ROS 1 (Noetic)")
        print("=" * 60)
    elif ROS_AVAILABLE == "ros2":
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print(f"检测到 ROS 版本: ROS 2 (支持 .db3/.mcap 格式)")
        print("=" * 60)
    else:
        print("=" * 60)
        print("ROS Bag 格式转换工具 (改进版 + ROS 2 支持)")
        print("错误: 未检测到 ROS 1 或 ROS 2 的 bag 库")
        print("  - ROS 1: 请安装 ros-noetic-rosbag")
        print("  - ROS 2: 请安装 rosbag2-py")
        print("=" * 60)
        exit(1)

    # 调用 main 函数处理命令行参数
    main()
