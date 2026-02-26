#!/usr/bin/env python3
"""检查 pcd 文件的点数"""

import os
import glob

# 使用与reshape.py相同的配置
SRC_BAG_DIR = "/home/lenovo/Documents/0203select/557/2025-11-28-14-21-42"
pcd_base_dir = os.path.join(SRC_BAG_DIR, "raw/pcd")

lidar_dirs = ["middle", "left", "right", "front", "back"]

for lidar in lidar_dirs:
    pcd_dir = os.path.join(pcd_base_dir, lidar)
    if not os.path.exists(pcd_dir):
        print(f"{lidar}: 目录不存在")
        continue

    pcd_files = glob.glob(os.path.join(pcd_dir, "*.pcd"))
    if not pcd_files:
        print(f"{lidar}: 没有 pcd 文件")
        continue

    # 读取第一个 pcd 文件统计点数
    first_file = pcd_files[0]
    point_count = 0
    with open(first_file, 'rb') as f:
        # 读取二进制 pcd 文件头
        header_lines = []
        while True:
            line = f.readline().decode('ascii').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        # 检查是否是二进制格式
        is_binary = any('binary' in line for line in header_lines)

        if is_binary:
            # 读取剩余数据
            data = f.read()
            # 根据字段数估算点数（假设每个点有 x,y,z 三个 float32 = 12字节）
            # 实际字段数需要从 header 中的 FIELDS 行获取
            fields_line = [line for line in header_lines if line.startswith('FIELDS')]
            if fields_line:
                num_fields = len(fields_line[0].split()) - 1  # 减去 "FIELDS"
                point_count = len(data) // (4 * num_fields)
            else:
                point_count = len(data) // 12  # 默认 3 个 float32
        else:
            # ASCII 格式，读取剩余行数
            lines = f.read().decode('ascii').strip().split('\n')
            point_count = len(lines)

    print(f"{lidar}: {point_count} 点 (文件: {os.path.basename(first_file)})")