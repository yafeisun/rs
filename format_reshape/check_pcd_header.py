#!/usr/bin/env python3
"""检查 pcd 文件头信息"""

import os
import glob

# 使用与reshape.py相同的配置
SRC_BAG_DIR = "/home/lenovo/Documents/0203select/557/2025-11-28-14-21-42"
pcd_base_dir = os.path.join(SRC_BAG_DIR, "raw/pcd")

lidar_dirs = ["middle", "left", "right", "front", "back"]

for lidar in lidar_dirs:
    pcd_dir = os.path.join(pcd_base_dir, lidar)
    if not os.path.exists(pcd_dir):
        continue

    pcd_files = glob.glob(os.path.join(pcd_dir, "*.pcd"))
    if not pcd_files:
        continue

    first_file = pcd_files[0]
    print(f"\n{'='*60}")
    print(f"{lidar}: {os.path.basename(first_file)}")
    print('='*60)

    with open(first_file, 'rb') as f:
        while True:
            line = f.readline().decode('ascii').strip()
            print(line)
            if line.startswith('DATA'):
                break