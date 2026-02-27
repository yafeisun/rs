#!/bin/bash
# ROS Bag 格式转换工具 - 运行脚本

# 设置变量
DATA_DIR="/home/geely/Documents/sunyafei/0203select"
OUTPUT_DIR="/home/geely/Documents/sunyafei/0203select_output"
IMAGE_NAME="rs-reshape:noetic"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

# 运行 Docker 容器
docker run --rm \
  --user $(id -u):$(id -g) \
  -v "$DATA_DIR:/data" \
  -v "$OUTPUT_DIR:/output" \
  "$IMAGE_NAME" \
  python3 reshape.py /data --target-root /output

echo ""
echo "Done! Output directory: $OUTPUT_DIR"
echo "All files have correct permissions (no root lock)"