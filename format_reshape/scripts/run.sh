#!/bin/bash
# ROS Bag 格式转换工具 - 运行脚本

# 设置变量
DATA_DIR="/home/geely/Documents/sunyafei/0203select"
OUTPUT_DIR="/home/geely/Documents/sunyafei/0203select_output"
CODE_DIR="/home/geely/Documents/sunyafei/rs/format_reshape"
IMAGE_NAME="rs-reshape:noetic"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

# 运行 Docker 容器
docker run --rm \
  --user $(id -u):$(id -g) \
  -v "$DATA_DIR:/data" \
  -v "$OUTPUT_DIR:/output" \
  -v "$CODE_DIR:/workspace" \
  -w /workspace \
  "$IMAGE_NAME" \
  python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/data', '--target-root', '/output']; main()"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "✅ 数据转换完成!"
echo "═══════════════════════════════════════════════════════════════"
echo "输出目录: $OUTPUT_DIR"
echo ""
echo "生成的可视化内容:"
echo "  - 轨迹图: */visualize/trajectory_visualization.png"
echo "  - 投影图: */visualize/projection_viz_*.png (10帧)"
echo ""
echo "所有文件权限正确 (无 root 锁定)"
echo "═══════════════════════════════════════════════════════════════"
