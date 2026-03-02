#!/bin/bash
# 一键完整部署脚本 - 重建Docker + 运行完整转换 + 生成可视化

set -e  # 遇到错误立即退出

echo "========================================"
echo "  ROS Bag 格式转换 - 完整部署流程"
echo "========================================"
echo ""

# 设置变量
DATA_DIR="/home/geely/Documents/sunyafei/0203select"
OUTPUT_DIR="/home/geely/Documents/sunyafei/0203select_output"
CODE_DIR="/home/geely/Documents/sunyafei/rs/format_reshape"
DOCKER_DIR="${CODE_DIR}/Docker"
IMAGE_NAME="rs-reshape:noetic"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

echo "步骤 1: 清理旧数据"
echo "----------------------------------------"
rm -rf "$OUTPUT_DIR"/*
echo "✓ 旧数据已清理"
echo ""

echo "步骤 2: 重建 Docker 镜像"
echo "----------------------------------------"
echo "这可能需要 5-10 分钟，请耐心等待..."
echo ""

cd "$DOCKER_DIR"
docker build -t "$IMAGE_NAME" -f Dockerfile .

echo ""
echo "✓ Docker 镜像构建完成"
echo ""

echo "步骤 3: 运行完整转换流程"
echo "----------------------------------------"
cd "$CODE_DIR"

docker run --rm \
  --user $(id -u):$(id -g) \
  -v "$DATA_DIR:/data" \
  -v "$OUTPUT_DIR:/output" \
  -v "$CODE_DIR:/workspace" \
  -w /workspace \
  "$IMAGE_NAME" \
  python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/data', '--target-root', '/output']; main()"

echo ""
echo "========================================"
echo "  ✅ 转换完成！"
echo "========================================"
echo ""
echo "输出目录: $OUTPUT_DIR"
echo ""
echo "检查可视化文件夹:"
ls -d "$OUTPUT_DIR"/*/visualize 2>/dev/null || echo "（未自动生成可视化文件夹）"
echo ""
echo "详细输出:"
ls -la "$OUTPUT_DIR"/*/ 2>/dev/null | head -5
