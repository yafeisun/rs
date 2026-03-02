#!/bin/bash
# ROS Bag 格式转换工具 - 运行脚本（本地运行版本）

# 设置变量
DATA_DIR="/home/geely/Documents/sunyafei/0203select"
OUTPUT_DIR="/home/geely/Documents/sunyafei/0203select_output"
CODE_DIR="/home/geely/Documents/sunyafei/rs/format_reshape"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

# 进入代码目录
cd "$CODE_DIR"

# 运行转换（本地环境）
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '$DATA_DIR', '--target-root', '$OUTPUT_DIR']; main()"

echo ""
echo "Done! Output directory: $OUTPUT_DIR"
echo "注意: 本地运行需要以下依赖:"
echo "  - cv2 (已安装: $(python3 -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo '未安装'))"
echo "  - numpy (已安装: $(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo '未安装'))"
echo "  - pyyaml (已安装: $(python3 -c 'import yaml; print(yaml.__version__)' 2>/dev/null || echo '未安装'))"
