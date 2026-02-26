
# 使用项目虚拟环境
PROJECT_ROOT=$(dirname $(dirname $(realpath $0)))
$PROJECT_ROOT/.venv/bin/python merge_pcd.py --InputPath $@