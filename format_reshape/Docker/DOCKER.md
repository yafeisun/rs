# 使用 Docker 运行 ROS Bag 格式转换工具

## 为什么需要 Docker？

由于你的数据是 ROS 1 格式的 .bag 文件，而 Ubuntu 24.04 只能安装 ROS 2，无法直接处理 ROS 1 的 bag 文件。使用 Docker 可以在容器中运行 ROS Noetic (ROS 1) 来处理这些数据。

## 使用方法

### 方法 1: 使用便捷脚本（推荐）

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape
bash run.sh
```

### 方法 2: 直接使用 docker run

```bash
docker run --rm \
  --user $(id -u):$(id -g) \
  -v /home/geely/Documents/sunyafei/0203select:/data \
  -v /home/geely/Documents/sunyafei/0203select_output:/output \
  rs-reshape:noetic \
  python3 reshape.py /data --target-root /output
```

### 方法 3: 使用 docker-compose

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape
docker-compose up
```

## 参数说明

- `--user $(id -u):$(id -g)`: 使用当前用户的 UID/GID，确保生成的文件权限正确
- `/data`: 源数据目录（包含 .bag 文件的目录）
- `/output`: 输出目录
- `--target-root`: 可选，指定输出根目录

## 修改配置

如果需要修改数据路径，请编辑以下文件：
- `run.sh`: 修改 `DATA_DIR` 和 `OUTPUT_DIR` 变量
- `docker-compose.yml`: 修改 `volumes` 部分

## 权限说明

- Docker 镜像配置了非 root 用户（UID 1000）
- 生成的文件和文件夹权限为 `geely:geely`，不会有 root 锁
- 如需修改权限，可使用：`sudo chown -R $(id -u):$(id -g) /path/to/output`

## 注意事项

- 确保 Docker 已安装并正在运行
- 首次运行会下载 ROS Noetic 镜像（约 2-3 GB），可能需要一些时间
- 输出目录会自动创建，无需预先创建