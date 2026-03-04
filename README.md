# RS - 无人驾驶传感器数据处理仓库

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

这是一个多项目仓库，用于无人驾驶传感器数据的处理和格式转换，支持从 Robosense 原始数据到 BEV4D 标准格式的批量转换。

---

## 📁 项目结构

```
rs/
├── format_reshape/     # ROS Bag 格式转换工具 (Robosense → BEV4D)
├── ipm/                # IPM (逆透视映射) 和斑马线检测
├── pipeline/           # 传感器数据处理流水线 (EA-LSS 相关)
├── script/             # 实用工具脚本
└── CLAUDE.md           # 详细的开发指南
```

---

## 🚀 主要功能

### 1. format_reshape - ROS Bag 格式转换

**核心功能：** 将 Robosense ROS Bag 数据批量转换为 BEV4D 格式，用于下游的 3D 目标检测任务。

**主要特性：**
- ✅ 批量转换相机图像（11 路相机）
- ✅ 批量转换激光雷达点云（5 个激光雷达 + 拼接点云）
- ✅ 标定参数转换（相机、激光雷达、虚拟相机）
- ✅ 位姿数据转换（在线/离线优化）
- ✅ 传感器数据提取（IMU/GNSS/轮速）
- ✅ 自动投影可视化验证

**坐标系转换：**
- 源坐标系：RFU（Y=前，X=右，Z=上）
- 目标坐标系：FLU（X=前，Y=左，Z=上）
- 转换矩阵：`[[0,1,0],[-1,0,0],[0,0,1]]`

**运行方式：**

```bash
# Docker 运行（推荐，用于 ROS 1 数据）
cd format_reshape
bash scripts/run.sh

# 本地运行
cd format_reshape
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/path/to/data_root']; main()"

# 指定输出目录
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/path/to/data_root', '--target-root', '/path/to/output']; main()"
```

**文档：**
- [格式转换说明](format_reshape/docs/README.md) - 核心转换逻辑、使用方法、输出功能
- [格式映射文档](format_reshape/docs/format_mapping.md) - Robosense → BEV4D 完整映射关系
- [源数据格式](format_reshape/docs/robosense.md) - Robosense 数据格式说明
- [可视化验证](format_reshape/docs/visualize.md) - 投影验证和坐标系详解

**输出目录结构：**

```
<output_root>/
├── sensor_data/
│   ├── camera/camera_*/                    # 11路相机图像
│   ├── lidar/lidar_*/                      # 5路激光雷达点云
│   ├── lidar/lidar_concat/                 # 拼接点云
│   ├── egopose_opt/camera_*/               # 相机位姿
│   ├── egopose_opt/egopose_optpose/        # 主雷达位姿
│   ├── imu.json                            # IMU 数据
│   ├── gnss_rtk.json                       # GNSS 数据
│   └── wheel_report.json                   # 轮速数据
├── calib_anno/                             # 相机标定 JSON
├── calib_anno_vc/                          # 虚拟相机标定（去畸变）
├── calibration/
│   ├── camera/                             # 相机标定 YAML
│   ├── lidar/                              # 激光雷达标定 YAML
│   └── virtual_camera/                     # 虚拟相机标定
└── node_output/peral-dataproc/
    └── result.json                         # 点云-相机映射表
```

---

### 2. ipm - 逆透视映射与斑马线检测

**核心功能：** 从相机图像生成鸟瞰图（BEV）并检测斑马线。

**主要特性：**
- ✅ IPM 变换生成 BEV 图像
- ✅ 斑马线检测
- ✅ 图像时间对齐
- ✅ 支持 ROS1 和 ROS2

**运行方式：**

```bash
cd ipm
python3 -m src.cli --help

# 生成 BEV 图像
python3 -m src.cli <data_directory> --car-yaml <path_to_car.yaml>
```

**文档：**
- [安装说明](ipm/static_zebra_checker/INSTALL.md) - 安装和配置指南

**关键模块：**
- `cli.py` - 命令行接口
- `proj2bev_.py` - IPM 变换逻辑
- `find_images.py` - 图像时间对齐
- `bev_plot.py` - BEV 可视化

---

### 3. pipeline - 传感器数据处理流水线

**核心功能：** EA-LSS 相关的传感器同步和数据处理流水线。

**主要特性：**
- ✅ 传感器同步数据处理
- ✅ EA-LSS 算法集成
- ✅ 点云合并处理
- ✅ 真值数据处理

**运行方式：**

```bash
cd pipeline
python3 rs_sdk_ctrl_main.py
```

**文档：**
- [Docker 安装指南](pipeline/readme.md) - 包含 GPU 支持的 Docker 环境配置

**关键组件：**
- `sensor_sync_data/` - 传感器同步模块
- `EA-LSS/` - EA-LSS 相关代码
- `merge_pcd/` - 点云合并
- `true_value/` - 真值处理

---

### 4. script - 实用工具脚本

**功能：** 各种数据处理的实用脚本集合。

**主要脚本：**
- `bag_location_checking.py` - Bag 文件位置检查
- `copy_od_frame.py` - 目标检测帧拷贝
- `find_bag_by_tag.py` - 按标签搜索 Bag 文件
- `rs_sdk_clip_stationary.py` - 静止场景剪裁
- `filter_pld_od_clip.py` - 数据过滤和剪裁
- `detect_human.py` / `detect_face.py` - 检测工具

---

## 📚 文档架构

```
rs/
├── CLAUDE.md                           # 仓库总览和开发指南
│
├── format_reshape/                     # 格式转换项目
│   ├── docs/
│   │   ├── README.md                   # 主要功能说明（中文）
│   │   ├── format_mapping.md          # Robosense→BEV4D 映射详解
│   │   ├── robosense.md               # Robosense 数据格式说明
│   │   ├── visualize.md               # 可视化验证指南
│   │   └── bev4d.md                   # BEV4D 格式规范
│   └── bev4d/
│       └── bev4d.md                   # BEV4D 目标格式规范
│
├── ipm/                                # IPM 和斑马线检测
│   └── static_zebra_checker/
│       └── INSTALL.md                 # 安装和使用说明
│
└── pipeline/                           # 传感器处理流水线
    └── readme.md                      # Docker 环境配置
```

### 详细文档说明

| 文档路径 | 说明 |
|---------|------|
| `CLAUDE.md` | 包含所有项目的概览、运行指令、架构设计、约定和详细文档链接 |
| `format_reshape/docs/README.md` | 格式转换工具的核心文档，详细说明了转换逻辑、使用方法和输出功能 |
| `format_reshape/docs/format_mapping.md` | 完整的 Robosense 到 BEV4D 格式映射规范，涵盖相机、激光雷达、标定、位姿等所有数据 |
| `format_reshape/docs/robosense.md` | Robosense 源数据的目录结构、文件命名规则和传感器分布 |
| `format_reshape/docs/visualize.md` | 点云投影到相机图像的可视化验证方法，包含坐标系转换说明 |
| `format_reshape/bev4d/bev4d.md` | BEV4D 目标数据格式的详细规范 |
| `ipm/static_zebra_checker/INSTALL.md` | IPM 和斑马线检测的安装、配置和使用指南 |
| `pipeline/readme.md` | Docker 环境配置，特别是 NVIDIA GPU 支持的安装步骤 |

---

## 🔧 环境要求

### format_reshape

- **Python**: 3.8+ (ROS Noetic) 或 3.12+
- **ROS 1**: Noetic（用于 rosbag 读取）
- **依赖包**:
  - rosbag
  - PyYAML
  - numpy
  - opencv-python

**安装方式：**

```bash
# ROS 1 (Ubuntu 20.04)
sudo apt install ros-noetic-rosbag python3-yaml python3-numpy

# ROS 2 (Ubuntu 22.04+)
sudo apt install ros-humble-rosbag2-storage python3-yaml python3-numpy

# 安装 opencv（可选，用于 Docker 内的投影验证）
pip3 install opencv-python
```

### ipm

- **Python**: 3.12+
- **依赖包**:
  - rosbags>=0.9.20
  - opencv-python>=4.8.0
  - numpy>=1.24.0
  - scipy>=1.10.0
  - PyYAML>=6.0
  - click>=8.0.0

**安装方式：**

```bash
cd ipm
pip3 install -r requirements.txt
```

### pipeline

- **Python**: 3.12+
- **依赖包**:
  - av
  - numpy
  - opencv-python
  - open3d
  - pycryptodome
  - python-gnupg
  - pyyaml
  - tqdm

**安装方式：**

```bash
cd pipeline
pip3 install -r requirements.txt
```

---

## 🧪 测试

### format_reshape 测试

```bash
cd format_reshape

# 运行所有测试
python3 -m pytest tests/ -v

# 运行特定测试
python3 tests/test_lidar_transformation.py
python3 tests/test_full_transformation.py
```

---

## 🐳 Docker 使用

### 构建 format_reshape 镜像

```bash
cd format_reshape
docker build -t rs-reshape:noetic -f Docker/Dockerfile .
```

### 运行 Docker

```bash
cd format_reshape
bash scripts/run.sh
```

Docker 镜像包含以下内容：
- ROS Noetic (ROS 1, Python 3.8)
- rosbag
- PyYAML、numpy、opencv-python

首次构建需要 5-10 分钟，后续运行会使用缓存镜像。

---

## 📊 快速开始

### 1. 格式转换 Robosense 数据到 BEV4D

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape

# 使用 Docker 运行（推荐）
bash scripts/run.sh

# 或本地运行
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/path/to/data_root']; main()"

# 转换完成后自动生成可视化验证
# 查看 visualize/ 目录中的投影图像
```

### 2. 生成 BEV 图像

```bash
cd /home/geely/Documents/sunyafei/rs/ipm
python3 -m src.cli <data_directory> --car-yaml <path_to/car.yaml>
```

### 3. 运行传感器数据处理流水线

```bash
cd /home/geely/Documents/sunyafei/rs/pipeline
python3 rs_sdk_ctrl_main.py
```

---

## 🔍 数据验证

转换完成后应验证以下时间戳对应关系是否正确：

1. **激光雷达位姿** - `sensor_data/egopose_opt/egopose_optpose/*.json`
2. **点云数据** - `sensor_data/lidar/lidar_concat/*.pcd`
3. **映射表** - `node_output/peral-dataproc/result.json`

这三个文件的时间戳编号应该是一致的，数据也应该是一一对应的。

详细验证方法请参考 [可视化验证文档](format_reshape/docs/visualize.md)。

---

## 💡 常见问题

### Q: 为什么主雷达位姿的 JSON 文件名是 PCD 时间戳而不是相机时间戳？

A: 因为 sync_sensors.txt 中的时间戳是相机时间戳，但为了与点云数据对齐，需要使用对应的 PCD 时间戳来命名文件。这样可以确保雷达位姿与点云数据的时间戳一致，便于后续数据处理和同步。

### Q: visualize 文件夹的时间戳为什么用相机时间戳？

A: visualize 文件夹使用 cam_front_right 的相机时间戳命名，这样便于与相机图像对齐查看。但内部的点云投影使用的是对应的 PCD 点云数据。

### Q: 如何验证时间戳是否正确？

A: 检查以下对应关系：
1. `result.json` 中的 PCD 时间戳与相机时间戳是否正确映射
2. `egopose_optpose/` 中的 JSON 文件名是否使用 PCD 时间戳
3. JSON 文件内容中的 `meta.timestamp_us` 是否与原始相机时间戳一致
4. `visualize/` 中的文件夹名是否是相机时间戳

### Q: Docker 镜像需要多久构建？

A: 首次构建需要较长时间（5-10 分钟），因为需要下载 ROS Noetic 基础镜像并安装依赖。后续运行会使用缓存的镜像，启动速度很快。

---

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request。

在提交代码之前，请确保：
1. 代码符合现有项目的风格和约定
2. 添加适当的测试用例
3. 更新相关文档

---

## 📜 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 📮 联系方式

如有问题或建议，请通过以下方式联系：
- 提交 GitHub Issue
- 发送邮件到维护者

---

## 🔗 相关链接

- [CLAUDE.md](CLAUDE.md) - 完整的开发指南和文档
- [format_reshape/docs/README.md](format_reshape/docs/README.md) - 格式转换详细文档
- [format_reshape/docs/format_mapping.md](format_reshape/docs/format_mapping.md) - 格式映射规范
