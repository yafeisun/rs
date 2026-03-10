# ROS Bag 格式转换工具

## 功能概述

批量转换 ROS Bag 数据格式，自动从数据根目录递归查找符合时间戳格式 (YYYY-MM-DD-HH-MM-SS) 且包含 .bag 文件的目录进行处理。

待转换目标格式，参考数据格式路径：`/home/geely/Documents/sunyafei/PLEX1H8791_event_lfdi_console_manual_event_20260205-165619_0`
测试数据路径：`/home/geely/Documents/sunyafei/0203select`
测试数据的目录文件说明：`doc/robosense.md`

2个测试数据都是向前运动，对pcd和pose坐标系的判断应该与目标格式保持一致
下游重点使用了目标格式里的 `sensor_data/lidar/lidar_undist`，pose用的是 `/sensor_data/egopose_opt/egopose_optpose`

---

## 核心转换逻辑

### node_output/peral-dataproc/result.json 的生成逻辑

数据来源：
1. `result/test_calibration/sync_sensors.txt`
2. `sensor_data/lidar/lidar_undist/*.pcd` 的所有文件，按时间戳编号排序

目标格式：`node_output/peral-dataproc/result.json`

生成逻辑：
1. 全量拷贝数据来源 1 里的内容
2. 按时间戳编号排序后，与数据来源 2 里的文件一一对应
3. 生成 `node_output/peral-dataproc/result.json`

### 去畸变后的拼接点云

下游使用 `sensor_data/lidar/lidar_undist`
- 数据来源：`result/test_calibration/middle/*.pcd`，自然编号
- 目标格式：`sensor_data/lidar/lidar_undist/*.pcd`，middle雷达时间戳编号
- 映射关系：`result/test_calibration/sync_sensors.txt`

### 主雷达位姿

`sensor_data/egopose_opt/egopose_optpose/*.json`
- 数据来源：`result/test_calibration/middle/sync_sensors.json`，列1 自然序号，列2 cam_front_right相机时间戳，列3+ 雷达位姿
- 目标格式：`sensor_data/egopose_opt/egopose_optpose/*.json`，middle雷达时间戳编号
- 映射关系：`result/test_calibration/sync_sensors.txt` 雷达和相机的时间戳编号对应关系

---

## 使用方法

### 本地运行

```bash
# 基本用法
cd /home/geely/Documents/sunyafei/rs/format_reshape
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/path/to/data_root']; main()"

# 指定目标根目录
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '/path/to/data_root', '--target-root', '/path/to/output']; main()"

# 查看帮助
python3 -c "import sys; sys.path.insert(0, '.'); from src.main import main; sys.argv = ['main.py', '-h']; main()"
```

### Docker 运行 (推荐用于 ROS 1 数据)

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape
bash scripts/run.sh
```

**注意**: 首次运行会自动构建 Docker 镜像，包含以下依赖：
- ROS Noetic (ROS 1, Python 3.8)
- rosbag
- PyYAML
- numpy
- opencv-python

---

## 输出功能

| 功能 | 输出路径 | 说明 |
|------|---------|------|
| 相机图像拷贝 | `sensor_data/camera/` | 11路相机图像 |
| 点云数据拷贝 | `sensor_data/lidar/` | 5路 lidar 点云 |
| 拼接点云拷贝 | `sensor_data/lidar/lidar_undist/` | 拼接点云 |
| 标定参数转换 | `calibration/camera/` | 相机标定 YAML |
| 标定参数转换 | `calibration/lidar/` | Lidar 标定 YAML |
| 标定参数转换 | `calibration/virtual_camera/` | 虚拟相机标定（去畸变） |
| calib_anno 生成 | `calib_anno/` | 相机标定 JSON (原始) |
| calib_anno_vc 生成 | `calib_anno_vc/` | 相机标定 JSON (去畸变) |
| Pose 在线 | `sensor_data/egopose.json` | 在线定位结果 |
| Pose 离线 | `sensor_data/egopose_opt.json` | 离线优化结果 |
| IMU 数据 | `sensor_data/imu.json` | IMU 传感器数据 |
| GNSS 数据 | `sensor_data/gnss_rtk.json` | GNSS 传感器数据 |
| 轮速数据 | `sensor_data/wheel_report.json` | 轮速传感器数据 |
| 时间戳映射 | `node_output/peral-dataproc/result.json` | 点云→相机映射 |
| 时间戳映射 | `node_output/peral-dataproc/result_full11v.json` | 点云→相机+result映射 |
| 相机位姿 | `sensor_data/egopose_opt/camera_*/` | 各相机位姿数据 |
| 主雷达位姿 | `sensor_data/egopose_opt/egopose_optpose/` | 主雷达位姿数据 |
| IPM 鸟瞰图 | `node_output/ipm/` | 4路鱼眼拼接的BEV图像 |
| 可视化验证 | `visualize/` | 点云投影验证图像（自动生成） |

---

## 可视化验证功能

### 自动生成

转换完成后，会自动调用 `scripts/project_calib.py` 生成可视化验证图像：

```bash
visualize/
├── <cam_front_right_timestamp>/    # 使用相机时间戳
│   ├── camera_front_far.jpeg
│   ├── camera_front_fisheye.jpeg
│   ├── ... (11路相机的投影图)
```

**特点**：
- 文件夹命名使用 cam_front_right 的时间戳（而非 PCD 时间戳）
- 默认生成所有帧（不限制帧数）
- 点云投影到相机图像，使用伪彩色表示深度
- 可用于验证外参、内参和数据映射关系的正确性

---

## IPM 鸟瞰图生成

### 功能说明

将4路鱼眼相机（前/后/左/右）拼接为鸟瞰图（Bird's Eye View），用于环视感知和可视化。

### 生成方式

```bash
# 为单个片段生成IPM
python3 src/ipm_generator.py /path/to/output/605/2025-11-21-16-15-45

# 限制生成帧数（用于测试）
python3 src/ipm_generator.py /path/to/output/605/2025-11-21-16-15-45 --max-frames 10
```

### 输出格式

```
node_output/ipm/
├── ipm_1763712945233000000.jpeg    # 使用前鱼眼时间戳命名
├── ipm_1763712945333000000.jpeg
└── ...
```

### 技术细节

- **投影策略**：全投影模式，所有4路鱼眼都投影到整个BEV区域，后投影覆盖先投影
- **投影顺序**：前鱼眼 → 后鱼眼 → 左鱼眼 → 右鱼眼
- **去畸变**：使用 `cv2.fisheye.initUndistortRectifyMap` 处理鱼眼畸变
- **BEV范围**：X[-20m, 30m], Y[-10m, 10m]，分辨率0.05m/pixel
- **坐标系**：FLU（X前，Y左，Z上），原点在车辆后轴中心
- **文件命名**：使用前鱼眼（camera_front_fisheye）的时间戳

### 参考实现

完全参考速腾IPM的SOTA实现方式，确保拼接效果无黑色空洞。

---

## 项目结构

```
format_reshape/
├── src/                          # 源代码目录
│   ├── __init__.py
│   ├── main.py                   # 主入口点
│   ├── bag_reader.py             # ROS Bag 读取器
│   ├── extractors.py             # 数据提取（相机、点云、标定）
│   ├── converters.py             # 数据转换（pose、IMU、GNSS、轮速）
│   ├── calibration.py           # 标定参数处理
│   ├── pose.py                  # Pose 数据处理
│   ├── ipm_generator.py          # IPM 鸟瞰图生成
│   └── utils.py                 # 工具函数
├── scripts/
│   ├── project_calib.py        # 可视化投影验证脚本
│   └── run.sh                   # Docker 运行脚本
├── Docker/
│   └── Dockerfile               # Docker 镜像配置
├── docs/
│   ├── README.md                # 本文档
│   ├── visualize.md             # 可视化功能说明
│   ├── format_mapping.md        # 格式映射文档
│   ├── bev4d.md                 # BEV4D 数据格式说明
│   └── robosense.md             # Robosense 数据格式说明
└── tests/                       # 测试目录
```

---

## 依赖

### Python 依赖

- `rosbag` (ROS Python bindings)
- `PyYAML`
- `numpy`
- `opencv-python`

### 安装

```bash
# ROS 1 (Ubuntu 20.04)
sudo apt install ros-noetic-rosbag python3-yaml python3-numpy python3-opencv
pip3 install opencv-python

# ROS 2 (Ubuntu 22.04+)
sudo apt install ros-humble-rosbag2-storage python3-yaml python3-numpy
pip3 install opencv-python
```

### Docker 构建

Docker 镜像包含所有必需的依赖，无需手动安装。

---

## 输出示例

```
Data root: /path/to/data_root
Target root: /path/to/output
Found 2 data directories

[1/2] /path/to/data_root/557/2025-11-28-14-21-42
  [Camera] Done: 2247 images
  [LiDAR] Done: 1022 files
  [LiDAR Concat] Done: 197 files
  [Calibration] Done: 11 cameras, 5 lidars
  [CalibAnno] Done: 11 cameras
  [CalibAnnoVC] Done: 11 virtual cameras
  [NodeOutput] Done: 197 frames mapped
  [CameraPoses] Done: 11 cameras, 2167 poses
  [LidarMainPose] Done: 197 poses
  [Pose Online] Done
  [Pose Offline] Done
  [IMU] Done: 2096 records
  [GNSS] Done: 2097 records
  [Wheel] Done: 1010 records
  [Visualization] Running projection verification...
                 Done: Visualization generated in /path/to/output/557/2025-11-28-14-21-42/visualize/

Done! Success: 2, Failed: 0
```

---

## 验证结果

### 验证方法

转换完成后，应验证以下时间戳对应关系是否正确：

1. `sensor_data/egopose_opt/egopose_optpose/*.json` (雷达位姿)
2. `sensor_data/lidar/lidar_undist/*.pcd` (点云 PCD)
3. `node_output/peral-dataproc/result.json` (映射表)

这3个文件的时间戳编号应该是一致的，数据也应该是一一对应的。

详细验证方法请参考文档中的验证章节。

---

## 常见问题

### Q: 为什么主雷达位姿的 JSON 文件名是 PCD 时间戳而不是相机时间戳？

A: 因为 sync_sensors.txt 中的时间戳是相机时间戳，但为了与点云数据对齐，需要使用对应的 PCD 时间戳来命名文件。这样可以确保：
- 雷达位姿与点云数据的时间戳一致
- 便于后续数据处理和同步

### Q: visualize 文件夹的时间戳为什么用相机时间戳？

A: visualize 文件夹使用 cam_front_right 的相机时间戳命名，这样便于与相机图像对齐查看。但内部的点云投影使用的是对应的 PCD 点云数据。

### Q: 如何验证时间戳是否正确？

A: 检查以下对应关系：
1. `result.json` 中的 PCD 时间戳与相机时间戳是否正确映射
2. `egopose_optpose/` 中的 JSON 文件名是否使用 PCD 时间戳
3. JSON 文件内容中的 `meta.timestamp_us` 是否与原始相机时间戳一致
4. `visualize/` 中的文件夹名是否是相机时间戳

### Q: Docker 镜像需要多久构建？

A: 首次构建 Docker 镜像需要较长时间（5-10分钟），因为需要下载 ROS Noetic 基础镜像并安装依赖。后续运行会使用缓存的镜像，启动速度很快。

### Q: 可以限制可视化生成的帧数吗？

A: 可以。修改 `scripts/project_calib.py` 中的 `--max-frames` 参数，或者修改 `src/main.py` 中调用脚本时的参数设置。默认值为 -1，表示生成所有帧。
