# ROS Bag 格式转换工具

## 功能概述

批量转换 ROS Bag 数据格式，自动从数据根目录递归查找符合时间戳格式 (YYYY-MM-DD-HH-MM-SS) 且包含 .bag 文件的目录进行处理。

## 输出功能

| 功能 | 输出路径 | 说明 |
|------|---------|------|
| 相机图像拷贝 | `sensor_data/camera/` | 11路相机图像 |
| 点云数据拷贝 | `sensor_data/lidar/` | 5路 lidar 点云 |
| 拼接点云拷贝 | `sensor_data/lidar/lidar_concat/` | 拼接点云 |
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

---

## 使用方法

```bash
# 基本用法
python3 reshape.py /path/to/data_root

# 指定目标根目录
python3 reshape.py /path/to/data_root --target-root /path/to/output

# 查看帮助
python3 reshape.py -h
```

---

## 数据关系详解

### 1. PCD 与相机的时间戳映射

**文件位置**: `node_output/peral-dataproc/result.json`

**数据结构**:
```json
{
  "1764310902799625984.pcd": {
    "camera_front_far": "1764310902895000000.jpeg",
    "camera_front_fisheye": "1764310902899000000.jpeg",
    ...
  }
}
```

**关键关系**:
- **PCD 时间戳**: `1764310902799625984` (纳秒级，19位)
- **相机时间戳**: `1764310902895000000` (纳秒级，19位)
- **对应关系**: 每个 PCD 时间戳对应多个相机时间戳

### 2. PCD-相机时间戳确定关系

**源文件**: `result/test_calibration/sync_sensors.txt`

**格式**:
```
#frame /middle /cam_front_right /cam_back /cam_side_left_front /cam_side_right_front /cam_side_left_back /cam_side_right_back /cam_front_left /cam_around_front /cam_around_left /cam_around_right /cam_around_back
1 1764310903499619840 1764310903495000000 1764310903493000000 1764310903493000000 1764310903493000000 1764310903493000000 1764310903493000000 1764310903495000000 1764310903499000000 1764310903499000000 1764310903499000000 1764310903499000000
```

**关键关系**:
- **第一列**: PCD 时间戳 (如 `1764310903499619840`)
- **第2列及之后**: 各相机时间戳 (如 `1764310903495000000`)
- **对应关系**: 每一行是确定的 PCD → 相机时间戳映射，不需要最近邻匹配

**用途**:
- 用于生成 `node_output/peral-dataproc/result.json`
- 用于主雷达位姿输出（通过 PCD 时间戳命名文件）

### 3. 主雷达位姿的时间戳映射

**源文件**: `result/test_calibration/middle/sync_sensors.txt`

**格式**:
```
#index timestamp x y z q_x q_y q_z q_w
1 1764310903495000000.jpeg -0.226895 1.130095 -1.998865 0.008977 0.008787 0.753109 0.657776
```

**关键关系**:
- **sync_sensors.txt 中的时间戳**: 相机时间戳（如 `1764310903495000000.jpeg`）
- **对应的 PCD 时间戳**: 通过 `result.json` 查找（如 `1764310903499619840`）
- **输出文件命名**: 使用 **PCD 时间戳** 命名（`1764310903499619840.json`）

**示例**:
```
sync_sensors.txt: 1764310903495000000.jpeg (相机时间戳)
          ↓ 查找 result.json
result.json: PCD 1764310903499619840.pcd → camera_front_far 1764310903495000000.jpeg
          ↓ 输出
egopose_optpose/1764310903499619840.json (使用 PCD 时间戳)
```

### 4. 相机位姿的时间戳映射

**源文件**: `result/test_calibration/cam_xxx/sync_sensors.txt`

**格式**: 与主雷达相同

**关键关系**:
- **sync_sensors.txt 中的时间戳**: 相机时间戳
- **输出文件命名**: 直接使用 **相机时间戳** 命名

**示例**:
```
sync_sensors.txt: 1764310903495000000.jpeg (相机时间戳)
          ↓ 输出
camera_front_right/1764310903495000000.json (使用相机时间戳)
```

### 4. 时间戳映射关系总结

| 数据类型 | 源时间戳类型 | 输出文件命名 | 说明 |
|---------|------------|------------|------|
| PCD 点云 | PCD 时间戳 | PCD 时间戳 | 直接使用 |
| 相机图像 | 相机时间戳 | 相机时间戳 | 直接使用 |
| 相机位姿 | 相机时间戳 | 相机时间戳 | 直接使用 |
| 主雷达位姿 | 相机时间戳 | **PCD 时间戳** | 需要通过 result.json 映射 |

### 5. 重要时间戳差异

**示例数据**:
```
PCD 时间戳: 1764310903499619840
相机时间戳: 1764310903495000000
差异: 4619840 纳秒 ≈ 4.6 毫秒
```

**原因**: 
- PCD 和相机采集时间不完全同步
- 需要建立映射关系来关联两者的数据

---

## 输出示例

```
Data root: /home/lenovo/Documents/0203select
Target root: /home/lenovo/Documents/0203select_output
Found 51 data directories

[1/51] /home/lenovo/Documents/0203select/557/2025-11-28-14-21-42
  [Camera] Src: /path/src/cam_*
           Dst: /path/dst/sensor_data/camera/
           Done: 2247 images
  [LiDAR] Src: /path/src/raw/pcd/*
          Dst: /path/dst/sensor_data/lidar/
          Done: 1022 files
  [LiDAR Concat] Src: /path/src/result/test_calibration/middle
                 Dst: /path/dst/sensor_data/lidar/lidar_concat
                 Done: 197 files
  [Calibration] Src: /path/src/car_xxx.yaml
                Dst: /path/dst/calibration/
                Done: 11 cameras, 5 lidars
  [CalibAnno] Src: /path/dst/calibration/camera
              Dst: /path/dst/calib_anno
              Done: 11 cameras
  [CalibAnnoVC] Src: /path/dst/calibration/virtual_camera
                Dst: /path/dst/calib_anno_vc
                Done: 11 virtual cameras
   [NodeOutput] Src: /path/src/result/test_calibration/sync_sensors.txt
                Dst: /path/dst/node_output/peral-dataproc/result.json, result_full11v.json
                Done: 197 frames mapped
  [CameraPoses] Src: /path/src/result/test_calibration/cam_*/sync_sensors.txt
               Dst: /path/dst/sensor_data/egopose_opt/camera_*/
               Done: 11 cameras, 637 poses
  [LidarMainPose] Src: /path/src/result/test_calibration/middle/sync_sensors.txt
                 Dst: /path/dst/sensor_data/egopose_opt/egopose_optpose/
                 Done: 197 poses (using PCD timestamps)
  [Pose Online] Src: /path/src/result/bev/BEV_pose.json
                Dst: /path/dst/sensor_data/egopose.json
  [Pose Offline] Src: /path/src/result/mapping/mapping_pose_quaterniond.txt
                 Dst: /path/dst/sensor_data/egopose_opt.json
  [IMU] Src: /path/src/xxx.bag (topic: /rs/imu)
        Dst: /path/dst/sensor_data/imu.json
        Done: 2096 records
  [GNSS] Src: /path/src/xxx.bag (topic: /rs/gps)
         Dst: /path/dst/sensor_data/gnss_rtk.json
         Done: 2097 records
  [Wheel] Src: /path/src/xxx.bag (topic: /ft_vehicle_data_v3)
          Dst: /path/dst/sensor_data/wheel_report.json
          Done: 1010 records

Done! Success: 51, Failed: 0
```

---

## calib_anno vs calib_anno_vc

| 字段 | calib_anno (原始) | calib_anno_vc (虚拟相机/去畸变) |
|------|-------------------|-------------------------------|
| extrinsic | 相同 | 相同 |
| intrinsic | `[fx, 0, cx, 0, fy, cy, 0, 0, 1]` | `[fx, 0, cx, 0, fx, cy, 0, 0, 1]` |
| distcoeff | `[kc2, kc3, kc4, kc5]` | `[0, 0, 0, 0]` |

**区别**:
- `intrinsic`: calib_anno_vc 中 `fy = fx`（去畸变后）
- `distcoeff`: calib_anno_vc 全为 0（已去畸变）

---

## 依赖

- `rosbag` (ROS Python bindings)
- `PyYAML`
- `numpy`

```bash
# ROS 1 (Ubuntu 20.04)
sudo apt install ros-noetic-rosbag python3-yaml python3-numpy

# ROS 2 (Ubuntu 22.04+)
sudo apt install ros-humble-rosbag2-storage python3-yaml python3-numpy
```

---

## Docker 支持

### 使用 Docker 运行 (推荐用于 ROS 1 数据)

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape

# 构建镜像
docker build -t rs-reshape:noetic .

# 运行
docker run --rm \
  --user $(id -u):$(id -g) \
  -v /home/geely/Documents/sunyafei/0203select:/data \
  -v /home/geely/Documents/sunyafei/0203select_output:/output \
  rs-reshape:noetic \
  python3 reshape.py /data --target-root /output
```

### 使用便捷脚本

```bash
cd /home/geely/Documents/sunyafei/rs/format_reshape
bash run.sh
```

---

## 文件结构

```
format_reshape/
├── reshape.py              # 主程序
├── Dockerfile              # Docker 配置
├── docker-compose.yml      # Docker Compose 配置
├── run.sh                  # 便捷运行脚本
├── check_pcd_header.py     # 检查PCD文件头
├── check_pcd_points.py     # 检查PCD点数
└── README.md               # 本文档
```

---

## 常见问题

### Q: 为什么主雷达位姿的 JSON 文件名是 PCD 时间戳而不是相机时间戳？

A: 因为 sync_sensors.txt 中的时间戳是相机时间戳，但为了与点云数据对齐，需要使用对应的 PCD 时间戳来命名文件。这样可以确保：
- 雷达位姿与点云数据的时间戳一致
- 便于后续数据处理和同步

### Q: 相机位姿为什么不用 PCD 时间戳？

A: 相机位姿是直接从 sync_sensors.txt 读取的，每个相机有独立的时间戳，不需要与 PCD 对齐。

### Q: 如何验证时间戳是否正确？

A: 检查以下对应关系：
1. `result.json` 中的 PCD 时间戳与相机时间戳是否正确映射
2. `egopose_optpose/` 中的 JSON 文件名是否使用 PCD 时间戳
3. JSON 文件内容中的 `meta.timestamp_us` 是否与原始相机时间戳一致

---

## 验证结果

### 验证方法

转换完成后，应验证以下时间戳对应关系是否正确：

### 1. 相机位姿与相机图像对应关系

**验证路径**:
- `sensor_data/egopose_opt/camera_*/` (相机位姿 JSON)
- `sensor_data/camera/camera_*/` (相机图像 JPEG)

**验证标准**:
- 所有相机位姿 JSON 文件的时间戳都能在对应相机文件夹中找到同名 JPEG
- 示例: `camera_front_far/1764310903495000000.json` → `camera_front_far/1764310903495000000.jpeg`

**预期结果**:
```
✓ 所有 11 路相机的 JSON 文件都有对应的 JPEG 图片
✓ camera_front_far: 197/197 匹配
✓ camera_front_fisheye: 197/197 匹配
✓ camera_front_wide: 197/197 匹配
✓ camera_left_fisheye: 197/197 匹配
✓ camera_left_front: 197/197 匹配
✓ camera_left_rear: 197/197 匹配
✓ camera_rear_fisheye: 197/197 匹配
✓ camera_rear_mid: 197/197 匹配
✓ camera_right_fisheye: 197/197 匹配
✓ camera_right_front: 197/197 匹配
✓ camera_right_rear: 197/197 匹配
```

**验证脚本**:
```python
import os

base_dir = '/path/to/output'
camera_name = 'camera_front_far'
json_dir = f'{base_dir}/sensor_data/egopose_opt/{camera_name}'
jpeg_dir = f'{base_dir}/sensor_data/camera/{camera_name}'

json_timestamps = {f.replace('.json', '') for f in os.listdir(json_dir) if f.endswith('.json')}
jpeg_timestamps = {f.replace('.jpeg', '').replace('.jpg', '') for f in os.listdir(jpeg_dir) if f.endswith(('.jpeg', '.jpg'))}

missing = json_timestamps - jpeg_timestamps
print(f"缺失: {len(missing)}")  # 应该是 0
```

### 2. 主雷达位姿与点云对应关系

**验证路径**:
- `sensor_data/egopose_opt/egopose_optpose/` (雷达位姿 JSON)
- `sensor_data/lidar/lidar_concat/` (点云 PCD)

**验证标准**:
- 所有雷达位姿 JSON 文件的时间戳都是 PCD 时间戳
- 所有 JSON 文件的时间戳都能在 lidar_concat 文件夹中找到同名 PCD
- 位姿坐标系：相对第一帧车体坐标系（第一帧 x=0,y=0,yaw=0°，x轴为车辆前向）

**预期结果**:
```
✓ 所有 egopose_optpose JSON 都有对应的 PCD 文件
✓ JSON 文件数: 197
✓ PCD 文件数: 197
✓ 匹配率: 197/197 (100%)
✓ 第一帧: x=0, y=0, yaw=0°
✓ x轴递增表示车辆前向行驶
```

**示例验证**:
```
✓ 1764310903499619840.json → 1764310903499619840.pcd
✓ 1764310903599625728.json → 1764310903599625728.pcd
✓ 1764310903699620096.json → 1764310903699620096.pcd
```

**验证脚本**:
```python
import os, json
import math

base_dir = '/path/to/output'
optpose_dir = f'{base_dir}/sensor_data/egopose_opt/egopose_optpose'
lidar_dir = f'{base_dir}/sensor_data/lidar/lidar_concat'

json_timestamps = {f.replace('.json', '') for f in os.listdir(optpose_dir) if f.endswith('.json')}
pcd_timestamps = {f.replace('.pcd', '') for f in os.listdir(lidar_dir) if f.endswith('.pcd')}

missing_pcd = json_timestamps - pcd_timestamps
print(f"JSON 找不到对应 PCD: {len(missing_pcd)}")  # 应该是 0

# 验证坐标系是否正确
files = sorted(os.listdir(optpose_dir))[:3] + sorted(os.listdir(optpose_dir))[-1:]
print("位姿验证:")
for f in files:
    with open(os.path.join(optpose_dir, f)) as fp:
        d = json.load(fp)
    p = d['position']['position_local']
    q = d['orientation']['quaternion_local']
    qx, qy, qz, qw = q['x'], q['y'], q['z'], q['w']
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    yaw_deg = math.degrees(yaw)
    print(f"  {f}: x={p['x']:.4f}, y={p['y']:.4f}, yaw={yaw_deg:.2f}deg")
```

### 3. PCD-相机时间戳映射验证

**验证路径**:
- `node_output/peral-dataproc/result.json` (映射表)
- `sensor_data/egopose_opt/egopose_optpose/` (雷达位姿)

**验证标准**:
- result.json 中记录的 PCD 时间戳应该与实际生成的 JSON 文件名一致
- JSON 文件内容中的 `meta.timestamp_us` 应该是原始相机时间戳（纳秒/1000）

**示例映射关系**:
```
sync_sensors.txt: 1764310903495000000.jpeg (相机时间戳)
         ↓ 查找 result.json
result.json: PCD 1764310903499619840.pcd → camera_front_far 1764310903495000000.jpeg
         ↓ 输出
egopose_optpose/1764310903499619840.json (使用 PCD 时间戳命名)
         ↓ JSON 内容
meta.timestamp_us: 1764310903495000 (原始相机时间戳，纳秒/1000)
```

### 4. 完整验证流程

**步骤 1**: 验证相机位姿对齐
```bash
python3 << 'EOF'
import os
base_dir = '/path/to/output'
for camera in os.listdir(f'{base_dir}/sensor_data/egopose_opt'):
    if camera.startswith('camera_'):
        json_ts = {f.replace('.json', '') for f in os.listdir(f'{base_dir}/sensor_data/egopose_opt/{camera}') if f.endswith('.json')}
        jpeg_ts = {f.replace('.jpeg', '').replace('.jpg', '') for f in os.listdir(f'{base_dir}/sensor_data/camera/{camera}') if f.endswith(('.jpeg', '.jpg'))}
        missing = json_ts - jpeg_ts
        print(f"{camera}: {'✓' if not missing else '✗'} ({len(json_ts)} JSON, {len(missing)} missing)")
EOF
```

**步骤 2**: 验证雷达位姿对齐
```bash
python3 << 'EOF'
import os
base_dir = '/path/to/output'
json_ts = {f.replace('.json', '') for f in os.listdir(f'{base_dir}/sensor_data/egopose_opt/egopose_optpose') if f.endswith('.json')}
pcd_ts = {f.replace('.pcd', '') for f in os.listdir(f'{base_dir}/sensor_data/lidar/lidar_concat') if f.endswith('.pcd')}
missing = json_ts - pcd_ts
print(f"egopose_optpose: {'✓' if not missing else '✗'} ({len(json_ts)} JSON, {len(missing)} missing)")
EOF
```

**步骤 3**: 验证映射关系
```bash
python3 << 'EOF'
import json
import os
base_dir = '/path/to/output'
with open(f'{base_dir}/node_output/peral-dataproc/result.json') as f:
    result = json.load(f)
expected_pcd = {pcd_key.replace('.pcd', '') for pcd_key in result.keys()}
actual_json = {f.replace('.json', '') for f in os.listdir(f'{base_dir}/sensor_data/egopose_opt/egopose_optpose') if f.endswith('.json')}
print(f"映射一致性: {'✓' if actual_json.issubset(expected_pcd) else '✗'}")
EOF
```

### 验证结果示例

以数据集 `557/2025-11-28-14-21-42` 为例：

| 验证项 | 结果 | 说明 |
|--------|------|------|
| 11 路相机位姿-图像对齐 | ✓ 197/197 | 所有相机 JSON 都有对应 JPEG |
| 雷达位姿-点云对齐 | ✓ 197/197 | 所有雷达 JSON 都有对应 PCD |
| PCD-相机映射一致性 | ✓ | 所有 JSON 使用正确的 PCD 时间戳 |

**注意**:
- result.json 包含确定的 PCD-相机对应关系（从 sync_sensors.txt 读取）
- lidar_concat 文件数 = egopose_optpose JSON 文件数 = result.json 条目数（完全对齐）
- 主雷达位姿已转换为相对第一帧车体坐标系，下游可直接使用（第一帧 x=0,y=0,yaw=0°，x轴为车辆前向）