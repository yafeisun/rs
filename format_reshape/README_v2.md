# ROS Bag 格式转换工具 (V2)

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
  [NodeOutput] Src: /path/dst/sensor_data/lidar/lidar
               Dst: /path/dst/node_output/peral-dataproc/result.json, result_full11v.json
               Done: 206 frames mapped
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

## 改进内容 (V2)

### 1. 批量处理
- 自动递归查找符合时间戳格式的目录
- 批量处理所有有效数据
- 显示处理进度和结果统计

### 2. 命令行参数
- `--src-dir` / `-s`: 指定源目录
- `--target-root` / `-t`: 指定目标根目录（可选）

### 3. 自动查找 bag 文件
- 无需手动指定 bag 文件名
- 自动扫描目录中的 .bag 文件

### 4. 新增输出功能
- **calib_anno**: 从 `calibration/camera/*.yaml` 生成标定 JSON
- **calib_anno_vc**: 生成去畸变后的虚拟相机标定
- **node_output/result.json**: 点云→相机时间戳映射
- **node_output/result_full11v.json**: 点云→相机+result 时间戳映射

### 5. 增强异常处理
- 所有文件操作添加 try-except
- Bag 操作使用 try-finally 确保资源释放
- 字段存在性检查，使用 `getattr()` 和 `hasattr()`

### 6. 改进时间戳转换
- 字符串分割避免浮点精度丢失
- 秒和微秒分别处理

### 7. 输出信息优化
- 详细显示源路径和目标路径
- 统计处理结果
- 英文输出

---

## 依赖

- `rosbag` (ROS Python bindings)
- `PyYAML`
- `numpy`

```bash
sudo apt install ros-noetic-rosbag python3-yaml python3-numpy
```

---

## 文件结构

```
format_reshape_v2/
├── reshape.py              # 主程序
├── check_pcd_header.py     # 检查PCD文件头
├── check_pcd_points.py     # 检查PCD点数
└── README_v2.md            # 本文档
```