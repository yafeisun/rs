# Robosense 到 BEV4D 格式映射文档

本文档详细描述从 Robosense 数据格式转换到 BEV4D 格式的完整映射关系和转换逻辑。

---

## 概述

### 源格式 (Robosense)

基于 ROS Bag 采集的原始数据，包含：
- 11 路相机图像 (`cam_*`)
- 5 路激光雷达点云 (`raw/pcd/*`)
- 标定参数 (`car_*.yaml`)
- 后处理结果 (`result/test_calibration/`)

### 目标格式 (BEV4D)

标准化数据格式，包含：
- 传感器数据 (`sensor_data/`)
- 标定参数 (`calib_anno/`, `calib_anno_vc/`)
- 定位信息 (`egopose_opt/`)
- 时间戳映射 (`node_output/`)

### 坐标系定义

| 坐标系 | 说明 |
|--------|------|
| 车体坐标系 | 原点: 后轴中心地面投影; X=前, Y=左, Z=上 |
| 第一帧坐标系 | 以第一帧车体位置为原点的相对坐标系 |

---

## 一、相机图像映射

### 1.1 目录映射表

| 源目录 | 目标目录 | 说明 |
|--------|----------|------|
| `cam_around_back/` | `sensor_data/camera/camera_rear_fisheye/` | 后向鱼眼 |
| `cam_around_front/` | `sensor_data/camera/camera_front_fisheye/` | 前向鱼眼 |
| `cam_around_left/` | `sensor_data/camera/camera_left_fisheye/` | 左侧鱼眼 |
| `cam_around_right/` | `sensor_data/camera/camera_right_fisheye/` | 右侧鱼眼 |
| `cam_back/` | `sensor_data/camera/camera_rear_mid/` | 后向中置 |
| `cam_front_left/` | `sensor_data/camera/camera_front_far/` | 前向远距 |
| `cam_front_right/` | `sensor_data/camera/camera_front_wide/` | 前向广角 |
| `cam_side_left_back/` | `sensor_data/camera/camera_left_rear/` | 左后侧 |
| `cam_side_left_front/` | `sensor_data/camera/camera_left_front/` | 左前侧 |
| `cam_side_right_back/` | `sensor_data/camera/camera_right_rear/` | 右后侧 |
| `cam_side_right_front/` | `sensor_data/camera/camera_right_front/` | 右前侧 |

### 1.2 文件转换

```
源文件格式: <timestamp>.jpeg / <timestamp>.jpg / <timestamp>.png
目标格式:   <timestamp>.jpeg

说明: 
- 文件名保持原始时间戳不变
- 非jpeg格式统一转换为jpeg
- 时间戳为曝光结束时间（纳秒级，19位）
```

---

## 二、激光雷达点云映射

### 2.1 单雷达点云映射

| 源目录 | 目标目录 | 说明 |
|--------|----------|------|
| `raw/pcd/middle/` | `sensor_data/lidar/lidar/` | 主雷达（顶上） |
| `raw/pcd/front/` | `sensor_data/lidar/lidar_front_up/` | 前向雷达 |
| `raw/pcd/back/` | `sensor_data/lidar/lidar_rear_up/` | 后向雷达 |
| `raw/pcd/right/` | `sensor_data/lidar/lidar_fr/` | 右前雷达 |
| `raw/pcd/left/` | `sensor_data/lidar/lidar_fl/` | 左前雷达 |

**转换逻辑:**
- 直接拷贝，文件名不变
- 时间戳命名规则：`<timestamp>.pcd`

### 2.2 拼接点云映射 (lidar_concat)

**关键映射 - 使用 sync_sensors.txt 重命名**

```
源路径: result/test_calibration/middle/*.pcd (自然序号命名: 1.pcd, 2.pcd, ...)
目标路径: sensor_data/lidar/lidar_concat/*.pcd (时间戳命名)

映射文件: result/test_calibration/sync_sensors.txt
```

**sync_sensors.txt 格式:**
```
#frame /middle /cam_front_right /cam_back ...
1 1764310903499619840 1764310903495000000 1764310903493000000 ...
2 1764310903599625728 1764310903595000000 1764310903593000000 ...
```

**转换规则:**
```
自然序号 1.pcd -> 时间戳 1764310903499619840.pcd (列2)
自然序号 2.pcd -> 时间戳 1764310903599625728.pcd
...
```

### 2.3 拼接点云地图映射 (lidar_map)

```
源路径: result/test_calibration/map_intensity_bev.pcd
目标路径: sensor_data/lidar/lidar_map/<first_timestamp>.pcd

命名规则: 使用 egopose_optpose 中第一个位姿文件的时间戳
```

---

## 三、标定参数映射

### 3.1 相机标定 YAML 映射

| 源话题 | 目标路径 |
|--------|----------|
| `cam_around_back/compressed` | `calibration/camera/camera_rear_fisheye.yaml` |
| `cam_around_front/compressed` | `calibration/camera/camera_front_fisheye.yaml` |
| `cam_around_left/compressed` | `calibration/camera/camera_left_fisheye.yaml` |
| `cam_around_right/compressed` | `calibration/camera/camera_right_fisheye.yaml` |
| `cam_back/compressed` | `calibration/camera/camera_rear_mid.yaml` |
| `cam_front_left/compressed` | `calibration/camera/camera_front_far.yaml` |
| `cam_front_right/compressed` | `calibration/camera/camera_front_wide.yaml` |
| `cam_side_left_back/compressed` | `calibration/camera/camera_left_rear.yaml` |
| `cam_side_left_front/compressed` | `calibration/camera/camera_left_front.yaml` |
| `cam_side_right_back/compressed` | `calibration/camera/camera_right_rear.yaml` |
| `cam_side_right_front/compressed` | `calibration/camera/camera_right_front.yaml` |

### 3.2 激光雷达标定 YAML 映射

| 源话题 | 目标路径 |
|--------|----------|
| `/middle/rslidar_packets_unique` | `calibration/lidar/lidar.yaml` |
| `/left/rslidar_packets_unique` | `calibration/lidar/lidar_fl.yaml` |
| `/right/rslidar_packets_unique` | `calibration/lidar/lidar_fr.yaml` |
| `/front/rslidar_packets_unique` | `calibration/lidar/lidar_front_up.yaml` |
| `/back/rslidar_packets_unique` | `calibration/lidar/lidar_rear_up.yaml` |

### 3.3 标定参数转换逻辑

#### 源格式 (car_*.yaml)
```yaml
sensors:
  camera:
    - topic: /cam_front_right/compressed
      calibration:
        CameraExt:
          x: 2.135, y: -0.008, z: 1.526
          roll: -1.210, pitch: 1.214, yaw: -1.200
        CameraIntMat: [1911.7, 0, 1917.4, 0, 1911.7, 1694.7, 0, 0, 1]
        DistCoeff: [-0.034, -0.001, 0, 0]
        ImageSize: [3840, 2160]
```

#### 目标格式 (calibration/camera/*.yaml)
```yaml
%YAML:1.0
---
sensor_name: "camera_front_wide"
sensor_type: "camera"
vehicle_xyz: "front_left_up"
r_s2b: [-1.210487, 1.214899, -1.200436]  # 旋转向量 (欧拉角转换)
t_s2b: [2.135, -0.008, 1.526]           # 平移向量 (米)
camera_model: "polyn"
fx: 1911.7
fy: 1911.7
cx: 1917.4
cy: 1694.7
kc2: -0.034  # 畸变系数 k1
kc3: -0.001  # 畸变系数 k2
kc4: 0       # 畸变系数 k3
kc5: 0       # 畸变系数 k4
is_fisheye: false
width: 3840
height: 2160
```

#### 欧拉角到旋转向量转换

使用 Rodrigues 公式:
```python
def euler_to_rotation_vector(roll, pitch, yaw):
    # 1. 构建旋转矩阵 R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # 2. 计算旋转角度 θ = arccos((trace(R) - 1) / 2)
    # 3. 计算旋转轴 [rx, ry, rz]
    # 4. 返回 [rx*θ, ry*θ, rz*θ]
```

### 3.4 calib_anno 生成 (JSON 格式)

**路径:** `calib_anno/camera_<name>.json`

**从 YAML 生成 JSON:**
```json
{
  "extrinsic": [r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz, 0, 0, 0, 1],
  "intrinsic": [fx, 0, cx, 0, fy, cy, 0, 0, 1],
  "distcoeff": [kc2, kc3, kc4, kc5]
}
```

**extrinsic 构建逻辑:**
```python
R = rotation_vector_to_matrix(r_s2b)  # 3x3 旋转矩阵
extrinsic = np.eye(4)                  # 4x4 单位矩阵
extrinsic[:3, :3] = R                  # 填入旋转
extrinsic[:3, 3] = t_s2b               # 填入平移
```

### 3.5 calib_anno_vc 生成 (虚拟相机/去畸变)

**路径:** `calib_anno_vc/camera_<name>.json`

**与 calib_anno 的区别:**
| 字段 | calib_anno | calib_anno_vc |
|------|------------|---------------|
| extrinsic | 相同 | 相同 |
| intrinsic | `[fx, 0, cx, 0, fy, cy, 0, 0, 1]` | `[fx, 0, cx, 0, fx, cy, 0, 0, 1]` (fy=fx) |
| distcoeff | `[kc2, kc3, kc4, kc5]` | `[0, 0, 0, 0]` (已去畸变) |

---

## 四、时间戳映射文件

### 4.1 sync_sensors.txt 格式解析

**源文件:** `result/test_calibration/sync_sensors.txt`

```
#frame /middle /cam_front_right /cam_back /cam_side_left_front ...
1 1764310903499619840 1764310903495000000 1764310903493000000 1764310903493000000 ...
```

| 列号 | 内容 | 说明 |
|------|------|------|
| 列1 | 帧号 | 自然序号 (1, 2, 3, ...) |
| 列2 | /middle | 主雷达时间戳 (PCD时间戳) |
| 列3 | /cam_front_right | 相机时间戳 |
| 列4-13 | 其他相机 | 各相机时间戳 |

### 4.2 result.json 生成

**目标路径:** `node_output/peral-dataproc/result.json`

**生成逻辑:**
```python
# 遍历 sync_sensors.txt 每一行
for frame, middle_ts, camera_ts_list in sync_data:
    # 检查 middle_ts 是否在 lidar_concat 中存在
    if middle_ts in pcd_files_set:
        pcd_name = f"{middle_ts}.pcd"
        
        # 构建相机映射
        frame_mapping = {
            "camera_front_right": f"{camera_ts_list[0]}.jpeg",
            "camera_rear_mid": f"{camera_ts_list[1]}.jpeg",
            ...
        }
        
        result[pcd_name] = frame_mapping
```

**输出格式:**
```json
{
  "1764310903499619840.pcd": {
    "camera_front_far": "1764310903495000000.jpeg",
    "camera_front_fisheye": "1764310903499000000.jpeg",
    "camera_front_wide": "1764310903495000000.jpeg",
    ...
  },
  "1764310903599625728.pcd": {
    ...
  }
}
```

### 4.3 相机列名映射表

| sync_sensors.txt 列名 | result.json 字段名 |
|----------------------|-------------------|
| `/middle` | (用于 PCD 时间戳，不输出) |
| `/cam_back` | `camera_rear_mid` |
| `/cam_side_left_front` | `camera_left_front` |
| `/cam_side_right_front` | `camera_right_front` |
| `/cam_side_left_back` | `camera_left_rear` |
| `/cam_side_right_back` | `camera_right_rear` |
| `/cam_front_left` | `camera_front_far` |
| `/cam_around_front` | `camera_front_fisheye` |
| `/cam_around_left` | `camera_left_fisheye` |
| `/cam_around_right` | `camera_right_fisheye` |
| `/cam_around_back` | `camera_rear_fisheye` |

---

## 五、位姿数据映射

### 5.1 相机位姿映射

**源文件:** `result/test_calibration/cam_<name>/sync_sensors.txt`

```
#index timestamp x y z q_x q_y q_z q_w
1 1764310903495000000.jpeg -0.229947 1.151212 -1.999342 0.009042 0.008805 0.753082 0.657805
```

**目标路径:** `sensor_data/egopose_opt/camera_<name>/<timestamp>.json`

**重要:** 文件名使用 **相机时间戳** (sync_sensors.txt 中的 timestamp 列)

**相机名称映射:**
| 源目录名 | 目标目录名 |
|----------|-----------|
| `cam_around_back` | `camera_rear_fisheye` |
| `cam_around_front` | `camera_front_fisheye` |
| `cam_back` | `camera_rear_mid` |
| `cam_front_left` | `camera_front_far` |
| `cam_front_right` | `camera_front_wide` |
| ... | ... |

### 5.2 主雷达位姿映射

**源文件:** `result/test_calibration/middle/sync_sensors.txt`

**目标路径:** `sensor_data/egopose_opt/egopose_optpose/<pcd_timestamp>.json`

**关键映射逻辑:**

```
sync_sensors.txt 中的时间戳是相机时间戳
          ↓
需要通过 result.json 找到对应的 PCD 时间戳
          ↓
使用 PCD 时间戳作为 JSON 文件名
```

**示例流程:**
```
1. 读取 sync_sensors.txt:
   相机时间戳: 1764310903495000000
   位姿: x=-0.226895, y=1.130095, z=-1.998865, q=(0.008977, 0.008787, 0.753109, 0.657776)

2. 查找 result.json:
   PCD 1764310903499619840.pcd -> camera_front_far 1764310903495000000.jpeg
   找到映射: 相机时间戳 1764310903495000000 -> PCD 时间戳 1764310903499619840

3. 输出文件:
   1764310903499619840.json (使用 PCD 时间戳命名)
```

### 5.3 位姿坐标系转换

**转换到第一帧车体坐标系:**

```python
def transform_poses_to_first_frame(pose_dict):
    """
    输入: {timestamp_ns: [x, y, z, qx, qy, qz, qw], ...}
    输出: 相对第一帧的位姿
    
    转换公式: T_rel_i = T_world_body0^{-1} * T_world_bodyi
    """
    # 第一帧位姿
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_dict[first_timestamp]
    R0 = quat_to_rotation_matrix(qw0, qx0, qy0, qz0)
    t0 = [x0, y0, z0]
    
    # T0^{-1}
    R0_inv = R0.T
    t0_inv = -R0_inv @ t0
    
    for each pose:
        # T_rel = T0^{-1} * Ti
        R_rel = R0_inv @ Ri
        t_rel = R0_inv @ ti + t0_inv
```

**结果验证:**
- 第一帧: x=0, y=0, yaw=0°
- 后续帧: x递增表示车辆前向行驶

### 5.4 位姿 JSON 结构

```json
{
  "header": {
    "seq": 0,
    "stamp": {"secs": 1764310903, "nsecs": 499619840},
    "frame_id": "{\"coordinate_system\":\"GCJ02\",\"version\":\"LTS_6188e01\"}"
  },
  "meta": {
    "timestamp_us": 1764310903499619,
    "seq": 0,
    "type": 0
  },
  "position": {
    "available": 2,
    "position_local": {"x": 0.0, "y": 0.0, "z": 0.0}
  },
  "orientation": {
    "available": 12,
    "euler_local": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "quaternion_local": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
  },
  ...
}
```

---

## 六、IMU/GNSS/轮速数据提取

### 6.1 IMU 数据

**源:** Bag 文件 topic `/rs/imu`

**目标:** `sensor_data/imu.json`

**字段映射:**
```json
{
  "header": {"seq", "stamp", "frame_id"},
  "meta": {"timestamp_us", "seq"},
  "info": {
    "imu_data_basic": {
      "accx": msg.linear_acceleration.x,
      "accy": msg.linear_acceleration.y,
      "accz": msg.linear_acceleration.z,
      "gyrox": msg.angular_velocity.x,
      "gyroy": msg.angular_velocity.y,
      "gyroz": msg.angular_velocity.z,
      "temperature": 25.0
    }
  }
}
```

### 6.2 GNSS 数据

**源:** Bag 文件 topic `/rs/gps`

**目标:** `sensor_data/gnss_rtk.json`

**字段映射:**
```json
{
  "info": {
    "gnss_data_basic": {
      "lat": msg.latitude,
      "lon": msg.longitude,
      "alt": msg.altitude,
      "status": msg.status.status,
      ...
    }
  }
}
```

### 6.3 轮速数据

**源:** Bag 文件 topic `/ft_vehicle_data_v3`

**目标:** `sensor_data/wheel_report.json`

**字段映射:**
```json
{
  "wheel_speed_report": {
    "wheel_speed_report_data": {
      "front_left": msg.wheelSpeedFrontLeft,
      "front_right": msg.wheelSpeedFrontRight,
      "rear_left": msg.wheelSpeedRearLeft,
      "rear_right": msg.wheelSpeedRearRight
    }
  },
  "vehicle_imu_report": {
    "vehicle_imu_report_data": {
      "linear_acceleration_lat": msg.lateralAcceleration,
      "linear_acceleration_lon": msg.acceleration,
      "angular_velocity_yaw": msg.yawRate
    }
  }
}
```

---

## 七、时间戳对应关系总结

### 7.1 时间戳类型

| 数据类型 | 时间戳来源 | 文件命名规则 |
|---------|-----------|-------------|
| PCD 点云 | PCD 时间戳 | PCD 时间戳 |
| 相机图像 | 相机时间戳 | 相机时间戳 |
| 相机位姿 | 相机时间戳 | **相机时间戳** |
| 主雷达位姿 | 相机时间戳 | **PCD 时间戳** (需映射) |

### 7.2 时间戳差异示例

```
PCD 时间戳:   1764310903499619840
相机时间戳:   1764310903495000000
差异:         4619840 ns ≈ 4.6 ms
```

### 7.3 一致性验证

转换完成后应验证:
1. `lidar_concat/*.pcd` 文件数 = `egopose_optpose/*.json` 文件数 = `result.json` 条目数
2. 所有 `egopose_optpose/*.json` 的时间戳都能在 `lidar_concat` 中找到对应 PCD
3. 所有相机位姿 JSON 的时间戳都能在对应相机文件夹找到 JPEG

---

## 八、转换流程图

```
输入: Robosense Bag 数据目录
      ├── cam_*/                    # 相机图像
      ├── raw/pcd/*/                # 点云数据
      ├── result/test_calibration/  # 后处理结果
      │   ├── sync_sensors.txt      # 时间戳映射
      │   ├── middle/*.pcd          # 拼接点云
      │   └── cam_*/sync_sensors.txt # 相机位姿
      └── car_*.yaml                # 标定参数

                    ↓ 转换 ↓

输出: BEV4D 格式目录
      ├── sensor_data/
      │   ├── camera/camera_*/           # 相机图像
      │   ├── lidar/lidar_*/             # 点云数据
      │   ├── lidar/lidar_concat/        # 拼接点云 (时间戳命名)
      │   └── egopose_opt/
      │       ├── camera_*/              # 相机位姿
      │       └── egopose_optpose/       # 主雷达位姿
      ├── calib_anno/                    # 标定 JSON
      ├── calib_anno_vc/                 # 虚拟相机标定
      ├── calibration/                   # 标定 YAML
      │   ├── camera/
      │   ├── lidar/
      │   └── virtual_camera/
      └── node_output/peral-dataproc/
          └── result.json                # PCD-相机映射
```

---

## 九、常见问题

### Q1: 为什么主雷达位姿用 PCD 时间戳命名？

A: 为了与点云数据对齐。sync_sensors.txt 中记录的是相机时间戳，但下游使用 PCD 时间戳关联数据，因此需要通过 result.json 映射后使用 PCD 时间戳命名。

### Q2: calib_anno 和 calib_anno_vc 有什么区别？

A: calib_anno 是原始标定参数，calib_anno_vc 是去畸变后的虚拟相机标定。虚拟相机的畸变系数为 0，内参中 fy=fx。

### Q3: 如何验证转换正确性？

A: 检查以下对应关系：
1. `result.json` 中的 PCD 时间戳与 `lidar_concat` 文件一致
2. `egopose_optpose` JSON 文件名与 `lidar_concat` PCD 文件名一致
3. 相机位姿 JSON 时间戳能在对应相机目录找到图像
