# BEV4D 数据说明

本文档描述 BEV4D 数据格式和目录结构，包含激光、图像、雷达等传感器的原始数据、标定信息及后处理数据。

## 概述

以一个数采包作为基本单元，包含激光、图像以及 GPS 等传感器的原始数据信息、传感器标定信息、以及运动补偿后的点云以及 egopose 等经过算法处理的信息。

**坐标系定义：**
- 所有传感器的时间戳均在同一个时间域下进行授时
- 所有传感器的外参均指相对于车体坐标系的旋转平移关系
- 车体坐标系原点定义为车体后轴中心在地面的投影

---

## 目录树状图

```
<event_name>/                                 # 数采包根目录（事件名称）
├── sensor_data/                              # 传感器数据目录
│   ├── lidar/                                # 激光雷达数据
│   │   ├── lidar_odist/                      # 去畸变顶上激光点云
│   │   │   └── *.pcd                         # PCD格式点云文件（时间戳命名）
│   │   └── lidar_map/                        # 顶上激光拼接点云地图
│   │       └── *.pcd                         # 拼接后的PCD格式点云
│   ├── radar/                                # 毫米波雷达数据
│   │   ├── radar_front/                      # 前向雷达数据
│   │   │   └── *                             # 雷达数据文件
│   │   ├── radar_lc/                         # 左中雷达数据
│   │   ├── radar_lr/                         # 左后雷达数据
│   │   ├── radar_rc/                         # 右中雷达数据
│   │   ├── radar_rear/                       # 后向雷达数据
│   │   └── radar_rr/                         # 右后雷达数据
│   └── camera/                               # 相机图像数据
│       ├── camera_<name>/                    # 各相机目录（普通相机）
│       │   └── <timestamp>.jpeg              # JPEG图像（时间戳命名，曝光结束时间）
│       └── camera_fisheye_<name>/            # 鱼眼相机目录
│           └── <timestamp>.jpeg              # JPEG图像（时间戳命名）
├── egopose_opt/                              # 自车定位信息（离线定位）
│   ├── egopose_opt.json                      # 离线定位信息汇总文件
│   └── egopose_optpose/                      # 各帧定位信息
│       └── <timestamp>.json                  # 单帧定位数据（JSON格式）
├── calib_anno/                               # 所有原始相机的内参 外参 畸变系数矩阵，来自于calibration
│   └── camera/                               # 相机标定参数
│       └── camera_<name>.yaml                # 相机标定文件（外参+内参）
├── calib_anno_vc/                            # 所有虚拟相机的内参 外参 畸变系数矩阵，来自于calibration
│   └── camera_<name>.json                    # 虚拟相机标定文件
├── calibration/                              # 在线标定数据
│   ├── camera/                               # 相机在线标定
│   ├── lidar/                                # 激光雷达在线标定
│   ├── localization/                         # 定位相关标定
│   └── virtual_camera/                       # 少量点云投影到各传感器下的JPG图像
├── visualize/                                # 可视化结果目录
│   └── <timestamp>/                          # 按时间戳分组的可视化结果
│       └── *.png                             # 可视化图像文件
├── logs/                                     # 日志文件目录
│   ├── peral-flow-bev4d.log                  # BEV4D处理日志
│   └── peral-optpose.log                     # 定位优化日志
└── node_output/                              # 节点输出目录
    └── peral-dataproc/                       # 数据处理节点输出
        ├── result.json                       # pcd与11v映射关系
        └── result_full11v.josn               # 暂时与result.json一致
```

---

## 传感器数据详解

### 1. 去畸变顶上激光点云

**路径：**
- 数据：`sensor_data/lidar/lidar_odist/`
- 定位：`sensor_data/egopose_opt/egopose_optpose/`

**文件格式：**
- 命名规则：`<timestamp>.pcd`
- 格式：PCD (Point Cloud Data) 二进制格式

**点云字段：**
| 字段 | 说明 |
|------|------|
| x | X坐标 |
| y | Y坐标 |
| z | Z坐标 |
| timestamp | 时间戳 |
| intensity | 反射强度 |
| ring | 激光线束编号 |
| azimuth | 方位角 |

**注意事项：**
- 激光点已通过外参转到车体坐标系
- 时间戳单位均为 μs（微秒）

### 2. 顶上激光拼接点云地图

**路径：**
- 数据：`sensor_data/lidar/lidar_map/`
- 定位：`sensor_data/egopose_opt/egopose_optpose/`

**文件格式：**
- 命名规则：`<timestamp>.pcd`

**点云字段：**
与去畸变点云相同，但激光点已转到第一帧车体坐标系。

### 3. 毫米波雷达

**数据路径与定位路径对应表：**

| 雷达名称 | 数据路径 | 定位路径 |
|---------|---------|---------|
| 前向雷达 | `sensor_data/radar/radar_front/` | `sensor_data/egopose_opt/radar_front/` |
| 左中雷达 | `sensor_data/radar/radar_lc/` | `sensor_data/egopose_opt/radar_lc/` |
| 左后雷达 | `sensor_data/radar/radar_lr/` | `sensor_data/egopose_opt/radar_lr/` |
| 右中雷达 | `sensor_data/radar/radar_rc/` | `sensor_data/egopose_opt/radar_rc/` |
| 后向雷达 | `sensor_data/radar/radar_rear/` | `sensor_data/egopose_opt/radar_rear/` |
| 右后雷达 | `sensor_data/radar/radar_rr/` | `sensor_data/egopose_opt/radar_rr/` |

### 4. 相机

**路径：**
- 数据：`sensor_data/camera/camera_<name>/`
- 定位：`sensor_data/egopose_opt/camera_<name>/`

**文件格式：**
- 命名规则：`<timestamp>.jpeg`
- 时间戳指的是曝光结束时间

**相机类型：**
- 普通相机：`camera_<name>/`
- 鱼眼相机：`camera_fisheye_<name>/`

---

## 标定参数详解

### 相机标定参数

**路径：** `calib_anno/camera/camera_<name>.yaml`

**YAML 配置示例：**

```yaml
CLOCK_calib_version: "1d987b7d05-dirty/master"    # 标定版本
CLOCK_calib_details: "output by CLOCK calibration"
CLOCK_calib_date: "2023-09-25-12:39:22"           # 标定日期

sensor_name: "camera_front_wide"                  # 传感器命名
sensor_type: "Camera"                             # 传感器类型
timestamp_shift: 0                                # 时间戳延延时（ms）

vehicle_xyz: "front_left_up"                      # 车体系定义（前左上，后轴中心接地点）

# 外参 - 旋转（旋转向量）
r_s2b: [-1.210487, 1.214899, -1.200436]           # 传感器到车体系的旋转

# 外参 - 平移（米）
t_s2b: [2.1352317030037655, -0.00870754000000000002, 1.526446]

# 内参
camera_model: "polyn"                             # 相机模型（等距相机模型）
fx: 1911.701748                                   # 焦距 fx（像素）
fy: 1911.701748                                   # 焦距 fy（像素）
cx: 1917.431243                                   # 主点 cx（像素）
cy: 1694.68327                                    # 主点 cy（像素）

# 畸变系数
kc2: -0.034213
kc3: -0.001692
kc4: 0
kc5: 0

is_fisheye: false                                 # 是否是鱼眼相机
line_exposure_delay: 0                            # 行曝光延迟（μs）
width: 3840                                       # 图像宽度（像素）
height: 2160                                      # 图像高度（像素）
suggested_rect_region_within_ROI: [0, 0, 3840, 2160]  # 建议使用的图像ROI
```

**字段说明：**

| 字段 | 说明 |
|------|------|
| `r_s2b` | 传感器到车体系的旋转向量，p_b = R(r_s2b) * p_s |
| `t_s2b` | 传感器到车体系的平移（单位：米） |
| `camera_model` | 相机模型，polyn 为等距相机模型 |
| `fx`, `fy` | 焦距（像素） |
| `cx`, `cy` | 主点坐标（像素） |
| `kc2`~`kc5` | 畸变系数 |

---

## 后处理信息

### 1. 各传感器数据对齐

**目录：** `od_destined/data_sync/`

**文件：** `radar.json`

**说明：**
- 该 JSON 是 bag 包级别的传感器数据对齐
- 实际应用过程中可进行抽帧
- 后续只会用到 radar.json 中涉及到的传感器及其定位信息

### 2. 单 bag 包拼接点云操作流程

**涉及文件：**
- `lidar/lidar_map/` - 拼接点云数据
- `egopose_opt/egopose_optpose/` - 离线定位信息
- `egopose_opt/egopose_opt.json` - 定位汇总信息

### 3. 自车定位信息 (egopose_opt.json)

**路径：** `sensor_data/egopose_opt/egopose_opt.json`

**JSON 结构：**

```json
{
  "header": {
    "seq": 444630,
    "stamp": {
      "secs": 1693311537,
      "nsecs": 528672473
    },
    "frame_id": "{\"coordinate_system\": \"GCJ02\", \"gnss_type\": \"rtk\", \"loc_type\": \"ddpf\"}"
  },
  "meta": {
    "timestamp_us": 1693311537543940,    // 传感器时间（微秒）
    "seq": 444630
  },
  "position": {
    "available": 3,
    "position_global": {
      "latitude": 27.91072136275722,     // GCJ02 坐标系纬度
      "longitude": 112.88816407484823,   // GCJ02 坐标系经度
      "altitude": 0.0
    },
    "position_local": {
      "x": 2226.2792000572035,           // 局部坐标系 X
      "y": -4803.918817306827,           // 局部坐标系 Y
      "z": -74.34943892907062            // 局部坐标系 Z
    }
  },
  "velocity": {
    "available": 3,
    "velocity_global": {
      "ve": 25.05373025113549,           // ENU 下速度
      "vn": 8.558763080511905,
      "vu": 0.30843588537803357
    },
    "velocity_local": {
      "vx": -3.18864986702602,
      "vy": 26.383956670211738,
      "vz": 0.20710309885589742
    }
  },
  "angular_velocity": {
    "available": 1,
    "angvelocity_local": {
      "vx": -0.004916933694351262,       // 角速度
      "vy": 0.0031466404497138226,
      "vz": 0.004512908882351219
    }
  },
  "orientation": {
    "available": 15,
    "euler_global": {
      "roll": -0.01486402913256871,      // 欧拉角
      "pitch": -0.011856329782549146,
      "yaw": 0.32863650341342565
    },
    "quaternion_global": {
      "w": 0.9864927321304374,           // 四元数
      "x": -0.0063620141927859435,
      "y": -0.007063812294703622,
      "z": 0.163528947850529
    },
    "euler_local": {...},
    "quaternion_local": {...}
  },
  "transform": {
    "available": 4,
    "transform_llh_to_boot": {...},       # LLH 到 boot 坐标系变换
    "transform_avp_map_to_boot": {...},   # AVP 地图到 boot 坐标系变换
    "transform_ego_motion_to_boot": {...} # 自车运动到 boot 坐标系变换
  }
}
```

**主要字段说明：**

| 字段 | 说明 |
|------|------|
| `position_global` | GCJ02 坐标系下的经纬度 |
| `position_local` | 局部坐标系下的 XYZ |
| `velocity_global` | ENU 坐标系下的速度 |
| `velocity_local` | 局部坐标系下的速度 |
| `orientation` | 姿态信息（欧拉角、四元数） |
| `transform` | 坐标变换矩阵 |

---

## 拼接点云定位信息

**文件：** `egopose_opt/egopose_optpose/*.json`

**merge_frame 示例：**

```json
{
  "merge_frame": {
    "1": "1705916159050071040.json",    // 拼接点云的参考坐标系，定义此帧位置为 base_pose
    "2": "1705916159650044928.json",    // 通过时间在 egopose_opt.json 寻找对应的旋转与位置信息
    "3": "1705916160150087936.json",
    ...
  }
}
```

---

## 单帧图像数据处理

**涉及文件：**
- `camera/*.jpeg` - 图像数据
- `calib_anno/camera/camera_<name>.yaml` - 相机标定参数
- `egopose_opt/camera_<name>/*.json` - 相机定位

**投影公式：**

```
P_image = K * (camera_ex_pose)^(-1) * (camera_pose)^(-1) * base_pose * P_lidar
```

其中：
- `P_lidar` - PCD 标注成果 [x, y, z]
- `camera_ex_pose` - 相机外参
- `camera_pose` - 相机位姿
- `base_pose` - 基准位姿
- `K` - 相机内参矩阵

---

## 注意事项

1. **时间戳单位：** 激光点云时间戳单位为 μs（微秒）
2. **坐标系：** 所有外参均相对于车体坐标系（后轴中心地面投影）
3. **相机模型：** 所有相机的 `camera_model` 都是等距相机模型（polyn）
4. **图像命名：** 时间戳为曝光结束时间
