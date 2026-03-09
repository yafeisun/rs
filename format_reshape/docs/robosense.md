# Robosense 测试数据目录结构

本文档描述 Robosense 测试数据的文件夹和文件结构。

## 目录树状图

```
2025-11-28-14-21-42/                           # 测试数据根目录（以采集时间命名）
├── cam_side_left_front/                       # 左前侧相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_around_back/                           # 后视环视相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_around_left/                           # 左视环视相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_back/                                  # 后视相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_front_left/                            # 左前相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_around_right/                          # 右视环视相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── cam_side_right_back/                       # 右后侧相机图像数据
│   └── *.jpeg                                 # 按时间戳命名的图像文件
├── state/                                     # 处理状态标记目录
│   ├── colormap_successed.txt                 # 颜色映射完成标记
│   ├── parse_successed.txt                    # 数据解析完成标记
│   ├── postprocess_successed.txt              # 后处理完成标记
│   ├── project_successed.txt                  # 投影处理完成标记
│   └── slam_successed.txt                     # SLAM处理完成标记
├── raw/                                       # 原始数据目录
│   ├── pcd/                                   # 点云数据目录
│   |   ├── front/                             # 前激光雷达点云 雷达坐标系 RFU
│   |   │   └── *.pcd                          # PCD格式点云文件（按时间戳命名）
│   |   ├── back/                              # 后激光雷达点云 雷达坐标系 RFU
│   |   │   └── *.pcd                          # PCD格式点云文件
│   |   ├── left/                              # 左激光雷达点云 雷达坐标系 RFU
│   |   │   └── *.pcd                          # PCD格式点云文件
│   |   ├── right/                             # 右激光雷达点云 雷达坐标系 RFU
│   |   │   └── *.pcd                          # PCD格式点云文件
│   |   └── middle/                            # 主激光雷达点云 雷达坐标系 RFU
│   |       └── *.pcd                          # PCD格式点云文件
│   ├── rtk_gps.txt                            # RTK GPS数据文件
│   ├── rtk_imu.txt                            # RTK IMU数据文件
│   ├── rtk_odom.txt                           # RTK 里程计数据文件
|   └── sync_time_list.txt                     # 主雷达和4个补盲雷达时间对应关系
└── result/                                    # 处理结果输出目录
    ├── bev/                                   # 鸟瞰图（Bird's Eye View）结果
    ├── debug/                                 # 调试信息目录
    │   ├── roadEdge/                          # 道路边缘检测结果
    │   └── lane/                              # 车道线检测结果
    ├── lane_image/                            # 车道线可视化图像
    ├── local_motion_pcdbin/                   # 局部运动点云（二进制格式）
    │   ├── front/                             # 前激光雷达局部运动点云
    │   ├── back/                              # 后激光雷达局部运动点云
    │   ├── left/                              # 左激光雷达局部运动点云
    │   └── right/                             # 右激光雷达局部运动点云
    ├── local_pcdbin/                          # 局部点云数据（二进制格式）
    ├── log/                                   # 处理日志目录
    ├── mapping/                               # 建图结果目录
    ├── motion_pcdbin/                         # 运动点云数据（二进制格式）
    ├── sensor_data/                           # 传感器同步数据
    │   └── sync_sensor_data.txt               # 传感器同步信息文件
    ├── slam_debug/                            # SLAM调试信息目录
    ├── slot_image/                            # 车位检测图像
    ├── test_calibration/                      # 标定测试结果目录
    │   ├── bbox/                              # 边界框标定结果
    │   ├── middle/                            # 主雷达和4个辅助雷达拼接后的点云，以自然序号排序，对应查看test_calibration/sync_sensors.txt
    |   |   ├── sync_sensors.txt               # 雷达位姿信息，主雷达和4个辅助雷达共享
    │   │   ├── sync_sensors.txt               # 雷达位姿信息，主雷达和4个辅助雷达共享
    │   │   └── *.pcd                          # PCD格式点云文件 车体坐标系 FLU (拼接后的点云，已从雷达RFU转换到车体FLU)
    │   ├── cam_*/                             # 各相机标定测试结果
    │   |   ├── sync_sensors.txt               # 相机传感器位姿信息
    |   ├── map_*.pcd                          # 全局地图点云
    │   ├── *.jpeg                             # 点云可视化图像
    |   └── sync_sensors.txt                   # 雷达和11V相机时间对应关系
    ├── tmp/                                   # 临时文件目录
    │   ├── index/                             # 索引文件
    │   ├── all_index/                         # 全部索引
    │   ├── peak_lane/                         # 车道线峰值检测
    │   │   └── txt/                           # 文本格式结果
    │   └── roadpoints/                        # 道路点数据
    ├── bumpy_log.txt                          # 颠簸检测日志
    ├── car_J6M_LB3783EZ8SA070557__2025-05-28.yaml  # 车辆配置文件（车型、VIN、日期）
    ├── exception_odom.txt                     # 里程计异常记录
    ├── global_GT.bin                          # 全局真值数据（二进制格式）
    ├── global_GT.csv                          # 全局真值数据（CSV格式）
    ├── GT.bin                                 # 局部真值数据（二进制格式）
    ├── GT.csv                                 # 局部真值数据（CSV格式）
    ├── GT_ROI.bin                             # ROI区域真值数据（二进制格式）
    └── GT_ROI.csv                             # ROI区域真值数据（CSV格式）
```



## 文件命名规则

### 图像文件
- 格式：`<timestamp>.jpeg`
- 时间戳为纳秒级 Unix 时间戳，如 `1764310902993000000.jpeg`

### 点云文件
- 格式：`<timestamp>.pcd`
- PCD（Point Cloud Data）格式，包含三维点云数据

## 坐标系说明

| 目录 | 坐标系 | 说明 |
|------|--------|------|
| `raw/pcd/*/` | 雷达RFU | 单独雷达点云，雷达自身坐标系（X右，Y前，Z上） |
| `result/test_calibration/middle/` | Body FLU | 拼接点云，车体坐标系（X前，Y左，Z上） |


## 传感器分布

| 目录名 | 传感器类型 | 方位 |
|--------|-----------|------|
| cam_side_left_front | 侧视相机 | 左前 |
| cam_side_right_back | 侧视相机 | 右后 |
| cam_front_left | 前视相机 | 左侧 |
| cam_back | 后视相机 | 正后方 |
| cam_around_front | 环视相机 | 前方 |
| cam_around_back | 环视相机 | 后方 |
| cam_around_left | 环视相机 | 左侧 |
| cam_around_right | 环视相机 | 右侧 |

## 激光雷达分布

| 目录名 | 方位 |
|--------|------|
| pcd/front | 前方 |
| pcd/back | 后方 |
| pcd/left | 左侧 |
| pcd/right | 右侧 |
| pcd/middle | 中间 |
