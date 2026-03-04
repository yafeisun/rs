# 格式转换后的点云图像投影验证

## 目标

验证从速腾 (Robosense) 格式转换到目标格式 (YAML/PCD) 后，点云到图像的投影是否正确。确保转换后的外参 (`r_s2b`, `t_s2b`)、内参 (`fx`, `fy`, `cx`, `cy`, `kc2-kc5`) 以及数据映射关系 (`result.json`) 是准确的。

---

## 坐标系说明（重要）

理解两套数据的坐标系差异是正确投影的前提。

### 速腾原始数据（pipeline/process_data.py）

| 数据 | 坐标系 |
|------|--------|
| `result/test_calibration/middle/*.pcd` | **Body RFU**（X右，Y前，Z上） |
| `calib.json` 中的 `extr` | **body_RFU → camera**（直接从 Body RFU 投影） |

速腾投影链：
```
PCD (Body RFU) → [extr_body_RFU_to_cam] → 相机坐标系 → 像素
```

### 转换后数据（format_reshape 输出）

| 数据 | 坐标系 |
|------|--------|
| `sensor_data/lidar/lidar_undist/*.pcd` | **Body RFU**（X右，Y前，Z上）⚠️ |
| `calib_anno/*.json` 中的 `extrinsic` | **camera → Body FLU**（经过 RFU→FLU 旋转转换） |
| `calib_anno_vc/*.json` 中的 `extrinsic` | **camera → Body FLU**（同上） |

> ⚠️ **关键说明**：`lidar_undist` 里的 PCD 直接拷贝自速腾的
> `result/test_calibration/middle/*.pcd`，该文件是 Body **RFU** 坐标系，
> 而非 Body FLU。`extractors.py` 中的注释"已是 Body FLU"是错误的。

转换后投影链：
```
PCD (Body RFU) → [R_rfu2flu] → PCD (Body FLU) → [inv(extrinsic_cam_to_FLU)] → 相机坐标系 → 像素
```

### 数学等价性验证

设 `R_rfu2flu = [[0,1,0],[-1,0,0],[0,0,1]]`，则：

```
inv(extrinsic_cam_to_FLU) @ R_rfu2flu == calib.json["extr"]  (body_RFU_to_cam)
```

这一等式经过数值验证完全成立（误差 < 1e-10），说明两套参数体系在数学上完全等价，只需在投影前对点云做坐标系转换即可。

---

## 数据来源

- **基准路径 (Base Directory)**: 转换输出目录，例如 `/path/to/0203select_output/605/2025-11-21-16-15-45/`
- **点云文件 (PCD)**: `sensor_data/lidar/lidar_undist/*.pcd`（Body RFU 坐标系）
- **图像文件 (Images)**: `sensor_data/camera/camera_*/`
- **标定文件来源和用途**:
  - **calib_anno（鱼眼相机用）**: `calib_anno/*.json` — extrinsic 为 camera→Body FLU，保留原始畸变系数
  - **calib_anno_vc（普通相机用）**: `calib_anno_vc/*.json` — extrinsic 为 camera→Body FLU，畸变系数清零
  - 原始 YAML: `calibration/camera/*.yaml` — 含 `is_fisheye`、分辨率等元信息
- **映射关系 (Mapping)**: `node_output/peral-dataproc/result.json`

---

## 投影实现

### 标定参数使用规则

| 相机类型 | 数量 | 投影函数 | 标定参数来源 | 畸变处理 |
|---------|------|---------|------------|---------|
| **鱼眼相机** | 4 | `cv2.fisheye.projectPoints` | `calib_anno/*.json` | 使用原始 kc2-5 |
| **普通相机** | 7 | 标准透视投影 | `calib_anno_vc/*.json` | distcoeff = 0 |

### 投影流程（`project_calib.py`）

```python
# 1. 加载点云（Body RFU 坐标系）
points = load_pcd(pcd_path)  # shape: (N, 3)

# 2. 过滤无效点
flag = (|x| <= 500) & (|y| <= 500) & (|z| <= 50)
points = points[flag]

# 3. ⚠️ 坐标系转换：Body RFU → Body FLU
#    R_rfu2flu = [[0,1,0],[-1,0,0],[0,0,1]]
#    x_flu =  y_rfu  (前方)
#    y_flu = -x_rfu  (左方)
#    z_flu =  z_rfu  (上方，不变)
R_rfu2flu = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
points = (R_rfu2flu @ points.T).T

# 4. 计算外参 body_FLU → camera
extrinsic = calib_anno["extrinsic"]  # camera → body FLU (4x4)
extr_b2c = inv(extrinsic)            # body FLU → camera

# 5. 投影（鱼眼 or 普通）
if is_fisheye:
    pixels = cv2.fisheye.projectPoints(points, rvec, tvec, K, D)
else:
    cam_coords = extr_b2c @ [points; 1]
    pixels = K @ cam_coords
```

### 深度过滤与颜色映射

- 深度过滤上限：全局 `SHOW_DEPTH = 150`（米）
- 颜色归一化：`zzz_norm = depth / show_depth * 255`
  - 普通相机：`show_depth = 150`
  - 鱼眼相机：`show_depth = 50`
- 颜色编码：伪彩色（近红→远蓝），参考速腾 `process_data.py` 的 `get_color()`

---

## 工具

### 脚本路径

```
format_reshape/scripts/project_calib.py
```

### 手动运行

```bash
# 生成所有帧（默认）
python3 format_reshape/scripts/project_calib.py \
  --data-dir /path/to/output_dir \
  --output-dir visualize

# 限制帧数（快速验证）
python3 format_reshape/scripts/project_calib.py \
  --data-dir /path/to/output_dir \
  --output-dir visualize \
  --max-frames 5
```

### 输出结构

```
visualize/
└── <camera_front_wide_timestamp>/
    ├── camera_front_far.jpeg
    ├── camera_front_fisheye.jpeg
    ├── camera_front_wide.jpeg
    ├── camera_left_fisheye.jpeg
    ├── camera_left_front.jpeg
    ├── camera_left_rear.jpeg
    ├── camera_rear_fisheye.jpeg
    ├── camera_rear_mid.jpeg
    ├── camera_right_fisheye.jpeg
    ├── camera_right_front.jpeg
    └── camera_right_rear.jpeg
```

子文件夹以 **camera_front_wide 的相机时间戳** 命名（来自 result.json 中的映射）。

---

## 速腾参考对比

速腾原始投影工具：
- 脚本：`pipeline/sensor_sync_data/process_data.py`
- 标定：`<bag_dir>/calib.json`（`CAM__front`、`CAM__a_front` 等键）
- 点云：clip 数据集中 `<frame>/<frame>.pcd`（Body RFU 坐标系）

速腾 `calib.json` 生成方式：
- 来源：`car_*.yaml` 中各相机的 `CameraExt`（roll/pitch/yaw，RFU body 系）
- 工具：`pipeline/sensor_sync_data/convert_yaml_2_json.py`
- extr 含义：**body_RFU → camera**（与 format_reshape 转换的方向和坐标系均不同）

---

## 注意事项

1. **点云坐标系**：`lidar_undist` PCD 是 Body RFU，投影前必须乘以 `R_rfu2flu`，否则点云会投影到错误位置（与实际物体有明显偏移）。

2. **外参方向**：`calib_anno` 存的是 camera→body（正向），投影时需取逆得到 body→camera。速腾 `calib.json` 存的是 body→camera（反向），可直接使用。

3. **鱼眼相机判断**：通过 `calibration/camera/*.yaml` 中的 `is_fisheye` 字段或相机名含 `fisheye` 判断，鱼眼相机使用 `calib_anno`，普通相机使用 `calib_anno_vc`。
