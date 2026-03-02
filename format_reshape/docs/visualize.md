# 格式转换后的点云图像投影验证

## 目标

验证从速腾 (Robosense) 格式转换到目标格式 (YAML/PCD) 后，点云到图像的投影是否正确。确保转换后的外参 (`r_s2b`, `t_s2b`)、内参 (`fx`, `fy`, `cx`, `cy`, `kc2-kc5`) 以及数据映射关系 (`result.json`) 是准确的。

---

## 数据来源

- **基准路径 (Base Directory)**: 转换输出目录，例如 `/home/geely/Documents/sunyafei/0203select_output/605/2025-11-21-16-15-45/`

- **点云文件 (PCD)**: `sensor_data/lidar/lidar_concat/*.pcd`

- **图像文件 (Images)**: `sensor_data/camera/camera_*/`

- **标定文件 (Calibration)**:
  - 雷达: `calibration/lidar/lidar.yaml` (包含 `r_s2b`, `t_s2b`)
  - 相机: `calibration/camera/*.yaml` (包含 `r_s2b`, `t_s2b`, `fx`, `fy`, `cx`, `cy`, `kc2-5`, `is_fisheye`)

- **映射关系 (Mapping)**: `node_output/peral-dataproc/result.json`

---

## 核心转换逻辑

与速腾原始数据不同，目标格式提供的是 Rodrigues 旋转向量：

1. **构建相机变换矩阵**: $T_{cam2b} = \text{build\_4x4}(\text{Rodrigues}(r\_s2b_{cam}), t\_s2b_{cam})$
2. **计算投影矩阵**: $T_{points2cam} = (T_{cam2b})^{-1}$ (因为 PCD 已在 Body FLU 系)
3. **投影点云**: $P_{cam} = T_{points2cam} \cdot P_{body}$

---

## 投影模型

- **鱼眼相机 (Fisheye)**: 使用 OpenCV 的 `cv2.fisheye.projectPoints` (Equidistant)
- **针孔相机 (Pinhole)**: 直接应用内参矩阵进行透视投影

---

## 验证标准

1. **可视性**: 投影出的点云应准确叠加在图像中的对应物体上
2. **颜色映射**: 根据深度进行伪彩色编码（参考深度映射算法）
3. **绘制风格**: 使用 `MARKER_CROSS` 风格标记投影点

---

## 工具实现

### 脚本路径

`/home/geely/Documents/sunyafei/rs/format_reshape/scripts/project_calib.py`

### 输出路径

数据目录下的 `visualize/` 文件夹，自动生成在转换流程的最后一步。

### 输出结构

```
visualize/
├── <cam_front_right_timestamp>/    # 使用 cam_front_right 相机时间戳
│   ├── camera_front_far.jpeg
│   ├── camera_front_fisheye.jpeg
│   ├── camera_front_wide.jpeg
│   ├── camera_left_fisheye.jpeg
│   ├── camera_left_front.jpeg
│   ├── camera_left_rear.jpeg
│   ├── camera_rear_fisheye.jpeg
│   ├── camera_rear_mid.jpeg
│   ├── camera_right_fisheye.jpeg
│   ├── camera_right_front.jpeg
│   └── camera_right_rear.jpeg
```

---

## 文件夹命名规则

### 使用 cam_front_right 时间戳

每个 visualize 子文件夹使用 **cam_front_right 相机的时间戳** 命名，而非 PCD 时间戳。

**原因**：
- 便于与相机图像直接对齐查看
- 相机时间戳与人眼观察直觉一致

**示例**：
```
sync_sensors.txt:
  PCD 时间戳: 1764310903499619840
  cam_front_right 时间戳: 1764310903495000000

文件夹命名:
  visualize/1764310903495000000/  (使用相机时间戳)
```

---

## 生成控制

### 默认行为

- **默认生成所有帧**: 不限制帧数，处理 result.json 中的所有映射
- 由主转换流程自动调用，无需手动运行

### 手动运行参数

```bash
# 生成所有帧（默认）
python3 scripts/project_calib.py \
  --data-dir /path/to/output \
  --output-dir visualize

# 限制帧数（用于快速测试）
python3 scripts/project_calib.py \
  --data-dir /path/to/output \
  --output-dir visualize \
  --max-frames 10

# 点云下采样（减少计算量）
python3 scripts/project_calib.py \
  --data-dir /path/to/output \
  --output-dir visualize \
  --subsample 50000
```

---

## 注意事项

1. **相机名称映射**: `camera_front_right` → `camera_front_wide` 文件夹（查找标定文件时）

2. **投影过滤**:
   - 需过滤相机后方点 ($z \le 0$)
   - 鱼眼相机深度限制：50m
   - 普通相机深度限制：150m

3. **性能优化**:
   - 可通过 `--subsample` 参数调整点云采样数量
   - 可通过 `--max-frames` 参数限制生成的帧数

4. **深度编码**:
   - 深度值归一化到 [0, 255] 范围
   - 使用伪彩色映射显示点云距离
