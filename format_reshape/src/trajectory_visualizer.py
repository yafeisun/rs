"""
轨迹可视化工具 - 车辆轨迹和车体坐标系可视化（俯视图，含第一帧PCD）
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import os
import math


def quat_to_rotation_matrix(w, x, y, z):
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)    ],
        [2*(x*y + z*w),        1 - 2*(x**2 + z**2), 2*(y*z - x*w)   ],
        [2*(x*z - y*w),        2*(y*z + x*w),     1 - 2*(x**2 + y**2)],
    ])
    return R


def load_pcd_xy(pcd_file, z_min=-3.0, z_max=8.0, T_world2local=None):
    """
    加载 PCD（在世界坐标系下），变换到局部坐标系后过滤高度范围。
    T_world2local: 4x4变换矩阵，将世界坐标系点变换到局部坐标系（可选）
    返回 (xy, z_vals) 均为 numpy array。
    """
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(pcd_file)
    pts = np.asarray(pcd.points)

    # 过滤 NaN
    valid = ~np.isnan(pts).any(axis=1)
    pts = pts[valid]

    # PCD坐标系中车头朝-Y，需旋转使车头对齐+X（FLU前向）
    # X_new = -Y_old, Y_new = X_old
    R_pcd2flu = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    pts = (R_pcd2flu @ pts.T).T

    # 变换到局部坐标系
    if T_world2local is not None:
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts = (T_world2local @ pts_h.T).T[:, :3]

    # 只过滤高度范围
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pts = pts[mask]

    return pts[:, :2], pts[:, 2]


def find_pcd_for_first_pose(pose_data, lidar_dir):
    """找到第一帧位姿对应的最近 PCD 文件"""
    if not pose_data:
        return None
    pcd_files = sorted([f for f in os.listdir(lidar_dir) if f.endswith(".pcd")])
    if not pcd_files:
        return None
    first_ts_us = pose_data[0]["meta"]["timestamp_us"]
    first_ts_ns = first_ts_us * 1000
    best = min(pcd_files, key=lambda f: abs(int(f.replace(".pcd", "")) - first_ts_ns))
    return os.path.join(lidar_dir, best)


def visualize_trajectory(pose_file, output_dir, pcd_file=None):
    """
    生成轨迹俯视图：在第一帧 PCD 背景上绘制轨迹和车体坐标系箭头。

    Args:
        pose_file: egopose_opt.json 路径
        output_dir: 输出目录
        pcd_file: 可选，手动指定 PCD 文件路径
    """
    with open(pose_file, "r") as f:
        poses = json.load(f)

    # 自动查找 lidar_undist 目录下第一帧 PCD
    if pcd_file is None:
        pose_dir = os.path.dirname(pose_file)
        lidar_dir = os.path.join(pose_dir, "lidar", "lidar_undist")
        if os.path.exists(lidar_dir):
            pcd_file = find_pcd_for_first_pose(poses, lidar_dir)
            if pcd_file:
                print(f"找到PCD文件: {pcd_file}")
            else:
                print("未找到PCD文件，将不显示点云")
    elif pcd_file and not os.path.exists(pcd_file):
        print(f"PCD文件不存在: {pcd_file}")
        pcd_file = None

    # 提取轨迹位置和四元数
    x  = np.array([p["position"]["position_local"]["x"] for p in poses])
    y  = np.array([p["position"]["position_local"]["y"] for p in poses])
    z  = np.array([p["position"]["position_local"]["z"] for p in poses])
    qw = np.array([p["orientation"]["quaternion_local"]["w"] for p in poses])
    qx = np.array([p["orientation"]["quaternion_local"]["x"] for p in poses])
    qy = np.array([p["orientation"]["quaternion_local"]["y"] for p in poses])
    qz = np.array([p["orientation"]["quaternion_local"]["z"] for p in poses])

    # 统计信息
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()
    traj_angle = math.degrees(math.atan2(y[-1] - y[0], x[-1] - x[0]))
    mid = len(poses) // 2
    R_mid = quat_to_rotation_matrix(qw[mid], qx[mid], qy[mid], qz[mid])
    body_x_angle = math.degrees(math.atan2(R_mid[1, 0], R_mid[0, 0]))
    align_err = abs(body_x_angle - traj_angle)
    if align_err > 180:
        align_err = 360 - align_err

    print(f"轨迹统计:")
    print(f"  点数: {len(x)}")
    print(f"  X 范围: {x_range:.2f}m, Y 范围: {y_range:.2f}m")
    print(f"  轨迹角度: {traj_angle:.1f}°, 车身X轴角度: {body_x_angle:.1f}°")
    print(f"  对齐误差: {align_err:.1f}°")

    # 构建第一帧逆变换：PCD在世界坐标系，轨迹在局部坐标系（以第一帧为原点）
    # T_world2local = inv(T_local2world) = [R0^T, -R0^T * t0; 0, 1]
    R0 = quat_to_rotation_matrix(qw[0], qx[0], qy[0], qz[0])
    t0 = np.array([x[0], y[0], z[0]])
    T_world2local = np.eye(4)
    T_world2local[:3, :3] = R0.T
    T_world2local[:3,  3] = -R0.T @ t0

    # 加载 PCD 并变换到局部坐标系
    pcd_xy = None
    pcd_z  = None
    if pcd_file and os.path.exists(pcd_file):
        try:
            pcd_xy, pcd_z = load_pcd_xy(pcd_file, T_world2local=T_world2local)
            print(f"PCD加载成功，有效点数: {len(pcd_xy)}")
        except Exception as e:
            print(f"PCD加载失败: {e}")

    # ---- 绘图 ----
    fig, ax = plt.subplots(figsize=(16, 12), dpi=150)

    # 1. PCD 背景（按高度着色，小点）
    if pcd_xy is not None and len(pcd_xy) > 0:
        sc = ax.scatter(
            pcd_xy[:, 0], pcd_xy[:, 1],
            c=pcd_z, cmap="viridis",
            s=0.5, alpha=0.6,
            linewidths=0, edgecolors="none",
            zorder=1,
        )
        cbar = plt.colorbar(sc, ax=ax, fraction=0.02, pad=0.01)
        cbar.set_label("Z (m)", fontsize=11)
        cbar.ax.tick_params(labelsize=9)

    # 2. 轨迹线
    ax.plot(x, y, "b-", linewidth=2.0, alpha=0.85, label="Trajectory", zorder=2)

    # 3. 轨迹点（按z高度着色）
    ax.scatter(x, y, c=z, cmap="plasma", s=15, alpha=0.7,
               linewidths=0, edgecolors="none", zorder=3)

    # 4. 起点（绿色圆）/ 终点（红色方块）
    ax.scatter([x[0]],  [y[0]],  c="lime", s=150, marker="o",
               edgecolors="black", linewidth=1.5, zorder=5)
    ax.scatter([x[-1]], [y[-1]], c="red",  s=150, marker="s",
               edgecolors="black", linewidth=1.5, zorder=5)
    ax.annotate("Start", (x[0],  y[0]),  xytext=(5, 5),  textcoords="offset points", fontsize=9)
    ax.annotate("End",   (x[-1], y[-1]), xytext=(5, 5),  textcoords="offset points", fontsize=9)

    # 5. 车体坐标系箭头（每隔10帧一个）
    arrow_len = max(x_range, y_range) * 0.06  # 箭头长度随轨迹范围自适应
    arrow_len = max(arrow_len, 1.0)
    for i in range(0, len(x), max(1, len(x) // 15)):
        R = quat_to_rotation_matrix(qw[i], qx[i], qy[i], qz[i])
        fwd  = R[:, 0]  # X: 向前
        left = R[:, 1]  # Y: 向左
        ax.arrow(x[i], y[i], fwd[0]*arrow_len,  fwd[1]*arrow_len,
                 head_width=arrow_len*0.25, head_length=arrow_len*0.2,
                 fc="red",   ec="red",   linewidth=1.2, alpha=0.85, zorder=4)
        ax.arrow(x[i], y[i], left[0]*arrow_len, left[1]*arrow_len,
                 head_width=arrow_len*0.25, head_length=arrow_len*0.2,
                 fc="green", ec="green", linewidth=1.2, alpha=0.85, zorder=4)

    # 6. 图例和标签
    ax.plot([], [], color="red",   linewidth=3, label="X-axis (Forward)")
    ax.plot([], [], color="green", linewidth=3, label="Y-axis (Left)")
    ax.set_xlabel("X Forward (m)", fontsize=12, fontweight="bold")
    ax.set_ylabel("Y Left (m)",    fontsize=12, fontweight="bold")
    ax.set_title(
        f"Vehicle Trajectory - Top-Down View (First Frame PCD)\n"
        f"Traj: {traj_angle:.1f}°  Body-X: {body_x_angle:.1f}°  Align err: {align_err:.1f}°",
        fontsize=13, fontweight="bold",
    )
    ax.legend(loc="upper left", fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.25, linestyle="--", linewidth=0.6)
    ax.set_aspect("equal")

    # 7. 视图范围：以轨迹为中心，padding = max(轨迹范围, 30m) * 0.3
    pad = max(x_range, y_range, 30.0) * 0.3
    cx, cy = (x.max() + x.min()) / 2, (y.max() + y.min()) / 2
    half = max(x_range, y_range) / 2 + pad
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)

    # 8. 统计文字框
    stats_text = (
        f"Poses: {len(x)}\n"
        f"X span: {x_range:.1f}m\n"
        f"Y span: {y_range:.1f}m\n"
        f"Align: {align_err:.1f}°\n"
        f"PCD pts: {len(pcd_xy) if pcd_xy is not None else 0}"
    )
    ax.text(0.01, 0.99, stats_text, transform=ax.transAxes,
            fontsize=9, verticalalignment="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.85, edgecolor="none"))

    # 保存
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "trajectory_visualization.png")
    plt.savefig(output_path, dpi=200, bbox_inches="tight",
                facecolor="white", edgecolor="none")
    print(f"轨迹图已保存: {output_path}")
    plt.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="车辆轨迹俯视图可视化")
    parser.add_argument("pose_file", type=str, help="egopose_opt.json 路径")
    parser.add_argument("--pcd", type=str, default=None, help="PCD文件路径（可选）")
    parser.add_argument("--output-dir", type=str, default=None, help="输出目录")
    args = parser.parse_args()

    if args.output_dir is None:
        pose_dir = os.path.dirname(args.pose_file)
        output_dir = os.path.join(os.path.dirname(pose_dir), "visualize")
    else:
        output_dir = args.output_dir

    visualize_trajectory(args.pose_file, output_dir, args.pcd)
