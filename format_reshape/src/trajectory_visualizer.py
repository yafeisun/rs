"""
轨迹可视化工具 - 车辆轨迹和车体坐标系可视化
俯视图版本（含第一帧PCD）
每个轨迹点只标记一个点, 文字较小, 不遮挡
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import open3d as o3d
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.axes_grid1 import make_axes_locatable


def quat_to_rotation_matrix(w, x, y, z):
    """
    从四元数 [w, x, y, z] 计算旋转矩阵
    """
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )
    return R


def load_pcd(pcd_file):
    """
    加载并可视化点云数据

    Args:
        pcd_file: PCD文件路径

    Returns:
        tuple: (xy_points, full_points) - XY平面点和完整点云
    """
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)

    # 只保留XY平面上的点（俯视图）
    xy_points = points[:, :2]  # 只取X和Y坐标

    return xy_points, points


def find_pcd_for_first_pose(pose_data, lidar_dir):
    """
    找到第一帧位姿对应的PCD文件

    Args:
        pose_data: 位姿数据列表
        lidar_dir: lidar目录路径

    Returns:
        str: PCD文件完整路径
    """
    if not pose_data:
        return None

    # 第一帧位姿的时间戳（微秒）
    first_timestamp_us = pose_data[0]["meta"]["timestamp_us"]
    # 转换为纳秒
    first_timestamp_ns = first_timestamp_us * 1000

    # 查找最接近的PCD文件
    pcd_files = sorted([f for f in os.listdir(lidar_dir) if f.endswith(".pcd")])
    if not pcd_files:
        return None

    # 找到最接近的PCD文件
    for pcd_file in pcd_files:
        pcd_timestamp_ns = int(pcd_file.replace(".pcd", ""))
        if abs(pcd_timestamp_ns - first_timestamp_ns) < 1000000:  # 1ms容差
            return os.path.join(lidar_dir, pcd_file)

    return pcd_files[0]  # 如果找不到精确匹配，返回第一个


def visualize_trajectory(pose_file, output_dir, pcd_file=None):
    """
    可视化轨迹和车体坐标系 - 俯视图版本（含第一帧PCD）

    Args:
        pose_file: egopose_opt.json 文件路径
        output_dir: 输出图像保存目录
        pcd_file: PCD文件路径（可选，如果未指定则自动查找）
    """
    # 读取 pose 数据
    with open(pose_file, "r") as f:
        poses = json.load(f)

    # 自动查找PCD文件（如果未指定）
    if pcd_file is None:
        pose_dir = os.path.dirname(pose_file)
        lidar_dir = os.path.join(pose_dir, "lidar", "lidar_fr")
        if os.path.exists(lidar_dir):
            pcd_file = find_pcd_for_first_pose(poses, lidar_dir)
            if pcd_file:
                print(f"找到PCD文件: {pcd_file}")
            else:
                print("⚠️  未找到PCD文件，将不显示点云")
    elif not os.path.exists(pcd_file):
        print(f"⚠️  PCD文件不存在: {pcd_file}")
        pcd_file = None

    # 提取位置数据
    x = np.array([p["position"]["position_local"]["x"] for p in poses])
    y = np.array([p["position"]["position_local"]["y"] for p in poses])
    z = np.array([p["position"]["position_local"]["z"] for p in poses])

    # 提取四元数
    qw = np.array([p["orientation"]["quaternion_local"]["w"] for p in poses])
    qx = np.array([p["orientation"]["quaternion_local"]["x"] for p in poses])
    qy = np.array([p["orientation"]["quaternion_local"]["y"] for p in poses])
    qz = np.array([p["orientation"]["quaternion_local"]["z"] for p in poses])

    # 计算统计信息
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()

    # 计算轨迹整体方向
    trajectory_dir = np.array([x[-1] - x[0], y[-1] - y[0]])
    trajectory_angle = math.degrees(math.atan2(trajectory_dir[1], trajectory_dir[0]))

    # 计算中点的车身朝向
    mid_idx = len(poses) // 2
    R_mid = quat_to_rotation_matrix(qw[mid_idx], qx[mid_idx], qy[mid_idx], qz[mid_idx])
    body_x_world = R_mid[:, 0]  # 车身X轴在世界坐标系中（第0列）
    body_x_angle = math.degrees(math.atan2(body_x_world[1], body_x_world[0]))

    print(f"轨迹统计:")
    print(f"  点数: {len(x)}")
    print(f"  X 范围: {x_range:.2f}m, Y 范围: {y_range:.2f}m")
    print(f"  轨迹角度: {trajectory_angle:.1f}°, 车身X轴角度: {body_x_angle:.1f}°")
    print(f"  对齐误差: {abs(body_x_angle - trajectory_angle):.1f}°")

    # 加载PCD数据（如果存在）
    pcd_xy = None
    pcd_full = None
    if pcd_file and os.path.exists(pcd_file):
        try:
            pcd_xy, pcd_full = load_pcd(pcd_file)
            print(f"✅ PCD加载成功，点数: {len(pcd_xy)}")
        except Exception as e:
            print(f"⚠️  PCD加载失败: {e}")
            pcd_xy = None

    # 创建图形（大幅提高分辨率）
    fig, ax = plt.subplots(figsize=(20, 16), dpi=300)

    # 设置高质量渲染
    plt.rcParams["figure.dpi"] = 300
    plt.rcParams["savefig.dpi"] = 300
    plt.rcParams["font.size"] = 14

    # 绘制PCD点云（高精度2D投影）
    scatter_pcd = None
    if pcd_xy is not None:
        # 使用PCD的Z轴信息来着色（按颜色表示高度）
        colors = pcd_full[:, 2]  # 使用Z坐标作为颜色

        # 创建颜色映射（Z轴高度）
        scatter_pcd = ax.scatter(
            pcd_xy[:, 0],
            pcd_xy[:, 1],
            c=colors,
            cmap="viridis",  # 使用viridis颜色映射
            s=0.8,  # 更小的点，显示更多细节
            alpha=0.6,  # 适当透明度
            label="First Frame PCD",
            zorder=1,
            marker="o",
            linewidths=0,
            edgecolors="none",
        )
        print(f"✅ PCD点云已绘制（{len(pcd_xy)}个点，使用Z轴高度着色）")

    # 绘制轨迹线（更粗更明显）
    ax.plot(x, y, "b-", linewidth=1.5, alpha=0.7, label="Trajectory")

    # 绘制所有轨迹点（用小点，按Z轴高度着色）
    scatter = ax.scatter(
        x,
        y,
        c=z,
        cmap="plasma",
        s=20,
        alpha=0.6,
        label="Trajectory Points",
        zorder=2,
        linewidths=0,
        edgecolors="none",
    )

    # 标记起点和终点（稍大的点）
    ax.scatter(
        [x[0]],
        [y[0]],
        c="lime",
        s=200,  # 更大的标记
        marker="o",
        zorder=5,
        edgecolors="black",
        linewidth=2.0,
    )
    ax.scatter(
        [x[-1]],
        [y[-1]],
        c="red",
        s=200,
        marker="s",
        zorder=5,
        edgecolors="black",
        linewidth=2.0,
    )

    # 在起点和终点标注坐标（更清晰的字体）
    ax.text(
        x[0],
        y[0] - 2.0,
        f"Start\n({x[0]:.2f}, {y[0]:.2f})\nZ={z[0]:.3f}",
        ha="center",
        va="top",
        fontsize=9,
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor="lime",
            alpha=0.8,
            edgecolor="black",
            linewidth=1.0,
        ),
    )
    ax.text(
        x[-1],
        y[-1] - 2.0,
        f"End\n({x[-1]:.2f}, {y[-1]:.2f})\nZ={z[-1]:.3f}",
        ha="center",
        va="top",
        fontsize=9,
        bbox=dict(
            boxstyle="round,pad=0.4",
            facecolor="red",
            alpha=0.8,
            edgecolor="black",
            linewidth=1.0,
        ),
    )

    # 每隔10个点绘制一个车身坐标系
    for i in range(5, len(x) - 5, 10):
        R = quat_to_rotation_matrix(qw[i], qx[i], qy[i], qz[i])

        # 车身坐标轴在世界坐标系下的表示（R的第k列）
        forward = R[:, 0]  # X轴向前
        left = R[:, 1]  # Y轴向左

        # 绘制X轴（红色，向前）- 更清晰的箭头
        ax.arrow(
            x[i],
            y[i],
            forward[0] * 2.0,  # 箭头长度更短
            forward[1] * 2.0,
            head_width=0.35,  # 稍大的箭头
            head_length=0.25,
            fc="#FF0000",
            ec="#FF0000",
            linewidth=2.0,
            alpha=0.9,
            zorder=4,
        )

        # 绘制Y轴（绿色，向左）
        ax.arrow(
            x[i],
            y[i],
            left[0] * 2.0,  # 箭头长度更短
            left[1] * 2.0,
            head_width=0.35,
            head_length=0.25,
            fc="#00FF00",
            ec="#00FF00",
            linewidth=2.0,
            alpha=0.9,
            zorder=4,
        )

    # 添加颜色条显示PCD高度信息
    if scatter_pcd is not None:
        # 使用更安全的方式创建颜色条
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="3%", pad=0.1)
        cbar = plt.colorbar(scatter_pcd, cax=cax)
        cbar.set_label("PCD Z (m)", rotation=90, labelpad=15, fontsize=12)
        cbar.ax.tick_params(labelsize=10)

    # 在起点和终点显示坐标轴图例
    ax.plot([], [], color="#FF0000", linewidth=4, label="X-axis (Forward)", alpha=0.8)
    ax.plot([], [], color="#00FF00", linewidth=4, label="Y-axis (Left)", alpha=0.8)

    # 设置坐标轴（更大字体）
    ax.set_xlabel("X Forward (m)", fontsize=14, fontweight="bold")
    ax.set_ylabel("Y Left (m)", fontsize=14, fontweight="bold")
    ax.set_title(
        f"Vehicle Trajectory - Top-Down High-Resolution View (with First Frame PCD)\n"
        f"(Trajectory: {trajectory_angle:.1f}°, Body X: {body_x_angle:.1f}°, Align: {abs(body_x_angle - trajectory_angle):.1f}°)",
        fontsize=16,
        fontweight="bold",
        pad=20,
    )

    ax.legend(loc="best", fontsize=11, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle="--", linewidth=0.8)
    ax.axis("equal")

    # 添加统计信息（适度字体）
    stats_text = (
        f"Points: {len(x)}\n"
        f"X: {x_range:.2f}m, Y: {y_range:.2f}m\n"
        f"Align error: {abs(body_x_angle - trajectory_angle):.2f}°\n"
        f"PCD: {len(pcd_xy) if pcd_xy is not None else 0} pts"
    )

    ax.text(
        0.02,
        0.98,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        fontfamily="monospace",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8, edgecolor="none"),
    )

    # 保存图像（高质量）
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "trajectory_visualization.png")
    plt.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.1,
        facecolor="white",
        edgecolor="none",
    )
    print(f"✅ 轨迹图已保存（高分辨率300 DPI）: {output_path}")

    # 同时保存一个无压缩的超高质量版本
    output_path_high = os.path.join(output_dir, "trajectory_visualization_hq.png")
    plt.savefig(
        output_path_high,
        dpi=450,
        bbox_inches="tight",
        pad_inches=0.1,
        facecolor="white",
        edgecolor="none",
    )
    print(f"✅ 轨迹图已保存（超高分辨率450 DPI）: {output_path_high}")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "trajectory_visualization.png")
    plt.savefig(

        output_path,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.1,
        facecolor="white",
        edgecolor="none",
    )
    print(f"✅ 轨迹图已保存（高分辨率300 DPI）: {output_path}")

    # 同时保存一个无压缩的超高质量版本
    output_path_high = os.path.join(output_dir, "trajectory_visualization_hq.png")
    plt.savefig(
        output_path_high,
        dpi=450,
        bbox_inches="tight",
        pad_inches=0.1,
        facecolor="white",
        edgecolor="none",
        compression=0,  # 无压缩（PNG格式）
    )
    print(f"✅ 轨迹图已保存（超高分辨率450 DPI，无压缩）: {output_path_high}")

    plt.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="可视化行车轨迹和车体坐标系（高精度版本）"
    )
    parser.add_argument("pose_file", type=str, help="egopose_opt.json 文件路径")
    parser.add_argument(
        "--pcd", type=str, default=None, help="第一帧PCD文件路径（可选，默认自动查找）"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="输出目录 (默认: pose文件所在目录的visualize)",
    )
    args = parser.parse_args()

    if args.output_dir is None:
        # 使用 pose 文件所在目录的 visualize 子目录
        pose_dir = os.path.dirname(args.pose_file)
        output_dir = os.path.join(os.path.dirname(pose_dir), "visualize")
    else:
        output_dir = args.output_dir

    print("\n" + "=" * 70)
    print("生成轨迹图（俯视图高精度版本，含第一帧PCD）")
    print("=" * 70)
    print(f"Pose 文件: {args.pose_file}")
    print(f"输出目录: {output_dir}")
    if args.pcd:
        print(f"PCD 文件: {args.pcd}")
    print()
    visualize_trajectory(args.pose_file, output_dir, args.pcd)
    print("\n" + "=" * 70)
    print("✅ 完成!")
    print("=" * 70)
