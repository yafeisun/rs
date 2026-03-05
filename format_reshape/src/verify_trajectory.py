"""
轨迹方向验证工具 - 验证车辆X轴（车头方向）是否与轨迹方向一致
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import os
import math


def quat_to_rotation_matrix(w, x, y, z):
    """从四元数 [w, x, y, z] 计算旋转矩阵"""
    R = np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )
    return R


def verify_trajectory_alignment(pose_file, output_dir):
    """
    验证车辆方向与轨迹方向的对齐情况

    Args:
        pose_file: egopose_opt.json 文件路径
        output_dir: 输出目录
    """
    # 读取 pose 数据
    with open(pose_file, "r") as f:
        poses = json.load(f)

    n = len(poses)
    if n < 3:
        print("警告: 轨迹点太少，无法进行有效验证")
        return

    # 提取位置和姿态数据
    x = np.array([p["position"]["position_local"]["x"] for p in poses])
    y = np.array([p["position"]["position_local"]["y"] for p in poses])
    z = np.array([p["position"]["position_local"]["z"] for p in poses])
    qw = np.array([p["orientation"]["quaternion_local"]["w"] for p in poses])
    qx = np.array([p["orientation"]["quaternion_local"]["x"] for p in poses])
    qy = np.array([p["orientation"]["quaternion_local"]["y"] for p in poses])
    qz = np.array([p["orientation"]["quaternion_local"]["z"] for p in poses])

    # 计算轨迹切线方向（使用前后5个点平滑）
    window = min(5, n // 10)  # 窗口大小，避免太宽
    alignment_errors = []
    body_x_angles = []
    trajectory_angles = []

    for i in range(n):
        # 计算轨迹切线方向（使用中心差分法）
        idx_start = max(0, i - window)
        idx_end = min(n - 1, i + window)

        if idx_end > idx_start:
            traj_dx = x[idx_end] - x[idx_start]
            traj_dy = y[idx_end] - y[idx_start]
            traj_angle = math.degrees(math.atan2(traj_dy, traj_dx))
        else:
            traj_angle = 0

        trajectory_angles.append(traj_angle)

        # 计算车身X轴方向
        R = quat_to_rotation_matrix(qw[i], qx[i], qy[i], qz[i])
        body_x = R[:, 0]  # X轴在世界坐标系中
        body_x_angle = math.degrees(math.atan2(body_x[1], body_x[0]))
        body_x_angles.append(body_x_angle)

        # 计算对齐误差（角度差的绝对值，考虑周期性）
        angle_diff = abs(body_x_angle - traj_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        alignment_errors.append(angle_diff)

    # 统计分析
    alignment_errors = np.array(alignment_errors)
    max_error = np.max(alignment_errors)
    min_error = np.min(alignment_errors)
    mean_error = np.mean(alignment_errors)
    std_error = np.std(alignment_errors)
    median_error = np.median(alignment_errors)
    p95_error = np.percentile(alignment_errors, 95)
    p99_error = np.percentile(alignment_errors, 99)

    # 统计各个误差范围的占比
    in_1deg = np.sum(alignment_errors < 1) / n * 100
    in_3deg = np.sum(alignment_errors < 3) / n * 100
    in_5deg = np.sum(alignment_errors < 5) / n * 100
    in_10deg = np.sum(alignment_errors < 10) / n * 100
    in_15deg = np.sum(alignment_errors < 15) / n * 100
    in_30deg = np.sum(alignment_errors < 30) / n * 100

    # 打印验证结果
    print("=" * 70)
    print("车辆轨迹方向验证报告")
    print("=" * 70)
    print(f"轨迹点数: {n}")
    print(
        f"轨迹范围: X({x.min():.2f} ~ {x.max():.2f}m), Y({y.min():.2f} ~ {y.max():.2f}m)"
    )
    print()
    print("对齐误差统计:")
    print(f"  最小误差: {min_error:.2f}°")
    print(f"  最大误差: {max_error:.2f}°")
    print(f"  平均误差: {mean_error:.2f}°")
    print(f"  标准差: {std_error:.2f}°")
    print(f"  中位数: {median_error:.2f}°")
    print(f"  95%点误差: {p95_error:.2f}°")
    print(f"  99%点误差: {p99_error:.2f}°")
    print()
    print("误差分布:")
    print(f"  误差 < 1°  : {in_1deg:.1f}% ({int(np.sum(alignment_errors < 1))} 个点)")
    print(f"  误差 < 3°  : {in_3deg:.1f}% ({int(np.sum(alignment_errors < 3))} 个点)")
    print(f"  误差 < 5°  : {in_5deg:.1f}% ({int(np.sum(alignment_errors < 5))} 个点)")
    print(f"  误差 < 10° : {in_10deg:.1f}% ({int(np.sum(alignment_errors < 10))} 个点)")
    print(f"  误差 < 15° : {in_15deg:.1f}% ({int(np.sum(alignment_errors < 15))} 个点)")
    print(f"  误差 < 30° : {in_30deg:.1f}% ({int(np.sum(alignment_errors < 30))} 个点)")
    print()

    # 结论
    print("验证结论:")
    if mean_error < 5 and p95_error < 10:
        print("  ✅ 优秀: 车辆方向与轨迹方向高度一致，对齐质量很好")
    elif mean_error < 10 and p95_error < 15:
        print("  ✔️ 良好: 车辆方向与轨迹方向基本一致，对齐质量良好")
    elif mean_error < 20 and p95_error < 30:
        print("  ⚠️  一般: 车辆方向与轨迹方向存在一定偏差，可接受")
    else:
        print("  ❌ 较差: 车辆方向与轨迹方向偏差较大，需要检查")
    print()

    # 生成验证图表
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle("车辆轨迹方向验证", fontsize=16, fontweight="bold", y=0.995)

    # 子图1: 轨迹图（带车身方向箭头）
    ax1 = axes[0, 0]
    ax1.plot(x, y, "b-", linewidth=1, alpha=0.5, label="Trajectory")
    ax1.scatter(
        x[0],
        y[0],
        c="lime",
        s=100,
        marker="o",
        zorder=5,
        edgecolors="black",
        linewidth=1.5,
        label="Start",
    )
    ax1.scatter(
        x[-1],
        y[-1],
        c="red",
        s=100,
        marker="s",
        zorder=5,
        edgecolors="black",
        linewidth=1.5,
        label="End",
    )

    # 每20个点绘制一个车身X轴方向
    step = max(1, n // 20)
    for i in range(10, n - 10, step):
        R = quat_to_rotation_matrix(qw[i], qx[i], qy[i], qz[i])
        body_x = R[:2, 0]  # 只取XY平面
        # 归一化并缩放
        scale = 3
        body_x = body_x / np.linalg.norm(body_x[:2]) * scale
        ax1.arrow(
            x[i],
            y[i],
            body_x[0],
            body_x[1],
            head_width=0.7,
            head_length=0.4,
            fc="r",
            ec="r",
            alpha=0.6,
            linewidth=1.5,
            zorder=3,
        )

    ax1.set_xlabel("X Forward (m)")
    ax1.set_ylabel("Y Left (m)")
    ax1.set_title("Vehicle Trajectory with Body X-axis (车头方向)")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")

    # 子图2: 对齐误差曲线
    ax2 = axes[0, 1]
    frame_indices = np.arange(n)
    ax2.plot(
        frame_indices,
        alignment_errors,
        "g-",
        linewidth=1,
        alpha=0.7,
        label="Alignment Error",
    )
    ax2.axhline(y=5, color="orange", linestyle="--", linewidth=2, label="5° threshold")
    ax2.axhline(
        y=mean_error,
        color="red",
        linestyle="--",
        linewidth=2,
        label=f"Mean: {mean_error:.2f}°",
    )
    ax2.axhline(y=0, color="black", linestyle="-", linewidth=0.5, alpha=0.3)
    ax2.set_xlabel("Frame Index")
    ax2.set_ylabel("Alignment Error (degrees)")
    ax2.set_title("Alignment Error along Trajectory")
    ax2.legend(loc="upper left")
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0, min(90, max_error * 1.2))

    # 子图3: 误差直方图
    ax3 = axes[1, 0]
    bins = np.linspace(0, min(90, max_error + 5), 30)
    ax3.hist(alignment_errors, bins=bins, edgecolor="black", alpha=0.7, color="skyblue")
    ax3.axvline(
        x=mean_error,
        color="red",
        linestyle="--",
        linewidth=2,
        label=f"Mean: {mean_error:.2f}°",
    )
    ax3.axvline(x=5, color="orange", linestyle="--", linewidth=2, label="5° threshold")
    ax3.axvline(
        x=p95_error,
        color="purple",
        linestyle="--",
        linewidth=2,
        label=f"95%: {p95_error:.2f}°",
    )
    ax3.set_xlabel("Alignment Error (degrees)")
    ax3.set_ylabel("Frequency")
    ax3.set_title("Error Distribution")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3, axis="y")

    # 子图4: 统计汇总
    ax4 = axes[1, 1]
    ax4.axis("off")
    stats_text = f"""
    轨迹验证统计汇总
    ────────────────────
    轨迹点数: {n}
    
    误差统计:
    •  最小: {min_error:.2f}°
    •  最大: {max_error:.2f}°
    •  平均: {mean_error:.2f}°
    •  标准差: {std_error:.2f}°
    •  中位数: {median_error:.2f}°
    •  95%点: {p95_error:.2f}°
    
    误差分布:
    •  < 1°  : {in_1deg:.1f}%
    •  < 3°  : {in_3deg:.1f}%
    •  < 5°  : {in_5deg:.1f}%
    •  < 10° : {in_10deg:.1f}%
    •  < 15° : {in_15deg:.1f}%
    •  < 30° : {in_30deg:.1f}%
    """
    ax4.text(
        0.1,
        0.9,
        stats_text,
        transform=ax4.transAxes,
        fontsize=12,
        verticalalignment="top",
        family="monospace",
        bbox=dict(
            boxstyle="round", facecolor="lightyellow", alpha=0.8, edgecolor="navy"
        ),
    )

    # 添加结论
    conclusion = (
        "✅ 优秀"
        if mean_error < 5 and p95_error < 10
        else "✔️ 良好"
        if mean_error < 10 and p95_error < 15
        else "⚠️  一般"
        if mean_error < 20 and p95_error < 30
        else "❌ 较差"
    )
    ax4.text(
        0.1,
        0.45,
        f"验证结论: {conclusion}\n"
        + f"平均误差 {mean_error:.2f}°, 95%点误差 {p95_error:.2f}°",
        transform=ax4.transAxes,
        fontsize=13,
        fontweight="bold",
        bbox=dict(
            boxstyle="round",
            facecolor="lightblue" if mean_error < 10 else "lightcoral",
            alpha=0.8,
            edgecolor="navy",
            linewidth=2,
        ),
    )

    plt.tight_layout()

    # 保存图像
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "trajectory_verification.png")
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"✅ 验证图表已保存: {output_path}")

    plt.close()

    # 生成详细数据报告
    report_path = os.path.join(output_dir, "trajectory_verification_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("=" * 70 + "\n")
        f.write("车辆轨迹方向验证报告\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"数据文件: {pose_file}\n")
        f.write(f"轨迹点数: {n}\n")
        f.write(f"轨迹范围:\n")
        f.write(
            f"  X: {x.min():.2f}m ~ {x.max():.2f}m (跨度: {x.max() - x.min():.2f}m)\n"
        )
        f.write(
            f"  Y: {y.min():.2f}m ~ {y.max():.2f}m (跨度: {y.max() - y.min():.2f}m)\n"
        )
        f.write(
            f"  Z: {z.min():.2f}m ~ {z.max():.2f}m (跨度: {z.max() - z.min():.2f}m)\n\n"
        )
        f.write("=" * 70 + "\n")
        f.write("对齐误差统计\n")
        f.write("=" * 70 + "\n")
        f.write(f"最小误差: {min_error:.4f}°\n")
        f.write(f"最大误差: {max_error:.4f}°\n")
        f.write(f"平均误差: {mean_error:.4f}°\n")
        f.write(f"标准差: {std_error:.4f}°\n")
        f.write(f"中位数: {median_error:.4f}°\n")
        f.write(f"95%误差: {p95_error:.4f}°\n")
        f.write(f"99%误差: {p99_error:.4f}°\n\n")
        f.write("=" * 70 + "\n")
        f.write("误差分布\n")
        f.write("=" * 70 + "\n")
        f.write(f"< 1° : {in_1deg:.2f}% ({int(np.sum(alignment_errors < 1))} 个点)\n")
        f.write(f"< 3° : {in_3deg:.2f}% ({int(np.sum(alignment_errors < 3))} 个点)\n")
        f.write(f"< 5° : {in_5deg:.2f}% ({int(np.sum(alignment_errors < 5))} 个点)\n")
        f.write(f"< 10°: {in_10deg:.2f}% ({int(np.sum(alignment_errors < 10))} 个点)\n")
        f.write(f"< 15°: {in_15deg:.2f}% ({int(np.sum(alignment_errors < 15))} 个点)\n")
        f.write(
            f"< 30°: {in_30deg:.2f}% ({int(np.sum(alignment_errors < 30))} 个点)\n\n"
        )
        f.write("=" * 70 + "\n")
        f.write("详细数据（前20个点）\n")
        f.write("=" * 70 + "\n")
        f.write(f"{'Index':<8} {'TrajAngle':<12} {'BodyXAngle':<12} {'Error(°)':<10}\n")
        f.write("-" * 50 + "\n")
        for i in range(min(20, n)):
            f.write(
                f"{i:<8} {trajectory_angles[i]:>10.2f}    {body_x_angles[i]:>10.2f}    {alignment_errors[i]:>8.2f}\n"
            )

    print(f"✅ 验证报告已保存: {report_path}")
    print("=" * 70)

    return {
        "mean_error": mean_error,
        "max_error": max_error,
        "p95_error": p95_error,
        "in_5deg": in_5deg,
        "in_10deg": in_10deg,
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="验证车辆轨迹方向与车头方向的对齐情况")
    parser.add_argument("pose_file", type=str, help="egopose_opt.json 文件路径")
    parser.add_argument("--output-dir", type=str, default=None, help="输出目录")
    args = parser.parse_args()

    if args.output_dir is None:
        # 使用 pose 文件所在目录的 visualize 子目录
        pose_dir = os.path.dirname(args.pose_file)
        output_dir = os.path.join(os.path.dirname(pose_dir), "visualize")
    else:
        output_dir = args.output_dir

    print("\n" + "=" * 70)
    print("开始验证车辆轨迹方向")
    print("=" * 70)
    print(f"Pose 文件: {args.pose_file}")
    print(f"输出目录: {output_dir}")
    print()

    verify_trajectory_alignment(args.pose_file, output_dir)

    print("\n" + "=" * 70)
    print("✅ 验证完成!")
    print("=" * 70)
