import os
import argparse
from pathlib import Path
from src.cli import run_ipm
from static_zebra_checker.batch_analyzer import analyze_single_bag, load_config, extract_zebra_frame_numbers
from static_zebra_checker.camera_size_checker import check_and_fix_camera_sizes


def gen_static_ipm(bag_dir, only_shape=False):
    """
    生成静止斑马线时段的 IPM 图像
    
    Args:
        bag_dir: bag 文件所在目录
        only_shape: 是否只处理形状，不处理加速度
    """
    # 加载配置
    config_path = Path(__file__).parent / 'static_zebra_checker' / 'config.yaml'
    config = load_config(str(config_path))
    
    # 分析 bag 文件
    print(f"\n{'='*80}")
    print(f"开始分析: {bag_dir}")
    print(f"{'='*80}")
    result = analyze_single_bag(bag_dir, config)
    
    # 提取有斑马线的静止时间段的图片序号
    camera_dir = Path(bag_dir) / config.get('camera', {}).get('camera_dir', 'cam_front_right')
    frm_nums = extract_zebra_frame_numbers(result, str(camera_dir))
    
    if not frm_nums:
        print("\n未找到有斑马线的静止时段")
        return
    
    print(f"\n总共选择了 {len(frm_nums)} 张图片用于生成 IPM")
    print(f"图片序号: {frm_nums}")
    
    # 修复相机尺寸配置
    check_and_fix_camera_sizes(bag_dir, dry_run=False)

    # 生成 IPM 图像
    output_dir = os.path.join(bag_dir, 'ipm')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    run_ipm(bag_dir, tuple(frm_nums), output_dir, only_shape)
    
    print(f"\nIPM 图像已生成到: {output_dir}")


def gen_random_ipm(bag_dir, num_frames=10, only_shape=False):
    """
    生成随机帧的 IPM 图像
    
    Args:
        bag_dir: bag 文件所在目录
        num_frames: 随机选择的帧数量
        only_shape: 是否只处理形状，不处理加速度
    """
    # 修复相机尺寸配置
    check_and_fix_camera_sizes(bag_dir, dry_run=False)
    
    # 使用空列表触发随机选择模式
    output_dir = os.path.join(bag_dir, 'ipm_random')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 传入空元组会让代码随机选择帧
    run_ipm(bag_dir, (), output_dir, only_shape)
    
    print(f"\n随机 IPM 图像已生成到: {output_dir}")


def gen_full_ipm(bag_dir, only_shape=False):
    """
    生成全量 IPM 图像
    
    Args:
        bag_dir: bag 文件所在目录
        only_shape: 是否只处理形状，不处理加速度
    """
    # 修复相机尺寸配置
    check_and_fix_camera_sizes(bag_dir, dry_run=False)
    
    # 计算全部帧范围
    cam_dir = os.path.join(bag_dir, 'cam_front_right')
    if not os.path.exists(cam_dir):
        print(f"错误: 找不到相机目录 {cam_dir}")
        return
    
    # 获取所有帧
    frames = [f for f in os.listdir(cam_dir) if f.endswith('.jpeg')]
    frames.sort()
    total_frames = len(frames)
    
    if total_frames == 0:
        print("错误: 未找到图像文件")
        return
    
    print(f"总帧数: {total_frames}")
    
    # 生成全量 IPM - 使用 (0, total_frames-1) 表示全量范围
    output_dir = os.path.join(bag_dir, 'ipm_full')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 使用 range 模式生成全量IPM
    run_ipm(bag_dir, tuple(range(0, min(total_frames, 100))), output_dir, only_shape)
    
    print(f"\n全量 IPM 图像已生成到: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description='IPM 图像生成工具')
    parser.add_argument('bag_dir', help='bag 文件所在目录')
    parser.add_argument('--mode', '-m', choices=['zebra', 'random', 'full'], default='zebra',
                        help='生成模式: zebra(斑马线静止时段,默认), random(随机帧), full(全量)')
    parser.add_argument('--num-frames', '-n', type=int, default=10,
                        help='随机模式下的帧数量 (默认: 10)')
    parser.add_argument('--only-shape', '-s', action='store_true',
                        help='是否只处理形状，不处理加速度')
    
    args = parser.parse_args()
    
    bag_dir = args.bag_dir
    
    if args.mode == 'zebra':
        print(f"模式: 斑马线静止时段 IPM 生成")
        gen_static_ipm(bag_dir, args.only_shape)
    elif args.mode == 'random':
        print(f"模式: 随机帧 IPM 生成 (共 {args.num_frames} 帧)")
        gen_random_ipm(bag_dir, args.num_frames, args.only_shape)
    elif args.mode == 'full':
        print("模式: 全量 IPM 生成")
        gen_full_ipm(bag_dir, args.only_shape)


if __name__ == '__main__':
    import sys
    # 默认bag路径
    default_bag_path = '/home/geely/Documents/sunyafei/0203select/557/2025-11-28-14-21-42'
    
    if len(sys.argv) > 1:
        # 有命令行参数，使用参数调用main
        main()
    else:
        # 无参数，默认使用斑马线模式
        gen_static_ipm(default_bag_path)