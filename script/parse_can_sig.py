import os
import subprocess

def call_can2csv(folder_path):
    """
    调用 can2csv.py 脚本，处理指定文件夹中的文件。
    :param folder_path: 包含 chassis.txt 和 4D_radar.txt 的文件夹路径
    """
    # 构建文件路径
    chassis_txt = os.path.join(folder_path, 'chassis.txt')
    radar_txt = os.path.join(folder_path, '4D_radar.txt')
    output_csv = os.path.join(folder_path, 'output.csv')

    # 构建命令
    command = f'python3 {os.path.join("/home/geely/Documents/robosense/script/can2csv", "can2csv.py")} {chassis_txt} {radar_txt} {output_csv}'
    print(command)
    try:
        # 执行命令
        subprocess.run(command, shell=True, check=True)
        if os.path.exists(output_csv):
            print(f"处理完成，输出文件: {output_csv}")
    except subprocess.CalledProcessError as e:
        print(f"执行命令时出错: {e}")

if __name__ == "__main__":
    # 示例调用，替换为实际的文件夹路径
    folder_path = '/media/geely/sda/20250304/1131_0036_20250304_1705_gaojia/2024-11-13-15-28-45'
    call_can2csv(folder_path)