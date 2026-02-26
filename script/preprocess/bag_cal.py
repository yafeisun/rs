import os
import re
import yaml
import csv
import pandas as pd

def is_valid_time_format(folder_name):
    """
    检查文件夹名称是否符合 YYYY-MM-DD-HH-MM-SS 的时间格式
    :param folder_name: 文件夹名称
    :return: 如果符合时间格式返回 True,否则返回 False
    """
    pattern = r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}$'
    return bool(re.match(pattern, folder_name))


def process_yaml_file(yaml_file_path):
    """
    处理 label.yaml 文件，计算每个 text 的时间总和
    :param yaml_file_path: label.yaml 文件的路径
    :return: 包含每个 text 时间总和的字典
    """
    bag_durations = {}
    if os.path.exists(yaml_file_path):
        with open(yaml_file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            # 在这里对 data 进行清洗，去掉 text 和 title 中的换行符
            data = remove_newlines_recursively(data)
            nodes = data.get('Interval', [])
            if not nodes:
                print(f"{yaml_file_path}的'Interval'节点为空")
            else:
                for node in nodes:
                    title = node.get('title', '').replace('\n', '').strip()
                    text = node.get('text', '').replace('\n', '').strip()
                    # 去掉 text 中的空格和破折号，并转换为大写
                    text = text.replace(' ', '').replace('–', '').replace('-', '').upper()
                    start_time = node.get('start_time', '')
                    end_time = node.get('end_time', '')

                    duration = 0
                    if start_time == "" or end_time == "":
                        duration = 0
                    else:
                        duration = end_time - start_time
                        # 修改保存逻辑，保存 duration 和 title
                        if text not in bag_durations:
                            bag_durations[text] = {'duration': duration, 'title': title}
                        else:
                            bag_durations[text]['duration'] += duration
    return bag_durations

def remove_newlines_recursively(data):
    """
    递归地去掉字典和列表中的换行符
    :param data: YAML 解析后的数据
    :return: 清洗后的数据
    """
    if isinstance(data, dict):
        return {key: remove_newlines_recursively(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [remove_newlines_recursively(item) for item in data]
    elif isinstance(data, str):
        return data.replace('\n', '').strip()
    else:
        return data


def write_dailydata_to_csv(all_results, sub_dir_names, input_folder):
    """
    将处理结果写入 input_folder 文件夹下的 result.csv 文件,第1列是text,第2列是对应title,第3列及以后是每个sub_dir_names的duration,最后加一列是所有duration的累加
    :param all_results: 包含所有处理结果的字典
    :param sub_dir_names: 子文件夹名称列表
    :param input_folder: 输入文件夹路径
    """
    # 收集所有可能的字段名
    all_texts = set()
    for times in all_results.values():
        all_texts.update(times.keys())

    # 创建一个临时字典来存储合并后的数据
    merged_data = {}

    for text in sorted(all_texts):
        row = {'text': text}
        title = None
        total_duration = 0
        for sub_dir_name in sub_dir_names:
            item = all_results.get(sub_dir_name, {}).get(text)
            if item:
                if title is None:
                    title = item['title']
                duration = item['duration']
                row[sub_dir_name] = duration
                total_duration += duration
            else:
                row[sub_dir_name] = 0  # 如果没有数据，设置为 0

        row['title'] = title if title else ''
        row['total_duration'] = total_duration
        merged_data[text] = row

    # 拼接文件路径
    csv_file_path = os.path.join(input_folder, 'result.csv')
    with open(csv_file_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        fieldnames = ['text', 'title'] + sorted(sub_dir_names) + ['total_duration']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for text, row in merged_data.items():
            writer.writerow(row)


def write_car_daily_sum_to_excel(car_daily_sum, input_folder):
    """
    将车辆每日累计数据写入 Excel 文件，sheet 页名称为 VIN 码
    :param car_daily_sum: 每辆车每日累计数据
    :param input_folder: 输入文件夹路径
    """
    # 建立 Excel 文件路径
    excel_file_path = os.path.join(input_folder, 'car_daily_sum.xlsx')
    
    # 使用 pandas 的 ExcelWriter 将每个 VIN_CODE 的数据写入不同的 sheet
    with pd.ExcelWriter(excel_file_path, engine='openpyxl') as writer:
        for vin_code, scenes_data in car_daily_sum.items():
            # 构造 DataFrame
            data = []
            for text, scene in scenes_data.items():
                data.append({
                    'text': text,
                    'title': scene['title'],
                    'duration': scene['duration']
                })
            if data:
                df = pd.DataFrame(data)
            
                # 将 duration 列重命名为 input_folder
                df.rename(columns={'duration': os.path.basename(input_folder)}, inplace=True)
                
                # 确保列顺序正确
                df = df[['text', 'title', os.path.basename(input_folder)]]
                
                # 将 DataFrame 写入 Excel，sheet 名称为此 vin_code
                df.to_excel(writer, sheet_name=str(vin_code), index=False)

def process_daily_folder(input_folder):
    """
    遍历输入文件夹下的所有文件夹，找出 E371 开头的文件夹并处理
    :param input_folder: 输入文件夹路径 日期格式YYYYMMDD
    :return: 包含所有处理结果的字典和子文件夹名称列表
    """
    daily_scene = {}
    car_daily_scene = {}    # 每辆车在当天的场景数据
    car_daily_sum = {}      # 当天每辆车的场景累计数据
    sub_dir_names = []
    bag_num = 0

    # 只遍历输入文件夹的直接子文件夹
    for root, dirs, files in os.walk(input_folder):
        if os.path.samefile(root, input_folder):
            # 处理当前目录下的所有子目录
            for dir_name in dirs:
                if dir_name.startswith('E371'):
                    e371_folder_path = os.path.join(root, dir_name)
                    # 只处理 E371 文件夹下一级的目录
                    sub_dirs = os.listdir(e371_folder_path)
                    for item in sub_dirs:
                        bag_folder_path = os.path.join(e371_folder_path, item)
                        if os.path.isdir(bag_folder_path):
                            bag_folder_name = item
                            if is_valid_time_format(bag_folder_name):
                                bag_num += 1
                                sub_dir_names.append(bag_folder_name)
                                yaml_file_path = os.path.join(bag_folder_path, 'label.yaml')
                                # 获取该BAG包对应车辆的VIN码
                                car_yaml_files = [f for f in os.listdir(bag_folder_path) 
                                                    if f.startswith('car_') and f.endswith('.yaml')]
                                VIN_CODE = car_yaml_files[0].split('_')[2]  # 按下划线分割，取第3个字段
                                scene_result = process_yaml_file(yaml_file_path)

                                # 累加当天每辆车的场景数据
                                if VIN_CODE not in car_daily_sum:
                                    car_daily_sum[VIN_CODE] = {}
                                for text, data in scene_result.items():
                                    if text not in car_daily_sum[VIN_CODE]:
                                        car_daily_sum[VIN_CODE][text] = {'duration': 0, 'title': data['title']}
                                    car_daily_sum[VIN_CODE][text]['duration'] += data['duration']

                                daily_scene[bag_folder_name] = scene_result
                                if VIN_CODE not in car_daily_scene:  # Initialize if VIN_CODE doesn't exist
                                    car_daily_scene[VIN_CODE] = {}
                                car_daily_scene[VIN_CODE][bag_folder_name] = scene_result
            # 避免继续递归遍历
            break
    # 写入car_daily_sum.xlsx
    # Filter out empty elements in car_daily_sum
    car_daily_filtered = {k: v for k, v in car_daily_sum.items() if v}
    if car_daily_filtered:
        write_car_daily_sum_to_excel(car_daily_filtered, input_folder)
    # 打印每天处理的文件夹数量
    print(f"{input_folder}共处理{bag_num}个bag")
    # 保存 daily_folder 的处理数据
    # Remove empty elements from daily_scene
    daily_scene = {k: v for k, v in daily_scene.items() if v}
    if daily_scene:
        write_dailydata_to_csv(daily_scene, sub_dir_names, input_folder)
    return daily_scene, car_daily_scene, car_daily_sum

def is_valid_daily_folder(folder_name):
    """
    检查文件夹名称是否符合 YYYYMMDD 格式
    :param folder_name: 文件夹名称
    :return: 如果符合格式返回 True,否则返回 False
    """
    pattern = r'^\d{8}$'
    return bool(re.match(pattern, folder_name))


def process_bag_folder(bag_folder):
    """
    处理指定 bag_folder 下的所有 daily_folder，按月份拆分并生成汇总结果保存到 Excel 文件
    :param bag_folder: 输入文件夹路径
    """
    total_result = {}
    daily_folders = [folder for folder in os.listdir(bag_folder) if is_valid_daily_folder(folder)]
    # 按从小到大排序
    daily_folders.sort()

    for daily_folder in daily_folders:
        month_key = daily_folder[:6]  # 提取 YYYYMM 作为月份键
        input_folder = os.path.join(bag_folder, daily_folder)
        daily_results, car_result, car_daily_sum = process_daily_folder(input_folder)



def write_monthly_data_to_excel(bag_folder):
    """
    遍历指定 bag_folder 下的每个 YYYYMMDD 文件夹，读取其中的 result.csv 文件，生成汇总文件 monthly_statistics.xlsx，
    并按 text 进行合并归类。
    :param bag_folder: 输入文件夹路径
    """
    monthly_data = {}  # 用于存储按月份分类的数据

    # 遍历 bag_folder 下的所有文件夹（YYYYMMDD 格式）
    daily_folders = os.listdir(bag_folder)
    daily_folders.sort()
    for daily_folder in daily_folders:
        folder_path = os.path.join(bag_folder, daily_folder)
        if os.path.isdir(folder_path) and daily_folder[:6].isdigit():
            month_key = daily_folder[:6]  # 提取月份（YYYYMM）
            result_csv_path = os.path.join(folder_path, 'result.csv')
            if os.path.exists(result_csv_path):
                # 读取 result.csv 文件
                df = pd.read_csv(result_csv_path)
                # 假设 result.csv 文件的列结构为：text, title, YYYYMMDD-1, YYYYMMDD-2, ..., total_duration, percent
                # 提取最后一列作为当天的统计数据列
                column_name = os.path.basename(folder_path)
                df[column_name] = df.iloc[:, -1]  # 假设倒数第1列是 total_duration

                # 将数据按月分类
                if month_key not in monthly_data:
                    monthly_data[month_key] = []
                # 保留 needed_columns 为 ['text', 'title', column_name]
                needed_columns = ['text', 'title', column_name]
                monthly_data[month_key].append(df[needed_columns])

    # 合并数据并写入 monthly_statistics.xlsx
    if monthly_data:
        excel_file_path = os.path.join(bag_folder, 'monthly_statistics.xlsx')
        with pd.ExcelWriter(excel_file_path, engine='openpyxl') as writer:
            for month_key, data_frames in monthly_data.items():
                # 合并所有 daily_data 的 DataFrame
                combined_df = pd.concat(data_frames, ignore_index=True)

                # 按 text 进行归类，计算每天的 total_duration 或其他统计值的总和
                # 按 text 分组，合并相同 text 的行
                # 假设需要保留 title 列，并对其他列进行求和
                # 使用 groupby 和 agg 函数
                aggregated_df = combined_df.groupby('text', as_index=False).agg(
                    {'title': 'first',  # 保留第一个 title
                     **{col: 'sum' for col in combined_df.columns if col not in ['text', 'title']}}
                )

                # 确保列顺序正确
                columns = ['text', 'title'] + [col for col in aggregated_df.columns if col not in ['text', 'title']]
                aggregated_df = aggregated_df[columns]

                # 新增一列 total，计算从第三列到最后一列的累加和
                aggregated_df['total'] = aggregated_df.iloc[:, 2:].sum(axis=1)

                # 新增一列 percent，计算每行 total 占总和的百分比
                total_sum = aggregated_df['total'].sum()
                if total_sum != 0:
                    aggregated_df['percent'] = (aggregated_df['total'] / total_sum).round(2)
                else:
                    aggregated_df['percent'] = 0.0

                # 确保列顺序正确，先 text 和 title，再其他列，最后是 total 和 percent
                columns_with_percent = ['text', 'title'] + [col for col in aggregated_df.columns if col not in ['text', 'title', 'total', 'percent']] + ['total', 'percent']
                aggregated_df = aggregated_df[columns_with_percent]

                # 将 DataFrame 写入 Excel，sheet 名为月份
                aggregated_df.to_excel(writer, sheet_name=month_key, index=False)

def write_car_monthly_sum_to_excel(bag_folder):
    """
    遍历指定 bag_folder 下的每个 YYYYMMDD 文件夹，读取其中的 car_daily_sum.xlsx 文件，生成汇总文件 car_monthly_sum.xlsx
    :param bag_folder: 输入文件夹路径
    """
    car_monthly_data = {}  # 用于存储按 VIN 码分类的每月数据

    # 遍历 bag_folder 下的所有文件夹（YYYYMMDD 格式）
    daily_folders = os.listdir(bag_folder)
    for daily_folder in daily_folders:
        if os.path.isdir(os.path.join(bag_folder, daily_folder)) and daily_folder[:6].isdigit():
            # 读取 daily_folder 下的 car_daily_sum.xlsx 文件
            daily_car_data_path = os.path.join(bag_folder, daily_folder, 'car_daily_sum.xlsx')
            if os.path.exists(daily_car_data_path):
                # 使用 pandas 读取每个 sheet 页（VIN 码）
                excel_file = pd.ExcelFile(daily_car_data_path)
                for sheet_name in excel_file.sheet_names:
                    df = excel_file.parse(sheet_name)
                    # 假设 car_daily_sum.xlsx 中的列名为：text, title, [daily_data_columns...]
                    df['date'] = daily_folder  # 添加日期列
                    # 按 VIN 码分类存储数据
                    if sheet_name not in car_monthly_data:
                        car_monthly_data[sheet_name] = []
                    car_monthly_data[sheet_name].append(df)

    # 合并数据并写入 car_monthly_sum.xlsx
    if car_monthly_data:
        excel_file_path = os.path.join(bag_folder, 'car_monthly_sum.xlsx')
        with pd.ExcelWriter(excel_file_path, engine='openpyxl') as writer:
            for vin_code, data_frames in car_monthly_data.items():
                # 合并所有 daily_data 的 DataFrame
                combined_df = pd.concat(data_frames, ignore_index=True)
                # 调整列顺序，确保 text 和 title 在前两列
                combined_df = combined_df[['text', 'title', 'date'] + [col for col in combined_df.columns if col not in ['text', 'title', 'date']]]
                # 将日期列转换为宽表格式（以日期为列）
                pivot_table = combined_df.pivot_table(index=['text', 'title'], columns='date', values=combined_df.columns[3:]).reset_index()
                # 重置列索引
                pivot_table.columns = ['text', 'title'] + [col[1] for col in pivot_table.columns[2:]]
                # 去除 None 列名
                pivot_table.rename(columns=lambda x: x if x else '', inplace=True)
                # 将结果写入 Excel，sheet 名为 VIN 码
                pivot_table.to_excel(writer, sheet_name=vin_code, index=False)

if __name__ == '__main__':
    # 输入文件夹路径
    bag_folder = '/home/geely/nas/J6M-3C/bag' 
    process_bag_folder(bag_folder)   
    # 生成 car_monthly_sum.xlsx
    write_car_monthly_sum_to_excel(bag_folder)
    # 生成每天的汇总文件 monthly_statistics.xlsx
    write_monthly_data_to_excel(bag_folder)