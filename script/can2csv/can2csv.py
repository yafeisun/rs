import cantools
import csv
import pandas as pd
import sys
import json
import os
from collections import defaultdict

argvTmp = sys.argv
if (len(argvTmp) != 4):
    print(len(argvTmp),'example:python3 chassis.txt 4D_radar.txt output.csv')
    exit(-1)

def parse_can_line(line):
    """
    解析单行CAN数据,返回字典格式。
    """
    parts = line.strip().split()
    if len(parts) < 7:
        raise ValueError("行字段不足，无法解析")

    # 提取基础字段
    timestamp = parts[0]
    channel = parts[1]
    direction = parts[2]
    frame_type = parts[4]
    can_id = int(parts[3], 16) if 'x' in parts[3] else int(parts[3])
    dlc = int(parts[6])
    
    # 提取数据字段并转换为字节数组
    hex_data = parts[7:7 + dlc]
    data_bytes = bytes.fromhex(''.join(hex_data))
    
    return {
        'timestamp': timestamp,
        'channel': channel,
        'direction': direction,
        'frame_type': frame_type,
        'can_id': can_id,
        'dlc': dlc,
        'data': data_bytes
    } 

def can_to_csv(dbc_path, input_txt, output_csv, mode, needCanID, needSignals):
    """
    核心转换函数：读取DBC文件，解析CAN数据并生成CSV。表格形式如下:
    Timestamp           SignalName                           Value
    1737510290.648059,  PinionSteerAgGroupPinionSteerAg1     -0.0078125
    """
    
    # 加载DBC数据库
    db = cantools.database.load_file(dbc_path)
    all_signals = set()
    
    # 收集所有可能的信号名
    # needCanID = [0x350, 0x195]
    for msg in db.messages:
        for signal in msg.signals:
            if signal.name in needSignals:
                all_signals.add(signal.name)
    all_signals = sorted(all_signals)
    
    # 准备CSV写入器
    with open(input_txt, 'r') as f_in, open(output_csv, mode, newline='', encoding='utf-8') as f_out:
        writer = csv.writer(f_out)
        if mode == 'w':
            writer.writerow((['Timestamp', 'SignalName', 'Value']))
        # 逐行解析
        for line in f_in:
            line = line.strip()
            if not line:
                continue
            
            try:
                parsed = parse_can_line(line)
                can_id = parsed['can_id']
                data = parsed['data']
                # print(can_id)
                if can_id not in needCanID:
                    continue
                # 解码信号
                try:
                    msg = db.get_message_by_frame_id(can_id)
                    decoded = msg.decode(data, decode_choices=False)
                except KeyError:
                    decoded = {}

                # 构建行数据
                # print(parsed['timestamp'])
                # 填充信号值（缺失则为空
                for sig in all_signals:
                    value = str(decoded.get(sig, ''))
                    if value != '' :
                        row = [
                            f"{float(parsed['timestamp']):.6f}",#需要从csv导入后转换为文本显示原始数据，实际csv内容是正确的
                            sig,
                            value
                        ]
                        writer.writerow(row)
                
            except Exception as e:
                print(f"解析失败的行：{line}\n错误：{e}")
                continue

if os.path.exists('./config.json'):
    try :
        # 解析 JSON 数据
        with open('./config.json', 'r') as file:
            data = json.load(file)
        for index, val in enumerate(data['config']):
            needCanId = []
            hex_values = val['needCanId']
            for hex_str in hex_values:
                if hex_str.startswith('0x'):
                    hex_str = hex_str[2:]
                needCanId.append(int(hex_str, 16))
            #python3 can2csv.py chassis.txt 4D_radar.txt output.csv
            if (index == 0):
                can_to_csv('./SDB23209_E371_PCMU_ADCU5_ChassisCAN1Cfg_240917.dbc', argvTmp[1], argvTmp[3], 'w', needCanId, val['needSignal'])
                pass
            else :
                can_to_csv('./SDB23209_E371_PCMU_ADCU5_SafetyCANFD2Cfg_240917.dbc', argvTmp[2], argvTmp[3], 'a', needCanId, val['needSignal'])
    except Exception as e:
        print(e)