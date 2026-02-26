import os
import shutil

# upper_paths = "/media/geely/4C42D6F542D6E32E/songbiao0326"
# upper_paths = "/media/geely/My Passport/20250325_4D_OD"
upper_paths = "/home/geely/nas/PLD-S7A/LD_OD_2HZ"
if os.path.isdir(upper_paths):
    od_frame_paths = [os.path.abspath(os.path.join(upper_paths, d)) for d in os.listdir(upper_paths)]
    od_frame_paths = [d for d in od_frame_paths if os.path.isdir(d)]
od_frame_paths = []
# od_frame_paths.append("/home/geely/nas/PLD-S7A/LD_OD_2HZ/P145_0509_20250812_1901")
od_frame_paths.append("/home/geely/nas/J6M-S6E/S10/下发/电动车后备箱/J6M_0576_20251224_1151")
# od_frame_paths.append("/home/geely/nas/PLD-S7A/LD_OD_2HZ/P145_0509_20250813_1557")
# od_frame_paths.append("/home/geely/nas/J6M-3C/4DOD-CC-0417-day/E371_6956_20250317_1241")

clip_num = 0
total_frame_num = 0
night_frame_num = 0
for dir in od_frame_paths:
    current_bags = [d for d in os.listdir(dir) if os.path.isdir(os.path.join(dir, d))]
    for item in current_bags:
        # 提取时分秒
        time_part = item.split('-')
        hour = int(time_part[3])
        minute = int(time_part[4])
        # for i in ["bev","pose"]:
        #     bev_path = os.path.join(dir, item, i)
        #     if os.path.exists(bev_path):
        #         shutil.rmtree(bev_path)
        current_clips = os.listdir(os.path.join(dir, item))
        current_clips = [d for d in current_clips if "2025" in d] 
        clip_num += len(current_clips)
        for clip in current_clips:
            # img_path = os.path.join(dir, item, clip, "cam_front_right")
            # if os.path.exists(img_path):
            #     shutil.rmtree(img_path)

            current_frames = [d for d in os.listdir(os.path.join(dir, item, clip))] 
            current_frames = [d for d in current_frames if os.path.isdir(os.path.join(dir, item, clip, d))]
            current_frames = [d for d in current_frames if d.isdigit()]
            # for frame in current_frames:
            #     frame_path = os.path.join(dir, item, clip, frame)
            #     for i in [".pcd", "__back.jpg","__back_left.jpg", "__back_right.jpg","__front_far.jpg","__front_left.jpg","__front_right.jpg"]:
            #         file_path = os.path.join(frame_path,frame + i)
            #         if os.path.exists(file_path):
            #             os.remove(file_path)

            total_frame_num += len(current_frames)
            # 判断是否晚于晚上六点半
            if hour > 18 or (hour == 18 and minute >= 30):
                night_frame_num += len(current_frames)
print(f"total: {total_frame_num} night: {night_frame_num}")

    