import os
import shutil

dirs = []
dirs.append("/home/geely/nas/PLD-S6B/OCC/0720-liuyang/liu-feng/P177")

for dir in dirs:
    folders = os.listdir(dir)
    folders = [d for d in folders if "-clip-20df" in d]
    for item in folders:
        item_path = os.path.join(dir, item)
        clips = os.listdir(item_path)
        for clip in clips:
            img_path = os.path.join(item_path, clip, "cam_front_right")
            if os.path.exists(img_path):
                shutil.rmtree(img_path)
        new_item_dir = os.path.join(dir, "0287-" + item)
        os.rename(item_path, new_item_dir)