import os
folder_path ="/home/robosense/nas/Geely-a/GL_2_9_X"
count = 0
for root,dirs,files in os.walk(folder_path):
        for file in files:
            if ".pcd" in file:
                count += 1

print(count)