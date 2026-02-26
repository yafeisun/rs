import os
import shutil

txt_path="label_info_engineering.txt"
with open(txt_path,"r") as f:
    lines=f.readlines()
    for line in lines:
        line = line.strip()
     #    line = line.replace("J6M-S8D", "J6M-S6A")
        if not os.path.isdir(line):
                print(f"错误：目标文件夹 {line} 不存在！")
                continue
                
        else:
             save_path = line.replace("J6M-S18", "bag_engineering-S18")
             
             print(save_path)
             os.makedirs(save_path, exist_ok=True)
             for file in os.listdir(line):
                  if file == "cam_front_right":
                    imgs = os.listdir(os.path.join(line, file))
                    #    for imgs in os.listdir(os.path.join(line, file)):
                         #    if imgs.endswith(".jpg"):
                         
                              
                    for idx,img in enumerate(imgs):
                         if img.endswith(".jpg"):
                              if idx %5 == 0: 
                                   shutil.copy(os.path.join(line, file, img), os.path.join(save_path, img))
                       
            
