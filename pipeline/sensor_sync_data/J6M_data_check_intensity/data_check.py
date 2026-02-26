import os, sys
from process_data import process_by_bag, process_stacking_frame
# from temp_test import process_by_bag, process_stacking_frame
import multiprocess as mp



if __name__ == '__main__':

    clip_paths = sys.argv[1:]

    vis_box = False
    vis_pcd = True
    vis_box_in_pcd = False
    clip_path_pairs = []

    for clip_path in clip_paths:
        if "visualization" in os.path.basename(clip_path):
            continue
        clip_path_spilt = clip_path.split("/")
        save_path = os.path.join(clip_path, "../..", clip_path_spilt[-2]+"_visualization_intensity", clip_path_spilt[-1]+"_visualization")
        
        clip_path_pairs.append((clip_path, save_path))

        process_by_bag(clip_path, save_path, vis_box, vis_pcd, vis_box_in_pcd)

        # process_stacking_frame(clip_path, 1)

    

    # with mp.Pool(processes=10) as pool:
    #     pool.starmap(process_by_bag, 
    #         [(clip_path, save_path, vis_box, vis_pcd, vis_box_in_pcd) 
    #             for clip_path, save_path in clip_path_pairs])

    #     pool.close()
    #     pool.join()
