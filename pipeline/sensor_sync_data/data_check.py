import os, sys
from .process_data import process_by_bag, process_stacking_frame
# from temp_test import process_by_bag, process_stacking_frame

if __name__ == '__main__':

    clip_paths = sys.argv[1:]

    # save_path = os.path.join(clip_path, "../processed_data")

    for clip_path in clip_paths:
        if "visualization" in os.path.basename(clip_path):
            continue

        # save_path = os.path.join(clip_path, "..", os.path.basename(clip_path)+"_visualization")
        clip_path_spilt = clip_path.split("/")
        save_path = os.path.join(clip_path, "../..", clip_path_spilt[-2]+"_visualization", clip_path_spilt[-1]+"_visualization")

        pcd_path = os.path.join(save_path, "vis_pcd")
        box_path = os.path.join(save_path, "vis_box")
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        if not os.path.exists(pcd_path):
            os.makedirs(pcd_path)
        if not os.path.exists(box_path):
            os.makedirs(box_path)

        process_by_bag(clip_path, save_path, vis_box=False, vis_pcd=True, vis_box_in_pcd=False)

        # process_by_bag(clip_path, save_path, vis_box=False, vis_pcd=False, vis_box_in_pcd=True)

        # process_stacking_frame(clip_path, 1)

    