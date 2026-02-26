import os, io
import av
import cv2
# import multiprocessing as mp
import gc
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

image_quality = 85
cam_topics = [
    "/CompressedVideo_0",
    "/CompressedVideo_1",
    "/CompressedVideo_2",
    "/CompressedVideo_3",
    "/CompressedVideo_4",
    "/CompressedVideo_5",
    "/CompressedVideo_6",
    "/CompressedVideo_7",
    "/CompressedVideo_8",
    "/CompressedVideo_9",
    "/CompressedVideo_10"
]

cam_topics_2_perspective = {
    "/CompressedVideo_0": "cam_front_right",
    "/CompressedVideo_1": "cam_front_left",
    "/CompressedVideo_2": "cam_back",
    "/CompressedVideo_3": "cam_side_left_front",
    "/CompressedVideo_4": "cam_side_right_front",
    "/CompressedVideo_5": "cam_side_left_back",
    "/CompressedVideo_6": "cam_side_right_back",
    "/CompressedVideo_7": "cam_around_front",
    "/CompressedVideo_8": "cam_around_back",
    "/CompressedVideo_9": "cam_around_left",
    "/CompressedVideo_10": "cam_around_right"
}

class H265Decoder:
    def __init__(self):
        self.container = None

    def decode_h265_frame(self, h265_data):
        """
        解码单个H.265编码的帧。
        :param h265_data: 包含H.265编码数据的字节串
        :return: 解码后的OpenCV格式图像
        """

        video_data = io.BytesIO()

        for frame_data in h265_data:
            video_data.write(frame_data)
        del h265_data
        gc.collect()

        print("Creating container...")
        self.container = av.open(video_data, format="hevc")
        del video_data
        gc.collect()
        stream = self.container.streams.video[0]
        fps = stream.base_rate
        frame_width = stream.width
        frame_height = stream.height

        frames = []
        print("Decoding frames...")
        for packet in self.container.demux():
            for frame in packet.decode():
                if isinstance(frame, av.VideoFrame):
                    img = frame.to_ndarray(format='bgr24')
                    frames.append(img)
        return frames


def create_save_paths(save_path, topics):
    return {topic: create_save_path(save_path, topic) for topic in topics}


def create_save_path(save_path, topic):
    path = os.path.join(save_path, cam_topics_2_perspective[topic])
    if os.path.exists(path):
        print(f"已处理过bag: {save_path} (输出目录已存在: {path})")
        exit(1)

    if not os.path.exists(path):
        os.makedirs(path)
    return path


def decode_H265_and_save(data, frame_name):
    container = av.open(io.BytesIO(data), format="hevc")

    for packet in container.demux():
        for frame in packet.decode():
            img = frame.to_ndarray(format="bgr24")
            cv2.imwrite(frame_name, img)


def decode_H265_images(bag_path, cam_topic, cam_save_path):
    decoder = H265Decoder()
    h265_data_all = []
    timestamps = []

    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == cam_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            vi_timestamp = msg.viTimestamp
            h265_data_all.append(msg.data)
            timestamps.append(vi_timestamp)

            if len(h265_data_all) >= 200:
                print(cam_topic)
                frames = decoder.decode_h265_frame(h265_data_all)
                print("saving images...")
                print(
                    f"len(h265_data_all): {len(h265_data_all)}, len(timestamps): {len(timestamps)}, len(frames): {len(frames)}")
                for frame, vi_timestamp in zip(frames, timestamps):
                    formatted_timestamp = f"{vi_timestamp:0<19}"
                    cv2.imwrite(os.path.join(cam_save_path, f"{formatted_timestamp}.jpeg"), frame,
                                [cv2.IMWRITE_JPEG_QUALITY, image_quality])

                # Explicitly free memory
                del h265_data_all, timestamps, frames
                h265_data_all = []
                timestamps = []
                gc.collect()

        if len(h265_data_all) > 0:
            print(cam_topic)
            frames = decoder.decode_h265_frame(h265_data_all)
            print("saving images...")
            print(
                f"len(h265_data_all): {len(h265_data_all)}, len(timestamps): {len(timestamps)}, len(frames): {len(frames)}")
            for frame, vi_timestamp in zip(frames, timestamps):
                formatted_timestamp = f"{vi_timestamp:0<19}"
                cv2.imwrite(os.path.join(cam_save_path, f"{formatted_timestamp}.jpeg"), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, image_quality])

            # Explicitly free memory
            del h265_data_all, timestamps, frames
            gc.collect()


def extract_images(bag_dir, thread_num=len(cam_topics)):
    print(f"正在提取图片: {bag_dir}")
    bag_name = os.path.join(bag_dir, os.path.basename(bag_dir) + ".bag")

    if not os.path.exists(bag_name):
        print(f"bag 不存在: {bag_name}")
        return

    cam_save_paths = {}
    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(bag_name)], default_typestore=typestore) as reader:
        connections = reader.connections
        available_topics = set(conn.topic for conn in connections)
        print("available topics: ", available_topics)

        for cam_topic in cam_topics:
            if cam_topic not in available_topics:
                continue
            cam_save_path = create_save_path(bag_dir, cam_topic)
            cam_save_paths[cam_topic] = cam_save_path
            topic_count = sum(1 for conn in connections if conn.topic == cam_topic)
            print(f"Extracting {cam_topic}...", "message count:", topic_count, "...")

    if len(cam_save_paths) <= 0:
        print(f"[Info] task num is zero")
        return

    # 线程池并行处理
    max_workers = min(len(cam_save_paths), thread_num)
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = []
        for cam_topic, cam_save_path in cam_save_paths.items():
            future = executor.submit(decode_H265_images, bag_name, cam_topic, cam_save_path)
            futures.append((cam_topic, future))
        for cam_topic, future in futures:
            try:
                future.result()
                print(f"{cam_topic} task exec finished")
            except Exception as e:
                print(f"{cam_topic} task exec failed: {str(e)}")
        # 显式关闭进程池，避免 Python 3.8 多进程清理错误
        executor.shutdown(wait=True)


def process_bag_files(bags):
    valid_bags = []
    for bag in bags:
        cam_folders = [f for f in os.listdir(bag) 
            if f.startswith('cam_') and os.path.isdir(os.path.join(bag, f))]
        if len(cam_folders) < len(cam_topics):
            valid_bags.append(bag)

    for bag_dir in valid_bags:
        try:
            extract_images(bag_dir)
        except Exception as e:
            print(f"处理文件 {bag_dir} 时出错: {str(e)}")


if __name__ == "__main__":

    bags = []
    bags.append("/home/lenovo/Documents/0203select/557/2025-11-28-15-25-49")
 
    process_bag_files(bags)