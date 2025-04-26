import cv2
import random
import time
from Lidar.Converter import *
from detect.Video import Video
from detect.Capture import Capture
if_video = True
if '__main__' == __name__:
    converter = Converter("Blue","/home/nvidia/RadarWorkspace/code/Hust_Radar_2025-main/configs/converter_config.yaml")
    binocular_camera_cfg_path = "./configs/bin_cam_config.yaml"
    video_path = "/home/nvidia/RadarWorkspace/code/Hust-Radar-2024/data/华科vs哈工大round1_原视频.avi"
    if if_video is False: 
        capture = Capture(binocular_camera_cfg_path,"new_cam") 
    else:
        capture = Video(video_path)

    # print(time.time())
    converter.camera_to_field_init(capture)
    # 随机生成一系列点：要求在图像的一半以下

