from detect.Detector import Detector
from detect.Video import Video
from detect.Capture import *
from .Lidar.Lidar import *
from Lidar.Converter import Converter
# from Log.Log import RadarLog
# from Lidar.PointCloud import PcdQueue
from Car.Car import *
# import numpy as np
# import pandas as pd
import threading
import cv2
import time
# import open3d as o3d
from collections import deque
from ruamel.yaml import YAML
# import os


mode = "test" # "video" or "camera"


if __name__ == '__main__':
    video_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/data/华科vs哈工大round1_原视频.avi"
    detector_config_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/detector_config.yaml"
    binocular_camera_cfg_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/bin_cam_config.yaml"
    main_config_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/main_config.yaml"
    converter_config_path = "/home/nvidia/RadarWorkspace/code/Radar_Develop/configs/converter_config.yaml"
    main_cfg = YAML().load(open(main_config_path, encoding='Utf-8', mode='r'))
    # 全局变量
    global_my_color = main_cfg['global']['my_color']
    is_debug = main_cfg['global']['is_debug']

    # 类初始化
    detector = Detector(detector_config_path)
    lidar = Lidar(main_cfg)
    converter = Converter(global_my_color,converter_config_path)
    carList = CarList(main_cfg)

    if mode == "video":
        capture = Video(video_path)
    elif mode == "camera":
        capture = Capture(binocular_camera_cfg_path, "new_cam")
    elif mode == "test":
        capture = ImageProcessor_test(binocular_camera_cfg_path, "new_cam")
        capture.run()

    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用mp4编码器
    # out = cv2.VideoWriter(f'only_detect{round}.mp4', fourcc, 24, (capture.width,capture.height))  # 文件名，编码器，帧率，帧大小

    # fps计算
    N = 10
    fps_queue = deque(maxlen=N)
    start_time = time.time()

    # 启动图像处理子线程
    threading.Thread(target=detector.detect_thread, args=(capture,), daemon=True).start()

    # 开启激光雷达线程
    # lidar.start()

    # 主循环

    # 主循环
    real_results = []
    while True:
        # 计算fps
        now = time.time()
        fps = 1 / (now - start_time)
        start_time = now
        # 将FPS值添加到队列中
        fps_queue.append(fps)
        # 计算平均FPS
        avg_fps = sum(fps_queue) / len(fps_queue)

        print("fps:", avg_fps)
        # 获得推理结果
        # print("try to get infer result")
        infer_result = detector.get_results()
        # 确保推理结果不为空且可以解包
        if infer_result is not None:
            # print(infer_result)
            result_img, results = infer_result

            if results is not None:
                print("results is not none")
                if lidar.obstacleQueue.point_num == 0:
                    print("no pcd")
                    continue
                # print("pcd num:",lidar.pcdQueue.point_num)
                pc_all = lidar.get_all_pc()
                for result in results:
                    print("result handle")
                    # 对每个检测框进行处理，获取对应点云
                    # 结果：[xyxy_box, xywh_box , track_id , label ]
                    xyxy_box, xywh_box, track_id, label = result  # xywh的xy是中心点的xy

                    # 如果没有分类出是什么车，或者是己方车辆，直接跳过
                    # if label == "NULL":
                    #     continue
                    # if global_my_color == "Red" and carList.get_car_id(label) < 100 and carList.get_car_id(label) != 7:
                    #     continue
                    # if global_my_color == "Blue" and carList.get_car_id(label) > 100 and carList.get_car_id(
                    #         label) != 107:
                    #     continue
                    # print("xyxy",xyxy_box)
                    # print("xywh",xywh_box)
                    # print("label",label)
                    result_ = converter.detection_main(xyxy_box,pcd=pc_all,if_lidar=False)
                    real_results.append(result_)

                    cv2.imshow("result", result_img)

                    print("out:", avg_fps)

                    if cv2.waitKey(1) == ord('q'):
                        break

                    # lidar.stop()
                    # out.release()
                print("real_results:",real_results)
                capture.release()

                cv2.destroyAllWindows()
