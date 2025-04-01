# 构建Lidar类，作为激光雷达接收类，构建一个ros节点持续订阅/centroid_points话题，把点云信息写入PcdQueue,整个以子线程形式运行
import sys
sys.path.append("/home/nvidia/RadarWorkspace/code/Hust-Radar-2024/Lidar")
from PointCloud import *
import threading
import rospy
import numpy as np
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class Process_Lidar:
    def __init__(self,cfg):
        # 标志位
        self.flag = False  # 激光雷达接收启动标志
        self.init_flag = False # 激光雷达接收线程初始化标志
        self.working_flag = False  # 激光雷达接收线程启动标志
        self.threading_1 = None  # 激光雷达接收子线程
        self.stop_event = threading.Event()  # 线程停止事件
        self.got_tf = False  # 是否接收到了tf


        # 参数
        self.height_threshold = cfg["lidar"]["height_threshold"]  # 自身高度，用于去除地面点云
        self.min_distance = cfg["lidar"]["min_distance"]  # 最近距离，距离小于这个范围的不要
        self.max_distance = cfg["lidar"]["max_distance"]  # 最远距离，距离大于这个范围的不要
        # self.lidar_topic_name = cfg["lidar"]["lidar_topic_name"] # 激光雷达话题名
        self.obstacle_topic_name = "/obstacle_cloud" # 障碍物发布话题名字 # 动态障碍物话题名

        # 点云队列
        self.pcdQueue = PcdQueue(max_size=10) # 将激光雷达接收的点云存入点云队列中，读写上锁？
        self.obstacleQueue = centroidsQueue(max_size=20) # 将动态障碍物的云加入队列中

        # 激光雷达线程
        self.lock = threading.Lock()  # 线程锁
        # transformation_matrix
        self.transformation_matrix = None

        # 中心点序列
        self.centroids = None




        if not self.init_flag:
            # 当雷达还未有一个对象时，初始化接收节点
            self.listener_begin()
            # print("listener_begin")
            self.init_flag = True
            self.threading_1 = threading.Thread(target=self.main_loop, daemon=True)


    # 线程启动
    def start(self):
        '''
        开始子线程，即开始spin
        '''
        if not self.working_flag:
            self.working_flag = True
            self.threading_1.start()

            # print("start@")

    # 线程关闭
    def stop(self):
        '''
        结束子线程
        '''
        if self.working_flag and self.threading_1 is not None: # 关闭时写错了之前，写成了if not self.working_flag
            self.stop_event.set()
            rospy.signal_shutdown('Stop requested')
            self.working_flag = False
            print("stop")




    # 节点启动
    def listener_begin(self):
        rospy.init_node('laser_listener', anonymous=True)
        # print('strategy')
        rospy.Subscriber("/centroid_points", PointCloud2, self.callback)

    # 订阅节点子线程
    def main_loop(self):
        # 通过将spin放入子线程来防止其对主线程的阻塞
        rospy.spin()

    def callback(self, msg):
        """
        回调函数，处理接收到的 PointCloud2 消息
        """
        # 解析 msg 中的点云数据
        # print('start')
        # 解析时间，并在结果的最后一维加上时间
        t = msg.header.stamp.to_sec()
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(points)
        centroids = np.array(points, dtype=np.float32)
        t_array = np.full((centroids.shape[0], 1), t)
        centroids = np.hstack((centroids,t_array))

        # 使用锁保护共享数据
        with self.lock:
            self.centroids = centroids
            # 根据需求处理数据
            self.obstacleQueue.add(self.centroids)  # 假设是一个队列

        # print("Received centroids as numpy array:\n", centroids)

    # 获取所有点云
    def get_all_pc(self):
        with self.lock:
            return self.obstacleQueue.get_all_pc()


    # del
    def __del__(self):
        self.stop()

# if __name__ == "__main__":
#     cfg = {
#         "lidar": {
#             "height_threshold": 0.05,
#             "min_distance": 0.1,
#             "max_distance": 20,
#             "lidar_topic_name": "/adjusted_cloud"
#         }
#     }
#     lidar = Process_Lidar(cfg)
#     lidar.start()
#     xyz = lidar.get_all_pc()

#     time.sleep(1000)

#     lidar.stop()
#     print("end")