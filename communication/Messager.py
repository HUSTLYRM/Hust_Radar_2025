import sys
sys.path.append("E:/Radar/hust_radar_2025/communication")
sys.path.append("E:/Radar/hust_radar_2025")
from .Sender import Sender
from .Receiver import Receiver
from .Topo_map.topo_lidar import *
from random import randint
import multiprocessing
import threading
import time
import logging
from shapely.geometry import Point, Polygon
import numpy as np
import cv2
from Log.Log import RadarLog
import copy
from Tools.Tools import Tools
from .assit_yaw_pitch import BallisticTrajectory
from .alert_our_hero import is_point_nearby_numpy
from .predictor import CarKalmanPredictor
from .hero_topo_predictor.predict_hero import Predictor


# from .assit_yaw_pitch import Hero_Assit

class Messager:
    def __init__(self, cfg, draw_queue):
        # 全局变量
        self.is_debug = cfg["global"]["is_debug"]
        # 创建共享内存变量
        self.shared_is_activating_double_effect = multiprocessing.Value('b', False)  # 共享内存，用于多进程
        self.shared_enemy_health_list = multiprocessing.Array('i', [100, 100, 100, 100, 100, 100])  # 对方1-5和7号的血量信息
        self.shared_enemy_marked_process_list = multiprocessing.Array('i',
                                                                      [0, 0, 0, 0, 0, 0, 0])  # 标记进度,对应对方1，2，3，4，5号车和哨兵
        self.shared_have_double_effect_times = multiprocessing.Value('i', 0)  # 拥有的双倍易伤次数
        self.shared_time_left = multiprocessing.Value('i', -1)  # 剩余时间
        self.shared_dart_target = multiprocessing.Value('i', 0)  # 飞镖目标
        self.draw_queue = draw_queue
        self.dart_target_times = [0, 0, 0]
        # log部分
        self.logger = RadarLog("Messager")
        self.status_logger = RadarLog("Messager_Status")

        # 发送部分
        self.sender = Sender(cfg)
        self.double_effect_times = 0  # 第几次发送双倍易伤效果决策,第一次发送值为1，第二次发送值为2，每局最多只能发送到2,不能发送3

        # 特殊flag
        # self.super_flag = cfg['others']['is_vs_qd'] # 打青岛大学的特殊flag
        # self.is_vs_hg = cfg['others']['is_vs_hg']

        # 接收部分
        self.receiver = Receiver(cfg, self.shared_is_activating_double_effect, self.shared_enemy_health_list,
                                 self.shared_enemy_marked_process_list, self.shared_have_double_effect_times,
                                 self.shared_time_left, self.shared_dart_target)

        # 数据存储
        self.our_car_infos = []  # 我方车辆信息 , our_car_id , our_center_xy , our_camera_xyz , our_field_xyz , our_color
        self.enemy_car_infos = []  # 敌方车辆信息，enemy_car_id , enemy_center_xy , enemy_camera_xyz , enemy_field_xyz , enemy_color = enemy_car_info
        self.sentinel_alert_info = []  # 哨兵预警信息，匹配sender的generate_sentinel_alert_info(self , carID , distance , quadrant):
        self.send_hero_assit_info = {}  # 英雄协助信息
        self.time_left = -1  #剩余时间
        self.last_time_left = -1  # 上次剩余时间 , 用于判断是否更新

        # 线程锁
        self.map_lock = threading.Lock()  # 小地图敌方车辆信息锁
        self.sentinel_lock = threading.Lock()  # 哨兵预警信息锁
        self.our_car_lock = threading.Lock()  # 我方车辆信息锁

        # predictor
        self.predictor = {1: None , 101: None , 2: None , 102: None , 3: None , 103: None , 4: None , 104: None , 5: None , 105: None , 7: None , 107: None}
        self.predictor_times = {1: 0 , 101: 0 , 2: 0 , 102: 0 , 3: 0 , 103: 0 , 4: 0 , 104: 0 , 5: 0 , 105: 0 , 7: 0 , 107: 0}

        # 次数记录
        self.hero_enter_times = 0

        # 我方颜色
        self.my_color = self.sender.my_color

        # 敌我初始化
        if self.my_color == "Red":  # 红方是1-7 ， 蓝方是101-107
            self.enemy_hero_id = 101
            self.enemy_id = [101, 102, 103, 104, 105, 107]
            self.my_cars_id = [1, 2, 3, 4, 5, 7]
            self.my_sentinel_id = 7
        elif self.my_color == "Blue":
            self.enemy_hero_id = 1
            self.enemy_id = [1, 2, 3, 4, 5, 7]
            self.my_cars_id = [101, 102, 103, 104, 105, 107]
            self.my_sentinel_id = 107

        else:
            print("检查main_config里己方颜色是否大写！")
            exit(0)

        # 线程
        self.threading = threading.Thread(target=self.main_loop, daemon=True)
        # 时间记录
        self.last_send_double_effect_time = time.time()
        self.last_send_map_time = time.time()
        self.last_update_time_left_time = time.time()
        self.last_main_loop_time = time.time()
        self.first_big_buff_send = False
        self.second_big_buff_send = False
        # 创建一个1-5,7,101-105,107的上次发送时间的字典
        if self.sender.my_color == "Blue":
            self.last_send_time_map = {1: time.time(), 2: time.time(), 3: time.time(), 4: time.time(), 5: time.time(),
                                       7: time.time()}
        elif self.sender.my_color == "Red":
            self.last_send_time_map = {101: time.time(), 102: time.time(), 103: time.time(), 104: time.time(),
                                       105: time.time(), 107: time.time()}
        else:
            print("color error , check upper character")
            exit(0)

        # 创建一个区域列表

        self.area_list_len = cfg["area"][self.my_color]["length"]
        self.area_list = []
        for i in range(self.area_list_len):
            area = cfg["area"][self.my_color][f"area{i}"]
            self.area_list.append(area)

        # 英雄预警相关
        self.find_hero_times = 0  # 英雄在区域内的次数
        self.hero_times_threshold = cfg["area"]["hero_times_threshold"]  # 英雄在区域内的次数阈值
        self.send_double_threshold = cfg["area"]["send_double_threshold"]  # 发送双倍易伤的阈值
        self.is_alert_hero = False  # 是否预警英雄

        self.our_hero_area = cfg["area"]["our_hero_area"]
        self.our_hero_xyz = None
        self.hero_state = 0 # 0,1,2
        self.hero_shooting_points = {1 : [18.0, 4.85] , 2 : [18.75 , 11.1]} if self.my_color == "Red" else {1 : [10.0 , 10.15] , 2 : [9.25 , 3.9]}
        self.hero_predictor = Predictor(self.my_color , 'hero')
        self.engine_predictor = Predictor(self.my_color , 'engine')
        # 发射速度
        self.hero_v0 = 16 + 0.1 * randint(a=-1, b=1)
        self.hero_assit = None
        self.is_assit_hero =False
        self.secure_our_hero = False

        # 发送小地图历史记录
        self.send_map_infos = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]  # 哨兵全局感知
        self.send_map_info_is_latest = [0, 0, 0, 0, 0, 0]  # 是否是最新的小地图信息

        # 赛场状态
        # 双倍易伤相关
        self.have_double_effect_times = 0  # 拥有的双倍易伤次数
        self.is_activating_double_effect = False  # 正在激活双倍易伤

        self.already_activate_double_effect_times = 0  # 已经激活了双倍易伤次数,请求时标号为这个数+1
        # 标记进度
        self.mark_progress = [0, 0, 0, 0, 0, 0]  # 标记进度,对应对方1，2，3，4，5号车和哨兵
        self.hero_is_marked = False  # 1号英雄是否被标记
        self.engineer_is_marked = False  # 2号工程车是否被标记
        self.standard_3_is_marked = False  # 3号步兵车是否被标记
        self.standard_4_is_marked = False  # 4号步兵车是否被标记
        self.standard_5_is_marked = False  # 5号步兵车是否被标记
        self.sentinel_is_marked = False  # 7号哨兵是否被标记
        self.marked_num = 0  # 被标记的数量
        # 敌方车辆的血量信息
        self.enemy_health_info = [100, 100, 100, 100, 100, 100]  # 对方1-5和7号的血量信息
        self.hero_is_dead = False  # 1号英雄是否死亡

        # 飞镖目标
        self.dart_target = 0

        # 拓扑图
        self.topo_map = Topology(self.my_color)
        # 打印区域列表

        # for area in self.area_list:
        #     print(area)

        # event
        # self.send_double_effect_decision_event = threading.Event()

        # flag
        self.working_flag = False
        # engine
        self.engine_location = []

    # 判断是否为下一秒
    def is_next_second(self):
        if self.time_left != self.last_time_left:
            self.last_time_left = self.time_left
            return True
        return False

    # 将想要绘值的图片放入队列
    def put_draw_queue(self, image):
        try:
            self.draw_queue.put(image)
        except Exception as e:
            self.logger.log(f"Put image into draw queue error:{e}")

    # 根据共享内存变量更新握在手上的决策信息
    def update_shared_info(self):
        self.update_shared_is_activating_double_effect_flags()
        self.update_shared_enemy_health_info()
        self.update_shared_mark_progress()
        self.update_shared_have_double_effect_times()
        self.update_shared_time_left()
        self.update_shared_dart_target()

    # 更新飞镖目标
    def update_shared_dart_target(self):
        index = self.shared_dart_target.value
        if not (self.dart_target == index):
            self.dart_target_times[index] += 1
        if self.dart_target_times[index] >= 2:
            self.dart_target = index
            self.dart_target_times = [0, 0, 0]

    # 更新敌方血量信息
    def update_shared_enemy_health_info(self):
        self.enemy_health_info = list(self.shared_enemy_health_list)

    # 更新标记进度
    def update_shared_mark_progress(self):
        self.mark_progress = list(self.shared_enemy_marked_process_list)

    # 更新双倍易伤次数
    def update_shared_have_double_effect_times(self):
        self.have_double_effect_times = self.shared_have_double_effect_times.value

    # 更新双倍易伤相关的flag , 用共享内存中转一下 , 如果是下降沿，已发送次数+1
    def update_shared_is_activating_double_effect_flags(self):
        if self.is_activating_double_effect == True and self.shared_is_activating_double_effect.value == False:
            self.is_activating_double_effect = False
            self.already_activate_double_effect_times += 1
        else:
            self.is_activating_double_effect = self.shared_is_activating_double_effect.value

    # 更新剩余时间
    def update_shared_time_left(self):
        if self.shared_time_left.value != self.time_left:
            self.time_left = self.shared_time_left.value
            self.logger.log(f"update time left{self.time_left}")
        # else:
        #     print(f"shared time left{self.shared_time_left} and local time left{self.time_left}")

    # hero_alert的辅助函数，将真实世界坐标转换为图像坐标
    def convert_to_image_coords(self, x, y, img_width, img_height, real_width, real_height):
        img_x = int((x / real_width) * img_width)
        img_y = int(img_height - (y / real_height) * img_height)
        return img_x, img_y


    # 解析hero_xyz信息
    def parse_hero_xyz(self):
        for info in self.our_car_infos:
            our_car_id, our_center_xy, our_camera_xyz, our_field_xyz, our_color = info
            if(our_car_id == self.my_cars_id[0]):
                self.our_hero_xyz = our_field_xyz
                if our_field_xyz[1] > 7.5:
                    self.hero_state = 2
                else:
                    self.hero_state = 1
                break

    def topo_info(self):
        return self.topo_map.find_attackable_regions(self.enemy_car_infos)

    # 包含可视化展示，仅DEBUG使用
    def hero_alert(self, image):
        # Function to convert real-world coordinates to image coordinates

        if self.find_hero_times < 0:  # 因为自然衰减机制，在小于0时，重置为0
            self.find_hero_times = 0

        if self.my_color == "Red":
            color = (255, 0, 0)
        elif self.my_color == "Blue":
            color = (0, 0, 255)
        else:
            color = (0, 255, 0)
        enemy_car_infos = self.enemy_car_infos

        if enemy_car_infos == []:
            self.logger.log("enemy_car_infos is empty-----------------------")
            print("enemy_car_infos is empty-----------------------")
            return

        hero_x = -1
        hero_y = -1
        if self.our_hero_xyz is not None:
            hero_x = self.our_hero_xyz[0]
            hero_y = self.our_hero_xyz[1]
            hero_x = max(0, min(hero_x, 28))
            hero_y = max(0, min(hero_y, 15))
            # self.logger.log(f"find hero at {hero_x} {hero_y}")

        # 对hero_x和hero_y的判空在内部进行了，其他地方若使用hero_x和hero_y，需要注意判空
        if self.is_in_areas(hero_x, hero_y):
            self.find_hero_times += 2

        # Draw the areas , DEBUG
        if self.is_debug:
            for index, points in enumerate(self.area_list):
                # 检查敌方hero的field_xyz是否在区域内
                img_points = [self.convert_to_image_coords(x, y, image.shape[1], image.shape[0], 28, 15) for x, y in
                              points]
                img_points = np.array(img_points, dtype=np.int32)
                cv2.polylines(image, [img_points], isClosed=True, color=color, thickness=2)
                if self.find_hero_times >= self.hero_times_threshold:
                    # 将本区域填充为color色,DEBUG
                    cv2.fillPoly(image, [img_points], color)

        # 判断是否预警英雄并自然衰减
        if self.find_hero_times >= self.hero_times_threshold:
            self.is_alert_hero = True
            self.logger.log("Alert hero")
        else:
            self.is_alert_hero = False

        self.find_hero_times -= 1

        # Display the image,DEBUG
        if self.is_debug:
            # cv2.imshow('areas', image)
            # cv2.waitKey(1)
            # self.put_draw_queue(image)
            # print("put in ")
            pass

        # cv2.destroyAllWindows()

    # 判断车辆是否在指定区域内
    def is_in_areas(self, x, y):
        if x < 0 or y < 0:
            return False
        point = Point(x, y)
        for area in self.area_list:
            polygon = Polygon(area)
            if polygon.contains(point):
                return True
        return False

    def assit_hero(self):
        if self.our_hero_xyz is None:
            return
        self.hero_assit = BallisticTrajectory(self.our_hero_xyz,self.hero_v0)
        pitch,yaw = self.hero_assit.find_optimal_parameters()
        self.send_hero_assit_info["pitch"] = pitch
        self.send_hero_assit_info["yaw"] = yaw
        self.is_assit_hero = True

    def generate_send_map_infos(self, enemy_infos):
        send_map_infos = [[0, 0] for _ in range(len(self.enemy_id))]
        for enemy_car_info in enemy_infos:
            # 提取car_id和field_xyz
            track_id, car_id, field_xyz, is_valid = enemy_car_info[0], enemy_car_info[1], enemy_car_info[4], \
                enemy_car_info[6]
            # if track_id == -1: # 如果track_id为-1，说明没有检测到，不发送
            #     # print("not init continue")
            #     continue
            if field_xyz == []:
                # self.logger.log("send map field_xyz is empty")
                continue
            # 将所有信息打印
            # print("car_id:",car_id , "field_xyz:",field_xyz , "is_valid:",is_valid)
            # 提取x和y
            x, y = field_xyz[0], field_xyz[1]
            # x的控制边界，让他在[0,28]m , y控制在[0,15]m
            x = max(0, min(x, 28))
            y = max(0, min(y, 15))

            # 将所有车的x，y信息打包
            for i, enemy_id in enumerate(self.enemy_id):
                if car_id == enemy_id:
                    send_map_infos[i] = [x, y]
                    break
        return send_map_infos

    def alert_our_hero(self):
        if self.our_hero_xyz is None:
            return
        self.secure_our_hero = is_point_nearby_numpy(self.our_hero_xyz, self.generate_send_map_infos(self.enemy_car_infos))





    # 开启线程
    def start(self):
        self.start = True
        self.threading.start()

    # 关闭线程
    def stop(self):
        self.logger.log("Messager stop")
        self.working_flag = False
        # self.receiver.stop()
        # self.threading.join()

    # 更新剩余时间
    # def update_time_left(self):
    #     self.time_left = self.receiver.get_time_left()

    # update engine state:
    def update_engine_state(self, engine_state):
        # 解析engine_state
        # print("update engine state")
        self.logger.log(f"update engine state{engine_state}")
        if engine_state == []:
            return
        # 解析engine_state
        with self.map_lock:
            if engine_state.len() == 1 :
                for i,id in enumerate(self.enemy_id):

                        self.send_map_infos[i] = engine_state.values()[0]
            else:
                for id, value in engine_state:
                    # TODO
                    if id == 2 and self.time_left > 330 :
                        for i,id in enumerate(self.enemy_id):
                            if id == 2 or id == 102:
                                self.send_map_infos[i] = value
                    elif id == 1 and self.time_left <= 330:
                        for i,id in enumerate(self.enemy_id):
                            if id == 2 or id == 102:
                                self.send_map_infos[i] = value


    # 更新敌方车辆信息
    def update_enemy_car_infos(self, enemy_car_infos):
        # 如果为空，直接返回
        with self.map_lock:
            self.enemy_car_infos = enemy_car_infos
            self.predict_enemy(enemy_car_infos)
        # print(f"enemy car info{self.enemy_car_infos}")
        # self.logger.log(f"update enemy car infos{self.enemy_car_infos}")

    def predict_enemy(self,enemy_infos):
        for enemy_info in enemy_infos:
            track_id, car_id, field_xyz, is_valid = enemy_info[0], enemy_info[1], enemy_info[4], enemy_info[6]
            x,y = field_xyz[0],field_xyz[1]
            if car_id % 100 == 1 :
                self.hero_predictor.update_cord([x,y])
            if car_id % 100 == 2:
                self.engine_predictor.update_cord([x,y])
                

    # 更新我方车辆信息
    def update_our_car_infos(self, our_car_infos):
        with self.map_lock:
            self.our_car_infos = our_car_infos
            self.parse_hero_xyz()
            

    # 更新哨兵预警信息
    def update_sentinel_alert_info(self, sentinel_alert_info):
        with self.sentinel_lock:
            # print("update",sentinel_alert_info)
            self.sentinel_alert_info = sentinel_alert_info


    # 新版本发送地方车辆位置，一次性发送全部车辆，需要补全
    def send_map(self, infos):
        self.sender.send_enemy_location(infos)
        self.logger.log(f'Sent map info: {infos}')

    # 发送哨兵预警角信息
    def send_sentry_alert_angle(self):

        sentinel_alert_info = []
        with self.sentinel_lock:
            sentinel_alert_info = self.sentinel_alert_info
        # print("alert info",sentinel_alert_info)
        if sentinel_alert_info == []:
            return
        carID, distance, quadrant = sentinel_alert_info
        self.sender.send_sentinel_alert_info(carID, distance, quadrant)
        # print("send_sentinel_alert_info")

    # 发送哨兵预警英雄信息
    def send_sentinel_alert_hero(self):
        if self.is_alert_hero:
            self.sender.send_hero_alert_info(self.is_alert_hero)

    def send_hero_assit(self):
        if self.is_assit_hero:
            self.sender.send_hero_assit_info(self.send_hero_assit_info,self.is_assit_hero)

    def send_alert_hero(self):
        if self.secure_our_hero:
            self.sender.send_alert_our_hero(self.secure_our_hero)



    # 更新flag，将共享内存中更新的信息解析，更新本地flag
    def update_flags(self):
        # 更新双倍易伤相关flag
        # self.update_double_effect_flags()
        # 更新标记进度
        self.parse_mark_process()
        # 更新血量信息
        self.parse_enemy_health_info()

    # 更新血量信息
    def parse_enemy_health_info(self):
        if self.enemy_health_info[0] <= 0:
            self.hero_is_dead = True
        else:
            self.hero_is_dead = False

    # 更新标记进度，用单flag太唐了
    def parse_mark_process(self):
        if self.mark_progress[0] >= 100:
            self.hero_is_marked = True
        else:
            self.hero_is_marked = False

        if self.mark_progress[1] >= 100:
            self.engineer_is_marked = True
        else:
            self.engineer_is_marked = False

        if self.mark_progress[2] >= 100:
            self.standard_3_is_marked = True
        else:
            self.standard_3_is_marked = False

        if self.mark_progress[3] >= 100:
            self.standard_4_is_marked = True
        else:
            self.standard_4_is_marked = False

        if self.mark_progress[4] >= 100:
            self.standard_5_is_marked = True
        else:
            self.standard_5_is_marked = False

        if self.mark_progress[5] >= 100:
            self.sentinel_is_marked = True
        else:
            self.sentinel_is_marked = False

        # 更新被标记的数量
        self.marked_num = sum([
            self.hero_is_marked,
            self.engineer_is_marked,
            self.standard_3_is_marked,
            self.standard_4_is_marked,
            self.standard_5_is_marked,
            self.sentinel_is_marked
        ])
        self.logger.log(f'Marked situation: {self.mark_progress}')

    # 根据已发送情况自动标号发双倍易伤
    def auto_send_double_effect_decision(self):
        # self.sender.send_radar_double_effect_info(self.already_activate_double_effect_times + 1)
        self.sender.send_radar_double_effect_info(2)
        self.sender.send_radar_double_effect_info(1)

    # 新双倍易伤发送机制
    def send_double_effect_decision(self):
        # 如果没有双倍易伤机会或正在触发双倍易伤，不发送
        if self.have_double_effect_times == 0 or self.is_activating_double_effect or self.already_activate_double_effect_times == 2:
            # print("no double times status",self.have_double_effect_times , self.is_activating_double_effect , self.already_activate_double_effect_times)
            # return
            pass

        # 如果对面英雄存活，且英雄在危险区域，且英雄被标记或发现次数超过阈值，发送双倍易伤
        if (self.is_alert_hero and (self.hero_is_marked or self.find_hero_times >= self.send_double_threshold)):
            # 关于已发送次数的计算，考虑读取是否正在触发双倍以上，如果从正在触发变为未触发，则计算已发送次数+1，在此期间请求的仍为上一次的次数，视作不合法不会触发第二次双倍易伤请求
            # 本次结束后，下降沿修改次数加一，则下次调用时自动加一，能触发第二次
            self.auto_send_double_effect_decision()
            if self.hero_is_marked:
                self.logger.log(
                    f'Sent double effect info: {self.already_activate_double_effect_times + 1} because hero is marked')
            else:
                self.logger.log(
                    f'Sent double effect info: {self.already_activate_double_effect_times + 1} because hero is in danger area more than {self.send_double_threshold} times')
            return
        else:
            # print("hero is dead",self.hero_is_dead)
            # print("is alert hero:" , self.is_alert_hero)
            # print("find_hero_times" , self.find_hero_times)
            pass

        # 为了好看，发送的分两种情况
        if self.marked_num >= 4:
            self.auto_send_double_effect_decision()
            self.logger.log(
                f'Sent double effect info: {self.already_activate_double_effect_times + 1} because marked num >= 4')
            return

        if self.dart_target >= 1:
            self.auto_send_double_effect_decision()
            print("发送双倍易伤请求，因为飞镖目标为2")


    # 频率控制
    def frequency_control(self, last_time, fps):
        time_interval = time.time() - last_time
        if time_interval < 1 / fps:
            time.sleep(1 / fps - time_interval)

    # 根据时间发送自主决策信息
    # 发送双倍易伤信息
    def send_double_effect_times_to_car(self):
        self.sender.send_double_effect_times_to_car(self.my_sentinel_id, self.have_double_effect_times)

    # 线程主函数
    def main_loop(self):
        # 问题出在这里，阻塞导致效率很低
        self.receiver.start()
        # 可视化
        try:
            map_image = cv2.imread("/home/nvidia/RadarWorkspace/code/Radar_Develop/data/map.jpg")
        except Exception as e:
            self.logger.log(f"Read map image error: {e}")
            print(f"Read map image error: {e}")
            # 随便创建一个全白的map_image
            map_image = np.ones((480, 640, 3), np.uint8) * 255
            self.status_logger.log(f"image create error {e}")

        while True:
            # 线程判断，主体代码不能超过这里
            if not self.working_flag:
                # print("messager stop")
                break
            # 主体代码在这里以下------------------------------------------------
            print("time left", self.time_left)
            # print("messager")
            self.last_main_loop_time = Tools.frame_control_sleep(self.last_main_loop_time, 10)

            # 更新共享内存变量
            try:
                self.update_shared_info()
                # 解析本地flag，更新flag
                self.update_flags()
            except Exception as e:
                self.logger.log(f"update error{e}")
            # 如果时间不为-1 ， 存剩余时间
            if self.time_left != -1:
                self.logger.log(f"Time left: {self.time_left}")
            # 更新英雄预警

            show_map_image = copy.deepcopy(map_image)

            # print("hero alert")

            # try:
            # self.logger.log("enter hero alert check")
            self.hero_alert(show_map_image)
            self.send_hero_assit()
            self.send_alert_hero()
            # self.logger.log("exit hero alert check")
            # except Exception as e:
            #     self.logger.log(f"Hero alert error: {e}")
            #     print(f"Hero alert error: {e}")

            # 发送自主决策信息
            self.send_double_effect_decision()
            # self.sender.send_radar_double_effect_info(2)
            self.sender.send_radar_double_effect_info(1)
            # self.logger.log("send double effect decision")

            # 发送哨兵预警信息
            # self.send_sentinel_alert_hero()
            # self.logger.log("send sentry alert hero")

            # 发送英雄辅助吊射信息
            # self.send_hero_assit()

            # 发送步兵双倍易伤信息
            self.send_double_effect_times_to_car()

            # 提取enemy_car_infos
            enemy_car_infos = self.enemy_car_infos
            # print("ready to send map info",enemy_car_infos)

            # 不可信的车辆信息，也发送，但是要控制在0.45s发送一次，先记录，最后再统一发送
            # 不可信的车辆信息
            # not_valid_enemy_car_infos = []

            # 新发送打包,嵌套列表，每个元素是一个列表，包含对应号的 x , y

            # print("send_map_infos" , send_map_infos)
            # 先将小地图发送信息的生命周期减1
            for i, enemy_id in enumerate(self.enemy_id):
                self.send_map_info_is_latest[i] -= 1 if self.send_map_info_is_latest[i] > 0 else 0

            for i, life_time in enumerate(self.send_map_info_is_latest):
                if life_time <= 0:
                    if i == 0 and 130 <= self.time_left <= 405:
                        self.send_map_infos[i] = self.hero_predictor.get_result()
                    elif i == 1 and 330 <= self.time_left <= 405:
                        self.send_map_infos[i] = self.engine_predictor.get_result()
                    else:
                        self.send_map_infos[i] = [0.0, 0.0]

            for enemy_car_info in enemy_car_infos:
                # 提取car_id和field_xyz
                track_id, car_id, field_xyz, is_valid = enemy_car_info[0], enemy_car_info[1], enemy_car_info[4], \
                enemy_car_info[6]
                # if track_id == -1: # 如果track_id为-1，说明没有检测到，不发送
                #     # print("not init continue")
                #     continue
                if field_xyz == []:
                    continue
                # 将所有信息打印
                # print("car_id:",car_id , "field_xyz:",field_xyz , "is_valid:",is_valid)
                # 提取x和y
                x, y = field_xyz[0], field_xyz[1]
                # x的控制边界，让他在[0,28]m , y控制在[0,15]m
                x = max(0, min(x, 28))
                y = max(0, min(y, 15))
                # 将所有车的x，y信息打包
                for i, enemy_id in enumerate(self.enemy_id):
                    if car_id == enemy_id:
                        self.send_map_infos[i] = [x, y]
                        self.send_map_info_is_latest[i] = 5
                        break

            # 一秒钟记录一次状态
            if self.is_next_second():
                self.status_logger.log(
                    f"status record:is activating double effect flag{self.is_activating_double_effect} , enemy health info{self.enemy_health_info} , mark progress{self.mark_progress} , have double effect times{self.have_double_effect_times} , time left{self.time_left} , dart target{self.dart_target}")

            # 打印打包好后的信息
            # print("send_map_infos",self.send_map_infos)
            # 发送 , 采用skip的方式控制发送频率，不用sleep影响主线程的帧率
            is_skip, self.last_send_map_time = Tools.frame_control_skip(self.last_send_map_time, 10)
            if not is_skip:
                self.send_map(self.send_map_infos)
            # print("send_map" ,  car_id , x , y)
            # 控制发送频率为5hz


        if not self.receiver.working_flag:
            self.receiver.stop()


