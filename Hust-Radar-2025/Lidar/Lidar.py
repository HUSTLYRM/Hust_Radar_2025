# -*- coding: utf-8 -*-
# 构建Lidar类，作为激光雷达接收类，构建一个ros节点持续订阅/livox/lidar话题，把点云信息写入PcdQueue,整个以子线程形式运行
from .PointCloud import *
import threading
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from PCLMatcher.pcl_matcher import PCL_Matcher
from PCLMatcher.clear import Clear


class Lidar:
    def __init__(self,cfg):
        # 标志位
        self.flag = False  # 激光雷达接收启动标志
        self.init_flag = False # 激光雷达接收线程初始化标志
        self.working_flag = False  # 激光雷达接收线程启动标志
        self.threading = None  # 激光雷达接收子线程
        self.stop_event = threading.Event()  # 线程停止事件

        # 配准用
        self.matcher = None
        self.trans_matrix = None
        self.clear = None


        # 参数
        self.height_threshold = cfg["lidar"]["height_threshold"]  # 自身高度，用于去除地面点云
        self.min_distance = cfg["lidar"]["min_distance"]  # 最近距离，距离小于这个范围的不要
        self.max_distance = cfg["lidar"]["max_distance"]  # 最远距离，距离大于这个范围的不要
        self.lidar_topic_name = cfg["lidar"]["lidar_topic_name"] # 激光雷达话题名

        # 点云队列
        self.pcdQueue = PcdQueue(max_size=10) # 将激光雷达接收的点云存入点云队列中，读写上锁？

        # 激光雷达线程
        self.lock = threading.Lock()  # 线程锁

        if not self.init_flag:
            # 当雷达还未有一个对象时，初始化接收节点
            self.listener_begin(self.lidar_topic_name)
            # print("listener_begin")
            self.init_flag = True
            self.threading = threading.Thread(target=self.main_loop, daemon=True)


    # 线程启动
    def start(self):
        '''
        开始子线程，即开始spin
        '''
        if not self.working_flag:
            self.working_flag = True
            self.threading.start()

            # print("start@")

    # 线程关闭
    def stop(self):
        '''
        结束子线程
        '''
        if self.working_flag and self.threading is not None: # 关闭时写错了之前，写成了if not self.working_flag
            self.stop_event.set()
            rospy.signal_shutdown('Stop requested')
            self.working_flag = False
            print("stop")

    # 安全关闭子线程
    # def _async_raise(self,tid, exctype):
    #     """raises the exception, performs cleanup if needed"""
    #     tid = ctypes.c_long(tid)
    #     if not inspect.isclass(exctype):
    #         exctype = type(exctype)
    #     res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    #     if res == 0:
    #         raise ValueError("invalid thread id")
    #     elif res != 1:
    #         # """if it returns a number greater than one, you're in trouble,
    #         # and you should call it again with exc=NULL to revert the effect"""
    #         ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
    #         raise SystemError("PyThreadState_SetAsyncExc failed")
    #
    # # 停止线程，中间方法
    # def stop_thread(self,thread):
    #     self._async_raise(thread.ident, SystemExit)


    # 节点启动
    def listener_begin(self,laser_node_name="/livox/lidar"):
        rospy.init_node('laser_listener', anonymous=True)
        # print("init ")
        rospy.Subscriber(laser_node_name, PointCloud2, self.callback)
        # print("sub")

    # 订阅节点子线程
    def main_loop(self):
        # 通过将spin放入子线程来防止其对主线程的阻塞
        rospy.spin()



    def callback(self,data):
        '''
        子线程函数，对于/livox/lidar topic数据的处理 , data是传入的
        '''

        if self.stop_event.is_set():
            print("stop is set")
            return
        if self.working_flag:

            # 获取点云
            pc = np.float32(point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)).reshape(
                -1, 3)

            # 过滤点云
            dist = np.linalg.norm(pc, axis=1)  # 计算点云距离

            pc = pc[dist > self.min_distance]  # 雷达近距离滤除
            # 第二次需要重新计算距离，否则会出现维度不匹配
            dist = np.linalg.norm(pc, axis=1)  # 计算点云距离
            pc = pc[dist < self.max_distance]  # 雷达远距离滤除

            # 如果在地面+5cm以上，才保留，在地面的点为-height_threshold,
            # pc = pc[pc[:, 2] > (-1 * self.height_threshold)]

            # 存入点云队列
            with self.lock:
                # 存入点云队列
                self.pcdQueue.add(pc)

    # 获取所有点云
    def get_all_pc(self):
        with self.lock:
            return self.pcdQueue.get_all_pc()


    # del
    def __del__(self):
        self.stop()

    def substract_pcd(self,pcd):
        self.pcdQueue.bg_rm(pcd)
        return self.pcdQueue.bg_rm.processed_pcd

# lidar = Lidar(main_cfg)
# lidar.start()
# while 1 :
#     pc = lidar.get_all_pc()
#     # print("get")


# class Radar(object):
#
#     # the global member of the Radar class
#     __init_flag = False  # 闆疯揪鍚姩鏍囧織
#     __working_flag = False  # 闆疯揪鎺ユ敹绾跨▼鍚姩鏍囧織
#     __threading = None  # 闆疯揪鎺ユ敹瀛愮嚎绋�
#
#     __lock = threading.Lock()  # 绾跨▼閿�
#     __queue = []  # 涓€涓垪琛紝瀛樻斁闆疯揪绫诲悇涓璞＄殑Depth Queue
#
#     __record_times = 0  # 宸插瓨鐐逛簯鐨勬暟閲�
#
#     __record_list = []
#
#     __record_max_times = 100  # 鏈€澶у瓨鐐逛簯鏁伴噺
#
#     def __init__(self, K_0, C_0, E_0, queue_size=200, imgsz=(3088, 2064)):
#         '''
#         闆疯揪澶勭悊绫伙紝瀵规瘡涓浉鏈洪兘瑕佸垱寤轰竴涓璞�
#
#         :param K_0:鐩告満鍐呭弬
#         :param C_0:鐣稿彉绯绘暟
#         :param E_0:闆疯揪鍒扮浉鏈哄鍙�
#         :param queue_size:闃熷垪鏈€澶ч暱搴�
#         :param imgsz:鐩告満鍥惧儚澶у皬
#         '''
#         if not Radar.__init_flag:
#             # 褰撻浄杈捐繕鏈湁涓€涓璞℃椂锛屽垵濮嬪寲鎺ユ敹鑺傜偣
#             Radar.__laser_listener_begin(LIDAR_TOPIC_NAME)
#             Radar.__init_flag = True
#             Radar.__threading = threading.Thread(target=Radar.__main_loop, daemon=True)
#         self._no = len(Radar.__queue)  # 璇ュ璞″搴斾簬鏁翠釜闆疯揪瀵硅薄鍒楄〃鐨勫簭鍙�
#         self._K_0 = K_0
#         self._C_0 = C_0
#         Radar.__queue.append(DepthQueue(queue_size, imgsz, K_0, C_0, E_0))
#
#     @staticmethod
#     def start():
#         '''
#         寮€濮嬪瓙绾跨▼锛屽嵆寮€濮媠pin
#         '''
#         if not Radar.__working_flag:
#             Radar.__threading.start()
#             Radar.__working_flag = True
#
#     @staticmethod
#     def stop():
#         '''
#         缁撴潫瀛愮嚎绋�
#         '''
#         if Radar.__working_flag:
#             stop_thread(Radar.__threading)
#             Radar.__working_flag = False
#
#     @staticmethod
#     def __callback(data):
#         '''
#         瀛愮嚎绋嬪嚱鏁帮紝瀵逛簬/livox/lidar topic鏁版嵁鐨勫鐞�
#         '''
#         if Radar.__working_flag:
#             Radar.__lock.acquire()
#
#             pc = np.float32(
#                 point_cloud2.read_points_list(data, field_names=("x", "y", "z"), skip_nans=True)).reshape(-1, 3)
#
#             dist = np.linalg.norm(pc, axis=1)
#
#             pc = pc[dist > 0.4]  # 闆疯揪杩戣窛绂绘护闄�
#             # do record
#             if Radar.__record_times > 0:
#
#                 Radar.__record_list.append(pc)
#                 print("[INFO] recording point cloud {0}/{1}".format(Radar.__record_max_times - Radar.__record_times,
#                                                                     Radar.__record_max_times))
#                 if Radar.__record_times == 1:
#                     try:
#                         if not os.path.exists(PC_STORE_DIR):
#                             os.mkdir(PC_STORE_DIR)
#                         with open("{0}/{1}.pkl"
#                                           .format(PC_STORE_DIR,
#                                                   datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M')),
#                                   'wb') as f:
#                             pkl.dump(Radar.__record_list, f)
#                         Radar.__record_list.clear()
#                         print("[INFO] record finished")
#                     except:  # 褰撳嚭鐜扮鐩樻湭鎸傝浇绛夋儏鍐碉紝瀵艰嚧鏂囦欢澶归兘鏃犳硶鍒涘缓
#                         print("[ERROR] The point cloud save dir even doesn't exist on this computer!")
#                 Radar.__record_times -= 1
#             # update every class object's queue
#             for q in Radar.__queue:
#                 q.push_back(pc)
#
#             Radar.__lock.release()
#
#     @staticmethod
#     def __laser_listener_begin(laser_node_name="/livox/lidar"):
#         rospy.init_node('laser_listener', anonymous=True)
#         rospy.Subscriber(laser_node_name, PointCloud2, Radar.__callback)
#
#     @staticmethod
#     def __main_loop():
#         # 閫氳繃灏唖pin鏀惧叆瀛愮嚎绋嬫潵闃叉鍏跺涓荤嚎绋嬬殑闃诲
#         rospy.spin()
#         # 褰搒pin璋冪敤鏃讹紝subscriber灏变細寮€濮嬭疆璇㈡帴鏀舵墍璁㈤槄鐨勮妭鐐规暟鎹紝鍗充笉鏂皟鐢╟allback鍑芥暟
#
#     @staticmethod
#     def start_record():
#         '''
#         寮€濮嬪綍鍒剁偣浜�
#         '''
#         if Radar.__record_times == 0:
#             Radar.__record_times = Radar.__record_max_times
#
#     def detect_depth(self, rects):
#         '''
#         鎺ュ彛鍑芥暟锛屼紶鍏ヨ鐢叉澘bounding box杩斿洖瀵瑰簲锛坸0,y0,z_c)鍊�
#         ps:杩欎釜x0,y0鏄綊涓€鍖栫浉鏈哄潗鏍囩郴涓€硷紝涓庝笅鍙傛暟涓寚浠ounding box宸︿笂鏂圭偣鍧愭爣涓嶅悓
#
#         :param rects: armor bounding box, format: (x0,y0,w,h)
#         '''
#         Radar.__lock.acquire()
#         # 閫氳繃self.no鏉ユ寚瀹氳瀵硅薄瀵瑰簲鐨勬繁搴﹂槦鍒�
#         results = Radar.__queue[self._no].detect_depth(rects)
#         Radar.__lock.release()
#         return results
#
#     def read(self):
#         '''
#         debug鐢紝杩斿洖娣卞害闃熷垪褰撳墠鐨勬繁搴﹀浘
#         '''
#         Radar.__lock.acquire()
#         depth = Radar.__queue[self._no].depth.copy()
#         Radar.__lock.release()
#         return depth
#
#     def check_radar_init(self):
#         '''
#         妫€鏌ヨ闃熷垪缁戝畾闃熷垪缃綅绗︼紝鏉ョ‘瀹氶浄杈炬槸鍚︽甯稿伐浣�
#         '''
#         if Radar.__queue[self._no].init_flag:
#             Radar.__queue[self._no].init_flag = False
#             return True
#         else:
#             return False
#
#     def __del__(self):
#         Radar.stop()
